import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from pnp_detect.pnp_estimator import estimate_pose,rectangle_model,_transform_points


class PnPTests(unittest.TestCase):
    def setUp(self):
        self.obj=rectangle_model(.42,.30)
        self.k=np.float64([[740,0,640],[0,735,360],[0,0,1]])
        self.dist=np.float64([.04,-.015,.001,-.002,0.])
        self.rv=np.float64([.48,-.35,.2]);self.tv=np.float64([.02,-.015,1.1])
        self.image=cv2.projectPoints(self.obj,self.rv,self.tv,self.k,self.dist)[0].reshape(-1,2)
        self.mapping={'confirmed':True,'model_to_image_indices':[0,1,2,3],'basis':'Known SYNTHETIC projection correspondence'}
        self.geometry={'confirmed':True,'status':'native','source_resolution_wh':[1280,720],
                       'input_resolution_wh':[1280,720],'source_to_input_3x3':np.eye(3).tolist()}

    def solve(self,**kwargs):
        opts=dict(physical_corner_mapping=self.mapping,input_geometry=self.geometry,calibration_resolution_wh=[1280,720])
        opts.update(kwargs)
        return estimate_pose(self.obj,self.image,self.k,self.dist,**opts)

    def test_recover_known_synthetic_pose(self):
        result=self.solve()
        self.assertTrue(result['success'],result['failure_reason'])
        np.testing.assert_allclose(result['pose']['tvec_m'],self.tv,atol=1e-7)
        np.testing.assert_allclose(result['pose']['rotation_matrix'],cv2.Rodrigues(self.rv)[0],atol=1e-7)
        self.assertLess(result['pose']['reprojection_rmse_px'],1e-6)
        self.assertFalse(result['robot_ready'])

    def test_nonuniform_resize_crop_restored_with_distortion(self):
        original=self.image.copy()
        h=np.float64([[.7,0,-100],[0,.8,-40],[0,0,1]])
        self.image=_transform_points(original,h)
        geometry=copy.deepcopy(self.geometry)
        geometry.update(status='transformed',input_resolution_wh=[800,500],source_to_input_3x3=h.tolist())
        result=self.solve(input_geometry=geometry)
        self.assertTrue(result['success'],result['failure_reason'])
        np.testing.assert_allclose(result['pose']['tvec_m'],self.tv,atol=1e-7)

    def test_explicit_permutation_not_image_labels(self):
        self.image=np.roll(self.image,1,axis=0)
        mapping=copy.deepcopy(self.mapping);mapping['model_to_image_indices']=[1,2,3,0]
        result=self.solve(physical_corner_mapping=mapping)
        self.assertTrue(result['success'])
        np.testing.assert_allclose(result['pose']['tvec_m'],self.tv,atol=1e-7)

    def test_missing_or_symmetric_unconfirmed_mapping_rejected(self):
        for mapping in (None,{'confirmed':False},dict(self.mapping,basis=None),dict(self.mapping,model_to_image_indices=[0,0,2,3])):
            with self.subTest(mapping=mapping),self.assertRaises(ValueError):self.solve(physical_corner_mapping=mapping)

    def test_unknown_or_wrong_image_geometry_rejected(self):
        for geometry in ({'status':'unknown'},dict(self.geometry,confirmed=False),
                         dict(self.geometry,source_resolution_wh=[640,480]),
                         dict(self.geometry,source_to_input_3x3=[[1,0,1],[0,1,0],[0,0,1]])):
            with self.subTest(geometry=geometry),self.assertRaises(ValueError):self.solve(input_geometry=geometry)

    def test_nonfinite_duplicate_and_nonplanar_points_rejected(self):
        for name in ('nan','duplicate','nonplanar','collinear'):
            old=self.obj.copy()
            if name=='nan':self.obj[0,0]=np.nan
            elif name=='duplicate':self.obj[0]=self.obj[1]
            elif name=='nonplanar':self.obj[0,2]=.1
            else:self.obj[:,1:]=0
            with self.subTest(name=name),self.assertRaises(ValueError):self.solve()
            self.obj=old

    def test_invalid_intrinsics_distortion_units(self):
        for key,value in [('length_unit','mm'),('calibration_resolution_wh',None)]:
            with self.subTest(key=key),self.assertRaises(ValueError):self.solve(**{key:value})
        k=self.k.copy();self.k[0,0]=-100
        with self.assertRaises(ValueError):self.solve()
        self.k=k
        self.dist=None
        with self.assertRaises(ValueError):self.solve()

    def test_pose_ambiguity_is_not_hidden(self):
        rv=np.float64([.02,-.01,.1]);tv=np.float64([0,0,2.5])
        self.image=cv2.projectPoints(self.obj,rv,tv,self.k,self.dist)[0].reshape(-1,2)
        result=self.solve()
        self.assertFalse(result['success']);self.assertIsNone(result['pose'])
        self.assertIn('ambiguous_planar_pose',result['failure_reason'])
        self.assertGreaterEqual(len(result['alternatives']),2)

    def test_inconsistent_quad_fails_reprojection_gate(self):
        self.image[0]+=[20,12]
        result=self.solve(ambiguity_margin_px=0,max_reprojection_rmse_px=.2,max_point_error_px=.5)
        self.assertFalse(result['success']);self.assertIsNone(result['pose'])
        self.assertIn('reprojection',result['failure_reason'])

    def test_unmeasured_rectangle_rejected(self):
        for dims in ((None,None),(0,.3),(-1,.3),(float('nan'),.3)):
            with self.assertRaises(ValueError):rectangle_model(*dims)

    def test_cli_synthetic_pose_and_exact_image_binding(self):
        import hashlib
        import yaml
        with tempfile.TemporaryDirectory() as tmp:
            tmp=Path(tmp);image=tmp/'synthetic.png'
            cv2.imwrite(str(image),np.zeros((720,1280,3),np.uint8))
            corners=tmp/'corners.json'
            record={'success':True,'source':'manual','image_sha256':hashlib.sha256(image.read_bytes()).hexdigest(),
                    'image_resolution_wh':[1280,720],'corners':self.image.tolist(),'data_kind':'SYNTHETIC_ONLY'}
            corners.write_text(json.dumps(record))
            cfg={'data_kind':'SYNTHETIC_ONLY','camera':{'confirmed':True,'calibration_source':'Known synthetic camera',
                 'model':'opencv_pinhole','resolution_wh':[1280,720],'camera_matrix':self.k.tolist(),'dist_coeffs':self.dist.tolist()},
                 'basket':{'confirmed':True,'measurement_source':'Known synthetic model','length_m':.42,'width_m':.30,
                           'corner_definition':'virtual_intersections_of_straight_outer_rim_edges'},
                 'correspondence':self.mapping,'input_geometry':self.geometry}
            config=tmp/'pnp.yaml';config.write_text(yaml.safe_dump(cfg))
            output=tmp/'pose.json';vis=tmp/'pose.png'
            cmd=[sys.executable,str(ROOT/'scripts/estimate_pose.py'),'--image',str(image),'--corners-file',str(corners),
                 '--pnp-config',str(config),'--output',str(output),'--visualization',str(vis)]
            run=subprocess.run(cmd,capture_output=True,text=True)
            self.assertEqual(run.returncode,0,run.stderr)
            result=json.loads(output.read_text());self.assertTrue(result['success'])
            self.assertEqual(result['data_kind'],'SYNTHETIC_ONLY')
            np.testing.assert_allclose(result['pose']['tvec_m'],self.tv,atol=1e-7)
            self.assertTrue(vis.is_file())
            record['image_sha256']='wrong';corners.write_text(json.dumps(record))
            output.unlink();vis.unlink()
            run=subprocess.run(cmd,capture_output=True,text=True)
            self.assertEqual(run.returncode,2)
            self.assertIn('SHA256',json.loads(output.read_text())['failure_reason'])
            self.assertFalse(vis.exists())

    def test_real_cli_refuses_placeholder_calibration(self):
        with tempfile.TemporaryDirectory() as tmp:
            image=Path(tmp)/'image.png';cv2.imwrite(str(image),np.zeros((100,100,3),np.uint8))
            output=Path(tmp)/'pose.json'
            run=subprocess.run([sys.executable,str(ROOT/'scripts/estimate_pose.py'),'--image',str(image),'--output',str(output)],capture_output=True,text=True)
            self.assertEqual(run.returncode,2)
            result=json.loads(output.read_text())
            self.assertFalse(result['success']);self.assertIsNone(result['pose'])
            self.assertIn('confirmed real data',result['failure_reason'])


if __name__=='__main__':unittest.main()
