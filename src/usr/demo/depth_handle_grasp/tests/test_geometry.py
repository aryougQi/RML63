import sys
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parents[1]
SRC=HERE.parents[2]
sys.path.insert(0,str(HERE/'scripts'))
from grasp_geometry import (transform,validate,optical_to_base,flange_target,
                            offset_target,GripperGeometry,rotation_matrix,
                            observed_obstacle_points,inward_grasp,collision_samples,raised_pad_grasp)


class GraspGeometryTests(unittest.TestCase):
    def test_raised_grasp_keeps_wood_inside_original_contact_pads(self):
        target=transform([-.3,0,.14],np.diag([1.,-1.,-1.]))
        pads=self.gripper.at_width(.016)['pad_bounds_tcp_m']
        for height in [0,.003,.006]:
            out=raised_pad_grasp(target,target[:3,3],[0,0,1],.007,pads,height)
            self.assertAlmostEqual(out[2,3],.14+height)
            np.testing.assert_array_equal(out[:3,:3],target[:3,:3])
        for height in [-.001,.01,float('nan')]:
            with self.assertRaises(ValueError):
                raised_pad_grasp(target,target[:3,3],[0,0,1],.007,pads,height)
        self.assertAlmostEqual(target[2,3],.14)

    def test_collision_sampling_preserves_nearby_native_points(self):
        points=np.array([[.001,.002,.003],[.002,.003,.004],[.101,.002,.003]])
        centers,detail=collision_samples(points,np.zeros(3),.005,.06)
        np.testing.assert_array_equal(detail,points[:2])
        self.assertEqual(centers.shape,(1,3))
        self.assertTrue(np.all(abs(points[2]-centers[0])<=.005/2+1e-12))
        original=points.copy()
        collision_samples(points,np.zeros(3),.005,.06)
        np.testing.assert_array_equal(points,original)
        centers,detail=collision_samples(np.empty((0,3)),np.zeros(3),.005,.06)
        self.assertEqual(centers.shape,(0,3));self.assertEqual(detail.shape,(0,3))
        centers,detail=collision_samples(points[:1],np.zeros(3),.005,.06)
        self.assertEqual(centers.shape,(0,3));self.assertEqual(detail.shape,(1,3))

    def test_inward_grasp_points_toward_base_and_keeps_center(self):
        target=transform([-.3,.05,.14],np.diag([1.,-1.,-1.]))
        for angle in [0,5,10,15]:
            tilted=inward_grasp(target,[0,0,1],[1,0,0],angle)
            validate(tilted)
            np.testing.assert_allclose(tilted[:3,3],target[:3,3])
            self.assertAlmostEqual(tilted[:3,2]@target[:3,2],np.cos(np.radians(angle)))
            if angle:
                self.assertGreater(tilted[:2,2]@(-target[:2,3]),0)
            self.assertGreater(tilted[:3,0]@[1,0,0],.95)
        np.testing.assert_array_equal(target[:3,:3],np.diag([1.,-1.,-1.]))

    def test_inward_grasp_rejects_limits_and_bad_surface(self):
        target=transform([-.3,0,.14],np.diag([1.,-1.,-1.]))
        for angle in [-5,16,float('nan')]:
            with self.assertRaises(ValueError):inward_grasp(target,[0,0,1],[1,0,0],angle)
        with self.assertRaises(ValueError):inward_grasp(target,[0,1,0],[1,0,0],10)
        with self.assertRaises(ValueError):inward_grasp(target,[0,0,1],[1,0,0],15,max_axis_approach_dot=.1)
        at_base=target.copy();at_base[:2,3]=0
        with self.assertRaises(ValueError):inward_grasp(at_base,[0,0,1],[1,0,0],5)

    def test_diagonal_approaches_preserve_tilt_and_contact_axis(self):
        target=transform([-.3,0,.14],np.diag([1.,-1.,-1.]))
        for azimuth in [-90,-45,45,90]:
            tilted=inward_grasp(target,[0,0,1],[1,0,0],10,azimuth_deg=azimuth)
            validate(tilted)
            self.assertAlmostEqual(tilted[:3,2]@target[:3,2],np.cos(np.radians(10)))
            self.assertGreaterEqual(tilted[0,2],-1e-12)
            np.testing.assert_array_equal(tilted[:3,3],target[:3,3])
        with self.assertRaises(ValueError):inward_grasp(target,[0,0,1],[1,0,0],10,azimuth_deg=100)

    @classmethod
    def setUpClass(cls):
        cls.gripper=GripperGeometry(SRC/'rml63_tools_description/upstream/EG2-4C2.urdf',
            SRC/'rml63_tools_description/meshes/eg2_4c2',[0,0,.006909],[0,0,-np.pi/3])

    def test_factory_color_depth_composition_direction(self):
        flange=transform([.4,.1,.3],rotation_matrix(Rotation.from_euler('xyz',[.3,.1,-.4])))
        handeye=transform([.08,.06,.05],rotation_matrix(Rotation.from_euler('xyz',[.2,-.5,.7])))
        native=transform([.001,.014,-.007],rotation_matrix(Rotation.from_euler('xyz',[.01,-.02,.003])))
        actual=optical_to_base(flange,handeye,dict(translation_m=native[:3,3],rotation_matrix=native[:3,:3]))
        p=np.array([.03,-.05,.4,1.])
        np.testing.assert_allclose(actual@p,flange@(handeye@(native@p)),atol=1e-12)
        self.assertGreater(np.linalg.norm(actual[:3,3]-(flange@handeye)[:3,3]),.01)

    def test_flange_tcp_inverse_and_pregrasp_sign(self):
        tcp=self.gripper.at_width(.016)['flange_tcp']
        target=transform([-.35,.05,.14],rotation_matrix(Rotation.from_euler('xyz',[np.pi,0,.2])))
        flange=flange_target(target,tcp)
        np.testing.assert_allclose(flange@tcp,target,atol=1e-12)
        pre=offset_target(target,target[:3,2],.1)
        self.assertAlmostEqual(pre[2,3],.24)
        self.assertGreater(flange[2,3],target[2,3])

    def test_original_finger_tcp_is_not_tip(self):
        g=self.gripper
        contact=g.at_width(.016)
        self.assertAlmostEqual(contact['opening_m'],.016,places=8)
        self.assertAlmostEqual(contact['flange_tcp'][2,3],.137149754,places=6)
        self.assertLess(contact['flange_tcp'][2,3],.145)
        pads=contact['pad_centers_flange_m']
        np.testing.assert_allclose(pads.mean(0),contact['flange_tcp'][:3,3],atol=1e-12)
        direction=(pads[1]-pads[0])/.016
        self.assertAlmostEqual(abs(direction@contact['flange_tcp'][:3,1]),1.,places=7)
        self.assertGreater(abs(g.at_width(.032)['flange_tcp'][2,3]-g.at_width(0)['flange_tcp'][2,3]),.004)

    def test_opening_inverse_and_limits(self):
        angles=[]
        for width in [0,.01,.016,.032,.06]:
            value=self.gripper.at_width(width)
            self.assertAlmostEqual(value['opening_m'],width,places=7)
            validate(value['flange_tcp'])
            angles.append(value['linkage_angle_rad'])
        self.assertTrue(np.all(np.diff(angles)>0))
        for width in [-.001,.07,np.nan]:
            with self.assertRaises(ValueError):self.gripper.at_width(width)

    def test_invalid_rigid_transform_rejected(self):
        for bad in [np.full((4,4),np.nan),np.zeros((4,4)),np.diag([2,1,1,1])]:
            with self.assertRaises(ValueError):validate(bad)
        with self.assertRaises(ValueError):offset_target(np.eye(4),[0,0,2],.1)

    def test_observed_obstacle_points_excludes_grasp_target(self):
        points=np.array([[0.,0.,.4],[.01,0.,.4],[.02,0.,.4],[.03,0.,.4]])
        nearby=np.array([True,True,True,False])
        above=np.array([True,True,False,True])
        target=np.array([True,False,True,False])
        result=observed_obstacle_points(points,nearby,above,target)
        np.testing.assert_allclose(result,[[.01,0.,.4]])

    def test_observed_obstacle_points_rejects_bad_filter_shape(self):
        with self.assertRaises(ValueError):
            observed_obstacle_points(np.zeros((2,3)),[True],[True,True],[False,False])

if __name__=='__main__':unittest.main()
