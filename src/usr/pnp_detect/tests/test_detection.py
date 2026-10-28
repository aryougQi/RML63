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
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from generate_synthetic import fixtures
from pnp_detect.corner_detector import detect
from pnp_detect.preprocessing import load_config,load_image
from pnp_detect.visualization import write_png


class DetectionTests(unittest.TestCase):
    def setUp(self):
        self.config=load_config(ROOT/'configs/detection.yaml')
        self.images,self.truth=fixtures()

    def test_perspective_and_determinism(self):
        a,_=detect(self.images['perspective'],self.config)
        b,_=detect(self.images['perspective'],self.config)
        self.assertTrue(a['success'],a['failure_reason'])
        self.assertEqual(a,b)
        self.assertLess(float(np.max(np.linalg.norm(np.array(a['corners'])-self.truth,axis=1))),5)

    def test_no_arbitrary_points_on_failure(self):
        for name in ('blank','occluded','ambiguous'):
            with self.subTest(name=name):
                result,_=detect(self.images[name],self.config)
                self.assertFalse(result['success'],result)
                self.assertEqual(result['corners'],[])
                self.assertIsNotNone(result['failure_reason'])

    def test_shifted_and_rotated_perspectives(self):
        for points in ([[80,150],[480,80],[550,300],[120,400]],[[250,70],[470,220],[340,400],[100,250]]):
            image=np.full((480,640,3),35,np.uint8)
            cv2.fillConvexPoly(image,np.array(points,np.int32),(170,170,170))
            result,_=detect(image,self.config)
            self.assertTrue(result['success'],result)

    def test_image_validation(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'bad.png';p.write_bytes(b'not an image')
            with self.assertRaises(ValueError):load_image(p)
            p.write_bytes(b'')
            with self.assertRaises(ValueError):load_image(p)
            write_png(p,np.zeros((100,100),np.uint16))
            with self.assertRaises(ValueError):load_image(p)
            write_png(p,np.zeros((100,100,4),np.uint8))
            with self.assertRaises(ValueError):load_image(p)

    def test_cli_artifacts_and_missing_input(self):
        with tempfile.TemporaryDirectory() as tmp:
            image=Path(tmp)/'synthetic.png';write_png(image,self.images['perspective'])
            out=Path(tmp)/'result'
            proc=subprocess.run([sys.executable,str(ROOT/'scripts/detect_image.py'),'--debug','--image',str(image),'--output',str(out)],capture_output=True,text=True)
            self.assertEqual(proc.returncode,0,proc.stderr)
            for name in ('original.png','edges.png','contours.png','detected_corners.png','corners.json'):
                self.assertTrue((out/name).exists())
            data=json.loads((out/'corners.json').read_text())
            self.assertFalse(data['pnp_ready']);self.assertIsNone(data['pose'])
            self.assertEqual(data['source'],'automatic')
            out2=Path(tmp)/'missing'
            proc=subprocess.run([sys.executable,str(ROOT/'scripts/detect_image.py'),'--debug','--image',str(Path(tmp)/'missing.png'),'--output',str(out2)],capture_output=True,text=True)
            self.assertEqual(proc.returncode,2)
            self.assertFalse(json.loads((out2/'corners.json').read_text())['success'])

    def test_candidate_overflow_fails_closed(self):
        cfg=copy.deepcopy(self.config);cfg['detection']['max_candidates']=1
        result,_=detect(self.images['perspective'],cfg)
        self.assertFalse(result['success']);self.assertIn('limit',result['failure_reason'])

    def test_manual_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            image=Path(tmp)/'synthetic.png';write_png(image,self.images['perspective'])
            out=Path(tmp)/'manual'
            proc=subprocess.run([sys.executable,str(ROOT/'scripts/manual_annotate.py'),'--image',str(image),'--output',str(out),'--points',json.dumps(self.truth.tolist())],capture_output=True,text=True)
            self.assertEqual(proc.returncode,0,proc.stderr)
            data=json.loads((out/'corners.json').read_text())
            self.assertEqual(data['source'],'manual');self.assertTrue(data['success'])


if __name__=='__main__':unittest.main()
