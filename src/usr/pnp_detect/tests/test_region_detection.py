"""New region candidates tested with known synthetic geometry and web-image equivariance."""
from pathlib import Path
import sys
import tempfile
import json
import subprocess
import unittest
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from pnp_detect.corner_detector import detect
from pnp_detect.preprocessing import load_config


def rounded_grid():
    im=np.full((1000,1000,3),255,np.uint8)
    cv2.rectangle(im,(110,200),(890,800),(20,20,20),-1)
    cv2.rectangle(im,(60,250),(940,750),(20,20,20),-1)
    for x,y in [(110,250),(890,250),(890,750),(110,750)]:cv2.circle(im,(x,y),50,(20,20,20),-1)
    for x in range(150,840,60):
        for y in range(300,720,80):cv2.rectangle(im,(x,y),(x+30,y+45),(255,255,255),-1)
    return im,np.float32([[60,200],[940,200],[940,800],[60,800]])


class RegionDetectionTests(unittest.TestCase):
    def setUp(self):self.cfg=load_config(ROOT/'configs/dark_region.yaml')

    def test_grid_is_not_mistaken_for_rim(self):
        image,truth=rounded_grid();result,_=detect(image,self.cfg)
        self.assertTrue(result['success'],result['failure_reason'])
        self.assertEqual(result['quality_metrics']['proposal_source'],'color_region_outer_boundary_fit')
        self.assertLess(np.max(np.linalg.norm(np.array(result['corners'])-truth,axis=1)),3)

    def test_known_synthetic_perspective(self):
        image,truth=rounded_grid();target=np.float32([[150,230],[840,170],[900,760],[120,820]])
        matrix=cv2.getPerspectiveTransform(truth,target)
        image=cv2.warpPerspective(image,matrix,(1000,1000),borderValue=(255,255,255))
        result,_=detect(image,self.cfg)
        self.assertTrue(result['success'],result['failure_reason'])
        self.assertLess(np.max(np.linalg.norm(np.array(result['corners'])-target,axis=1)),4)

    def test_resolutions_preserve_original_pixel_coordinates(self):
        image,truth=rounded_grid()
        for side in [500,750,1200]:
            resized=cv2.resize(image,(side,side))
            result,_=detect(resized,self.cfg)
            self.assertTrue(result['success'],result['failure_reason'])
            expected=truth*(side/1000.)
            self.assertLess(np.max(np.linalg.norm(np.array(result['corners'])-expected,axis=1)),4)
            self.assertAlmostEqual(result['parameter_scale'],side/1000.)

    def test_default_cli_only_saves_final_png(self):
        image,_=rounded_grid()
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'image.png';cv2.imwrite(str(source),image)
            output=Path(tmp)/'final';metadata=Path(tmp)/'metadata.json'
            proc=subprocess.run([sys.executable,str(ROOT/'scripts/detect_image.py'),'--image',str(source),
                                 '--config',str(ROOT/'configs/dark_region.yaml'),'--output',str(output),'--json',str(metadata)],capture_output=True,text=True)
            self.assertEqual(proc.returncode,0,proc.stderr)
            self.assertEqual([p.name for p in output.iterdir()],['detected_corners.png'])
            self.assertTrue(json.loads(metadata.read_text())['success'])

    @unittest.skipUnless((ROOT/'data/images/web/regency_top_view.jpg').is_file(),'web sample not downloaded')
    def test_downloaded_web_photo_and_equivariance(self):
        # A check on one tuning sample, NOT an independent accuracy benchmark.
        image=cv2.imread(str(ROOT/'data/images/web/regency_top_view.jpg'))
        baseline,_=detect(image,self.cfg)
        self.assertTrue(baseline['success'],baseline['failure_reason'])
        again,_=detect(image,self.cfg);self.assertEqual(baseline,again)
        for target in [np.float32([[150,230],[840,170],[900,760],[120,820]]),
                       np.float32([[150,250],[850,250],[850,730],[150,730]])]:
            h=cv2.getPerspectiveTransform(np.float32(baseline['corners']),target)
            warped=cv2.warpPerspective(image,h,(1000,1000),borderValue=(255,255,255))
            result,_=detect(warped,self.cfg)
            self.assertTrue(result['success'],result['failure_reason'])
            self.assertLess(np.max(np.linalg.norm(np.array(result['corners'])-target,axis=1)),4)


if __name__=='__main__':unittest.main()
