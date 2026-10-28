import copy
import hashlib
import json
from pathlib import Path
import sys
import unittest
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from complex_fixtures import complex_scene
from pnp_detect.preprocessing import load_config
from pnp_detect.corner_detector import detect
from pnp_detect.line_candidates import propose_quads


class ComplexDetectionTests(unittest.TestCase):
    def setUp(self):
        self.cfg=load_config(ROOT/'configs/complex_dark_rim.yaml')

    def test_rounded_rim_crossing_handle_and_clutter(self):
        im,truth=complex_scene();r,_=detect(im,self.cfg)
        self.assertTrue(r['success'],r['failure_reason'])
        self.assertLess(np.max(np.linalg.norm(np.asarray(r['corners'])-truth,axis=1)),3)
        self.assertEqual(r['quality_metrics']['proposal_source'],'hough_line_combination')
        self.assertIsNone(r['quality_metrics']['contour_coverage'])
        # Reject fake lower sides formed by distant background lines.
        for c in r['diagnostics']['accepted_candidates']:
            self.assertLess(np.max(np.array(c['corners'])[:,1]),400)

    def test_position_and_perspective_without_roi(self):
        for destination in ([[105,125],[425,125],[425,395],[105,395]],
                            [[190,80],[480,120],[450,355],[145,330]]):
            with self.subTest(destination=destination):
                im,truth=complex_scene(destination);r,_=detect(im,self.cfg)
                self.assertTrue(r['success'],r['failure_reason'])
                self.assertLess(np.max(np.linalg.norm(np.asarray(r['corners'])-truth,axis=1)),4)

    def test_severe_occlusion_and_missing_side_rejected(self):
        for kw in ({'missing_side':True},{'handle_width':110}):
            im,_=complex_scene(**kw);r,_=detect(im,self.cfg)
            self.assertFalse(r['success']);self.assertEqual(r['corners'],[])

    def test_blank_and_lines_do_not_invent_quad(self):
        for im in [np.full((480,640,3),155,np.uint8), np.zeros((480,640,3),np.uint8)]:
            r,_=detect(im,self.cfg);self.assertFalse(r['success'])
        quads,stats=propose_quads([[10,10,600,10],[10,100,600,100],[10,200,600,200]],640,480,self.cfg['detection'])
        self.assertEqual(quads,[])

    def test_two_plausible_rims_are_ambiguous(self):
        im=np.full((480,640,3),155,np.uint8)
        for x in [45,355]:
            cv2.rectangle(im,(x,100),(x+235,350),(20,20,20),14)
            cv2.line(im,(x+115,80),(x+115,370),(130,130,130),17)
        r,_=detect(im,self.cfg)
        self.assertFalse(r['success']);self.assertEqual(r['corners'],[])
        self.assertTrue(r['failure_reason'].startswith('ambiguous_rims'))

    def test_line_search_budget_fails_closed(self):
        cfg=copy.deepcopy(self.cfg);cfg['detection']['max_candidates']=2
        im,_=complex_scene();r,_=detect(im,cfg)
        self.assertFalse(r['success']);self.assertEqual(r['corners'],[])
        self.assertIn('limit',r['failure_reason'])

    def test_deterministic_complex(self):
        im,_=complex_scene()
        a,_=detect(im,self.cfg);b,_=detect(im,self.cfg)
        self.assertEqual(a,b)

    def test_rejected_debug_images(self):
        im,_=complex_scene(missing_side=True);r,debug=detect(im,self.cfg)
        self.assertFalse(r['success'])
        self.assertTrue(any(name.startswith('rejected_') for name in debug))

    @unittest.skipUnless((ROOT/'data/images/basket.png').is_file(),'local real screenshot not supplied')
    def test_local_real_photo_smoke_only(self):
        # A smoke/determinism check, not an accuracy benchmark or physical ground truth.
        cfg=load_config(ROOT/'configs/basket_real.yaml')
        im=cv2.imread(str(ROOT/'data/images/basket.png'))
        a,_=detect(im,cfg);b,_=detect(im,cfg)
        self.assertEqual(a,b)
        self.assertTrue(a['success'],a['failure_reason'])
        self.assertEqual(len(a['corners']),4)


if __name__=='__main__':unittest.main()
