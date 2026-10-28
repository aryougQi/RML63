from pathlib import Path
import hashlib
import json
import subprocess
import sys
import unittest
from unittest.mock import patch
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from pnp_detect.docdetect_adapter import docdetect_proposals
from pnp_detect.preprocessing import load_config


class DocdetectAdapterTests(unittest.TestCase):
    def test_pinned_source_matches_manifest_and_license_retained(self):
        root=ROOT/'third_party/docdetect';manifest=json.loads((root/'UPSTREAM.json').read_text())
        self.assertEqual(manifest['commit'],'dc29391bcc1494d662e3fe7cabd630b708ab2a70')
        for name,digest in manifest['files'].items():
            self.assertEqual(hashlib.sha256((root/name).read_bytes()).hexdigest(),digest,name)
        self.assertIn('MIT License',(root/'LICENSE').read_text())

    def test_disabled_backend_does_not_spawn_worker(self):
        with patch('pnp_detect.docdetect_adapter.subprocess.run') as run:
            quads,stats=docdetect_proposals(np.zeros((100,100,3),np.uint8),{})
            self.assertEqual(quads,[]);self.assertFalse(stats['enabled']);run.assert_not_called()

    def test_timeout_is_explicit_and_has_no_corners(self):
        cfg=load_config(ROOT/'configs/docdetect.yaml')['detection']
        with patch('pnp_detect.docdetect_adapter.subprocess.run',side_effect=subprocess.TimeoutExpired('worker',10)):
            quads,stats=docdetect_proposals(np.zeros((100,100,3),np.uint8),cfg)
        self.assertEqual(quads,[]);self.assertEqual(stats['failure_reason'],'docdetect_timeout')

    def test_upstream_synthetic_projection_and_resize_mapping(self):
        image=np.full((800,1000,3),255,np.uint8)
        # Upstream groups lines by rho alone (30 px); use a positive fixture
        # with separated rho values. The known limitation is documented, not patched.
        truth=np.float32([[120,200],[880,200],[880,620],[120,620]])
        cv2.fillConvexPoly(image,truth.astype(np.int32),(25,25,25))
        before=image.copy();cfg=load_config(ROOT/'configs/docdetect.yaml')['detection']
        a,stats=docdetect_proposals(image,cfg)
        b,again=docdetect_proposals(image,cfg)
        self.assertNotIn('failure_reason',stats)
        self.assertGreater(len(a),0)
        self.assertEqual(stats,again)
        self.assertEqual([q.tolist() for q in a],[q.tolist() for q in b])
        np.testing.assert_array_equal(image,before)
        self.assertTrue(stats['source_unmodified']);self.assertEqual(stats['upstream_parameter_overrides'],{})
        self.assertEqual(stats['working_resolution_wh'],[640,512])
        best=min(float(np.max(np.linalg.norm(q-truth,axis=1))) for q in a)
        self.assertLess(best,6)


if __name__=='__main__':unittest.main()
