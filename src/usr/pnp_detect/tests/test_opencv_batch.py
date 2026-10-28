from pathlib import Path
import copy
import json
import subprocess
import sys
import tempfile
import unittest
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from pnp_detect.preprocessing import load_config
from pnp_detect.opencv_quads import opencv_quad_proposals


class OpenCVBatchTests(unittest.TestCase):
    def setUp(self):self.config=load_config(ROOT/'configs/general.yaml')

    def test_multichannel_proposals_are_deterministic_and_do_not_modify_image(self):
        image=np.full((481,641,3),230,np.uint8)
        cv2.rectangle(image,(90,100),(540,390),(30,30,30),-1)
        original=image.copy();mask=np.full(image.shape[:2],255,np.uint8)
        a,stats=opencv_quad_proposals(image,self.config['detection'],mask)
        b,other=opencv_quad_proposals(image,self.config['detection'],mask)
        self.assertGreater(len(a),0)
        self.assertEqual(stats,other)
        self.assertEqual([q.tolist() for q,_,_ in a],[q.tolist() for q,_,_ in b])
        np.testing.assert_array_equal(image,original)
        self.assertTrue(all(source=='opencv_multichannel_threshold_contour' for _,_,source in a))

    def test_proposal_budget_truncation_is_explicit(self):
        image=np.full((480,640,3),230,np.uint8)
        cv2.rectangle(image,(60,60),(240,380),(20,20,20),-1)
        cv2.rectangle(image,(360,60),(580,380),(20,20,20),-1)
        cfg=copy.deepcopy(self.config['detection']);cfg['opencv_quads']['max_proposals']=1
        result,stats=opencv_quad_proposals(image,cfg,np.full(image.shape[:2],255,np.uint8))
        self.assertTrue(stats['truncated']);self.assertLessEqual(len(result),1)

    def test_batch_uses_one_config_continues_and_saves_only_final_pngs(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp=Path(tmp);inputs=tmp/'inputs';inputs.mkdir()
            cv2.imwrite(str(inputs/'first.png'),np.full((120,180,3),200,np.uint8))
            cv2.imwrite(str(inputs/'second.png'),np.full((120,180,3),180,np.uint8))
            (inputs/'broken.png').write_bytes(b'not a PNG')
            original={p.name:p.read_bytes() for p in inputs.iterdir()}
            output=tmp/'final';report=tmp/'report.json'
            cmd=[sys.executable,str(ROOT/'scripts/detect_batch.py'),'--input',str(inputs),'--output',str(output),'--report',str(report)]
            run=subprocess.run(cmd,capture_output=True,text=True)
            self.assertEqual(run.returncode,2,run.stderr)
            r=json.loads(report.read_text())
            self.assertTrue(r['completed']);self.assertTrue(r['same_configuration_for_all_images'])
            self.assertFalse(r['per_image_overrides']);self.assertEqual(r['total'],3)
            self.assertEqual(r['rejected'],3)
            self.assertTrue(all(row['corners']==[] and row['pose'] is None for row in r['results']))
            self.assertEqual(len(list(output.iterdir())),2)
            self.assertTrue(all(p.suffix=='.png' for p in output.iterdir()))
            for p in inputs.iterdir():self.assertEqual(p.read_bytes(),original[p.name])
            before=report.read_bytes()
            again=subprocess.run(cmd,capture_output=True,text=True)
            self.assertEqual(again.returncode,2);self.assertEqual(report.read_bytes(),before)


if __name__=='__main__':unittest.main()
