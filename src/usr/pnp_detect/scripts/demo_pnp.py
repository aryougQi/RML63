#!/usr/bin/env python3
"""Known synthetic projection only. Never estimates a real basket pose."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from _common import ROOT
from pnp_detect.pnp_estimator import rectangle_model,estimate_pose


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',default=str(ROOT/'reports/pnp_synthetic_demo.json'))
    args=parser.parse_args();out=Path(args.output)
    if out.exists():parser.error('Refusing to overwrite existing result')
    obj=rectangle_model(.42,.30)
    k=np.float64([[740,0,640],[0,735,360],[0,0,1]])
    dist=np.float64([.04,-.015,.001,-.002,0.])
    rv=np.float64([.48,-.35,.2]);tv=np.float64([.02,-.015,1.1])
    image=cv2.projectPoints(obj,rv,tv,k,dist)[0].reshape(-1,2)
    geometry={'confirmed':True,'status':'native','source_resolution_wh':[1280,720],
              'input_resolution_wh':[1280,720],'source_to_input_3x3':np.eye(3).tolist()}
    mapping={'confirmed':True,'model_to_image_indices':[0,1,2,3],'basis':'Known SYNTHETIC projection correspondence'}
    result=estimate_pose(obj,image,k,dist,physical_corner_mapping=mapping,input_geometry=geometry,calibration_resolution_wh=[1280,720])
    result.update(data_kind='SYNTHETIC_ONLY',real_image_used=False,
                  known_rvec=rv.tolist(),known_tvec_m=tv.tolist(),
                  synthetic_camera_matrix=k.tolist(),synthetic_dist_coeffs=dist.tolist(),
                  synthetic_object_points_m=obj.tolist(),synthetic_image_points=image.tolist())
    if result['success']:
        result['translation_error_m']=float(np.linalg.norm(np.array(result['pose']['tvec_m'])-tv))
        result['rotation_matrix_error_frobenius']=float(np.linalg.norm(np.array(result['pose']['rotation_matrix'])-cv2.Rodrigues(rv)[0]))
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2,allow_nan=False))
    print(f"SYNTHETIC ONLY: {out}, success={result['success']}")
    return 0 if result['success'] else 2


if __name__=='__main__':raise SystemExit(main())
