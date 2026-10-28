#!/usr/bin/env python3
"""Run validated PnP; refuses unmeasured dimensions or uncalibrated screenshots."""
import argparse
import json
from pathlib import Path
import sys
import yaml
import cv2
import numpy as np
from _common import ROOT,base_result,attach_image
from pnp_detect.preprocessing import load_image,load_config
from pnp_detect.corner_detector import detect
from pnp_detect.pnp_estimator import estimate_pose,rectangle_model
from pnp_detect.visualization import overlay,write_png


def main():
    parser=argparse.ArgumentParser(description='Calibrated planar basket PnP; no robot control')
    parser.add_argument('--image',required=True)
    parser.add_argument('--detection-config',default=str(ROOT/'configs/basket_real.yaml'))
    parser.add_argument('--pnp-config',default=str(ROOT/'configs/pnp.yaml'))
    parser.add_argument('--corners-file',help='Optional successful detection/manual JSON from this exact image')
    parser.add_argument('--output',default=str(ROOT/'reports/pnp_result.json'))
    parser.add_argument('--visualization',help='Optional pose PNG, created only for an accepted pose')
    args=parser.parse_args()
    target=Path(args.output)
    if target.exists() or (args.visualization and Path(args.visualization).exists()):
        print('Refusing to overwrite an existing result',file=sys.stderr);return 2
    result={'success':False,'pose':None,'robot_ready':False,'failure_reason':None,'image_path':str(Path(args.image).resolve())}
    try:
        image,meta=load_image(args.image)
        with open(args.pnp_config,encoding='utf-8') as f:settings=yaml.safe_load(f)
        if not isinstance(settings,dict) or any(not isinstance(settings.get(key),dict) for key in ('camera','basket','correspondence','input_geometry')):
            raise ValueError('PnP YAML requires camera/basket/correspondence/input_geometry mappings')
        result['data_kind']=settings.get('data_kind','real')
        for section,source_key in [('camera','calibration_source'),('basket','measurement_source')]:
            if settings[section].get('confirmed') is not True or not str(settings[section].get(source_key) or '').strip():
                raise ValueError(f'{section}: confirmed real data and {source_key} are required')
        cam=settings['camera'];basket=settings['basket']
        if cam.get('model')!='opencv_pinhole':raise ValueError('Only OpenCV pinhole distortion model supported; do not substitute fisheye/inverse Brown coefficients')
        if basket.get('corner_definition')!='virtual_intersections_of_straight_outer_rim_edges':raise ValueError('Basket model must use the same outer straight-edge intersection corners')
        obj=rectangle_model(basket['length_m'],basket['width_m'])
        geom=settings['input_geometry']
        if geom.get('input_resolution_wh')!=[meta['width'],meta['height']]:raise ValueError('PnP input resolution does not match actual image')
        config=load_config(args.detection_config)
        detection=base_result(args.image,config,'automatic');attach_image(detection,args.image,meta)
        if args.corners_file:
            supplied=json.loads(Path(args.corners_file).read_text())
            if supplied.get('image_sha256')!=detection['image_sha256'] or supplied.get('image_resolution_wh')!=detection['image_resolution_wh']:
                raise ValueError('Corner JSON must come from the exact same image (SHA256 and resolution)')
            detection.update(supplied)
        else:
            found,_=detect(image,config);detection.update(found)
        if not detection['success']:raise ValueError(f"Corner detection rejected: {detection['failure_reason']}")
        result.update(estimate_pose(obj,detection['corners'],cam['camera_matrix'],cam['dist_coeffs'],
            physical_corner_mapping=settings['correspondence'],input_geometry=geom,
            calibration_resolution_wh=cam['resolution_wh'],**settings.get('solver',{})))
        result.update(image_sha256=detection['image_sha256'],image_resolution_wh=detection['image_resolution_wh'],
                      corners_source=detection['source'],settings=settings)
        if result['success'] and args.visualization:
            pose=result['pose'];view=overlay(image,detection)
            # drawFrameAxes expects source pixels. Project axes there, then apply recorded H.
            length=min(basket['length_m'],basket['width_m'])*.4
            axes=np.float64([[0,0,0],[length,0,0],[0,length,0],[0,0,length]])
            p=cv2.projectPoints(axes,np.array(pose['rvec']),np.array(pose['tvec_m']),np.array(cam['camera_matrix'],float),np.array(cam['dist_coeffs'],float))[0].reshape(-1,2)
            from pnp_detect.pnp_estimator import _transform_points
            p=_transform_points(p,np.array(geom['source_to_input_3x3'],float));p=np.rint(p).astype(int)
            for point,color in zip(p[1:],[(0,0,255),(0,255,0),(255,0,0)]):cv2.line(view,tuple(p[0]),tuple(point),color,2)
            dest=Path(args.visualization);dest.parent.mkdir(parents=True,exist_ok=True);write_png(dest,view)
    except (ValueError,KeyError,TypeError,OSError,yaml.YAMLError,cv2.error) as exc:
        result.update(success=False,pose=None,failure_reason=str(exc))
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
    print(f"{'成功' if result['success'] else '拒绝计算'}: {target}")
    if result['failure_reason']:print(result['failure_reason'],file=sys.stderr)
    return 0 if result['success'] else 2


if __name__=='__main__':raise SystemExit(main())
