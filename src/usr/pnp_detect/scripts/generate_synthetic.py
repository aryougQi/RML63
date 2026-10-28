#!/usr/bin/env python3
"""Explicitly SYNTHETIC test fixtures, never real basket evidence."""
import argparse
import numpy as np
import cv2
from _common import new_output
from pnp_detect.visualization import write_png


def fixtures():
    points=np.array([[145,110],[490,145],[535,365],[105,335]],np.int32)
    image=np.full((480,640,3),35,np.uint8)
    cv2.fillConvexPoly(image,points,(170,170,170))
    blank=np.full_like(image,35)
    occluded=image.copy()
    cv2.rectangle(occluded,(260,80),(390,190),(35,35,35),-1)
    ambiguous=blank.copy()
    cv2.rectangle(ambiguous,(45,100),(285,350),(170,170,170),-1)
    cv2.rectangle(ambiguous,(355,100),(595,350),(170,170,170),-1)
    return {"perspective":image,"blank":blank,"occluded":occluded,"ambiguous":ambiguous},points


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    args=parser.parse_args();out=new_output(args.output)
    images,truth=fixtures()
    for name,image in images.items():write_png(out/f'{name}.png',image)
    import json
    (out/'SYNTHETIC_ONLY.json').write_text(json.dumps({'synthetic':True,'perspective_truth':truth.tolist()},indent=2))
    print(out)
