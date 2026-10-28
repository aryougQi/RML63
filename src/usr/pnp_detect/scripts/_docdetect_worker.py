#!/usr/bin/env python3
"""Isolated execution of pinned, unmodified MIT docdetect source."""
import argparse
import json
import hashlib
from pathlib import Path
import sys
import warnings
import resource


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--image',required=True)
    parser.add_argument('--max-side',type=int,default=640)
    parser.add_argument('--max-proposals',type=int,default=500)
    args=parser.parse_args()
    resource.setrlimit(resource.RLIMIT_AS,(2*1024**3,2*1024**3))
    # NumPy warns about a legacy upstream array-to-int conversion; it is still
    # supported in the pinned environment. No upstream algorithm code is patched.
    warnings.filterwarnings('ignore',category=DeprecationWarning)
    root=Path(__file__).resolve().parents[1]
    sys.path[:0]=[str(root/'third_party/docdetect'),str(root/'src')]
    manifest=json.loads((root/'third_party/docdetect/UPSTREAM.json').read_text())
    for name,expected in manifest['files'].items():
        actual=hashlib.sha256((root/'third_party/docdetect'/name).read_bytes()).hexdigest()
        if actual!=expected:raise ValueError(f'Vendored source changed: {name}')
    import cv2
    import numpy as np
    import docdetect
    from pnp_detect.geometry import order_corners
    cv2.setNumThreads(1);cv2.setRNGSeed(0)
    image=cv2.imread(args.image,cv2.IMREAD_COLOR)
    if image is None:raise ValueError('Temporary image decode failed')
    h,w=image.shape[:2];scale=min(1.,args.max_side/max(h,w))
    ww,hh=max(1,round(w*scale)),max(1,round(h*scale))
    work=cv2.resize(image,(ww,hh)) if (ww,hh)!=(w,h) else image
    raw=docdetect.process(work)  # All upstream thresholds remain their defaults.
    unique=[];seen=set();invalid=0;truncated=False
    sx,sy=ww/w,hh/h
    for candidate in raw:
        try:q=order_corners(candidate)
        except ValueError:invalid+=1;continue
        key=tuple(q.ravel())
        if key in seen:continue
        if len(unique)>=args.max_proposals:truncated=True;break
        seen.add(key)
        # OpenCV resize maps source pixel centres to work pixel centres.
        q=(q+.5)/[sx,sy]-.5
        unique.append(q.tolist())
    print(json.dumps({'proposals':unique,'stats':{'enabled':True,'commit':manifest['commit'],
          'source_unmodified':True,'upstream_parameter_overrides':{},
          'source_resolution_wh':[w,h],'working_resolution_wh':[ww,hh],
          'source_to_work_3x3':[[sx,0,(sx-1)/2],[0,sy,(sy-1)/2],[0,0,1]],
          'raw_quad_count':len(raw),'invalid_quads':invalid,'proposal_count':len(unique),'truncated':truncated}},allow_nan=False))


if __name__=='__main__':main()
