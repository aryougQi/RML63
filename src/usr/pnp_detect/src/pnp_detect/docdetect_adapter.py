"""Candidate adapter for pinned unmodified alessandrozamberletti/docdetect.

This is a document-geometry candidate source, not a basket semantic detector.
No frame-specific thresholds, HSV or ROI; independent existing gates decide acceptance.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import cv2
import numpy as np


def docdetect_proposals(image,config):
    opts=config.get('docdetect',{})
    if not opts.get('enabled',False):return [],{'enabled':False}
    size=opts.get('max_side',640);maximum=opts.get('max_proposals',500);timeout=opts.get('timeout_seconds',10)
    if type(size) is not int or not 64<=size<=2048 or type(maximum) is not int or not 1<=maximum<=5000:
        raise ValueError('Invalid docdetect working size or proposal budget')
    if not np.isfinite(timeout) or not 0<timeout<=60:raise ValueError('Invalid docdetect timeout')
    root=Path(__file__).resolve().parents[2]
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    with tempfile.TemporaryDirectory(prefix='pnp_docdetect_') as tmp:
        source=Path(tmp)/'input.png'
        ok,encoded=cv2.imencode('.png',image)
        if not ok:raise ValueError('Cannot encode temporary input')
        encoded.tofile(str(source))
        command=[sys.executable,str(root/'scripts/_docdetect_worker.py'),'--image',str(source),
                 '--max-side',str(size),'--max-proposals',str(maximum)]
        try:run=subprocess.run(command,env=env,capture_output=True,text=True,timeout=timeout)
        except subprocess.TimeoutExpired:
            return [],{'enabled':True,'failure_reason':'docdetect_timeout','timeout_seconds':timeout}
    if run.returncode:
        return [],{'enabled':True,'failure_reason':'docdetect_worker_error','detail':run.stderr[-2000:]}
    try:result=json.loads(run.stdout)
    except json.JSONDecodeError:
        return [],{'enabled':True,'failure_reason':'docdetect_invalid_worker_response'}
    quads=[np.array(q,dtype=float) for q in result['proposals']]
    return quads,result['stats']
