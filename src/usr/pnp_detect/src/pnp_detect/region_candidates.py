"""Color-region outer boundary proposals; no bounding rectangle corners."""
import cv2
import numpy as np
from .geometry import order_corners,line_intersection,validate_quad


def region_proposals(image,cfg,search_mask):
    opts=cfg.get('region_proposals',{})
    if not opts.get('enabled',False):return [],{}
    lower=np.asarray(opts['hsv_lower'],np.uint8);upper=np.asarray(opts['hsv_upper'],np.uint8)
    mask=cv2.inRange(cv2.cvtColor(image,cv2.COLOR_BGR2HSV),lower,upper)
    mask=cv2.bitwise_and(mask,search_mask)
    k=int(opts.get('close_kernel',3))
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((k,k),np.uint8))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    out=[];h,w=mask.shape
    for c in sorted(contours,key=lambda c:-cv2.contourArea(c)):
        if not cfg['min_area_ratio']<=cv2.contourArea(c)/(h*w)<=cfg['max_area_ratio']:continue
        hull=cv2.convexHull(c)
        for ratio in cfg['approx_epsilon_ratios']:
            p=cv2.approxPolyDP(hull,cv2.arcLength(hull,True)*ratio,True)
            if len(p)!=4:continue
            try:q=order_corners(p.reshape(4,2))
            except ValueError:continue
            points=c.reshape(-1,2).astype(float);fitted=[]
            for a,b in zip(q,np.roll(q,-1,axis=0)):
                v=b-a;length=np.linalg.norm(v);u=v/length
                rel=points-a;along=rel@u;dist=np.abs(u[0]*rel[:,1]-u[1]*rel[:,0])
                near=points[(along>length*.18)&(along<length*.82)&(dist<length*opts.get('fit_band_ratio',.08))]
                if len(near)<12:break
                vx,vy,x,y=cv2.fitLine(near.astype(np.float32),cv2.DIST_HUBER,0,.01,.01).ravel()
                fitted.append(((x,y),(vx,vy)))
            if len(fitted)!=4:continue
            try:refined=order_corners([line_intersection(fitted[(i-1)%4],fitted[i]) for i in range(4)])
            except ValueError:continue
            if np.max(np.linalg.norm(refined-q,axis=1))>min(np.linalg.norm(np.roll(q,-1,axis=0)-q,axis=1))*.15:continue
            if not validate_quad(refined,w,h,**cfg)[0]:continue
            out.append((refined,c,'color_region_outer_boundary_fit'))
    return out,{'region_mask':mask}
