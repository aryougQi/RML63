"""Multi-channel/threshold proposals inspired by the OpenCV squares.cpp sample.

Reference: https://github.com/opencv/opencv/blob/4.13.0/samples/cpp/squares.cpp
Independent Python implementation of its pyramid, per-channel Canny/threshold,
contour and approxPolyDP pipeline. Unlike the sample, this project uses the existing
perspective-quad geometry checks rather than assuming projected right angles.
All candidates must pass the independent original-image edge/line checks.
"""
import cv2
import numpy as np
from .geometry import order_corners,validate_quad


def opencv_quad_proposals(image,config,search_mask):
    opts=config.get('opencv_quads',{})
    if not opts.get('enabled',False):return [],{'enabled':False}
    levels=opts.get('threshold_levels',11)
    if type(levels) is not int or not 2<=levels<=32:raise ValueError('threshold_levels must be an integer in [2,32]')
    maximum=opts.get('max_proposals',500)
    contour_budget=opts.get('max_examined_contours',20000)
    if type(maximum) is not int or maximum<1 or type(contour_budget) is not int or contour_budget<1:
        raise ValueError('OpenCV proposal/contour budgets must be positive integers')
    h,w=image.shape[:2]
    smooth=cv2.pyrUp(cv2.pyrDown(image),dstsize=(w,h))
    proposals=[];seen=set();examined=0;truncated=False
    for channel in cv2.split(smooth):
        for level in range(levels):
            if level==0:
                mask=cv2.Canny(channel,0,opts.get('canny_high',50),apertureSize=5)
                mask=cv2.dilate(mask,None)
            else:mask=np.uint8(channel>=(level+1)*255/levels)*255
            mask=cv2.bitwise_and(mask,search_mask)
            contours,_=cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
            for contour in sorted(contours,key=lambda c:(-abs(cv2.contourArea(c)),cv2.boundingRect(c))):
                examined+=1
                if examined>contour_budget:truncated=True;break
                area=abs(cv2.contourArea(contour))/(w*h)
                if not config['min_area_ratio']<=area<=config['max_area_ratio']:continue
                approx=cv2.approxPolyDP(contour,.02*cv2.arcLength(contour,True),True)
                if len(approx)!=4 or not cv2.isContourConvex(approx):continue
                try:q=order_corners(approx.reshape(4,2))
                except ValueError:continue
                if not validate_quad(q,w,h,**config)[0]:continue
                key=tuple(np.rint(q/2).astype(int).ravel())
                if key in seen:continue
                if len(proposals)>=maximum:truncated=True;break
                seen.add(key);proposals.append((q,contour,'opencv_multichannel_threshold_contour'))
            if truncated:break
        if truncated:break
    return proposals,{'enabled':True,'reference':'opencv/4.13.0/samples/cpp/squares.cpp',
                      'threshold_levels':levels,'channels':3,'examined_contours':examined,
                      'proposal_count':len(proposals),'truncated':truncated}
