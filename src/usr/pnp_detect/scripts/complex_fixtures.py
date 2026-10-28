"""Deterministic rounded dark rim + crossing handle + distractor lines (SYNTHETIC)."""
import cv2
import numpy as np


def complex_scene(destination=None, handle_width=19, missing_side=False):
    source=np.full((480,640,3),155,np.uint8)
    mask=np.zeros((480,640),np.uint8)
    # Rounded outer polygon whose virtual straight-side intersections are known.
    cv2.rectangle(mask,(173,90),(467,360),255,-1)
    cv2.rectangle(mask,(160,103),(480,347),255,-1)
    for center in [(173,103),(467,103),(467,347),(173,347)]:cv2.circle(mask,center,13,255,-1)
    inner=np.zeros_like(mask)
    cv2.rectangle(inner,(180,107),(460,343),255,-1)
    mask[inner>0]=0
    source[mask>0]=(20,20,20)
    cv2.line(source,(310,65),(310,390),(130,130,130),handle_width)
    cv2.line(source,(20,430),(610,440),(25,25,25),3)
    cv2.line(source,(30,180),(85,390),(30,30,30),4)
    if missing_side:cv2.rectangle(source,(450,130),(500,330),(155,155,155),-1)
    truth=np.float32([[160,90],[480,90],[480,360],[160,360]])
    if destination is not None:
        matrix=cv2.getPerspectiveTransform(truth,np.float32(destination))
        source=cv2.warpPerspective(source,matrix,(640,480),borderValue=(155,155,155))
        truth=np.float32(destination)
    return source,truth
