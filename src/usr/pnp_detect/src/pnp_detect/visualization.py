import json
from pathlib import Path
import cv2
import numpy as np


def write_png(path, image):
    ok,data=cv2.imencode(".png", image)
    if not ok: raise OSError(f"PNG 编码失败: {path}")
    data.tofile(str(path))


def overlay(image, result):
    out=image.copy()
    if result["success"]:
        pts=np.rint(result["corners"]).astype(np.int32)
        cv2.polylines(out,[pts],True,(0,255,0),2)
        for i,(x,y) in enumerate(pts):
            cv2.circle(out,(int(x),int(y)),5,(0,0,255),-1)
            text=f"{i}: ({x},{y})"
            (tw,th),baseline=cv2.getTextSize(text,cv2.FONT_HERSHEY_SIMPLEX,.5,2)
            tx=max(0,min(int(x)-20,out.shape[1]-tw-2))
            ty=max(th+2,min(int(y)-10,out.shape[0]-baseline-2))
            cv2.putText(out,text,(tx,ty),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
    label=("AUTO" if result["source"]=="automatic" else "MANUAL") + (" OK" if result["success"] else " FAILED")
    cv2.putText(out,label,(10,25),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0) if result["success"] else (0,0,255),2)
    if not result["success"]:
        reason=str(result.get("failure_reason") or "unknown").split(":",1)[0]
        cv2.putText(out,reason,(10,min(48,out.shape[0]-5)),cv2.FONT_HERSHEY_SIMPLEX,0.42,(0,0,255),1)
    return out


def save_outputs(output, image, result, debug=None, *, image_only=False):
    output=Path(output); output.mkdir(parents=True,exist_ok=True)
    if image_only:
        if image is not None: write_png(output/"detected_corners.png",overlay(image,result))
        return
    # 调用方要求新目录，避免旧运行的 PNG 被误认为当前输出。
    if image is not None:
        write_png(output/"original.png",image)
        write_png(output/"detected_corners.png",overlay(image,result))
    for name,img in (debug or {}).items(): write_png(output/f"{name}.png",img)
    (output/"corners.json").write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding="utf-8")
