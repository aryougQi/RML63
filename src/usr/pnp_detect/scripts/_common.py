from pathlib import Path
import sys
import hashlib
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from pnp_detect.geometry import ORDER, ORDER_RULE


def new_output(path):
    out=Path(path)
    if out.exists() and any(out.iterdir()):
        raise ValueError(f"输出目录非空，拒绝覆盖，请换新目录: {out}")
    out.mkdir(parents=True,exist_ok=True)
    return out


def base_result(image_path,config,source):
    import cv2
    return {"success":False,"source":source,"image_path":str(Path(image_path).resolve()),
            "image_resolution_wh":None,"image_metadata":None,"corners":[],
            "corner_order":ORDER,"ordering_rule":ORDER_RULE,"coordinate_system":"input image pixels, origin top-left, x right, y down",
            "physical_corner_mapping":None,"orientation_ambiguous":True,
            "pnp_ready":False,"pose":None,
            "input_geometry":config.get("input_geometry",{"status":"unknown"}),
            "method":None,"quality_metrics":{},"failure_reason":None,
            "configuration":config,"versions":{"opencv":cv2.__version__,"numpy":np.__version__},
            "limitations":"Geometry is a rim hypothesis, not semantic basket confirmation. Image ordering does not identify physical model corners."}


def attach_image(result, image_path, metadata):
    result["image_metadata"]=metadata
    result["image_resolution_wh"]=[metadata["width"],metadata["height"]]
    result["image_sha256"]=hashlib.sha256(Path(image_path).read_bytes()).hexdigest()
    geom=result["input_geometry"]
    if geom["status"]=="native":
        if geom["source_resolution_wh"] != result["image_resolution_wh"] or not np.allclose(geom["source_to_input_3x3"],np.eye(3)):
            raise ValueError("native 输入必须匹配源分辨率并使用单位变换")
