#!/usr/bin/env python3
"""Headless manual annotation via a browser HTML image or explicit pixel coordinates."""
import argparse
import json
import sys
from _common import ROOT,new_output,base_result,attach_image
from pnp_detect.preprocessing import load_config,load_image
from pnp_detect.geometry import order_corners,validate_quad
from pnp_detect.visualization import save_outputs


def main():
    parser=argparse.ArgumentParser(description="人工对照工具：显式坐标或生成离线浏览器点击页面")
    parser.add_argument("--image",required=True); parser.add_argument("--output",required=True)
    parser.add_argument("--points",help='四点 JSON 字符串，例如 [[100,100],[400,100],[400,300],[100,300]]')
    parser.add_argument("--points-file",help="浏览器导出的 manual_points.json")
    parser.add_argument("--export-html",help="生成含嵌入图像的离线 HTML 点击工具（可下载到本地浏览器）")
    parser.add_argument("--compare",help="可选：自动检测的 corners.json")
    args=parser.parse_args()
    if sum(bool(x) for x in (args.points,args.points_file,args.export_html)) != 1:
        parser.error("必须且只能选择 --points / --points-file / --export-html 之一")
    image,meta=load_image(args.image)
    if args.export_html:
        from pathlib import Path
        import base64
        import cv2
        target=Path(args.export_html)
        if target.exists(): raise ValueError("拒绝覆盖现有 HTML")
        _,encoded=cv2.imencode(".png",image)
        html=(ROOT/"scripts/manual_template.html").read_text(encoding="utf-8")
        target.write_text(html.replace("__IMAGE__",base64.b64encode(encoded).decode()).replace("__WIDTH__",str(meta["width"])).replace("__HEIGHT__",str(meta["height"])),encoding="utf-8")
        print(f"离线人工点击页面: {target}，导出坐标后用 --points-file 生成正式人工结果。")
        return 0
    out=new_output(args.output)
    config=load_config(ROOT/"configs/detection.yaml"); result=base_result(args.image,config,"manual")
    attach_image(result,args.image,meta)
    try:
        points=json.loads(args.points) if args.points else json.loads(__import__('pathlib').Path(args.points_file).read_text())
        if isinstance(points,dict):
            if points.get("image_resolution_wh") != result["image_resolution_wh"]: raise ValueError("标注分辨率不匹配")
            points=points["corners"]
        q=order_corners(points)
        ok,metrics=validate_quad(q,meta["width"],meta["height"],min_area_ratio=0.00001,max_area_ratio=1.,min_side_px=2,border_margin_px=0,min_angle_deg=1,max_angle_deg=179)
        if not ok: raise ValueError(metrics["reason"])
        result.update(success=True,corners=q.tolist(),method="manual_annotation",quality_metrics=metrics)
        if args.compare:
            import numpy as np
            auto=json.loads(__import__('pathlib').Path(args.compare).read_text())
            if auto.get("image_sha256") != result["image_sha256"]: raise ValueError("对照必须使用同一原图")
            result["automatic_comparison"]={"automatic_success":auto["success"]}
            if auto["success"]:
                delta=np.linalg.norm(q-np.asarray(auto["corners"]),axis=1)
                result["automatic_comparison"].update(per_corner_error_px=delta.tolist(),mean_error_px=float(delta.mean()))
    except (ValueError,OSError,KeyError,TypeError) as exc:
        result.update(success=False,corners=[],failure_reason=str(exc))
    save_outputs(out,image,result)
    print(out/"corners.json")
    return 0 if result["success"] else 2


if __name__=="__main__":
    try: raise SystemExit(main())
    except (ValueError,OSError) as exc:
        print(str(exc),file=sys.stderr); raise SystemExit(2)
