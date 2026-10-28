#!/usr/bin/env python3
import argparse
import sys
import json
from pathlib import Path
from _common import ROOT,new_output,base_result,attach_image
from pnp_detect.preprocessing import load_config,load_image
from pnp_detect.corner_detector import detect
from pnp_detect.visualization import save_outputs


def main():
    parser=argparse.ArgumentParser(description="Stage 1: conservative basket rim detection; no pose estimation")
    parser.add_argument("--image",required=True)
    parser.add_argument("--output",required=True)
    parser.add_argument("--config",default=str(ROOT/"configs/general.yaml"))
    parser.add_argument("--debug",action="store_true",help="Save full intermediate PNGs and corners.json (default: final PNG only)")
    parser.add_argument("--json",dest="json_path",help="Optional separate metadata JSON, e.g. reports/detection.json")
    args=parser.parse_args()
    if args.json_path and Path(args.json_path).exists():
        print("Refusing to overwrite metadata JSON",file=sys.stderr);return 2
    try: out=new_output(args.output)
    except (ValueError,OSError) as exc:
        print(str(exc),file=sys.stderr); return 2
    image=None; debug={}; result=base_result(args.image,{},"automatic")
    try:
        config=load_config(args.config); result=base_result(args.image,config,"automatic")
        image,metadata=load_image(args.image); attach_image(result,args.image,metadata)
        detection,debug=detect(image,config); result.update(detection)
    except (ValueError,OSError,KeyError,TypeError,OverflowError) as exc:
        result.update(success=False,corners=[],failure_reason=f"invalid_input_or_config: {exc}")
    except Exception as exc:
        # 保存错误并返回非零；保留完整 traceback 用于程序错误诊断。
        import traceback
        traceback.print_exc()
        result.update(success=False,corners=[],failure_reason=f"processing_error: {type(exc).__name__}: {exc}")
    save_outputs(out,image,result,debug,image_only=not args.debug)
    if args.json_path:
        target=Path(args.json_path);target.parent.mkdir(parents=True,exist_ok=True)
        target.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding="utf-8")
    print(f"{'成功' if result['success'] else '失败'}: {out}")
    if result["failure_reason"]: print(result["failure_reason"],file=sys.stderr)
    return 0 if result["success"] else 2


if __name__=="__main__": raise SystemExit(main())
