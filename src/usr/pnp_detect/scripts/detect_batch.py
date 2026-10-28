#!/usr/bin/env python3
"""Apply one unchanged detector configuration to every image in a directory."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import sys
import time
from _common import ROOT,new_output,base_result,attach_image
from pnp_detect.preprocessing import load_config,load_image
from pnp_detect.corner_detector import detect
from pnp_detect.visualization import overlay,write_png


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',required=True)
    parser.add_argument('--output',required=True,help='New directory containing final PNGs only')
    parser.add_argument('--report',required=True,help='New report JSON, preferably under reports/')
    parser.add_argument('--config',default=str(ROOT/'configs/general.yaml'),help='One shared config, no per-image overrides')
    args=parser.parse_args()
    source=Path(args.input);target=Path(args.report)
    if not source.is_dir():parser.error(f'Input directory does not exist: {source}')
    if target.exists():parser.error('Report already exists; refusing to overwrite')
    # Keep the final-image directory free of JSON files even if the caller mistypes paths.
    if target.resolve().is_relative_to(Path(args.output).resolve()):parser.error('Report must be outside the final-image directory')
    config_path=Path(args.config)
    try:config=load_config(config_path)
    except (ValueError,OSError) as exc:parser.error(str(exc))
    extensions={'.png','.jpg','.jpeg','.webp','.bmp','.tif','.tiff'}
    files=sorted((p for p in source.rglob('*') if p.is_file() and not p.is_symlink() and p.suffix.lower() in extensions),key=lambda p:p.relative_to(source).as_posix())
    if not files:parser.error('No supported image files found')
    try:output=new_output(args.output)
    except (ValueError,OSError) as exc:parser.error(str(exc))
    started=time.perf_counter();rows=[]
    for index,path in enumerate(files,1):
        begin=time.perf_counter();record=base_result(path,config,'automatic');image=None
        try:
            image,metadata=load_image(path);attach_image(record,path,metadata)
            found,_=detect(image,config);record.update(found)
        except Exception as exc:
            record.update(success=False,corners=[],failure_reason=f'{type(exc).__name__}: {exc}')
        slug=re.sub(r'[^\w.-]+','_',path.stem).strip('._') or 'image'
        destination=output/f'{index:03d}_{slug}_detected.png'
        if image is not None:write_png(destination,overlay(image,record))
        diagnostics=record.get('diagnostics',{})
        row={key:record.get(key) for key in ('success','image_resolution_wh','image_metadata','image_sha256',
             'corners','corner_order','ordering_rule','method','quality_metrics','failure_reason','source','pnp_ready','pose')}
        row.update(input=str(path.resolve()),relative_input=path.relative_to(source).as_posix(),
                   output_image=str(destination.resolve()) if image is not None else None,
                   elapsed_seconds=time.perf_counter()-begin,
                   opencv_quads=diagnostics.get('opencv_quads'),
                   docdetect=diagnostics.get('docdetect'),
                   rejected_reasons=dict(Counter(c['reason'] for c in diagnostics.get('rejected_candidates',[]))),
                   semantic_basket_confirmation=False)
        rows.append(row)
        print(f"[{index}/{len(files)}] {'PASS' if record['success'] else 'REJECT'} {path.name}",flush=True)
    passed=sum(row['success'] for row in rows)
    report={'completed':True,'total':len(rows),'passed_image_checks':passed,'rejected':len(rows)-passed,
            'configuration_path':str(config_path.resolve()),'configuration_sha256':hashlib.sha256(config_path.read_bytes()).hexdigest(),
            'same_configuration_for_all_images':True,'per_image_overrides':False,'configuration':config,
            'elapsed_seconds':time.perf_counter()-started,'results':rows,
            'limitations':'PASS means image checks only, not semantic/physical correctness or an accuracy measurement. No PnP on uncalibrated web images.'}
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    print(f'Processed {len(rows)}; passed={passed}; rejected={len(rows)-passed}; report={target}')
    return 0 if passed==len(rows) else 2


if __name__=='__main__':raise SystemExit(main())
