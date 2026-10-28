#!/usr/bin/env python3
"""Sequential replay acceptance matrix. No execution option is exposed."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--baseline',type=Path,required=True,help='archived directory containing scripts/ and config/')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    cases=[('optimized_A','20261027_163257',[],0),
        ('baseline_A','20261027_163257',['--baseline',str(args.baseline)],0),
        ('coarse_A','20261027_163257',['--coarse'],0),
        ('unreachable_B','20261027_163257',['--scenario','unreachable'],2),
        ('collision_C','20261027_162608',['--offset-y','.05'],2),
        ('timeout_empty_D','20261027_163257',['--budget','.00001'],2),
        ('timeout_saved_D','20261027_163257',['--mode','best_within_budget','--budget','7'],0),
        ('optimized_E1','20261027_164248',[],0),
        ('baseline_E1','20261027_164248',['--baseline',str(args.baseline)],0),
        ('optimized_E2','20261027_165801',[],0),
        ('baseline_E2','20261027_165801',['--baseline',str(args.baseline)],0)]
    rows=[]
    for name,run,flags,expected in cases:
        cmd=[sys.executable,str(HERE/'tools/replay_planner_benchmark.py'),'--run',str(HERE/'reports'/run),
             '--output',str(args.output/name)]+flags
        with (args.output/(name+'.log')).open('w') as log:
            result=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,timeout=90)
        f=args.output/name/'benchmark.json';data=json.loads(f.read_text()) if f.exists() else {}
        rows.append(dict(case=name,passed=result.returncode==expected,exit_code=result.returncode,result=data))
        (args.output/'matrix.json').write_text(json.dumps(rows,indent=2)+'\n')
        print(name,'exit',result.returncode,'expected',expected,flush=True)
    for name,source in [('oracle_A','optimized_A'),('oracle_coarse_A','coarse_A'),('oracle_E1','optimized_E1'),('oracle_E2','optimized_E2')]:
        data=json.loads((args.output/source/'benchmark.json').read_text())
        if not data['status']['success']:continue
        cmd=[sys.executable,str(HERE/'tools/replay_planner_benchmark.py'),'--run',data['run'],
            '--baseline',str(args.baseline),'--validate-run',data['output_run'],'--output',str(args.output/name)]
        with (args.output/(name+'.log')).open('w') as log:result=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,timeout=90)
        rows.append(dict(case=name,passed=result.returncode==0,exit_code=result.returncode))
        (args.output/'matrix.json').write_text(json.dumps(rows,indent=2)+'\n')
    return 0 if all(r['passed'] for r in rows) else 1

if __name__=='__main__':sys.exit(main())
