#!/usr/bin/env python3
"""Reproduce the three controlled profiles, preserving failed shutdown evidence."""
import argparse
import datetime
import json
import subprocess
import sys
from pathlib import Path
import yaml
from common import write_json

parser=argparse.ArgumentParser()
parser.add_argument('--output',type=Path)
args=parser.parse_args()
here=Path(__file__).resolve().parent
cfg=yaml.safe_load((here/'quality.yaml').read_text())
ws=Path(cfg['workspace'])
out=(args.output or here/'results'/('optimization_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))).resolve()
out.mkdir(parents=True,exist_ok=False)
statuses={}
for name in ['baseline','fast','ground']:
    command=[sys.executable,str(here/'run_quality.py'),'--config',str(here/('optimization_'+name+'.yaml')),
             '--bag',str(ws/'bags/室外闭环'),'--output',str(out/name),'--navigation']
    with (out/(name+'_runner.log')).open('w') as log:
        returncode=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT).returncode
    status=json.loads((out/name/'run_status.json').read_text())
    statuses[name]=dict(runner_returncode=returncode,**status)
    write_json(out/'suite_status.json',statuses)
    if not status.get('measurement_complete'):
        raise RuntimeError('Incomplete replay: '+name)
    # 原生基线版本的退出崩溃保留为失败；不把测量完成解释为稳定性通过
    if 'shutdown_error' in status:print(name,'shutdown failure retained:',status['shutdown_error'],flush=True)
    subprocess.run([sys.executable,str(here/'analyze_quality.py'),'--run',str(out/name),
                    '--output',str(out/'analysis'/name)],check=True)
subprocess.run([sys.executable,str(here/'compare_optimization.py'),'--root',str(out)],check=True)
print('Artifacts:',out,flush=True)
if any('shutdown_error' in s for s in statuses.values()):raise SystemExit(1)
