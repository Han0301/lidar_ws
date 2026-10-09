"""Serial deterministic map sweeps on one recorded set; no concurrent FAST-LIO jobs."""
import argparse
import subprocess
from pathlib import Path
import yaml
from common import sha256, write_json

p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--profiles',nargs='*');a=p.parse_args()
profiles=yaml.safe_load(Path(__file__).with_name('mapping_sweep.yaml').read_text())
manifest={}
for name,cfg in profiles.items():
    if a.profiles and name not in a.profiles:continue
    output=a.root/'mapping_core'/name
    if output.exists():raise RuntimeError('Existing result refused: '+str(output))
    command=[str(a.root/'benchmark_mapping'),str(a.root/'input/mapping'),str(output),
             str(int(cfg['cache'])),str(cfg['prune_interval']),str(cfg['max_ray_endpoints']),str(cfg['frame_stride']),'1',str(int(cfg['incremental_inner']))]
    subprocess.run(command,check=True)
    manifest[name]=dict(settings=cfg,benchmark_sha256=sha256(a.root/'benchmark_mapping'))
    write_json(a.root/'mapping_sweep_manifest.json',manifest)
