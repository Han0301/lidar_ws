"""Strict identical-intensity input comparison of geometry, boxes and reference gates."""
import argparse
import csv
from pathlib import Path
from common import describe, sha256, write_csv, write_json

p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args()
base=a.root/'perception_baseline';candidate=a.root/'perception_optimized'
rows=lambda p:list(csv.DictReader((p/'metrics.csv').open()))
x,y=rows(base),rows(candidate)
if [r['frame'] for r in x]!=[r['frame'] for r in y]:raise ValueError('Different input frames')
quality=[]
for b,c in zip(x,y):
    frame=b['frame'];different=[k for k in b if k!='ms' and b[k]!=c[k]]
    files={name:sha256(base/(frame+'_'+name+'.'+('csv' if name=='objects' else 'bin')))==sha256(candidate/(frame+'_'+name+'.'+('csv' if name=='objects' else 'bin'))) for name in ['ground','obstacles','clearing','removed','objects']}
    quality.append(dict(frame=frame,metadata_equal=int(not different),different_fields=' '.join(different),**{k+'_identical':int(v) for k,v in files.items()}))
write_csv(a.root/'perception_paired_quality.csv',quality)
result=dict(frames=len(x),true_intensity=True,metadata_identical_frames=sum(r['metadata_equal'] for r in quality),
    geometry_identical_frames={k:sum(r[k+'_identical'] for r in quality) for k in ['ground','obstacles','clearing','removed','objects']},
    baseline=dict(valid_frames=sum(int(r['ground_valid']) for r in x),processing_ms=describe([float(r['ms']) for r in x])),
    optimized=dict(valid_frames=sum(int(r['ground_valid']) for r in y),processing_ms=describe([float(r['ms']) for r in y])),
    accuracy_truth=False)
write_json(a.root/'perception_paired.json',result)
print(result)
