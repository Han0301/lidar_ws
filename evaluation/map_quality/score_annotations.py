#!/usr/bin/env python3
"""Optional real manual/surveyed cell truth; never generate truth from prediction."""
import argparse
from pathlib import Path
import numpy as np
from common import sha256, write_json


def score(prediction, truth):
    with np.load(prediction) as p, np.load(truth) as t:
        required={'occupied','evaluation_mask','origin','resolution','provenance'}
        if not required.issubset(t.files):
            raise ValueError('Truth requires occupied, evaluation_mask, origin, resolution, provenance')
        if not str(t['provenance']).strip():
            raise ValueError('Document human/surveyed truth provenance')
        if p['grid'].shape!=t['occupied'].shape or p['grid'].shape!=t['evaluation_mask'].shape:
            raise ValueError('Different grid dimensions')
        if not np.allclose(p['origin'],t['origin'],rtol=0,atol=1e-7) or abs(float(p['resolution'])-float(t['resolution']))>1e-7:
            raise ValueError('Different grid origin/resolution; explicit registration required')
        mask=t['evaluation_mask'].astype(bool);actual=t['occupied'].astype(bool)
        if not mask.any():
            raise ValueError('No annotated evaluation cells')
        # 原始主地图中只以 254 为障碍；膨胀不是几何障碍标签，未知不当自由
        predicted=p['grid']==254
        tp=int(np.sum(mask&predicted&actual));fp=int(np.sum(mask&predicted&~actual));fn=int(np.sum(mask&~predicted&actual))
        return dict(tp=tp,fp=fp,fn=fn,annotated_cells=int(mask.sum()),
            precision=tp/(tp+fp) if tp+fp else None,recall=tp/(tp+fn) if tp+fn else None,
            iou=tp/(tp+fp+fn) if tp+fp+fn else None,unknown_annotated_cells=int(np.sum(mask&(p['grid']==255))),
            truth_provenance=str(t['provenance']),prediction_sha256=sha256(prediction),truth_sha256=sha256(truth),
            note='Geometric occupancy within manually evaluated cells; unknown predicted cells are missed positives when truth is occupied. Inflation ignored as obstacle truth; not path safety or person detection.')


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--prediction',type=Path,required=True)
    p.add_argument('--truth',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    write_json(a.output,score(a.prediction,a.truth))
