"""Pure offline metrics; no ROS or production algorithm dependency."""
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def describe(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return {"count": 0, "mean": None, "p50": None, "p95": None, "max": None}
    return dict(count=len(a), mean=float(a.mean()), p50=float(np.percentile(a, 50)),
                p95=float(np.percentile(a, 95)), max=float(a.max()), min=float(a.min()))


def write_json(path, data):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')


def write_csv(path, rows):
    if not rows:
        Path(path).write_text('', encoding='utf-8')
        return
    with Path(path).open('w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def rotation(q):
    return Rotation.from_quat(q).as_matrix()


def read_pcd(path):
    """Read binary or ASCII scalar PCD; explicitly reject compressed/vector fields."""
    with Path(path).open('rb') as f:
        header = {}
        while True:
            line = f.readline().decode('ascii').strip()
            if not line:
                raise ValueError('Incomplete PCD header')
            if line.startswith('#'):
                continue
            parts = line.split()
            header[parts[0]] = parts[1:]
            if parts[0] == 'DATA':
                break
        if any(int(v) != 1 for v in header.get('COUNT', ['1'] * len(header['FIELDS']))):
            raise ValueError('Vector fields not supported')
        dtype = np.dtype([(name, '<' + {'F': 'f', 'I': 'i', 'U': 'u'}[kind] + size)
                          for name, kind, size in zip(header['FIELDS'], header['TYPE'], header['SIZE'])])
        if header['DATA'] == ['binary']:
            a = np.frombuffer(f.read(), dtype=dtype)
            if len(a) != int(header['POINTS'][0]):
                raise ValueError('PCD point count mismatch')
            xyz = np.column_stack([a[k] for k in ['x', 'y', 'z']])
        elif header['DATA'] == ['ascii']:
            a = np.loadtxt(f)
            xyz = a[:, [header['FIELDS'].index(k) for k in ['x', 'y', 'z']]]
        else:
            raise ValueError('Compressed PCD not supported; convert with PCL')
    return xyz[np.isfinite(xyz).all(axis=1)].astype(float)


def fit_plane(points, config, seed=None):
    """RANSAC then TLS refit; residuals returned for all ROI points and inliers."""
    p = np.asarray(points, float)
    p = p[np.isfinite(p).all(axis=1)]
    if len(p) < config['min_points']:
        return None
    rng = np.random.default_rng(config['seed'] if seed is None else seed)
    sample = p
    if len(p) > config['max_fit_points']:
        sample = p[rng.choice(len(p), config['max_fit_points'], replace=False)]
    best = None
    best_normal = None
    score = -1
    for _ in range(config['iterations']):
        x = sample[rng.choice(len(sample), 3, replace=False)]
        n = np.cross(x[1] - x[0], x[2] - x[0])
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if config.get('normal_kind') == 'vertical' and abs(n[2])>.35:
            continue
        if config.get('normal_kind') == 'horizontal' and abs(n[2])<.95:
            continue
        d = -n @ x[0]
        mask = np.abs(sample @ n + d) <= config['threshold_m']
        if int(mask.sum()) > score:
            score = int(mask.sum())
            best = mask
            best_normal = n.copy()
    if best is None or score < config['min_points']:
        return None
    center = sample[best].mean(axis=0)
    _, singular, vh = np.linalg.svd(sample[best] - center, full_matrices=False)
    if singular[0]<=1e-9 or singular[1]/singular[0]<.01:
        return None
    n = vh[-1]
    if abs(n@best_normal)<np.cos(np.radians(15)):
        return None
    if config.get('normal_kind') == 'vertical' and abs(n[2])>.35:
        return None
    if config.get('normal_kind') == 'horizontal' and abs(n[2])<.95:
        return None
    if n[np.argmax(np.abs(n))] < 0:
        n = -n
    d = -float(n @ center)
    signed = p @ n + d
    mask = np.abs(signed) <= config['threshold_m']
    return n, d, signed, mask


def residual_metrics(signed, threshold):
    a = np.abs(signed)
    return dict(points=len(a), rms_m=float(np.sqrt(np.mean(a*a))),
                mean_abs_m=float(a.mean()), p50_m=float(np.percentile(a,50)),
                p95_m=float(np.percentile(a, 95)), max_m=float(a.max()),
                thickness_90_m=float(np.percentile(signed, 95) - np.percentile(signed, 5)),
                outlier_fraction=float(np.mean(a > threshold)))


def overlap(a, b, margin=2):
    """Compare exact same world cells of axis aligned, equally sized rolling grids."""
    res = float(a['resolution'])
    if abs(res-float(b['resolution'])) > 1e-7:
        raise ValueError('Resolution changed')
    shift = (np.asarray(b['origin'])-np.asarray(a['origin']))/res
    if np.max(np.abs(shift-np.rint(shift))) > 1e-4:
        raise ValueError('Origins do not share a lattice')
    dx, dy = np.rint(shift).astype(int)
    ah, aw = a['grid'].shape
    bh, bw = b['grid'].shape
    x0, x1 = max(margin, dx+margin), min(aw-margin, dx+bw-margin)
    y0, y1 = max(margin, dy+margin), min(ah-margin, dy+bh-margin)
    if x1 <= x0 or y1 <= y0:
        return np.array([], np.uint8), np.array([], np.uint8)
    return (a['grid'][y0:y1, x0:x1], b['grid'][y0-dy:y1-dy, x0-dx:x1-dx])
