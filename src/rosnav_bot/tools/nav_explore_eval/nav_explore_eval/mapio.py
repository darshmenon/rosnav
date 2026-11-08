"""Occupancy grids as numpy arrays: -1 unknown, 0 free, 100 occupied (OccupancyGrid semantics)."""
import os
from dataclasses import dataclass

import numpy as np


@dataclass
class Grid:
    data: np.ndarray       # (h, w) int8, row 0 = lowest y
    resolution: float
    ox: float              # world x of cell (0,0) corner
    oy: float

    @property
    def shape(self):
        return self.data.shape


def from_msg(msg):
    i = msg.info
    d = np.asarray(msg.data, dtype=np.int8).reshape(i.height, i.width)
    return Grid(d, float(i.resolution), float(i.origin.position.x), float(i.origin.position.y))


def _read_pgm(path):
    with open(path, 'rb') as f:
        raw = f.read()
    toks, pos = [], 0
    while len(toks) < 4:                       # magic, w, h, maxval
        while raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b'#':
            pos = raw.index(b'\n', pos) + 1
            continue
        end = pos
        while not raw[end:end + 1].isspace():
            end += 1
        toks.append(raw[pos:end]); pos = end
    magic, w, h, mx = toks[0], int(toks[1]), int(toks[2]), int(toks[3])
    if magic == b'P5':
        pix = np.frombuffer(raw[pos + 1:pos + 1 + w * h], dtype=np.uint8)
    elif magic == b'P2':
        pix = np.array(raw[pos:].split(), dtype=np.uint16)[:w * h]
    else:
        raise ValueError(f'{path}: only PGM (P2/P5) maps supported')
    return pix.reshape(h, w).astype(float) / mx


def load_yaml_map(yaml_path):
    import yaml
    y = yaml.safe_load(open(yaml_path))
    img = y['image']
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img)
    v = _read_pgm(img)
    p = v if y.get('negate', 0) else 1.0 - v           # occupancy probability
    occ_t, free_t = y.get('occupied_thresh', 0.65), y.get('free_thresh', 0.196)
    g = np.full(p.shape, -1, dtype=np.int8)
    g[p > occ_t] = 100
    g[p < free_t] = 0
    g = g[::-1].copy()                                  # image row 0 is top
    return Grid(g, float(y['resolution']), float(y['origin'][0]), float(y['origin'][1]))
