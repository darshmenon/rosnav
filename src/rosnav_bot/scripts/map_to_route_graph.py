#!/usr/bin/env python3
"""Generate a nav2_route GeoJSON graph from a saved occupancy map.

Nodes are placed on a coarse grid wherever the free-space clearance (distance
transform) is at least --clearance; edges join 8-neighbours whose straight
segment keeps that clearance. Result: a lane graph that stays mid-aisle.

  map_to_route_graph.py maps/map_cafe.yaml -o maps/route_cafe.geojson
"""
import argparse
import json
import math
import os

import numpy as np
import yaml
from scipy import ndimage


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img = meta['image']
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(yaml_path), img)
    with open(img, 'rb') as f:
        assert f.readline().strip() == b'P5', 'only binary PGM supported'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        data = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    free_thresh = meta.get('free_thresh', 0.25)
    occ = (255 - data) / 255.0 if not meta.get('negate', 0) else data / 255.0
    free = occ < free_thresh
    return free[::-1], meta['resolution'], meta['origin'][:2]  # row0 = bottom


def build_graph(free, res, origin, clearance=0.45, spacing=1.0):
    """Return (nodes[(x, y)], edges[(i, j)]) in map-frame metres."""
    dist = ndimage.distance_transform_edt(free) * res
    step = max(1, int(round(spacing / res)))
    ok = dist >= clearance
    idx, nodes = {}, []
    for r in range(step // 2, free.shape[0], step):
        for c in range(step // 2, free.shape[1], step):
            if ok[r, c]:
                idx[(r, c)] = len(nodes)
                nodes.append((origin[0] + (c + 0.5) * res, origin[1] + (r + 0.5) * res))
    edges = []
    for (r, c), i in idx.items():
        for dr, dc in ((0, step), (step, 0), (step, step), (step, -step)):
            j = idx.get((r + dr, c + dc))
            if j is None:
                continue
            n = max(abs(dr), abs(dc))
            rr = np.rint(np.linspace(r, r + dr, n + 1)).astype(int)
            cc = np.rint(np.linspace(c, c + dc, n + 1)).astype(int)
            if ok[rr, cc].all():
                edges.append((i, j))
    return nodes, edges


def to_geojson(nodes, edges):
    feats = [{'type': 'Feature', 'properties': {'id': i, 'frame': 'map'},
              'geometry': {'type': 'Point', 'coordinates': [x, y]}}
             for i, (x, y) in enumerate(nodes)]
    eid = len(nodes)
    for a, b in edges:
        for s, e in ((a, b), (b, a)):  # bidirectional
            feats.append({'type': 'Feature',
                          'properties': {'id': eid, 'startid': s, 'endid': e},
                          'geometry': {'type': 'LineString',
                                       'coordinates': [[list(nodes[s]), list(nodes[e])]]}})
            eid += 1
    return {'type': 'FeatureCollection', 'name': 'graph', 'features': feats}


def largest_component(nodes, edges):
    """Keep only the biggest connected set so the router never strands a goal."""
    parent = list(range(len(nodes)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for a, b in edges:
        parent[find(a)] = find(b)
    roots = [find(i) for i in range(len(nodes))]
    best = max(set(roots), key=roots.count)
    keep = [i for i, r in enumerate(roots) if r == best]
    remap = {old: new for new, old in enumerate(keep)}
    return ([nodes[i] for i in keep],
            [(remap[a], remap[b]) for a, b in edges if a in remap and b in remap])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('map_yaml')
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--clearance', type=float, default=0.45)
    ap.add_argument('--spacing', type=float, default=1.0)
    a = ap.parse_args()
    free, res, origin = load_map(a.map_yaml)
    nodes, edges = largest_component(*build_graph(free, res, origin, a.clearance, a.spacing))
    with open(a.out, 'w') as f:
        json.dump(to_geojson(nodes, edges), f)
    print(f'route graph: {len(nodes)} nodes, {len(edges)} lanes -> {a.out}')


if __name__ == '__main__':
    main()
