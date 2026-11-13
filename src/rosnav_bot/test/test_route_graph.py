"""Route graph generator: clearance, connectivity, GeoJSON shape (no ROS needed)."""
import importlib.util
import os

import numpy as np

_p = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'map_to_route_graph.py')
_s = importlib.util.spec_from_file_location('map_to_route_graph', _p)
m = importlib.util.module_from_spec(_s)
_s.loader.exec_module(m)


def _room():
    free = np.zeros((60, 60), bool)
    free[5:55, 5:55] = True          # big room
    free[28:32, 5:30] = False        # wall stub
    return free


def test_nodes_respect_clearance():
    nodes, _ = m.build_graph(_room(), 0.05, (0.0, 0.0), clearance=0.4, spacing=0.5)
    assert nodes
    for x, y in nodes:
        assert 0.25 + 0.3 <= x <= 2.75 - 0.3 + 1e-6
        assert 0.25 + 0.3 <= y <= 2.75 - 0.3 + 1e-6


def test_edges_avoid_wall_and_one_component():
    nodes, edges = m.build_graph(_room(), 0.05, (0.0, 0.0), clearance=0.2, spacing=0.4)
    nodes, edges = m.largest_component(nodes, edges)
    gj = m.to_geojson(nodes, edges)
    pts = [f for f in gj['features'] if 'startid' not in f['properties']]
    lines = [f for f in gj['features'] if 'startid' in f['properties']]
    assert len(pts) == len(nodes) and len(lines) == 2 * len(edges)
    ids = {f['properties']['id'] for f in gj['features']}
    assert len(ids) == len(gj['features'])          # unique ids
    for a, b in edges:                                # no lane crosses the wall
        (x1, y1), (x2, y2) = nodes[a], nodes[b]
        for t in np.linspace(0, 1, 20):
            x, y = x1 + t * (x2 - x1), y1 + t * (y2 - y1)
            assert not (1.4 <= y <= 1.6 and x <= 1.5)
