"""Build an RMF nav graph from locations + lane list."""

from __future__ import annotations

from typing import Dict, List, Tuple

import rmf_adapter.graph as graph

from rosnav_bot.rmf.config import RmfFleetConfig


def build_graph(cfg: RmfFleetConfig) -> Tuple[graph.Graph, Dict[str, int]]:
    """Return ``(rmf Graph, name→waypoint_index)``."""
    g = graph.Graph()
    index_of: Dict[str, int] = {}

    for name, coords in cfg.locations.items():
        x, y = float(coords[0]), float(coords[1])
        wp = g.add_waypoint(cfg.map_name, [x, y])
        if name == cfg.charger:
            wp.set_charger(True)
        index_of[name] = wp.index
        g.add_key(name, wp.index)

    for a, b in cfg.lanes:
        if a not in index_of or b not in index_of:
            print(f'[rmf] skipping lane {a}<->{b}: location missing from locations.yaml')
            continue
        g.add_bidir_lane(index_of[a], index_of[b])

    print(f'[rmf] graph: {g.num_waypoints} waypoint(s), {g.num_lanes} lane(s) '
          f'on map {cfg.map_name!r}: {sorted(index_of)}')
    return g, index_of
