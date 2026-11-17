import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))
MAP = os.path.join(HERE, '..', 'maps', 'map_hospital.yaml')
pytestmark = pytest.mark.skipif(not os.path.exists(MAP), reason='hospital map missing')

import social_force_core as sf  # noqa: E402


@pytest.fixture(scope='module')
def world():
    return sf.MapWorld(MAP)


def test_route_found_and_avoids_walls(world):
    rng = np.random.default_rng(0)
    a = world.sample_free(rng, 0.8)
    b = world.sample_free(rng, 0.8, reachable_from=a)
    r = world.route(a, b)
    assert r and all(world.clearance(*p) > 0.2 for p in r[:-1])


def test_crowd_stays_out_of_walls_and_moves(world):
    c = sf.Crowd(world, 8, seed=1)
    start = c.pos.copy()
    for _ in range(600):                      # 60 s at 10 Hz
        c.step(0.1)
        assert all(world.clearance(*p) > 0.1 for p in c.pos)
    assert np.linalg.norm(c.pos - start, axis=1).max() > 2.0   # somebody walked somewhere
    assert np.isfinite(c.pos).all()


def test_people_keep_personal_distance(world):
    c = sf.Crowd(world, 10, seed=2)
    mn = 1e9
    for _ in range(400):
        c.step(0.1)
        d = np.linalg.norm(c.pos[:, None] - c.pos[None], axis=2) + np.eye(10) * 1e3
        mn = min(mn, d.min())
    assert mn > 0.2                           # bodies (r=0.25 each) rarely interpenetrate hard


def test_clearance_many_matches_scalar(world):
    rng = np.random.default_rng(3)
    pts = np.array([world.sample_free(rng, 0.3) for _ in range(20)])
    assert np.allclose(world.clearance_many(pts), [world.clearance(*p) for p in pts])
    assert world.clearance_many(np.array([[1e4, 1e4]]))[0] == 0.0     # outside the map


def test_remove_small_obstacles_keeps_walls():
    w = sf.MapWorld(MAP)
    from scipy import ndimage
    def big(b):
        lab, _ = ndimage.label(b, structure=np.ones((3, 3)))
        return sum(1 for sl in ndimage.find_objects(lab)
                   if max(sl[0].stop - sl[0].start, sl[1].stop - sl[1].start) * w.res > 0.9)
    before = big(w.blocked)
    n = w.remove_small_obstacles(0.9)
    assert n > 0 and big(w.blocked) == before            # speckle gone, every wall segment kept
    assert w.clearance(*w.sample_free(np.random.default_rng(0), 0.5)) >= 0.5
