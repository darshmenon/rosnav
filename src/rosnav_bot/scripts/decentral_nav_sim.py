#!/usr/bin/env python3
"""
decentral_nav_sim.py — pure-Python (no ROS/Gazebo) decentralized multi-robot
navigation: every robot has its OWN goal and only sees neighbours' position +
velocity (no central planner). Same spirit as social_nav_core.py.

Controllers (all holonomic velocity commands, 10 Hz):
  pref        goal-seek along a BFS distance field, no avoidance (baseline)
  vo          sampled velocity obstacle (ORCA-like, reactive, 1-step horizon)
  mppi        plain MPPI over a 1.2 s horizon, zero-mean noise (uninformed sampling)
  mppi_prior  MPPI whose sampling mean is the VO velocity (CoRL-MPPI idea with a
              heuristic prior instead of a learned one) + id-priority yielding
              in head-on symmetry (our twist: breaks deadlocks without comms)

Scenarios:
  swap     antipodal circle swap (every robot crosses the centre)
  maze     random starts/goals on a MovingAI map crop
  convoy   robot 0 drives to its goal; follower k tracks the leader's trail k*1.2 m behind
           its head (queue seeded behind the leader). Success = leader at goal AND every
           follower parked in its slot behind it, no collisions.

Metrics per run: success (all arrive), makespan, mean delay vs shortest path,
robot-robot collisions (overlap ticks), deadlock (timeout).

  python3 decentral_nav_sim.py bench --n 8 --seeds 10
  python3 decentral_nav_sim.py gif --scenario swap --ctrl mppi_prior --n 8
"""
import argparse
import math
import os
import sys
from collections import deque

import numpy as np
from scipy.ndimage import distance_transform_edt

DT = 0.1
R = 0.30            # robot radius
VMAX = 1.0
OUT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'images', 'decentral_nav'))
MAP_DIR = os.path.expanduser('~/inspiration/movingai_maps/maps')
CTRLS = ('pref', 'vo', 'mppi', 'mppi_prior')
SCENARIOS = ('swap', 'maze', 'convoy', 'corridor', 'crowd')
BASE_SCENARIOS = ('swap', 'maze', 'convoy')   # default bench set (unchanged)
PED_R = 0.25                                  # pedestrian radius
PERSONAL_SPACE = 1.2                          # m centre-to-centre, same as social_nav_core.py


# ── world ────────────────────────────────────────────────────────────────────
class World:
    """Occupancy grid, 1 cell = 1 m. free[y, x]."""

    def __init__(self, free):
        self.free = free.astype(bool)
        self.h, self.w = free.shape
        self.clear = distance_transform_edt(self.free)   # metres to nearest obstacle centre
        self._cache = {}

    def ok(self, p, r=R):
        """True iff a disc of radius r at p overlaps no blocked cell (cells are
        axis-aligned 1 m squares centred on integer coords; off-map = blocked)."""
        x, y = float(p[0]), float(p[1])
        ix, iy = int(math.floor(x + 0.5)), int(math.floor(y + 0.5))
        r2 = r * r
        for cy in (iy - 1, iy, iy + 1):
            for cx in (ix - 1, ix, ix + 1):
                if 0 <= cx < self.w and 0 <= cy < self.h and self.free[cy, cx]:
                    continue
                dx = max(abs(x - cx) - 0.5, 0.0)
                dy = max(abs(y - cy) - 0.5, 0.0)
                if dx * dx + dy * dy < r2 - 1e-12:
                    return False
        return True

    def dist_field(self, goal):
        key = (int(round(goal[0])), int(round(goal[1])))
        if key in self._cache:
            return self._cache[key]
        d = np.full(self.free.shape, np.inf)
        q = deque([(key[1], key[0])])
        d[key[1], key[0]] = 0.0
        while q:
            y, x = q.popleft()
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < self.h and 0 <= nx < self.w and self.free[ny, nx] and d[ny, nx] == np.inf:
                    d[ny, nx] = d[y, x] + 1.0
                    q.append((ny, nx))
        self._cache[key] = d
        return d

    def pref_dir(self, p, goal):
        """Unit vector along the BFS gradient, looking ~2 cells ahead."""
        d = self.dist_field(goal)
        ix, iy = int(round(p[0])), int(round(p[1]))
        if not (0 <= ix < self.w and 0 <= iy < self.h) or d[iy, ix] == np.inf:
            g = np.array(goal) - p
            return g / (np.linalg.norm(g) + 1e-9)
        cx, cy = ix, iy
        for _ in range(2):
            best, bx, by = d[cy, cx], cx, cy
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    nx, ny = cx + dx, cy + dy
                    if dx and dy and not (0 <= nx < self.w and 0 <= ny < self.h
                                          and self.free[cy, nx] and self.free[ny, cx]):
                        continue                      # no diagonal squeeze past a corner
                    if 0 <= nx < self.w and 0 <= ny < self.h and d[ny, nx] < best:
                        best, bx, by = d[ny, nx], nx, ny
            cx, cy = bx, by
        if d[cy, cx] == 0 and (cx, cy) == (ix, iy):
            g = np.array(goal) - p
            return g / (np.linalg.norm(g) + 1e-9)
        v = np.array([cx, cy], float) - p
        return v / (np.linalg.norm(v) + 1e-9)


class CorridorWorld(World):
    """Two rooms joined by a corridor whose walls are inset so the clear width is
    1.5 m (the 1 m grid alone could only give 1 or 2 m). Inset = extra wall slabs."""

    def __init__(self, free, x0, x1, y0, width=1.5):
        super().__init__(free)
        gap = (2.0 - width) / 2.0                 # wall thickness added per side
        self.slabs = [(x0 - 0.5, x1 + 0.5, y0 - 0.5, y0 - 0.5 + gap),
                      (x0 - 0.5, x1 + 0.5, y0 + 1.5 - gap, y0 + 1.5)]

    def ok(self, p, r=R):
        if not super().ok(p, r):
            return False
        x, y = float(p[0]), float(p[1])
        for xa, xb, ya, yb in self.slabs:
            dx = max(xa - x, 0.0, x - xb)
            dy = max(ya - y, 0.0, y - yb)
            if dx * dx + dy * dy < r * r - 1e-12:
                return False
        return True


def open_world(size):
    f = np.ones((size, size), bool)
    f[0, :] = f[-1, :] = f[:, 0] = f[:, -1] = False
    return World(f)


def load_map(name='den312d.map', size=40, seed=0):
    path = os.path.join(MAP_DIR, name)
    with open(path) as fh:
        lines = fh.read().splitlines()
    i = lines.index('map') + 1
    g = np.array([[c in '.GS' for c in row] for row in lines[i:]])
    rng = np.random.default_rng(seed)
    for _ in range(200):
        y = int(rng.integers(0, max(1, g.shape[0] - size)))
        x = int(rng.integers(0, max(1, g.shape[1] - size)))
        crop = g[y:y + size, x:x + size]
        if crop.mean() > 0.55:
            break
    # keep the largest connected free component so every goal is reachable
    from scipy.ndimage import label
    lab, n = label(crop)
    if n:
        big = np.argmax(np.bincount(lab.ravel())[1:]) + 1
        crop = lab == big
    return World(crop)


# ── scenarios ────────────────────────────────────────────────────────────────
def make_scenario(kind, n, seed):
    rng = np.random.default_rng(seed)
    if kind == 'swap':
        world = open_world(max(16, int(2 * (n * 0.45 + 3))))
        c = np.array([world.w, world.h]) / 2.0
        rad = c[0] - 2.5
        ang = 2 * math.pi * np.arange(n) / n + rng.uniform(0, 0.2)
        starts = c + rad * np.stack([np.cos(ang), np.sin(ang)], 1)
        goals = c - rad * np.stack([np.cos(ang), np.sin(ang)], 1)
        return world, starts, goals, None
    if kind == 'corridor':
        return _corridor_scenario(n, rng)
    if kind == 'crowd':
        return _crowd_scenario(n, rng)
    world = load_map(seed=seed) if kind in ('maze', 'convoy') else open_world(30)
    cells = np.argwhere(world.clear > 1.5)[:, ::-1].astype(float)
    if len(cells) < 2 * n:
        cells = np.argwhere(world.free)[:, ::-1].astype(float)
    pick = []
    for idx in rng.permutation(len(cells)):
        if all(np.linalg.norm(cells[idx] - cells[j]) > 1.4 for j in pick):
            pick.append(idx)
        if len(pick) == 2 * n:
            break
    pts = cells[pick]
    if kind == 'convoy':
        return _convoy_scenario(world, cells, n, rng)
    return world, pts[:n], pts[n:2 * n], None


def _spread(cands, k, rng, gap=1.4):
    """Pick k points from cands with pairwise spacing >= gap (relaxed if too dense)."""
    for g in (gap, 1.1, 0.9):
        pick = []
        for idx in rng.permutation(len(cands)):
            if all(np.linalg.norm(cands[idx] - cands[j]) >= g for j in pick):
                pick.append(idx)
            if len(pick) == k:
                return cands[pick]
    raise ValueError('room too small for %d robots' % k)


def _corridor_scenario(n, rng, length=10):
    """Left room | 1.5 m x `length` m corridor | right room. Group A (left) and
    group B (right) swap sides, i.e. everyone must pass someone head-on inside."""
    rw, h = 6, 8
    w = 2 * rw + length + 2
    f = np.zeros((h, w), bool)
    f[1:h - 1, 1:1 + rw] = True
    f[1:h - 1, w - 1 - rw:w - 1] = True
    x0, x1 = 1 + rw, w - 2 - rw
    f[3:5, x0:x1 + 1] = True
    world = CorridorWorld(f, x0, x1, 3)
    na, nb = (n + 1) // 2, n // 2
    left = np.array([(x, y) for x in range(1, 1 + rw) for y in range(1, h - 1)], float)
    right = left.copy()
    right[:, 0] = (w - 1) - right[:, 0]
    a = _spread(left, na, rng)
    b = _spread(right, nb, rng)
    starts = np.vstack([a, b])
    goals = np.vstack([_spread(right, na, rng), _spread(left, nb, rng)])
    return world, starts, goals, None


def _crowd_scenario(n, rng, n_peds=None, w=26, h=14):
    """Hall; robots cross left->right (stratified in y) through patrolling
    pedestrians that walk straight at constant speed (top<->bottom and
    right<->left), so robots must treat them as moving obstacles."""
    f = np.zeros((h, w), bool)
    f[1:-1, 1:-1] = True
    world = World(f)
    ys = np.linspace(2, h - 3, n) + rng.uniform(-0.3, 0.3, n)
    starts = np.stack([np.full(n, 2.0), ys], 1)
    goals = np.stack([np.full(n, w - 3.0), rng.permutation(ys)], 1)
    m = n if n_peds is None else n_peds
    pa, pb = [], []
    for k in range(m):
        if k % 3 == 2:                                       # head-on walker
            y = rng.uniform(2, h - 3)
            pa.append((w - 4.0, y))
            pb.append((3.0, y + rng.uniform(-1, 1)))
        else:                                                # cross-flow walker
            x = rng.uniform(5, w - 6)
            up = k % 2 == 0
            pa.append((x, 1.5 if up else h - 2.5))
            pb.append((x + rng.uniform(-1.5, 1.5), h - 2.5 if up else 1.5))
    peds = dict(pos=np.array(pa, float), goal=np.array(pb, float),
                speed=rng.uniform(0.6, 1.0, m))
    return world, starts, goals, dict(peds=peds)


class Peds:
    """Constant-speed patrolling walkers with weak social-force-like reaction:
    they sway away from robots inside 1.2 m but never stop or yield fully."""

    def __init__(self, spec):
        self.pos = spec['pos'].copy()
        self.a, self.b = spec['pos'].copy(), spec['goal'].copy()
        self.speed = spec['speed']
        self.vel = np.zeros_like(self.pos)

    def step(self, robots):
        tgt = self.b
        d = tgt - self.pos
        dist = np.linalg.norm(d, axis=1, keepdims=True)
        dirv = d / np.maximum(dist, 1e-9)
        rep = np.zeros_like(self.pos)
        for q in robots:
            r = self.pos - q
            rd = np.linalg.norm(r, axis=1, keepdims=True)
            rep += np.where(rd < PERSONAL_SPACE, (PERSONAL_SPACE - rd) / np.maximum(rd, 1e-9) * r, 0.0)
        v = dirv + 0.5 * rep
        v = v / np.maximum(np.linalg.norm(v, axis=1, keepdims=True), 1e-9) * self.speed[:, None]
        self.pos = self.pos + v * DT
        self.vel = v
        arrived = dist[:, 0] < 0.3
        if arrived.any():                                    # patrol: swap endpoints
            self.a[arrived], self.b[arrived] = self.b[arrived].copy(), self.a[arrived].copy()


CONVOY_SPACING = 1.2


class Trail:
    """Polyline of the leader's past positions with cumulative arc length."""

    def __init__(self, pts):
        self.pts = [np.asarray(p, float) for p in pts]
        self.cum = [0.0]
        for a, b in zip(self.pts[:-1], self.pts[1:]):
            self.cum.append(self.cum[-1] + float(np.linalg.norm(b - a)))

    def push(self, p):
        d = float(np.linalg.norm(p - self.pts[-1]))
        if d > 1e-6:
            self.pts.append(np.asarray(p, float).copy())
            self.cum.append(self.cum[-1] + d)

    def at_lag(self, lag):
        """Point on the trail `lag` metres of path behind the leader's head."""
        a = max(0.0, self.cum[-1] - lag)
        k = int(np.searchsorted(self.cum, a))
        if k <= 0:
            return self.pts[0].copy()
        if k >= len(self.pts):
            return self.pts[-1].copy()
        seg = self.cum[k] - self.cum[k - 1]
        f = (a - self.cum[k - 1]) / seg if seg > 1e-9 else 1.0
        return self.pts[k - 1] + f * (self.pts[k] - self.pts[k - 1])


def _convoy_scenario(world, cells, n, rng):
    """Leader 0 drives start->goal. Followers queue on a real path BEHIND the
    leader (spacing CONVOY_SPACING along that path); that queue seeds the trail."""
    cell_ok = [c for c in cells if world.clear[int(c[1]), int(c[0])] > 1.5]
    cells = np.array(cell_ok) if len(cell_ok) > 2 else cells
    need = (n - 1) * CONVOY_SPACING + 2.0
    best = None
    for idx in rng.permutation(len(cells))[:60]:
        s0 = cells[idx]
        ds = world.dist_field(s0)
        reach = np.isfinite(ds)
        far = np.argwhere(reach & (ds >= need) & (ds <= need + 2))
        goal_c = np.argwhere(reach & (ds >= 15))
        if len(far) and len(goal_c):
            best = (s0, ds, far, goal_c)
            break
    if best is None:                            # tiny/cramped map: take the most spacious start
        s0 = cells[int(rng.integers(len(cells)))]
        ds = world.dist_field(s0)
        fin = np.argwhere(np.isfinite(ds))
        far = fin[np.argsort(-ds[fin[:, 0], fin[:, 1]])[:5]]
        goal_c = far
        best = (s0, ds, far, goal_c)
    s0, ds, far, goal_c = best
    goal = goal_c[int(rng.integers(len(goal_c)))][::-1].astype(float)
    dg = world.dist_field(goal)
    tail = far[np.argmax(dg[far[:, 0], far[:, 1]])]          # farthest-from-goal candidate
    cy, cx = int(tail[0]), int(tail[1])
    path = [np.array([cx, cy], float)]
    while ds[cy, cx] > 0:                                    # descend back to the leader start
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ny, nx = cy + dy, cx + dx
            if 0 <= ny < world.h and 0 <= nx < world.w and ds[ny, nx] == ds[cy, cx] - 1:
                cy, cx = ny, nx
                break
        path.append(np.array([cx, cy], float))
    tr = Trail(path)                                          # tail -> leader start
    sp = min(CONVOY_SPACING, tr.cum[-1] / max(1, n))
    starts = np.array([tr.at_lag(k * sp) for k in range(n)])
    goals = np.array([goal] * n)
    return world, starts, goals, dict(trail=Trail(path), spacing=sp)


# ── controllers ──────────────────────────────────────────────────────────────
def vo_velocity(i, pos, vel, goal, world, pref, k=121, horizon=3.0):
    """Sample velocities; cost = deviation from pref + time-to-collision penalty."""
    ang = np.linspace(0, 2 * math.pi, 40, endpoint=False)
    cand = [np.zeros(2)]
    for sp in (0.35 * VMAX, 0.7 * VMAX, VMAX):
        cand += [sp * np.array([math.cos(a), math.sin(a)]) for a in ang]
    cand = np.array(cand)
    cost = np.linalg.norm(cand - pref, axis=1)
    for j in range(len(pos)):
        if j == i:
            continue
        rel_p = pos[j] - pos[i]
        if np.linalg.norm(rel_p) > 6.0:
            continue
        # reciprocal: each takes half the avoidance -> assume other keeps its velocity
        rv = cand - vel[j]
        a = (rv * rv).sum(1)
        b = -2 * (rv @ rel_p)
        c = rel_p @ rel_p - (2 * R + 0.1) ** 2
        disc = b * b - 4 * a * c
        ttc = np.full(len(cand), np.inf)
        hit = (disc > 0) & (a > 1e-9)
        t = (-b[hit] - np.sqrt(disc[hit])) / (2 * a[hit])
        ttc[hit] = np.where(t >= 0, t, np.inf)
        # already inside the inflated radius: only moving closer counts as a hit
        # (lets a packed cluster peel apart instead of freezing)
        inside = c < 0
        ttc = np.where(inside, np.where(rv @ rel_p > 0, 0.0, np.inf), ttc)
        cost += np.where(ttc < horizon, 2.5 * (horizon - ttc) / horizon * 3 + 1.0, 0.0)
        # right-hand rule: when j is ahead, prefer velocities that pass on its left
        # (j to my right => rotate right). Breaks the symmetric head-on stand-off.
        dj = np.linalg.norm(rel_p)
        if dj < 4.0 and rel_p @ pref > 0:
            cross = cand[:, 0] * rel_p[1] - cand[:, 1] * rel_p[0]     # >0: passing j on its right
            cost += 0.35 * (1.0 - dj / 4.0) * np.where(cross > 0, 1.0, 0.0) * (np.linalg.norm(cand, axis=1) > 0.05)
    cost += 0.15 * np.linalg.norm(cand - vel[i], axis=1)      # smoothness / hysteresis
    if np.linalg.norm(vel[i]) < 0.1:                           # stalled: standing still is not free
        cost[0] += 1.2
    # keep candidates out of walls over the next 0.5 s
    for m, v in enumerate(cand):
        if not world.ok(pos[i] + v * 0.5):
            cost[m] += 4.0
    return cand[int(np.argmin(cost))]


def mppi_velocity(i, pos, vel, goal, world, pref, rng, prior=None, K=96, H=12, lam=0.35, sigma=0.55):
    mean = pref if prior is None else prior
    u = mean[None, :] + rng.normal(0, sigma, (K, 2))
    if prior is not None:                       # keep a few uninformed + pref samples
        u[: K // 8] = pref + rng.normal(0, sigma, (K // 8, 2))
    sp = np.linalg.norm(u, axis=1, keepdims=True)
    u = u * np.minimum(1.0, VMAX / np.maximum(sp, 1e-9))
    others = [j for j in range(len(pos)) if j != i and np.linalg.norm(pos[j] - pos[i]) < 7.0]
    d_goal = world.dist_field(goal)
    cost = np.zeros(K)
    p = np.repeat(pos[i][None, :], K, 0)
    for h in range(H):
        p = p + u * DT * 2.0                     # 0.2 s per MPPI step
        for j in others:
            q = pos[j] + vel[j] * DT * 2.0 * (h + 1)
            dd = np.linalg.norm(p - q, axis=1)
            cost += np.where(dd < 2 * R + 0.15, 60.0, 0.0) + np.where(dd < 2 * R + 0.7, 4.0 * (2 * R + 0.7 - dd), 0.0)
    ix = np.clip(np.rint(p[:, 0]).astype(int), 0, world.w - 1)
    iy = np.clip(np.rint(p[:, 1]).astype(int), 0, world.h - 1)
    dg = d_goal[iy, ix]
    cost += 1.2 * np.where(np.isinf(dg), 1e3, dg) + np.where(world.clear[iy, ix] < 1.0, 30.0, 0.0)
    cost += 0.05 * np.linalg.norm(u - pref, axis=1) * H
    w = np.exp(-(cost - cost.min()) / (lam * max(cost.std(), 1e-6) * 3))
    w /= w.sum()
    return (w[:, None] * u).sum(0)


def command(ctrl, i, pos, vel, goals, world, rng):
    g = goals[i]
    if np.linalg.norm(g - pos[i]) < 0.25:
        return np.zeros(2)
    pref = VMAX * world.pref_dir(pos[i], g)
    if ctrl == 'pref':
        return pref
    if ctrl == 'vo':
        return vo_velocity(i, pos, vel, g, world, pref)
    if ctrl == 'mppi':
        return mppi_velocity(i, pos, vel, g, world, pref, rng)
    # mppi_prior: VO velocity as prior + id-priority yield in head-on symmetry
    prior = vo_velocity(i, pos, vel, g, world, pref)
    for j in range(len(pos)):
        if j == i:
            continue
        rel = pos[j] - pos[i]
        dist = np.linalg.norm(rel)
        if dist < 2.2 and np.linalg.norm(vel[j]) < 0.15 and np.linalg.norm(vel[i]) < 0.15 and i > j:
            side = np.array([-rel[1], rel[0]]) / (dist + 1e-9)   # lower-priority robot sidesteps
            prior = 0.6 * VMAX * side
    return mppi_velocity(i, pos, vel, g, world, pref, rng, prior=prior)


# ── simulation ───────────────────────────────────────────────────────────────
def simulate(kind, ctrl, n=8, seed=0, max_t=90.0, record=False,
             obs_noise=0.0, delay_steps=0, act_noise=0.0):
    """obs_noise: sigma (m) of Gaussian on neighbours' observed position, and
    sigma/DT-scaled... velocity sigma = obs_noise (m/s); delay_steps: neighbour
    observations are that many ticks stale; act_noise: sigma (m/s) added to each
    commanded velocity. All 0 -> identical to the noise-free simulator."""
    world, starts, goals, extra = make_scenario(kind, n, seed)
    convoy = extra if extra is not None and 'trail' in extra else None
    peds = Peds(extra['peds']) if extra is not None and 'peds' in extra else None
    nrng = np.random.default_rng(seed + 2000)           # separate stream: rng untouched
    n = len(starts)
    rng = np.random.default_rng(seed + 1000)
    pos = starts.astype(float).copy()
    vel = np.zeros_like(pos)
    done_t = np.full(n, np.nan)
    ideal = np.array([world.dist_field(goals[i])[int(round(starts[i][1])), int(round(starts[i][0]))] for i in range(n)]) / VMAX
    ideal = np.maximum(0.8 * ideal, np.linalg.norm(goals - starts, axis=1) / VMAX)   # 4-conn BFS overestimates diagonals
    ideal = np.where(np.isinf(ideal), np.nan, ideal)
    if convoy is not None:
        ideal[:] = ideal[0]                   # whole convoy is bounded by the leader's path
    coll_ticks = 0
    pair_hit = set()
    ped_hit, min_ped, intr_s = set(), np.inf, 0.0
    hist = deque(maxlen=max(1, delay_steps + 1))
    ptrail = [peds.pos.copy()] if (record and peds is not None) else None
    trail = [pos.copy()] if record else None
    goals = goals.astype(float).copy()
    if convoy is not None:
        ctrail, lag = convoy['trail'], convoy['spacing']
        for k in range(1, n):
            goals[k] = ctrail.at_lag(k * lag)
    steps = int(max_t / DT)
    for s in range(steps):
        t = s * DT
        if convoy is not None:                    # follower k aims k*spacing behind the leader's head on its trail
            ctrail.push(pos[0])
            for k in range(1, n):
                goals[k] = ctrail.at_lag(k * lag)
        cmds = np.zeros_like(pos)
        allp = pos if peds is None else np.vstack([pos, peds.pos])
        allv = vel if peds is None else np.vstack([vel, peds.vel])
        hist.append((allp.copy(), allv.copy()))
        sp_, sv_ = hist[0]                                  # oldest = delay_steps stale
        for i in range(n):
            if not np.isnan(done_t[i]):
                continue
            if obs_noise > 0 or delay_steps > 0:
                op, ov = sp_.copy(), sv_.copy()
                if obs_noise > 0:
                    op += nrng.normal(0, obs_noise, op.shape)
                    ov += nrng.normal(0, obs_noise, ov.shape)
                op[i], ov[i] = pos[i], vel[i]               # own state is exact
            else:
                op, ov = allp, allv
            cmds[i] = command(ctrl, i, op, ov, goals, world, rng)
            if act_noise > 0:
                cmds[i] = cmds[i] + nrng.normal(0, act_noise, 2)
        newpos = pos.copy()
        for i in range(n):
            step = cmds[i] * DT
            for cand in (pos[i] + step, pos[i] + np.array([step[0], 0]), pos[i] + np.array([0, step[1]])):
                if world.ok(cand):
                    newpos[i] = cand
                    break
        vel = (newpos - pos) / DT
        pos = newpos
        if peds is not None:
            peds.step(pos)
            dp = np.linalg.norm(pos[:, None] - peds.pos[None], axis=2)
            min_ped = min(min_ped, float(dp.min()))
            ped_hit.update(map(tuple, np.argwhere(dp < R + PED_R)))
            intr_s += DT * int((dp.min(axis=1) < PERSONAL_SPACE).sum())
            if record:
                ptrail.append(peds.pos.copy())
        for i in range(n):
            if np.isnan(done_t[i]) and np.linalg.norm(goals[i] - pos[i]) < 0.35:
                if convoy is not None and i > 0 and np.isnan(done_t[0]):
                    continue                      # slots only count once the leader has arrived
                done_t[i] = t
        d = np.linalg.norm(pos[:, None] - pos[None], axis=2) + np.eye(n) * 99
        hit = np.argwhere(np.triu(d < 2 * R - 0.02))
        coll_ticks += len(hit)
        pair_hit.update(map(tuple, hit))
        if record:
            trail.append(pos.copy())
        if not np.isnan(done_t).any():
            break
    ok = not np.isnan(done_t).any()
    res = dict(success=ok, makespan=float(np.nanmax(done_t)) if ok else max_t,
               delay=float(np.nanmean(np.where(np.isnan(done_t), max_t, done_t) - ideal)),
               collisions=len(pair_hit), coll_ticks=coll_ticks, n=n,
               ped_contacts=len(ped_hit), min_ped_dist=(min_ped if peds is not None else float('nan')),
               ped_intrusion_s=intr_s)
    if record:
        res['trail'] = np.array(trail)
        res['ped_trail'] = np.array(ptrail) if ptrail is not None else None
        res['world'] = world
        res['starts'], res['goals'] = starts, goals
    return res


# ── benchmark + visuals ──────────────────────────────────────────────────────
def bench(args):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    os.makedirs(OUT, exist_ok=True)
    rows = []
    for kind in args.scenarios:
        for ctrl in args.ctrls:
            rs = [simulate(kind, ctrl, args.n, sd, args.max_t, obs_noise=args.obs_noise,
                           delay_steps=args.delay_steps, act_noise=args.act_noise)
                  for sd in range(args.seeds)]
            row = dict(scenario=kind, ctrl=ctrl,
                       success=100 * np.mean([r['success'] for r in rs]),
                       makespan=np.mean([r['makespan'] for r in rs if r['success']] or [np.nan]),
                       delay=np.mean([r['delay'] for r in rs]),
                       collisions=np.mean([r['collisions'] for r in rs]),
                       ped_contacts=np.mean([r['ped_contacts'] for r in rs]),
                       intrusion=np.mean([r['ped_intrusion_s'] for r in rs]),
                       min_ped=float(np.nanmin([r['min_ped_dist'] for r in rs])) if kind == 'crowd' else float('nan'))
            rows.append(row)
            print(f"{kind:7s} {ctrl:11s} succ={row['success']:5.1f}% makespan={row['makespan']:6.1f}s "
                  f"delay={row['delay']:6.1f}s coll_pairs={row['collisions']:.2f} "
                  f"ped_contacts={row['ped_contacts']:.2f} intr={row['intrusion']:.1f}s", flush=True)
    md = ['| scenario | controller | success % | makespan s | delay s | collision pairs | ped contacts | ped intrusion s | min ped dist m |', '|---|---|---|---|---|---|---|---|---|']
    md += [f"| {r['scenario']} | {r['ctrl']} | {r['success']:.0f} | {r['makespan']:.1f} | {r['delay']:.1f} | {r['collisions']:.2f} | {r['ped_contacts']:.2f} | {r['intrusion']:.1f} | {r['min_ped']:.2f} |" for r in rows]
    with open(os.path.join(OUT, f'bench{args.tag}.md'), 'w') as fh:
        fh.write(f'n={args.n} robots, {args.seeds} seeds, max_t={args.max_t}s, obs_noise={args.obs_noise}, delay_steps={args.delay_steps}, act_noise={args.act_noise}\n\n' + '\n'.join(md) + '\n')
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.6))
    kinds = list(dict.fromkeys(r['scenario'] for r in rows))
    for ax, key, title in ((axs[0], 'success', 'success rate (%)'), (axs[1], 'collisions', 'robot-robot collision pairs / run')):
        for ci, ctrl in enumerate(args.ctrls):
            ys = [next(r[key] for r in rows if r['scenario'] == k and r['ctrl'] == ctrl) for k in kinds]
            ax.bar(np.arange(len(kinds)) + ci * 0.2 - 0.3, ys, 0.2, label=ctrl)
        ax.set_xticks(range(len(kinds)))
        ax.set_xticklabels(kinds)
        ax.set_title(title)
    axs[0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, f'bench{args.tag}.png'), dpi=130)
    print('wrote', OUT)


def robust(args):
    """Robustness bench: vo / mppi_prior / pref under clean, 0.1 m obs noise,
    3-tick delay, on corridor + crowd. Writes robustness.md + robustness.png."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    os.makedirs(OUT, exist_ok=True)
    ctrls = ('pref', 'vo', 'mppi_prior')
    conds = (('clean', {}), ('noise0.1', dict(obs_noise=0.1)), ('delay3', dict(delay_steps=3)))
    rows = []
    for kind in ('corridor', 'crowd'):
        for ctrl in ctrls:
            for cname, kw in conds:
                rs = [simulate(kind, ctrl, args.n, sd, args.max_t, **kw) for sd in range(args.seeds)]
                ok = [r['makespan'] for r in rs if r['success']]
                row = dict(scenario=kind, ctrl=ctrl, cond=cname,
                           success=100 * np.mean([r['success'] for r in rs]),
                           makespan=float(np.mean(ok)) if ok else float('nan'),
                           coll=np.mean([r['collisions'] for r in rs]),
                           ped=np.mean([r['ped_contacts'] for r in rs]),
                           intr=np.mean([r['ped_intrusion_s'] for r in rs]),
                           minped=float(np.nanmin([r['min_ped_dist'] for r in rs])) if kind == 'crowd' else float('nan'))
                rows.append(row)
                print(row, flush=True)
    md = ['| scenario | ctrl | condition | success % | makespan s | robot coll pairs | ped contacts | ped intrusion s | min ped dist m |',
          '|---|---|---|---|---|---|---|---|---|']
    md += [f"| {r['scenario']} | {r['ctrl']} | {r['cond']} | {r['success']:.0f} | {r['makespan']:.1f} | {r['coll']:.2f} | "
           f"{r['ped']:.2f} | {r['intr']:.1f} | {r['minped']:.2f} |" for r in rows]
    with open(os.path.join(OUT, 'robustness_table.md'), 'w') as fh:
        fh.write(f'n={args.n}, {args.seeds} seeds, max_t={args.max_t}s\n\n' + '\n'.join(md) + '\n')
    fig, axs = plt.subplots(1, 3, figsize=(12, 3.6))
    labels = [f"{k}\n{c}" for k in ('corridor', 'crowd') for c, _ in conds]
    for ax, key, title in ((axs[0], 'success', 'success (%)'), (axs[1], 'coll', 'robot-robot collision pairs / run'),
                           (axs[2], 'intr', 'ped intrusion (robot-s, crowd only)')):
        for ci, ctrl in enumerate(ctrls):
            ys = [next(r[key] for r in rows if r['scenario'] == k and r['ctrl'] == ctrl and r['cond'] == c)
                  for k in ('corridor', 'crowd') for c, _ in conds]
            ax.bar(np.arange(len(ys)) + ci * 0.27 - 0.27, ys, 0.27, label=ctrl)
        ax.set_xticks(range(len(labels)))
        ax.set_xticklabels(labels, fontsize=6)
        ax.set_title(title, fontsize=9)
    axs[0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'robustness.png'), dpi=130)
    print('wrote', OUT)


CTRL_NAMES = {
    'pref': 'Goal-seek, no avoidance (baseline)',
    'vo': 'Sampled velocity obstacle (ORCA-like)',
    'mppi': 'MPPI, uninformed sampling',
    'mppi_prior': 'MPPI + velocity-obstacle prior + yield rule',
}



def _heading_track(tr, dt):
    """Per-frame heading (rad) from smoothed motion direction; held while (almost) stationary so the body does not spin."""
    k_, n_ = tr.shape[0], tr.shape[1]
    yaw = np.zeros((k_, n_))
    last = np.zeros(n_)
    for i in range(n_):
        for k in range(k_):
            a, b = max(0, k - 3), min(k_ - 1, k + 3)
            v = (tr[b, i] - tr[a, i]) / max(dt * (b - a), 1e-9)
            if np.hypot(*v) > 0.06:
                last[i] = math.atan2(v[1], v[0])
            yaw[k, i] = last[i]
    return yaw


def _draw_diffdrive(ax, pos, yaw, color, zorder=6):
    """Top-down differential-drive base: rounded chassis, two drive wheels, lidar puck, white 'front' wedge."""
    from matplotlib.patches import FancyBboxPatch, Circle, Polygon, Rectangle
    from matplotlib.transforms import Affine2D
    L, W = 2 * R * 1.05, 2 * R * 0.82                                    # chassis length / width (m)
    tf = Affine2D().rotate(yaw).translate(*pos) + ax.transData
    ax.add_patch(Rectangle((-L * 0.38, W / 2 - 0.01), L * 0.76, 0.06, fc='#1f2937', ec='none', zorder=zorder, transform=tf))   # wheels
    ax.add_patch(Rectangle((-L * 0.38, -W / 2 - 0.05), L * 0.76, 0.06, fc='#1f2937', ec='none', zorder=zorder, transform=tf))
    ax.add_patch(FancyBboxPatch((-L / 2, -W / 2), L, W, boxstyle='round,pad=0,rounding_size=0.09', fc=color, ec='white',
                                lw=1.1, zorder=zorder + 0.1, transform=tf))
    ax.add_patch(Polygon([[L * 0.50, 0], [L * 0.22, W * 0.30], [L * 0.22, -W * 0.30]], closed=True, fc='white', ec='none',
                         alpha=0.95, zorder=zorder + 0.2, transform=tf))                                                  # front
    ax.add_patch(Circle((-L * 0.12, 0), 0.075, fc='#111827', ec='white', lw=0.6, zorder=zorder + 0.3, transform=tf))      # lidar


def make_gif(args):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import animation
    from matplotlib.collections import LineCollection
    os.makedirs(OUT, exist_ok=True)
    r = simulate(args.scenario, args.ctrl, args.n, args.seed, args.max_t, record=True)
    tr, world, goals = r['trail'], r['world'], r['goals']
    ptr = r.get('ped_trail')
    n = tr.shape[1]
    cols = plt.cm.tab10(np.arange(n) % 10) if n <= 10 else plt.cm.tab20(np.arange(n) % 20)
    bg, wall = '#f6f7f9', '#2b3340'
    dist = np.linalg.norm(tr[:, :, None] - tr[:, None], axis=3) + np.eye(n)[None] * 99
    col_now = (dist < 2 * R - 0.02).sum((1, 2)) // 2                      # overlapping pairs per frame
    col_total = np.maximum.accumulate(col_now)
    arrived = np.linalg.norm(tr - goals[None], axis=2) < 0.35
    span = max(world.w, world.h)
    fig, ax = plt.subplots(figsize=(6.0 * world.w / span + 0.4, 6.0 * world.h / span + 0.9), dpi=100)
    fig.patch.set_facecolor(bg)
    frames = list(range(0, len(tr), 3)) + [len(tr) - 1] * 8              # hold the last frame
    tail = 45                                                           # ~4.5 s fading trail
    yaws = _heading_track(tr, DT)

    def draw(k):
        k = min(k, len(tr) - 1)
        ax.clear()
        ax.set_facecolor(bg)
        ax.imshow(~world.free, cmap=matplotlib.colors.ListedColormap([bg, wall]), origin='lower',
                  extent=(-0.5, world.w - 0.5, -0.5, world.h - 0.5), interpolation='nearest')
        for i in range(n):
            c = cols[i]
            gx, gy = goals[i]
            ax.plot([gx], [gy], marker='*', ms=11, mfc=c, mec='white', mew=0.8, zorder=3, alpha=0.35 if arrived[k, i] else 1)
            seg = tr[max(0, k - tail):k + 1, i]
            if len(seg) > 1:
                lc = LineCollection(np.stack([seg[:-1], seg[1:]], 1), colors=[(*c[:3], a) for a in np.linspace(0.05, 0.9, len(seg) - 1)],
                                    linewidths=2.0, capstyle='round', zorder=4)
                ax.add_collection(lc)
            _draw_diffdrive(ax, tr[k, i], yaws[k, i], c)
            if arrived[k, i]:
                ax.text(*tr[k, i], '✓', ha='center', va='center', fontsize=7, color='white', weight='bold', zorder=8)
        if ptr is not None:                                             # pedestrians + personal-space ring
            for q in ptr[min(k, len(ptr) - 1)]:
                ax.add_patch(plt.Circle(q, 1.2, fc='#f59e0b', alpha=0.12, ec='#f59e0b', lw=0.8, ls='--', zorder=2))
                ax.add_patch(plt.Circle(q, PED_R, fc='#d97706', ec='white', lw=1.0, zorder=5))
        for i, j in np.argwhere(np.triu(dist[k] < 2 * R - 0.02)):       # collision flash
            ax.add_patch(plt.Circle((tr[k, i] + tr[k, j]) / 2, 0.55, fc='none', ec='#dc2626', lw=2.0, zorder=9))
        ax.set_xlim(-0.5, world.w - 0.5)
        ax.set_ylim(-0.5, world.h - 0.5)
        ax.set_aspect('equal')
        ax.axis('off')
        done = 'ALL ARRIVED' if arrived[k].all() else f'{int(arrived[k].sum())}/{n} arrived'
        ax.set_title(f"{args.scenario}  ·  {CTRL_NAMES.get(args.ctrl, args.ctrl)}", fontsize=10.5, weight='bold', color=wall, loc='left', pad=22)
        ax.text(0, 1.015, f"t = {k * DT:4.1f} s   ·   {done}   ·   collisions: {int(col_total[k])}", transform=ax.transAxes,
                fontsize=8.5, color='#dc2626' if col_total[k] else '#475569', va='bottom')
        ax.add_patch(plt.Rectangle((0, -0.035), k / max(len(tr) - 1, 1), 0.012, transform=ax.transAxes, fc='#3b82f6', ec='none', clip_on=False))

    ani = animation.FuncAnimation(fig, draw, frames=frames, interval=60)
    path = os.path.join(OUT, f'{args.scenario}_{args.ctrl}.gif')
    ani.save(path, writer='pillow', fps=16)
    plt.close(fig)
    print('wrote', path, {k: v for k, v in r.items() if k in ('success', 'makespan', 'collisions')})


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    b = sub.add_parser('bench')
    b.add_argument('--n', type=int, default=8)
    b.add_argument('--seeds', type=int, default=10)
    b.add_argument('--max-t', type=float, default=60.0)
    b.add_argument('--scenarios', nargs='+', default=list(BASE_SCENARIOS), choices=SCENARIOS)
    b.add_argument('--obs-noise', type=float, default=0.0, help='sigma (m, m/s) on neighbours observed pos/vel')
    b.add_argument('--delay-steps', type=int, default=0, help='neighbour observations stale by k ticks (0.1 s each)')
    b.add_argument('--act-noise', type=float, default=0.0, help='sigma (m/s) on commanded velocity')
    b.add_argument('--tag', default='', help='suffix for bench output filenames')
    r_ = sub.add_parser('robust')
    r_.add_argument('--n', type=int, default=8)
    r_.add_argument('--seeds', type=int, default=6)
    r_.add_argument('--max-t', type=float, default=60.0)
    b.add_argument('--ctrls', nargs='+', default=list(CTRLS))
    g = sub.add_parser('gif')
    g.add_argument('--scenario', default='swap', choices=SCENARIOS)
    g.add_argument('--ctrl', default='mppi_prior', choices=CTRLS)
    g.add_argument('--n', type=int, default=8)
    g.add_argument('--seed', type=int, default=0)
    g.add_argument('--max-t', type=float, default=60.0)
    a = ap.parse_args()
    {'bench': bench, 'gif': make_gif, 'robust': robust}[a.cmd](a)


if __name__ == '__main__':
    sys.exit(main())
