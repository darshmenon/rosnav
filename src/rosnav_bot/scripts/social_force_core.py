#!/usr/bin/env python3
"""
social_force_core.py — pure-Python pedestrian crowd for a mapped indoor world.

No ROS / Gazebo. Used by social_force_peds.py (drives Gazebo models) and tested
offline (test/test_social_force_core.py).

What makes the walking human-like rather than a back-and-forth patrol:
  * routes: every person picks a random free goal and follows an A* route on the
    map's inflated occupancy grid (so they go through doors and corridors);
  * social forces: goal drive, person-person repulsion with an anisotropic
    "field of view" (people react more to what is in front of them), wall
    repulsion from the map's distance field, optional robot repulsion;
  * individuality: preferred speed 0.8-1.4 m/s, reaction time, "distracted"
    walkers who react to the robot at only a fraction of normal strength;
  * behaviour: pause at goals (0-4 s), occasional hesitation (slow-down), and
    pairs/groups that walk together (group cohesion toward a shared leader).
"""
import heapq
import math

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage


class MapWorld:
    def __init__(self, yaml_path):
        meta = yaml.safe_load(open(yaml_path))
        img_path = yaml_path.rsplit('/', 1)[0] + '/' + meta['image']
        img = np.array(Image.open(img_path).convert('L'), np.float32)
        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        occ_p = (255 - img) / 255.0 if not meta.get('negate', 0) else img / 255.0
        occ = occ_p > float(meta.get('occupied_thresh', 0.65))
        unknown = (occ_p >= float(meta.get('free_thresh', 0.25))) & ~occ
        self.blocked = np.flipud(occ | unknown)           # row 0 = y min
        self.h, self.w = self.blocked.shape
        self.edt = ndimage.distance_transform_edt(~self.blocked) * self.res
        gy, gx = np.gradient(self.edt)
        self.gx, self.gy = gx / self.res, gy / self.res    # d(edt)/dx, d(edt)/dy (m/m)
        self._coarse = None

    # -- coordinates
    def to_cell(self, x, y):
        return int((y - self.oy) / self.res), int((x - self.ox) / self.res)

    def to_world(self, r, c):
        return self.ox + (c + 0.5) * self.res, self.oy + (r + 0.5) * self.res

    def clearance(self, x, y):
        r, c = self.to_cell(x, y)
        if not (0 <= r < self.h and 0 <= c < self.w):
            return 0.0
        return float(self.edt[r, c])

    def remove_small_obstacles(self, max_extent_m=0.9):
        """Drop compact blobs (bounding box <= max_extent_m in both directions): lidar speckle and
        stray little boxes in a saved map. Walls are long in one direction, so they stay.
        Returns the number of blobs removed; the distance field is recomputed."""
        lab, _ = ndimage.label(self.blocked, structure=np.ones((3, 3)))
        removed = 0
        for i, sl in enumerate(ndimage.find_objects(lab), 1):
            h, w = (sl[0].stop - sl[0].start) * self.res, (sl[1].stop - sl[1].start) * self.res
            if max(h, w) <= max_extent_m:
                self.blocked[sl][lab[sl] == i] = False
                removed += 1
        self.edt = ndimage.distance_transform_edt(~self.blocked) * self.res
        gy, gx = np.gradient(self.edt)
        self.gx, self.gy = gx / self.res, gy / self.res
        self._coarse = None
        return removed

    def clearance_many(self, P):
        """Vectorised clearance for an (...,2) array of world points (0 outside the map)."""
        P = np.asarray(P, float)
        r = ((P[..., 1] - self.oy) / self.res).astype(int)
        c = ((P[..., 0] - self.ox) / self.res).astype(int)
        ok = (r >= 0) & (r < self.h) & (c >= 0) & (c < self.w)
        out = np.zeros(P.shape[:-1])
        out[ok] = self.edt[r[ok], c[ok]]
        return out

    def wall_dir(self, x, y):
        r, c = self.to_cell(x, y)
        if not (0 <= r < self.h and 0 <= c < self.w):
            return 0.0, 0.0
        return float(self.gx[r, c]), float(self.gy[r, c])

    # -- sampling & routing
    def sample_free(self, rng, min_clear=0.7, reachable_from=None, tries=500):
        for _ in range(tries):
            r, c = int(rng.integers(0, self.h)), int(rng.integers(0, self.w))
            if self.edt[r, c] >= min_clear:
                p = self.to_world(r, c)
                if reachable_from is None or self.route(reachable_from, p):
                    return p
        raise RuntimeError('no free cell found (map too small / too tight?)')

    def route(self, a, b, clear=0.45, k=4):
        """A* on a k-times coarser grid; returns world waypoints [(x,y)...] or []."""
        if self._coarse is None or self._coarse[0] != (clear, k):
            hh, ww = self.h // k, self.w // k
            ok = np.zeros((hh, ww), bool)
            for i in range(hh):
                for j in range(ww):
                    ok[i, j] = self.edt[i * k:(i + 1) * k, j * k:(j + 1) * k].min() >= clear
            self._coarse = ((clear, k), ok)
        ok = self._coarse[1]
        hh, ww = ok.shape

        def cell(p):
            r, c = self.to_cell(*p)
            return r // k, c // k
        s, g = cell(a), cell(b)
        if not (0 <= s[0] < hh and 0 <= s[1] < ww and 0 <= g[0] < hh and 0 <= g[1] < ww):
            return []
        if not ok[g]:
            return []
        if not ok[s]:                       # start inside the inflated zone: nearest ok cell
            idx = np.argwhere(ok)
            s = tuple(idx[np.argmin(((idx - np.array(s)) ** 2).sum(1))])
        openq, came, cost = [(0.0, s)], {s: None}, {s: 0.0}
        while openq:
            _, cur = heapq.heappop(openq)
            if cur == g:
                break
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == dc == 0:
                        continue
                    n = (cur[0] + dr, cur[1] + dc)
                    if not (0 <= n[0] < hh and 0 <= n[1] < ww) or not ok[n]:
                        continue
                    nc = cost[cur] + math.hypot(dr, dc)
                    if nc < cost.get(n, 1e9):
                        cost[n], came[n] = nc, cur
                        heapq.heappush(openq, (nc + math.hypot(g[0] - n[0], g[1] - n[1]), n))
        if g not in came:
            return []
        path, cur = [], g
        while cur is not None:
            path.append(cur)
            cur = came[cur]
        path.reverse()
        pts = [self.to_world(r * k + k // 2, c * k + k // 2) for r, c in path]
        pts = pts[::2] + [pts[-1]]
        pts.append(tuple(b))
        return pts


class Crowd:
    def __init__(self, world: MapWorld, n, seed=0, robot_weight=0.6, group_frac=0.3,
                 spawn_avoid=None):
        self.w = world
        self.rng = np.random.default_rng(seed)
        self.n = n
        self.robot_weight = robot_weight
        self.pos = np.zeros((n, 2))
        for i in range(n):
            for _ in range(200):
                p = np.array(world.sample_free(self.rng, 0.8))
                if spawn_avoid is None or np.linalg.norm(p - np.asarray(spawn_avoid)) > 2.5:
                    break
            self.pos[i] = p
        self.vel = np.zeros((n, 2))
        self.vdes = self.rng.uniform(0.8, 1.4, n)
        self.tau = self.rng.uniform(0.4, 0.8, n)
        self.distracted = self.rng.random(n) < 0.3          # react weakly to the robot
        self.path = [[] for _ in range(n)]
        self.pause = np.zeros(n)
        self.hesitate = np.zeros(n)
        self.leader = np.full(n, -1)
        for i in range(1, n):                               # pairs walking together
            if self.leader[i] < 0 and self.rng.random() < group_frac and self.leader[i - 1] < 0:
                self.leader[i] = i - 1
        for i in range(n):
            if self.leader[i] < 0:
                self._new_goal(i)
        for i in range(n):
            if self.leader[i] >= 0:                         # follower starts near its leader
                self.pos[i] = self.pos[self.leader[i]] + self.rng.normal(0, 0.4, 2)
                self.vdes[i] = self.vdes[self.leader[i]]

    def _new_goal(self, i):
        for _ in range(10):
            g = self.w.sample_free(self.rng, 0.8)
            r = self.w.route(tuple(self.pos[i]), g)
            if r:
                self.path[i] = r
                return
        self.path[i] = [tuple(self.pos[i])]

    def _target(self, i):
        j = self.leader[i]
        if j >= 0:
            return self.pos[j] - 0.5 * self._dir(j)         # trail half a metre behind
        while len(self.path[i]) > 1 and np.linalg.norm(np.array(self.path[i][0]) - self.pos[i]) < 0.5:
            self.path[i].pop(0)
        return np.array(self.path[i][0]) if self.path[i] else self.pos[i]

    def _dir(self, i):
        v = self.vel[i]
        n = np.linalg.norm(v)
        return v / n if n > 0.05 else np.array([1.0, 0.0])

    def step(self, dt, robot=None, robot_vel=None):
        n = self.n
        F = np.zeros((n, 2))
        for i in range(n):
            if self.pause[i] > 0:
                self.pause[i] -= dt
                F[i] = -self.vel[i] / 0.3
                continue
            tgt = self._target(i)
            g = tgt - self.pos[i]
            d = np.linalg.norm(g)
            if self.leader[i] < 0 and len(self.path[i]) <= 1 and d < 0.6:
                self.pause[i] = self.rng.uniform(0.0, 4.0)   # arrive, linger, pick new goal
                self._new_goal(i)
                continue
            if self.rng.random() < 0.002:
                self.hesitate[i] = self.rng.uniform(0.5, 1.5)
            sp = self.vdes[i] * (0.3 if self.hesitate[i] > 0 else 1.0)
            self.hesitate[i] = max(0.0, self.hesitate[i] - dt)
            F[i] = (g / max(d, 1e-6) * sp - self.vel[i]) / self.tau[i]
        # person-person repulsion with anisotropic field of view
        dxy = self.pos[:, None] - self.pos[None]
        dist = np.linalg.norm(dxy, axis=2) + np.eye(n) * 1e3
        for i in range(n):
            e = self._dir(i)
            cosang = -(dxy[i] @ e) / dist[i]                 # >0 when the other is in front
            fov = 0.4 + 0.6 * (cosang > -0.3)                # people ignore what's behind
            mag = 2.2 * np.exp((0.55 - dist[i]) / 0.35) * fov
            F[i] += (mag[:, None] * dxy[i] / dist[i][:, None]).sum(0)
        # walls
        for i in range(n):
            c = self.w.clearance(*self.pos[i])
            gx, gy = self.w.wall_dir(*self.pos[i])
            F[i] += 4.0 * math.exp((0.35 - c) / 0.2) * np.array([gx, gy])
        # robot
        if robot is not None:
            rob = np.asarray(robot, float)
            for i in range(n):
                d = self.pos[i] - rob
                dn = np.linalg.norm(d) + 1e-6
                wgt = self.robot_weight * (0.25 if self.distracted[i] else 1.0)
                F[i] += wgt * 3.0 * math.exp((0.7 - dn) / 0.45) * d / dn
        self.vel += F * dt
        sp = np.linalg.norm(self.vel, axis=1, keepdims=True)
        self.vel *= np.minimum(1.0, 1.7 / np.maximum(sp, 1e-6))
        new = self.pos + self.vel * dt
        for i in range(n):                                    # never step into a wall
            if self.w.clearance(*new[i]) < 0.18:
                self.vel[i] *= 0.2
                new[i] = self.pos[i]
        self.pos = new
        return self.vel.copy()
