"""MuJoCo crowd-navigation env for a DIFFERENTIAL-DRIVE robot (kinematic, no Gazebo).

Split of responsibilities (read this before trusting a number from it):
  * MuJoCo (``mujoco`` python package): robot body (box + 2 wheel cylinders + a cylinder collision
    geom, slide-x / slide-y / hinge-yaw + 2 wheel hinges), 12 pedestrian capsules on mocap bodies,
    robot<->pedestrian contact detection (mj_forward contacts) and the pedestrian part of the
    36-beam lidar (mj_ray).  The robot is moved KINEMATICALLY: wheel speeds -> unicycle (v, w) ->
    qpos each step; we do not simulate wheel/ground dynamics.
  * numpy / MapWorld (scripts/social_force_core.py): walls.  The hospital map has tens of
    thousands of occupied cells, so turning them into geoms is pointless; wall collision
    (distance-field clearance < robot radius) and the wall part of the lidar (grid ray march) use
    the occupancy grid.  Lidar = min(wall range, mj_ray pedestrian range).
  * Pedestrians: social_force_core.Crowd (social forces, A* routes), robot repels them via
    Crowd.step(dt, robot_xy).  Inactive pedestrians (n is 4..12 per reset) are parked far away.

Action (physical units): (v, w), v in [0, 0.8] m/s, w in [-1.8, 1.8] rad/s -> wheel speeds
vl = v - w*track/2, vr = v + w*track/2 (track 0.36) -> back to (v, w) -> pose.
Observation (45): 36 lidar ranges /5 m | goal dist/10 (clipped 3), sin, cos of bearing |
last v/0.8, w/1.8 | nearest ped rel. position /5 and rel. velocity /2 in the robot frame.
Episode ends on goal, collision (pedestrian OR wall) or timeout (400 steps at 10 Hz).
"""
from __future__ import annotations

import math
import os
from typing import Optional

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError as exc:  # pragma: no cover
    raise ImportError('pip install gymnasium') from exc
import mujoco

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.abspath(os.path.join(_HERE, '..', '..', 'scripts'))
if _SCRIPTS not in __import__('sys').path:
    __import__('sys').path.insert(0, _SCRIPTS)
import social_force_core as sf  # noqa: E402

DEFAULT_MAP = os.path.abspath(os.path.join(_HERE, '..', '..', 'maps', 'map_hospital.yaml'))
MAX_PEDS = 12
ROBOT_R, PED_R, PERSONAL_SPACE = 0.30, 0.22, 1.2
TRACK, WHEEL_R = 0.36, 0.07
LIDAR_N, LIDAR_R = 36, 5.0
V_MAX, W_MAX = 0.8, 1.8
DT = 0.1
_WORLDS: dict = {}


def _xml() -> str:
    peds = ''.join(
        f'<body name="ped{i}" mocap="true" pos="{1000 + 3 * i} 1000 0.3">'
        f'<geom name="pedg{i}" type="capsule" fromto="0 0 -0.3 0 0 0.3" size="{PED_R}" group="1" '
        f'contype="0" conaffinity="1" rgba="0.9 0.5 0.2 1"/></body>' for i in range(MAX_PEDS))
    return f"""<mujoco model="social_nav">
  <option timestep="{DT}" gravity="0 0 0"/>
  <worldbody>
    <body name="robot" pos="0 0 0.15">
      <joint name="x" type="slide" axis="1 0 0"/><joint name="y" type="slide" axis="0 1 0"/>
      <joint name="yaw" type="hinge" axis="0 0 1"/>
      <geom name="robot_col" type="cylinder" size="{ROBOT_R} 0.15" group="2" contype="1" conaffinity="0"/>
      <geom type="box" size="0.25 0.15 0.08" contype="0" conaffinity="0" rgba="0.2 0.7 0.3 1"/>
      <body name="wl" pos="0 {TRACK / 2} -0.08"><joint name="wl" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" euler="90 0 0" size="{WHEEL_R} 0.025" contype="0" conaffinity="0"/></body>
      <body name="wr" pos="0 {-TRACK / 2} -0.08"><joint name="wr" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" euler="90 0 0" size="{WHEEL_R} 0.025" contype="0" conaffinity="0"/></body>
    </body>
    {peds}
  </worldbody>
</mujoco>"""


class SocialNavMujocoEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, map_yaml: str = DEFAULT_MAP, max_steps: int = 400, goal_tol: float = 0.4,
                 goal_range=(2.0, 8.0), n_peds_range=(4, 12), keep_small_obstacles: bool = False,
                 seed: Optional[int] = None):
        super().__init__()
        key = (map_yaml, keep_small_obstacles)
        if key not in _WORLDS:
            w = sf.MapWorld(map_yaml)
            if not keep_small_obstacles:
                w.remove_small_obstacles(0.9)       # lidar speckle, same as render_social_demo
            _WORLDS[key] = w
        self.world = _WORLDS[key]
        self.max_steps, self.goal_tol = max_steps, goal_tol
        self.goal_range, self.n_peds_range = goal_range, n_peds_range
        self.n_beams, self.max_range, self.v_max, self.w_max = LIDAR_N, LIDAR_R, V_MAX, W_MAX
        self.model = mujoco.MjModel.from_xml_string(_xml())
        self.data = mujoco.MjData(self.model)
        m = self.model
        self._rid = m.body('robot').id
        self._ped_geoms = [m.geom(f'pedg{i}').id for i in range(MAX_PEDS)]
        self._robot_geom = m.geom('robot_col').id
        self._ped_geom_set = set(self._ped_geoms)
        self._geomgroup = np.array([0, 1, 0, 0, 0, 0], dtype=np.uint8)   # rays see pedestrians only
        self._ray_ang = np.linspace(-np.pi, np.pi, LIDAR_N, endpoint=False)
        self._rr = np.linspace(0.2, LIDAR_R, 50)
        self.observation_space = spaces.Box(-np.inf, np.inf, (LIDAR_N + 9,), np.float32)
        self.action_space = spaces.Box(np.array([0.0, -W_MAX], np.float32),
                                       np.array([V_MAX, W_MAX], np.float32))
        self.crowd = None
        self.n_peds = 0
        if seed is not None:
            self.reset(seed=seed)

    # -- helpers -----------------------------------------------------------------------------
    def _sync_mujoco(self):
        d = self.data
        d.qpos[0:2], d.qpos[2] = self.robot_xy, self.theta
        d.qpos[3], d.qpos[4] = self._wl, self._wr
        for i in range(MAX_PEDS):
            if i < self.n_peds:
                d.mocap_pos[i] = (*self.crowd.pos[i], 0.3)
            else:
                d.mocap_pos[i] = (1000 + 3 * i, 1000, 0.3)
        mujoco.mj_forward(self.model, d)

    def _lidar(self):
        w, th, p = self.world, self.theta, self.robot_xy
        ang = th + self._ray_ang
        dirs = np.stack([np.cos(ang), np.sin(ang)], -1)
        P = p[None, None] + self._rr[None, :, None] * dirs[:, None]
        r = ((P[..., 1] - w.oy) / w.res).astype(int)
        c = ((P[..., 0] - w.ox) / w.res).astype(int)
        inside = (r >= 0) & (r < w.h) & (c >= 0) & (c < w.w)
        hit = w.blocked[r.clip(0, w.h - 1), c.clip(0, w.w - 1)] | ~inside
        first = np.where(hit.any(1), hit.argmax(1), len(self._rr) - 1)
        ranges = self._rr[first].copy()
        ranges[~hit.any(1)] = LIDAR_R
        if self.n_peds:
            geomid = np.zeros(1, np.int32)
            pnt = np.array([p[0], p[1], 0.15])
            for k in range(LIDAR_N):
                vec = np.array([dirs[k, 0], dirs[k, 1], 0.0])
                dist = mujoco.mj_ray(self.model, self.data, pnt, vec, self._geomgroup, 1,
                                     self._rid, geomid)
                if 0 <= dist < ranges[k]:
                    ranges[k] = dist
        return np.minimum(ranges, LIDAR_R)

    def _ped_dists(self):
        return np.linalg.norm(self.crowd.pos[:self.n_peds] - self.robot_xy, axis=1)

    def _robot_frame(self, v):
        c, s = math.cos(self.theta), math.sin(self.theta)
        return np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1]])

    def _obs(self):
        to_goal = self.goal - self.robot_xy
        dist = float(np.linalg.norm(to_goal))
        bearing = math.atan2(to_goal[1], to_goal[0]) - self.theta
        d = self._ped_dists()
        j = int(np.argmin(d))
        rel = self._robot_frame(self.crowd.pos[j] - self.robot_xy)
        relv = self._robot_frame(self.crowd.vel[j])
        scalar = [min(dist / 10.0, 3.0), math.sin(bearing), math.cos(bearing),
                  self.last_v / V_MAX, self.last_w / W_MAX]
        o = np.concatenate([self._lidar() / LIDAR_R, scalar,
                            np.clip(rel / 5.0, -1, 1), np.clip(relv / 2.0, -1, 1)])
        return o.astype(np.float32)

    def _collided(self):
        """MuJoCo contacts robot<->pedestrian, or wall (distance field) below robot radius."""
        for k in range(self.data.ncon):
            c = self.data.contact[k]
            if (c.geom1 == self._robot_geom and c.geom2 in self._ped_geom_set) or \
               (c.geom2 == self._robot_geom and c.geom1 in self._ped_geom_set):
                return True
        return self.world.clearance(*self.robot_xy) < ROBOT_R

    # -- gym API ------------------------------------------------------------------------------
    def reset(self, *, seed: Optional[int] = None, options=None):
        super().reset(seed=seed)
        rng = self.np_random
        w = self.world
        self.n_peds = int(rng.integers(self.n_peds_range[0], self.n_peds_range[1] + 1))
        for _ in range(100):
            start = np.array(w.sample_free(rng, 0.7))
            goal = None
            for _ in range(60):
                g = np.array(w.sample_free(rng, 0.7))
                if self.goal_range[0] <= np.linalg.norm(g - start) <= self.goal_range[1] \
                        and w.route(tuple(start), tuple(g)):
                    goal = g
                    break
            if goal is not None:
                break
        else:
            raise RuntimeError('could not sample a reachable start/goal pair')
        self.robot_xy, self.goal = start, goal
        self.theta = float(rng.uniform(-math.pi, math.pi))
        self.route = w.route(tuple(start), tuple(goal))
        self.crowd = sf.Crowd(w, self.n_peds, seed=int(rng.integers(0, 2 ** 31 - 1)),
                              spawn_avoid=start)
        self._wl = self._wr = 0.0
        self.last_v = self.last_w = 0.0
        self.t = 0
        self.intrusion_s, self.collided = 0.0, False
        self.prev_dist = float(np.linalg.norm(goal - start))
        self._sync_mujoco()
        self.min_clear = float(self._ped_dists().min() - ROBOT_R - PED_R)
        return self._obs(), self._info(False)

    def _info(self, success):
        return {'collided': self.collided, 'success': success,
                'min_clearance': float(self.min_clear), 'intrusion_s': float(self.intrusion_s)}

    def step(self, action):
        a = np.asarray(action, np.float64).reshape(2)
        v = float(np.clip(a[0], 0.0, V_MAX))
        om = float(np.clip(a[1], -W_MAX, W_MAX))
        vl, vr = v - om * TRACK / 2, v + om * TRACK / 2           # wheel linear speeds
        v, om = (vl + vr) / 2, (vr - vl) / TRACK                  # unicycle constraint
        self._wl += vl / WHEEL_R * DT
        self._wr += vr / WHEEL_R * DT
        self.theta = (self.theta + om * DT + math.pi) % (2 * math.pi) - math.pi
        self.robot_xy = self.robot_xy + v * DT * np.array([math.cos(self.theta), math.sin(self.theta)])
        self.crowd.step(DT, self.robot_xy)
        self._sync_mujoco()
        self.t += 1

        d = self._ped_dists()
        dmin = float(d.min())
        self.min_clear = min(self.min_clear, dmin - ROBOT_R - PED_R)
        intr = dmin < PERSONAL_SPACE
        self.intrusion_s += DT * intr
        collided = self._collided()
        self.collided = self.collided or collided
        dist = float(np.linalg.norm(self.goal - self.robot_xy))
        reached = dist < self.goal_tol

        r = 2.0 * (self.prev_dist - dist)                                  # goal progress
        r -= 0.01                                                          # time
        r -= 0.02 * (abs(v - self.last_v) / V_MAX + abs(om - self.last_w) / W_MAX)   # smoothness
        if intr:
            r -= 0.1 * (PERSONAL_SPACE - dmin) / (PERSONAL_SPACE - ROBOT_R - PED_R)  # intrusion
        if reached:
            r += 20.0
        if collided:
            r -= 20.0
        self.prev_dist, self.last_v, self.last_w = dist, v, om
        terminated = bool(reached or collided)
        truncated = bool(not terminated and self.t >= self.max_steps)
        return self._obs(), float(r), terminated, truncated, self._info(bool(reached and not collided))

    def close(self):
        pass
