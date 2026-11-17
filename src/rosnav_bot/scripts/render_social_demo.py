#!/usr/bin/env python3
"""
render_social_demo.py — animated video of the social-navigation stack (no Gazebo/ROS needed).

Hospital map + social-force crowd (social_force_core) + a DIFFERENTIAL-DRIVE robot (unicycle:
it must turn toward the planner's chosen direction before it drives, so heading and wheel speeds
are real) driven by the risk-aware planner using the LEARNED pedestrian predictor. Draws: people (coloured), the learned
predicted path (orange) with conformal uncertainty discs, the constant-velocity prediction
(blue, dotted) for comparison, the robot (green), its trail, the 1.2 m personal-space ring and live
metrics (clearance, intrusion seconds, collisions).

  python3 render_social_demo.py --out social_demo.mp4 --seconds 60 --peds 10
"""
import argparse, json, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, PillowWriter
from matplotlib.patches import Circle, Polygon

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import social_force_core as sf
import social_nav_core as s
import train_ped_predictor as tp
import eval_learned_planner as ev

ap = argparse.ArgumentParser()
ap.add_argument('--map', default=os.path.join(HERE, '..', 'maps', 'map_hospital.yaml'))
ap.add_argument('--weights', default=os.path.join(HERE, '..', '..', '..', 'outputs', 'ped_predictor_n24.pt'))
ap.add_argument('--out', default='social_demo.mp4')
ap.add_argument('--seconds', type=float, default=60)
ap.add_argument('--peds', type=int, default=10)
ap.add_argument('--seed', type=int, default=3)
ap.add_argument('--clean', action='store_true', help='minimal look: no lidar rays / CV lines, one-line title, smaller GIF')
ap.add_argument('--keep-small-obstacles', action='store_true', help='keep lidar speckle / tiny boxes in the map')
ap.add_argument('--start', type=float, nargs=2, default=[-9.0, 0.0])
ap.add_argument('--goals', type=float, nargs='+', default=[8.0, 0.4, -8.0, -0.4])
a = ap.parse_args()

world = sf.MapWorld(a.map)
if not a.keep_small_obstacles:
    print('removed', world.remove_small_obstacles(0.9), 'small obstacle blobs from the map')
crowd = sf.Crowd(world, a.peds, seed=a.seed, spawn_avoid=a.start)
pred = ev.LearnedPredictor(a.weights)
cj = a.weights + '.conformal.json'
conf = json.load(open(cj))['radii'] if os.path.exists(cj) else None
planner = s.RiskPlanner(conformal_radii=conf, sigma_scale=0.5, clear_fn=world.clearance_many)
conf_r = np.array(conf) if conf else np.full(12, 0.5)

DT = 0.1
TRACK, LIDAR_N, LIDAR_R = 0.36, 120, 5.0
robot = np.array(a.start, float)
theta = 0.0
lin = ang = 0.0
goals = [np.array(a.goals[i:i + 2]) for i in range(0, len(a.goals), 2)]
gi = 0
route = world.route(tuple(robot), tuple(goals[0]))
hist = [crowd.pos.copy() for _ in range(32)]
trail, collisions, intrusion, in_contact, min_clear = [], 0, 0.0, False, 1e9

fig, ax = plt.subplots(figsize=(9, 5.0) if a.clean else (11, 6.2), dpi=80 if a.clean else 90)
img = np.where(world.blocked, 0.25, 0.97)
ext = [world.ox, world.ox + world.w * world.res, world.oy, world.oy + world.h * world.res]
cols = plt.cm.tab10(np.arange(a.peds) % 10)

def lidar(robot, theta, peds):
    ang = theta + np.linspace(-np.pi, np.pi, LIDAR_N, endpoint=False)
    rr = np.linspace(0.2, LIDAR_R, 50)
    P = robot[None, None] + rr[None, :, None] * np.stack([np.cos(ang), np.sin(ang)], -1)[:, None]
    cell_r = ((P[..., 1] - world.oy) / world.res).astype(int).clip(0, world.h - 1)
    cell_c = ((P[..., 0] - world.ox) / world.res).astype(int).clip(0, world.w - 1)
    hit = world.blocked[cell_r, cell_c]
    dmin = np.linalg.norm(P[:, :, None] - peds[None, None], axis=-1).min(-1) < 0.22
    hit |= dmin
    first = np.where(hit.any(1), hit.argmax(1), len(rr) - 1)
    return P[np.arange(LIDAR_N), first], hit.any(1)


def body_polys(robot, theta):
    c, s_ = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s_], [s_, c]])
    box = lambda cx, cy, w, h: (np.array([[cx - w / 2, cy - h / 2], [cx + w / 2, cy - h / 2],
                                          [cx + w / 2, cy + h / 2], [cx - w / 2, cy + h / 2]]) @ R.T + robot)
    return box(0, 0, 0.50, 0.30), box(-0.02, TRACK / 2, 0.16, 0.05), box(-0.02, -TRACK / 2, 0.16, 0.05)


steps = int(a.seconds / DT)
writer = (FFMpegWriter(fps=10, bitrate=2500) if a.out.endswith('.mp4') else PillowWriter(fps=10))
with writer.saving(fig, a.out, dpi=80 if a.clean else 90):
    for k in range(steps):
        crowd.step(DT, robot)
        hist.append(crowd.pos.copy()); hist.pop(0)
        h = np.stack(hist[::-1][0::4][:tp.OBS][::-1], 1)                  # (n,8,2) @ 0.4 s
        lp = pred(h, 3.0)
        lp = [p if p is not None else s.predict_cv(crowd.pos[i], crowd.vel[i], 3.0) for i, p in enumerate(lp)]
        # local goal: look ahead along the A* route
        while len(route) > 1 and np.linalg.norm(np.array(route[0]) - robot) < 1.0:
            route.pop(0)
        tgt = np.array(route[min(2, len(route) - 1)]) if route else goals[gi]
        if np.linalg.norm(goals[gi] - robot) < 0.6:
            gi = (gi + 1) % len(goals); route = world.route(tuple(robot), tuple(goals[gi]))
        v = planner.plan(robot, tgt, crowd.pos, crowd.vel, preds=lp)
        v = s.safety_filter(v, robot, crowd.pos)
        sp = float(np.linalg.norm(v))                       # differential drive: heading follows the plan
        if sp > 0.02:
            err = (np.arctan2(v[1], v[0]) - theta + np.pi) % (2 * np.pi) - np.pi
            ang = float(np.clip(3.0 * err, -1.8, 1.8))
            lin = sp * max(0.0, np.cos(err)) if abs(err) < 1.1 else 0.0   # turn in place when far off
        else:
            lin = ang = 0.0
        theta += ang * DT
        new = robot + lin * DT * np.array([np.cos(theta), np.sin(theta)])
        if world.clearance(*new) > 0.3:
            robot = new
        trail.append(robot.copy())
        d = np.linalg.norm(crowd.pos - robot, axis=1)
        clear = d.min() - 0.30 - 0.22
        min_clear = min(min_clear, clear)
        intrusion += DT * (d.min() < 1.2)
        hit = clear < 0
        collisions += int(hit and not in_contact); in_contact = hit

        if k % (3 if a.clean else 2):                                      # render every 2nd step (5 fps sim-time x2)
            continue
        ax.clear()
        ax.imshow(img, cmap='gray', vmin=0, vmax=1, origin='lower', extent=ext, interpolation='nearest')
        for i, p in enumerate(crowd.pos):
            ax.add_patch(Circle(p, 0.22, color=cols[i], zorder=5))
            if d[i] < 7:
                cv = s.predict_cv(p, crowd.vel[i], 3.0)
                if not a.clean:
                    ax.plot(*cv.T, ':', color='tab:blue', lw=1.2, zorder=3)
                path = lp[i]
                ax.plot(*path.T, '-', color='tab:orange', lw=2, zorder=4)
                for step_s in (1.2, 2.4):
                    j = min(len(path) - 1, int(step_s / DT) - 1)
                    kk = min(len(conf_r) - 1, int(step_s / 0.4) - 1)
                    ax.add_patch(Circle(path[j], conf_r[kk] * 0.5, fill=False, ec='tab:orange', lw=0.8, alpha=0.6, zorder=3))
        ax.add_patch(Circle(robot, 1.2, fill=False, ls='--', ec='tab:green', lw=1, alpha=0.7, zorder=4))
        pts, hit_any = lidar(robot, theta, crowd.pos)
        for q, hh in (zip([], []) if a.clean else zip(pts, hit_any)):
            ax.plot([robot[0], q[0]], [robot[1], q[1]], '-', color='tab:cyan', lw=0.4, alpha=0.35, zorder=2)
            if hh:
                ax.plot(*q, '.', color='tab:red', ms=2, zorder=3)
        body, wl, wr = body_polys(robot, theta)
        ax.add_patch(Polygon(body, fc='tab:green', ec='k', lw=1.2, zorder=6))
        ax.add_patch(Polygon(wl, fc='k', zorder=7)); ax.add_patch(Polygon(wr, fc='k', zorder=7))
        ax.arrow(robot[0], robot[1], 0.45 * np.cos(theta), 0.45 * np.sin(theta), width=0.05,
                 head_width=0.18, fc='white', ec='k', lw=0.6, zorder=8)
        if len(trail) > 2:
            ax.plot(*np.array(trail[-120:]).T, '-', color='tab:green', lw=1.5, alpha=0.6, zorder=2)
        ax.plot(*goals[gi], '*', color='gold', ms=14, mec='k', zorder=7)
        ax.set_xlim(robot[0] - 8, robot[0] + 8); ax.set_ylim(robot[1] - 4.6, robot[1] + 4.6)
        ax.set_aspect('equal'); ax.set_xticks([]); ax.set_yticks([])
        vl, vr = lin - ang * TRACK / 2, lin + ang * TRACK / 2
        if a.clean:
            ax.set_title(f'robot (green) navigates a crowd   t={k*DT:4.0f}s   collisions {collisions}   '
                         f'closest {min_clear:+.2f} m', fontsize=10)
            writer.grab_frame()
            continue
        ax.set_title(f't={k*DT:5.1f}s   collisions {collisions}   min clearance {min_clear:+.2f} m   '
                     f'intrusion {intrusion:.1f} s\n'
                     f'diff-drive: v={lin:+.2f} m/s  w={ang:+.2f} rad/s   wheels L {vl:+.2f}  R {vr:+.2f} m/s',
                     fontsize=10.5)
        ax.text(0.01, 0.02, 'orange = learned prediction (+conformal discs)   blue dotted = constant velocity   '
                'green = diff-drive robot, cyan = 2D lidar (ring = 1.2 m personal space)', transform=ax.transAxes, fontsize=8,
                bbox=dict(fc='white', alpha=0.8, ec='none'))
        writer.grab_frame()
print(f'wrote {a.out}  collisions={collisions} min_clearance={min_clear:.2f} intrusion_s={intrusion:.1f}')
