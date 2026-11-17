#!/usr/bin/env python3
"""
render_nav_showcase.py — dashboard-style GIF of the social-nav stack (matplotlib only, no Gazebo / ROS).

Top left : whole hospital map — every pedestrian, the robot, its A* route, trail and goal (where am I going).
Top right: robot-centred zoom — learned predictions with 90% uncertainty discs, the 1.2 m personal-space ring,
           the differential-drive body + heading + 2D lidar hits, pedestrians coloured by closeness.
Bottom   : distance to the nearest person over time against the 1.2 m personal-space line and the 0.52 m contact line.

  python3 render_nav_showcase.py --out nav_showcase.gif --seconds 60
"""
import argparse, json, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter
from matplotlib.colors import ListedColormap
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
ap.add_argument('--out', default='nav_showcase.gif')
ap.add_argument('--seconds', type=float, default=60)
ap.add_argument('--peds', type=int, default=10)
ap.add_argument('--seed', type=int, default=3)
ap.add_argument('--start', type=float, nargs=2, default=[-9.0, 0.0])
ap.add_argument('--goals', type=float, nargs='+', default=[8.0, 0.4, -8.0, -0.4])
ap.add_argument('--every', type=int, default=3, help='render every Nth 0.1 s step')
a = ap.parse_args()

world = sf.MapWorld(a.map)
world.remove_small_obstacles(0.9)
crowd = sf.Crowd(world, a.peds, seed=a.seed, spawn_avoid=a.start)
pred = ev.LearnedPredictor(a.weights)
cj = a.weights + '.conformal.json'
conf = json.load(open(cj))['radii'] if os.path.exists(cj) else None
planner = s.RiskPlanner(conformal_radii=conf, sigma_scale=0.5, clear_fn=world.clearance_many)
conf_r = np.array(conf) if conf else np.full(12, 0.5)

DT, TRACK, LIDAR_N, LIDAR_R = 0.1, 0.36, 90, 4.5
robot, theta, lin, ang = np.array(a.start, float), 0.0, 0.0, 0.0
goals = [np.array(a.goals[i:i + 2]) for i in range(0, len(a.goals), 2)]
gi, route = 0, world.route(tuple(robot), tuple(goals[0]))
hist = [crowd.pos.copy() for _ in range(32)]
trail, ts, dmins = [], [], []
collisions, intrusion, in_contact, min_clear, laps = 0, 0.0, False, 1e9, 0

ext = [world.ox, world.ox + world.w * world.res, world.oy, world.oy + world.h * world.res]
img = world.blocked.astype(float)
cmap = ListedColormap(['#f7f7f2', '#3b3f46'])
rows = np.where(world.blocked.any(1))[0]; cols_ = np.where(world.blocked.any(0))[0]
xlim = (world.ox + cols_.min() * world.res - 0.5, world.ox + cols_.max() * world.res + 0.5)
ylim = (world.oy + rows.min() * world.res - 0.5, world.oy + rows.max() * world.res + 0.5)

fig = plt.figure(figsize=(12.5, 6.4), dpi=70)
gs = fig.add_gridspec(2, 2, height_ratios=[3.2, 1], width_ratios=[1.35, 1], hspace=0.28, wspace=0.08,
                      left=0.03, right=0.985, top=0.9, bottom=0.08)
axm, axz, axt = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, :])


def lidar(robot, theta, peds):
    an = theta + np.linspace(-np.pi, np.pi, LIDAR_N, endpoint=False)
    rr = np.linspace(0.2, LIDAR_R, 45)
    P = robot[None, None] + rr[None, :, None] * np.stack([np.cos(an), np.sin(an)], -1)[:, None]
    r_ = ((P[..., 1] - world.oy) / world.res).astype(int).clip(0, world.h - 1)
    c_ = ((P[..., 0] - world.ox) / world.res).astype(int).clip(0, world.w - 1)
    hit = world.blocked[r_, c_] | (np.linalg.norm(P[:, :, None] - peds[None, None], axis=-1).min(-1) < 0.22)
    first = np.where(hit.any(1), hit.argmax(1), len(rr) - 1)
    return P[np.arange(LIDAR_N), first], hit.any(1)


def body(robot, theta):
    c, s_ = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s_], [s_, c]])
    box = lambda cx, cy, w, h: (np.array([[cx - w / 2, cy - h / 2], [cx + w / 2, cy - h / 2],
                                          [cx + w / 2, cy + h / 2], [cx - w / 2, cy + h / 2]]) @ R.T + robot)
    return box(0, 0, 0.50, 0.30), box(-0.02, TRACK / 2, 0.16, 0.05), box(-0.02, -TRACK / 2, 0.16, 0.05)


def ped_color(d):
    return '#d62728' if d < 1.2 else ('#ff9f1c' if d < 2.5 else '#4c78a8')


writer = PillowWriter(fps=12)
with writer.saving(fig, a.out, dpi=70):
    for k in range(int(a.seconds / DT)):
        crowd.step(DT, robot)
        hist.append(crowd.pos.copy()); hist.pop(0)
        h = np.stack(hist[::-1][0::4][:tp.OBS][::-1], 1)
        lp = pred(h, 3.0)
        lp = [p if p is not None else s.predict_cv(crowd.pos[i], crowd.vel[i], 3.0) for i, p in enumerate(lp)]
        while len(route) > 1 and np.linalg.norm(np.array(route[0]) - robot) < 1.0:
            route.pop(0)
        tgt = np.array(route[min(2, len(route) - 1)]) if route else goals[gi]
        if np.linalg.norm(goals[gi] - robot) < 0.6:
            gi = (gi + 1) % len(goals); laps += gi == 0
            route = world.route(tuple(robot), tuple(goals[gi]))
        v = s.safety_filter(planner.plan(robot, tgt, crowd.pos, crowd.vel, preds=lp), robot, crowd.pos)
        sp = float(np.linalg.norm(v))
        if sp > 0.02:
            err = (np.arctan2(v[1], v[0]) - theta + np.pi) % (2 * np.pi) - np.pi
            ang = float(np.clip(3.0 * err, -1.8, 1.8))
            lin = sp * max(0.0, np.cos(err)) if abs(err) < 1.1 else 0.0
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
        ts.append(k * DT); dmins.append(d.min())
        if k % a.every:
            continue

        # ---- overview
        axm.clear()
        axm.imshow(img, cmap=cmap, origin='lower', extent=ext, interpolation='nearest')
        if route:
            axm.plot(*np.array([robot] + route).T, '--', color='gold', lw=1.6, zorder=3)
        axm.plot(*np.array(trail).T, '-', color='tab:green', lw=1.6, alpha=.7, zorder=3)
        for i, p in enumerate(crowd.pos):
            axm.add_patch(Circle(p, 0.3, color=ped_color(d[i]), zorder=5))
        axm.add_patch(Polygon(body(robot, theta)[0] * 1.0, fc='tab:green', ec='k', zorder=7))
        axm.add_patch(Circle(robot, 0.55, fc='none', ec='tab:green', lw=1.5, zorder=7))
        for g in goals:
            axm.plot(*g, '*', color='gold', ms=13, mec='k', zorder=6)
        axm.set_xlim(*xlim); axm.set_ylim(*ylim); axm.set_aspect('equal'); axm.set_xticks([]); axm.set_yticks([])
        axm.set_title('hospital map: route (gold), trail (green), people', fontsize=10)

        # ---- zoom
        axz.clear()
        axz.imshow(img, cmap=cmap, origin='lower', extent=ext, interpolation='nearest')
        for i, p in enumerate(crowd.pos):
            if d[i] > 7:
                continue
            path = lp[i]
            axz.plot(*path.T, '-', color='tab:orange', lw=2, alpha=.9, zorder=4)
            for step_s in (1.2, 2.4):
                j = min(len(path) - 1, int(step_s / DT) - 1)
                kk = min(len(conf_r) - 1, int(step_s / 0.4) - 1)
                axz.add_patch(Circle(path[j], conf_r[kk] * 0.5, fill=False, ec='tab:orange', lw=.8, alpha=.55, zorder=3))
            axz.add_patch(Circle(p, 0.22, color=ped_color(d[i]), zorder=5))
            vn = np.linalg.norm(crowd.vel[i])
            if vn > .1:
                axz.arrow(p[0], p[1], *(crowd.vel[i] / vn * .4), width=.03, head_width=.12, fc='k', ec='k', zorder=6)
        axz.add_patch(Circle(robot, 1.2, fill=False, ls='--', ec='tab:green', lw=1.1, alpha=.8, zorder=4))
        pts, hh = lidar(robot, theta, crowd.pos)
        for q, hv in zip(pts, hh):
            axz.plot([robot[0], q[0]], [robot[1], q[1]], '-', color='tab:cyan', lw=.35, alpha=.3, zorder=2)
            if hv:
                axz.plot(*q, '.', color='tab:red', ms=2.5, zorder=3)
        b, wl, wr = body(robot, theta)
        axz.add_patch(Polygon(b, fc='tab:green', ec='k', lw=1.2, zorder=7))
        axz.add_patch(Polygon(wl, fc='k', zorder=8)); axz.add_patch(Polygon(wr, fc='k', zorder=8))
        axz.arrow(robot[0], robot[1], .5 * np.cos(theta), .5 * np.sin(theta), width=.05, head_width=.18,
                  fc='white', ec='k', lw=.6, zorder=9)
        axz.plot(*goals[gi], '*', color='gold', ms=14, mec='k', zorder=8)
        axz.set_xlim(robot[0] - 4.2, robot[0] + 4.2); axz.set_ylim(robot[1] - 3.2, robot[1] + 3.2)
        axz.set_aspect('equal'); axz.set_xticks([]); axz.set_yticks([])
        vl, vr = lin - ang * TRACK / 2, lin + ang * TRACK / 2
        axz.set_title(f'robot view: v {lin:+.2f} m/s  w {ang:+.2f} rad/s  wheels L {vl:+.2f} R {vr:+.2f}', fontsize=10)

        # ---- distance timeline
        axt.clear()
        axt.fill_between(ts, 0, 0.52, color='#d62728', alpha=.18, lw=0)
        axt.fill_between(ts, 0.52, 1.2, color='#ff9f1c', alpha=.15, lw=0)
        axt.plot(ts, dmins, '-', color='#1f3b57', lw=1.6)
        axt.set_xlim(0, a.seconds); axt.set_ylim(0, 5)
        axt.set_ylabel('nearest person (m)', fontsize=9); axt.set_xlabel('time (s)', fontsize=9)
        axt.text(a.seconds * .995, .1, 'contact', ha='right', fontsize=8, color='#a01818')
        axt.text(a.seconds * .995, .75, 'personal space (1.2 m)', ha='right', fontsize=8, color='#a86400')
        axt.grid(alpha=.25); axt.tick_params(labelsize=8)
        fig.suptitle(f'Social navigation: learned pedestrian prediction + risk-aware planner     t = {k*DT:4.0f} s     '
                     f'collisions {collisions}     closest {min_clear:+.2f} m     in personal space {intrusion:.1f} s',
                     fontsize=11.5)
        writer.grab_frame()
print(f'wrote {a.out}  collisions={collisions} min_clearance={min_clear:.2f} intrusion_s={intrusion:.1f} laps={laps}')
