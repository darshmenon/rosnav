#!/usr/bin/env python3
"""Matplotlib simulations of the leaderless swarm formation (decentral_nav_node.SwarmState).

Not a ROS node: a ROS-free harness (N unicycles driven by the node's own `compute_velocity`, `track_unicycle`
and `SwarmState`, open floor or a wall with a gap) that writes GIFs and PNGs to images/swarm_formation/.
Results are simulation only (no Gazebo); numbers are reproduced by running this script, nothing is hardcoded.

    python3 gen_swarm_formation_viz.py ../../../images/swarm_formation          # everything
    python3 gen_swarm_formation_viz.py OUT --only anim_run,scaling              # a subset

Outputs: anim_run.gif anim_morph.gif anim_squeeze.gif cohesion_time.png scaling.png assignment.png
         leader_vs_leaderless.png robustness.png (anim_stall8 / base_vs_new / smoothness via --only)
"""
import argparse
import math
import os
import sys
from multiprocessing import Pool

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import decentral_nav_node as nd  # noqa: E402

plt.rcParams.update({'font.family': 'DejaVu Sans', 'axes.edgecolor': '#c7ccd4', 'axes.linewidth': 0.8,
                     'text.color': '#10161f', 'axes.labelcolor': '#4a5568', 'xtick.color': '#4a5568',
                     'ytick.color': '#4a5568', 'axes.spines.top': False, 'axes.spines.right': False,
                     'axes.grid': True, 'grid.color': '#e6e9ee', 'grid.linewidth': 0.7})
C_BASE, C_NEW, C_ACC, C_BAD = '#9aa4b2', '#2f6fed', '#17a673', '#d64545'
FORMS = ('wedge', 'line', 'circle', 'diamond')
BASE_CFG = dict(cruise=1.0, alpha=1.0, group_tol=1e9)      # no smoothing / no group cohesion
NEW_CFG = dict()                                           # shipped defaults
GOAL_X = 14.0
DT = 0.1
WALL_X = 7.0


# ----------------------------------------------------------------------------------------------- simulation
def wall_points(x=7.0, gap=3.4, half=7.0, step=0.1):
    ys = np.arange(gap / 2, half, step)
    return np.array([[x, y] for y in ys] + [[x, -y] for y in ys])


def simulate(N, form, cfg=None, ctrl='orca', T=90.0, seed=0, walls=None, drop=None, obs_noise=0.0, delay=0,
             sched=None, spacing=1.5, init_gap=0.9, record=True, goal_x=GOAL_X):
    """One run. drop: {time_s: robot_index} robots that die (stop, vanish from TF). sched: {time_s: form}.
    Returns dict of arrays (t, pos, yaw, v, err, slot, alive) + summary scalars."""
    cfg = NEW_CFG if cfg is None else cfg
    rng = np.random.default_rng(seed)
    pos = np.array([[rng.uniform(-0.4, 0.4), (k - (N - 1) / 2) * init_gap + rng.uniform(-0.25, 0.25)]
                    for k in range(N)])
    yaw, v, w = np.zeros(N), np.zeros(N), np.zeros(N)
    vel = np.zeros((N, 2))
    names = [f'robot{k + 1}' for k in range(N)]
    st = [nd.SwarmState(form, spacing, **cfg) for _ in range(N)]
    alive = np.ones(N, bool)
    drop = dict(drop or {})
    sched = dict(sched or {})
    hist = [pos.copy()]
    R = {k: [] for k in ('t', 'pos', 'yaw', 'v', 'err', 'slot', 'alive', 'w', 'cen')}
    settle_t, t, mind = None, 0.0, 9.0
    while t < T:
        for td in [k for k in drop if k <= t]:
            alive[drop.pop(td)] = False
        for ts in [k for k in sched if k <= t]:
            for s_ in st:
                s_.form = sched[ts]
            sched.pop(ts)
        obs = hist[max(0, len(hist) - 1 - delay)]
        newv, neww = v.copy(), w.copy()
        slots = np.full((N, 2), np.nan)
        errs = np.zeros(N)
        for i in range(N):
            if not alive[i]:
                newv[i] = neww[i] = 0.0
                continue
            oth = [j for j in range(N) if j != i and alive[j]]
            P = [list(pos[i])] + [list(obs[j] + rng.normal(0, obs_noise, 2)) for j in oth]
            V = [list(vel[i])] + [list(vel[j]) for j in oth]
            nm = [names[i]] + [names[j] for j in oth]
            if walls is not None and pos[i][0] < WALL_X - 0.5:      # like a Nav2 plan: route through the gap, not the wall
                a1 = np.linspace(pos[i], [WALL_X, 0.0], 30)
                a2 = np.linspace([WALL_X, 0.0], [goal_x, 0.0], 30)[1:]
                pl = np.vstack([a1, a2])
            else:
                pl = np.stack([np.linspace(pos[i][0], goal_x, 57), np.linspace(pos[i][1], 0, 57)], 1)
            scan, clear = None, None
            if walls is not None:
                near = walls[np.linalg.norm(walls - pos[i], axis=1) < 4.0]
                scan = near if len(near) else np.empty((0, 2))
                clear = float(np.min(np.linalg.norm(near - pos[i], axis=1))) if len(near) else None
            pl2, vm = st[i].plan(names[i], nm, P, pl, 0.5, clearance=clear, obstacles=scan)
            ids = [sorted(nm, key=lambda s_: (len(s_), s_)).index(x) for x in nm]
            vx, vy, _ = nd.compute_velocity(ctrl, 0, np.array(P), np.array(V), pl2, scan, rng, vm, 1.0, [], ids=ids)
            newv[i], neww[i] = nd.track_unicycle((vx, vy), yaw[i], v[i], w[i], 0.5, 1.0, dt=DT)
            if st[i].slot is not None:
                slots[i] = st[i].slot
            errs[i] = st[i].err
        v, w = newv, neww
        yaw = yaw + w * DT
        vel = np.stack([v * np.cos(yaw), v * np.sin(yaw)], 1)
        pos = pos + vel * DT
        hist.append(pos.copy())
        t += DT
        a = np.where(alive)[0]
        if len(a) > 1 and t > 10.0:          # ignore the tight initial line-up
            d = min(np.linalg.norm(pos[i] - pos[j]) for i in a for j in a if j < i)
            mind = min(mind, d)
        if record:
            R['t'].append(t); R['pos'].append(pos.copy()); R['yaw'].append(yaw.copy()); R['v'].append(v.copy())
            R['w'].append(w.copy()); R['err'].append(errs.copy()); R['slot'].append(slots.copy())
            R['alive'].append(alive.copy())
            R['cen'].append(pos[a].mean(axis=0))
        if len(a) and pos[a, 0].mean() > goal_x - 1.5 and np.max(np.abs(v[a])) < 0.03 and not drop:
            settle_t = t
            break
    out = {k: np.array(x) for k, x in R.items()}
    a = alive
    out.update(N=N, settle=settle_t, success=settle_t is not None, mind=mind, T=t,
               final_err=float(np.mean(out['err'][-5:, a])) if record else float('nan'),
               progress=float(pos[a, 0].mean()))
    if record:
        e = out['err'][:, :]
        al = out['alive']
        out['mean_err_t'] = np.array([e[k][al[k]].mean() for k in range(len(e))])
        out['worst_err_t'] = np.array([e[k][al[k]].max() for k in range(len(e))])
        dv = np.abs(np.diff(out['v'], axis=0)) / DT
        dw = np.abs(np.diff(out['w'], axis=0)) / DT
        out['jerk'] = float(dv.mean() + dw.mean())
    return out


def _job(a):
    kw = dict(a)
    r = simulate(**kw)
    return {k: r[k] for k in ('N', 'settle', 'success', 'mind', 'T', 'final_err', 'progress', 'jerk')} | \
        {'mean_err': float(np.mean(r['mean_err_t'][len(r['mean_err_t']) // 3:])),
         'mean_err_t': r['mean_err_t'], 'worst_err_t': r['worst_err_t']}


def sweep(jobs):
    with Pool(min(20, len(jobs))) as p:
        return p.map(_job, jobs)


# ----------------------------------------------------------------------------------------------- drawing
def err_color(e):
    return C_ACC if e <= 0.15 else ('#e0a100' if e <= 0.45 else C_BAD)


def segs_cross(a, b, c, d):
    def ccw(p, q, r):
        return (r[1] - p[1]) * (q[0] - p[0]) > (q[1] - p[1]) * (r[0] - p[0])
    return ccw(a, c, d) != ccw(b, c, d) and ccw(a, b, c) != ccw(a, b, d)


def animate(rec, path, title, walls=None, step=3, fps=15, highlight_stall=False, xlim=(-3, 17), ylim=(-8, 8)):
    fig, ax = plt.subplots(figsize=(8.5, 5.4))
    ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.set_aspect('equal')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.plot([GOAL_X, GOAL_X], [ylim[0], ylim[1]], color='#c7ccd4', ls='--', lw=1)
    if walls is not None:
        ax.scatter(walls[:, 0], walls[:, 1], s=6, color='#10161f', zorder=2)
    N = rec['N']
    frames = list(range(0, len(rec['t']), step)) + [len(rec['t']) - 1]
    circles = [plt.Circle((0, 0), 0.3, fc='white', ec='#10161f', lw=1.2, zorder=5) for _ in range(N)]
    for c in circles:
        ax.add_patch(c)
    heads = [ax.plot([], [], color='#10161f', lw=1.4, zorder=6)[0] for _ in range(N)]
    marks = ax.scatter([], [], marker='x', s=36, color=C_NEW, zorder=3)
    links = [ax.plot([], [], color='#b8c0cc', lw=0.8, zorder=1)[0] for _ in range(N)]
    trail = [ax.plot([], [], color='#d5dbe5', lw=0.8, zorder=0)[0] for _ in range(N)]
    ttl = ax.set_title('')

    def draw(fi):
        k = frames[fi]
        p, y, e, s, al = rec['pos'][k], rec['yaw'][k], rec['err'][k], rec['slot'][k], rec['alive'][k]
        v = rec['v'][k]
        stalled = np.zeros(N, bool)
        if highlight_stall:
            stalled = (v < 0.05) & (e > 1.0) & al
        for i in range(N):
            circles[i].center = p[i]
            circles[i].set_facecolor(('#e6e9ee' if not al[i] else err_color(e[i])) if not stalled[i] else C_BAD)
            circles[i].set_edgecolor('#10161f' if al[i] else '#9aa4b2')
            heads[i].set_data([p[i][0], p[i][0] + 0.45 * math.cos(y[i])], [p[i][1], p[i][1] + 0.45 * math.sin(y[i])])
            if al[i] and not np.isnan(s[i, 0]):
                links[i].set_data([p[i][0], s[i, 0]], [p[i][1], s[i, 1]])
            else:
                links[i].set_data([], [])
            tr = rec['pos'][max(0, k - 60):k + 1, i]
            trail[i].set_data(tr[:, 0], tr[:, 1])
        ok = al & ~np.isnan(s[:, 0])
        marks.set_offsets(s[ok] if ok.any() else np.empty((0, 2)))
        extra = f'   stalled {int(stalled.sum())}' if highlight_stall else ''
        ttl.set_text(f'{title}   t={rec["t"][k]:.0f}s   mean slot err {rec["mean_err_t"][k]:.2f} m{extra}')
        return []

    anim = FuncAnimation(fig, draw, frames=len(frames), blit=False)
    anim.save(path, writer=PillowWriter(fps=fps), dpi=80)
    plt.close(fig)
    print('wrote', path)


def save(fig, out, name):
    path = os.path.join(out, name)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    print('wrote', path)


# ----------------------------------------------------------------------------------------------- jobs
def anim_run(out):
    animate(simulate(5, 'wedge'), os.path.join(out, 'anim_run.gif'), 'Leaderless wedge, 5 robots')


def anim_morph(out):
    rec = simulate(6, 'line', sched={18: 'wedge', 34: 'circle'}, T=55)
    animate(rec, os.path.join(out, 'anim_morph.gif'), 'Live morph line -> wedge (18 s) -> circle (34 s)')


def anim_squeeze(out):
    w = wall_points()
    rec = simulate(4, 'wedge', cfg=dict(clear_full=4.0), walls=w, T=70)
    al = rec['alive'][0]
    hit = min(np.linalg.norm(w - p, axis=1).min() for p in rec['pos'].reshape(-1, 2)[::7])
    print('squeeze: min robot-to-wall-point distance', round(float(hit), 2), 'm (robot radius 0.30)')
    animate(rec, os.path.join(out, 'anim_squeeze.gif'), 'Squeeze through a 3.4 m gap, wedge of 4', walls=w)
    return hit


def anim_stall8(out):
    rec = simulate(8, 'line', T=60)
    animate(rec, os.path.join(out, 'anim_stall8.gif'), 'Stall diagnosis, 8 robots (red = stalled)', highlight_stall=True,
            xlim=(-3, 17), ylim=(-10, 10))
    k = len(rec['t']) // 2
    p, s, al = rec['pos'][k], rec['slot'][k], rec['alive'][k]
    cr = sum(segs_cross(p[i], s[i], p[j], s[j]) for i in range(8) for j in range(i) if not np.isnan(s[i, 0]))
    print('stall8: crossing robot->slot segments at mid-run:', cr)
    return cr


def plot_base_vs_new(out, res):
    fig, axs = plt.subplots(1, 3, figsize=(12, 3.8))
    Ns = res['Ns']
    for ax, key, lab in zip(axs, ('mean_err', 'jerk', 'T'), ('mean slot error [m]', 'command jerk [m/s^2 + rad/s^2]',
                                                             'run time [s] (90 = not settled)')):
        for off, cfgn, c in ((-0.18, 'base', C_BASE), (0.18, 'new', C_NEW)):
            ys = [np.mean([r[key] for r in res[(cfgn, n)]]) for n in Ns]
            ax.bar(np.arange(len(Ns)) + off, ys, width=0.34, color=c, label=cfgn)
        ax.set_xticks(range(len(Ns))); ax.set_xticklabels([f'N={n}' for n in Ns]); ax.set_ylabel(lab)
    axs[0].legend(frameon=False)
    fig.suptitle('Base vs improved (smoothing + cohesion), mean over 4 formations x 2 seeds', y=1.02)
    save(fig, out, 'base_vs_new.png')


def plot_cohesion_time(out, res):
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.8), sharey=True)
    for ax, n in zip(axs, (5, 12)):
        for cfgn, c in (('base', C_BASE), ('new', C_NEW)):
            r = [x for x in res[(cfgn, n)]][0]
            t = np.arange(len(r['mean_err_t'])) * DT
            ax.plot(t, r['mean_err_t'], color=c, label=f'{cfgn} mean')
            ax.plot(t, r['worst_err_t'], color=c, ls=':', label=f'{cfgn} worst')
        ax.set_title(f'N={n}'); ax.set_xlabel('time [s]')
    axs[0].set_ylabel('slot error [m]'); axs[0].legend(frameon=False)
    save(fig, out, 'cohesion_time.png')


def plot_smoothness(out):
    fig, axs = plt.subplots(2, 1, figsize=(9, 5.2), sharex=True)
    for cfgn, cfg, c in (('base', BASE_CFG, C_BASE), ('new', NEW_CFG, C_NEW)):
        r = simulate(5, 'wedge', cfg=cfg, T=40)
        t = r['t']
        axs[0].plot(t, r['v'][:, 2], color=c, label=cfgn)
        axs[1].plot(t[1:], np.diff(r['v'][:, 2]) / DT, color=c, lw=0.9)
    axs[0].set_ylabel('v robot3 [m/s]'); axs[1].set_ylabel('accel [m/s^2]'); axs[1].set_xlabel('time [s]')
    axs[0].legend(frameon=False)
    save(fig, out, 'smoothness.png')


def plot_scaling(out, res):
    Ns = res['Ns']
    fig, axs = plt.subplots(1, 3, figsize=(12, 3.8))
    for cfgn, c in (('base', C_BASE), ('new', C_NEW)):
        axs[0].plot(Ns, [100 * np.mean([r['success'] for r in res[(cfgn, n)]]) for n in Ns], 'o-', color=c, label=cfgn)
        axs[1].plot(Ns, [np.mean([r['progress'] for r in res[(cfgn, n)]]) for n in Ns], 'o-', color=c)
        axs[2].plot(Ns, [np.mean([r['final_err'] for r in res[(cfgn, n)]]) for n in Ns], 'o-', color=c)
    axs[0].set_ylabel('settled within 90 s [%]'); axs[1].set_ylabel(f'mean x reached [m] (goal {GOAL_X:.0f})')
    axs[2].set_ylabel('final slot error [m]')
    for a in axs:
        a.set_xlabel('robots N'); a.set_xticks(Ns)
    axs[0].legend(frameon=False)
    save(fig, out, 'scaling.png')


def plot_assignment(out):
    rng = np.random.default_rng(3)
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.6))
    n = 12
    slots = nd.swarm_slots((6, 0), 0.0, nd.swarm_offsets('wedge', n, 1.6))
    pos = rng.uniform([-1, -4], [3, 4], size=(n, 2))

    def greedy(p, s):
        free, out_ = set(range(n)), []
        for i in range(n):
            j = min(free, key=lambda j: np.hypot(p[i][0] - s[j][0], p[i][1] - s[j][1]))
            out_.append(j); free.discard(j)
        return out_
    for ax, (lab, asg) in zip(axs, (('greedy (old, n>7)', greedy(pos, slots)),
                                    ('Hungarian (now)', nd.assign_slots(pos, slots)))):
        cross = 0
        tot = sum(np.hypot(pos[i][0] - slots[asg[i]][0], pos[i][1] - slots[asg[i]][1]) for i in range(n))
        for i in range(n):
            for j in range(i):
                cross += segs_cross(pos[i], slots[asg[i]], pos[j], slots[asg[j]])
        for i in range(n):
            ax.plot([pos[i][0], slots[asg[i]][0]], [pos[i][1], slots[asg[i]][1]], color='#9aa4b2', lw=1)
        ax.scatter(*pos.T, color=C_NEW, zorder=3, label='robots'); ax.scatter(*np.array(slots).T, marker='x', color=C_BAD, zorder=3, label='slots')
        ax.set_title(f'{lab}: total {tot:.1f} m, {cross} crossings'); ax.set_aspect('equal')
    axs[0].legend(frameon=False)
    save(fig, out, 'assignment.png')


def leader_run(N, T=70.0, drop_t=25.0, form='wedge', spacing=1.5):
    """Leader-based convoy (the leader_follower idea): followers chase slots on robot 0's pose, stop when it is lost."""
    pos = np.array([[0.0, (k - (N - 1) / 2) * 0.9] for k in range(N)])
    yaw, v, w, vel = np.zeros(N), np.zeros(N), np.zeros(N), np.zeros((N, 2))
    offs = nd.swarm_offsets(form, N, spacing)
    xs, t, alive0 = [], 0.0, True
    names = [f'robot{k + 1}' for k in range(N)]
    last = (pos[0].copy(), 0.0)
    while t < T:
        if t >= drop_t:
            alive0 = False
        if alive0:
            last = (pos[0].copy(), 0.0)
        slots = nd.swarm_slots(last[0], last[1], offs)       # followers only know the last leader pose
        newv, neww = v.copy(), w.copy()
        for i in range(N):
            if i == 0:
                tgt = np.array([GOAL_X, 0.0]) if alive0 else pos[0]
            else:
                tgt = np.array(slots[i]) if alive0 else pos[i]   # leader lost -> hold (watchdog)
            if not alive0 and i == 0:
                newv[i] = neww[i] = 0.0
                continue
            P = [list(pos[i])] + [list(pos[j]) for j in range(N) if j != i and (alive0 or j != 0)]
            V = [list(vel[i])] + [list(vel[j]) for j in range(N) if j != i and (alive0 or j != 0)]
            pl = np.stack([np.linspace(pos[i][0], tgt[0], 40), np.linspace(pos[i][1], tgt[1], 40)], 1)
            if np.linalg.norm(pl[-1] - pl[0]) < 0.05:
                newv[i], neww[i] = nd.track_unicycle((0, 0), yaw[i], v[i], w[i], 0.5, 1.0, dt=DT)
                continue
            vx, vy, _ = nd.compute_velocity('orca', 0, np.array(P), np.array(V), pl, None, None, 0.5, 1.0, [])
            newv[i], neww[i] = nd.track_unicycle((vx, vy), yaw[i], v[i], w[i], 0.5, 1.0, dt=DT)
        v, w = newv, neww
        yaw = yaw + w * DT
        vel = np.stack([v * np.cos(yaw), v * np.sin(yaw)], 1)
        pos = pos + vel * DT
        t += DT
        xs.append(pos[1:, 0].mean())
        _ = names
    return np.array(xs)


def plot_leader(out):
    N, drop_t = 6, 25.0
    xl = leader_run(N, drop_t=drop_t)
    rl = simulate(N, 'wedge', T=70, drop={drop_t: 0})
    al = rl['alive']
    xs = np.array([rl['pos'][k][al[k], 0].mean() for k in range(len(rl['t']))])
    fig, ax = plt.subplots(figsize=(8.5, 4))
    ax.plot(np.arange(len(xl)) * DT, xl, color=C_BAD, label='leader-based (followers hold when the leader is lost)')
    ax.plot(rl['t'], xs, color=C_NEW, label='leaderless (same robot removed)')
    ax.axvline(drop_t, color='#9aa4b2', ls='--'); ax.text(drop_t + 0.5, 12.5, 'robot 1 dies', color='#4a5568')
    ax.axhline(GOAL_X, color='#c7ccd4', ls=':')
    ax.set_xlabel('time [s]'); ax.set_ylabel('mean x of surviving robots [m]'); ax.legend(frameon=False, loc='lower right')
    save(fig, out, 'leader_vs_leaderless.png')
    return float(xl[-1]), float(xs[-1])


def plot_robustness(out):
    N = 6
    cases = [('clean', {}), ('noise 0.1 m', dict(obs_noise=0.1)), ('noise 0.3 m', dict(obs_noise=0.3)),
             ('delay 0.3 s', dict(delay=3)), ('delay 1 s', dict(delay=10)),
             ('1 dropout', dict(drop={20.0: 2})), ('2 dropouts', dict(drop={20.0: 2, 30.0: 4}))]
    jobs = [dict(N=N, form='wedge', T=80, seed=s, **kw) for _, kw in cases for s in range(3)]
    res = sweep(jobs)
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.8))
    lab = [c[0] for c in cases]
    err = [np.mean([r['final_err'] for r in res[3 * i:3 * i + 3]]) for i in range(len(cases))]
    mind = [np.min([r['mind'] for r in res[3 * i:3 * i + 3]]) for i in range(len(cases))]
    axs[0].bar(lab, err, color=C_NEW); axs[0].set_ylabel('final slot error [m]')
    axs[1].bar(lab, mind, color=C_ACC); axs[1].axhline(0.6, color=C_BAD, ls='--'); axs[1].set_ylabel('min robot gap [m] (red = contact 0.6)')
    for a in axs:
        a.tick_params(axis='x', rotation=35)
    save(fig, out, 'robustness.png')
    return dict(zip(lab, zip(err, mind)))


# anim_stall8, base_vs_new and smoothness are kept as functions but not generated by default: after the plan-length
# fix there is no stall to show and base/new are identical for N<=8 (their old PNGs/GIF were removed).
ALL = ['anim_run', 'anim_morph', 'anim_squeeze', 'cohesion_time', 'scaling', 'assignment', 'leader', 'robustness']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--only', default='')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    want = set(a.only.split(',')) if a.only else set(ALL)
    notes = {}
    for name, fn in (('anim_run', anim_run), ('anim_morph', anim_morph)):
        if name in want:
            fn(a.out)
    if 'anim_squeeze' in want:
        notes['squeeze_min_wall_dist'] = anim_squeeze(a.out)
    if 'anim_stall8' in want:
        notes['stall8_crossings'] = anim_stall8(a.out)
    if want & {'base_vs_new', 'cohesion_time', 'scaling'}:
        Ns = (3, 5, 8, 12)
        keys, jobs = [], []
        for cfgn, cfg in (('base', BASE_CFG), ('new', NEW_CFG)):
            for n in Ns:
                for f in FORMS:
                    for s in range(2):
                        keys.append((cfgn, n))
                        jobs.append(dict(N=n, form=f, cfg=cfg, T=90, seed=s))
        out_ = sweep(jobs)
        res = {'Ns': Ns}
        for k, r in zip(keys, out_):
            res.setdefault(k, []).append(r)
        if 'base_vs_new' in want:
            plot_base_vs_new(a.out, res)
        if 'cohesion_time' in want:
            plot_cohesion_time(a.out, res)
        if 'scaling' in want:
            plot_scaling(a.out, res)
        notes['scaling'] = {f'{c}-N{n}': dict(success=float(np.mean([r['success'] for r in res[(c, n)]])),
                                            final_err=float(np.mean([r['final_err'] for r in res[(c, n)]])),
                                            jerk=float(np.mean([r['jerk'] for r in res[(c, n)]])))
                            for c in ('base', 'new') for n in Ns}
    if 'smoothness' in want:
        plot_smoothness(a.out)
    if 'assignment' in want:
        plot_assignment(a.out)
    if 'leader' in want:
        notes['leader_vs_leaderless_final_x'] = plot_leader(a.out)
    if 'robustness' in want:
        notes['robustness(err,mind)'] = plot_robustness(a.out)
    print('NOTES', notes)


if __name__ == '__main__':
    main()
