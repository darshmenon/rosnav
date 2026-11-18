#!/usr/bin/env python3
"""CoRL-MPPI-style learned sampling prior for decentral_nav_sim.

Imitates the mppi_prior controller's chosen velocity with a small MLP, then
uses the net output as the MPPI prior mean ('mppi_learned').
  collect/train/eval:  nice -n 19 python3 decentral_nav_learned_prior.py all
"""
import argparse
import math
import os
import sys
import time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import decentral_nav_sim as sim  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
MODEL = os.path.join(ROOT, 'local', 'decentral_learned_prior.npz')
REPORT = os.path.join(sim.OUT, 'learned_prior.md')
K_NB = 4
NFEAT = 3 + 2 + 3 + 4 + 4 * K_NB     # vel(2) dist(1) | ahead clear/free(2) | yield flag+side(3) | ring clear(4) | nbrs
N_BEST = 4                           # expert: best-of-N candidates
_ERNG = np.random.default_rng(12345)


def yield_info(i, pos, vel):
    """Same trigger as sim.command's mppi_prior yield rule: (flag, unit side vec in world frame)."""
    for j in range(len(pos)):
        if j == i:
            continue
        rel = pos[j] - pos[i]
        dist = np.linalg.norm(rel)
        if dist < 2.2 and np.linalg.norm(vel[j]) < 0.15 and np.linalg.norm(vel[i]) < 0.15 and i > j:
            return 1.0, np.array([-rel[1], rel[0]]) / (dist + 1e-9)
    return 0.0, np.zeros(2)


def _clear_at(world, p):
    ix = int(np.clip(round(p[0]), 0, world.w - 1))
    iy = int(np.clip(round(p[1]), 0, world.h - 1))
    return min(world.clear[iy, ix], 3.0) / 3.0 if world.free[iy, ix] else 0.0


def features(i, pos, vel, goals, world):
    """Goal-aligned frame: x axis = BFS pref direction. Returns (feat, rotation R)."""
    g = goals[i]
    d = np.linalg.norm(g - pos[i])
    pd = world.pref_dir(pos[i], g)
    Rm = np.array([[pd[0], pd[1]], [-pd[1], pd[0]]])   # world -> goal frame
    f = list(Rm @ vel[i]) + [min(d, 10.0) / 10.0]
    ahead = pos[i] + 1.0 * pd
    ix = int(np.clip(round(ahead[0]), 0, world.w - 1))
    iy = int(np.clip(round(ahead[1]), 0, world.h - 1))
    f += [min(world.clear[iy, ix], 3.0) / 3.0, 1.0 if world.free[iy, ix] else 0.0]
    yf, side = yield_info(i, pos, vel)
    f += [yf] + list(Rm @ side)
    for ang in (0, 90, 180, 270):                      # wall clearance ring in goal frame
        a = math.radians(ang)
        dirg = np.array([math.cos(a), math.sin(a)])
        f.append(_clear_at(world, pos[i] + 0.8 * (Rm.T @ dirg)))
    others = [j for j in range(len(pos)) if j != i]
    others.sort(key=lambda j: np.linalg.norm(pos[j] - pos[i]))
    for j in others[:K_NB]:
        rp = Rm @ (pos[j] - pos[i])
        rv = Rm @ (vel[j] - vel[i])
        if np.linalg.norm(rp) > 6.0:
            f += [0, 0, 0, 0]
        else:
            f += [rp[0] / 3, rp[1] / 3, rv[0], rv[1]]
    f += [0.0] * (4 * (K_NB - min(K_NB, len(others))))
    return np.array(f, np.float32), Rm


# ── expert: best-of-N with lookahead rollout ────────────────────────────────
def _lookahead_cost(v, i, pos, vel, goal, world, T=2.0, steps=10):
    """Cost of holding velocity v for T s (others constant-velocity)."""
    dt = T / steps
    p = pos[i].copy()
    c = 0.0
    dg = world.dist_field(goal)
    for h in range(1, steps + 1):
        p = p + v * dt
        if not world.ok(p):
            c += 40.0
            p = p - v * dt
            continue
        for j in range(len(pos)):
            if j == i or np.linalg.norm(pos[j] - pos[i]) > 7.0:
                continue
            dd = np.linalg.norm(p - (pos[j] + vel[j] * dt * h))
            if dd < 2 * sim.R + 0.1:
                c += 60.0
            elif dd < 2 * sim.R + 0.7:
                c += 6.0 * (2 * sim.R + 0.7 - dd)
    ix = int(np.clip(round(p[0]), 0, world.w - 1))
    iy = int(np.clip(round(p[1]), 0, world.h - 1))
    e = dg[iy, ix]
    c += 1.5 * (1e3 if np.isinf(e) else e)
    if world.clear[iy, ix] < 1.0:
        c += 5.0
    return c


def expert_label(i, pos, vel, goals, world, orig_cmd):
    """Best of {mppi_prior x N seeds, vo, yield-side, pref} by lookahead cost."""
    g = goals[i]
    pref = sim.VMAX * world.pref_dir(pos[i], g)
    cands = [orig_cmd('mppi_prior', i, pos, vel, goals, world, _ERNG) for _ in range(N_BEST)]
    cands.append(sim.vo_velocity(i, pos, vel, g, world, pref))
    cands.append(pref)
    yf, side = yield_info(i, pos, vel)
    if yf:
        cands.append(0.6 * sim.VMAX * side)
    costs = [_lookahead_cost(c, i, pos, vel, g, world) for c in cands]
    return cands[int(np.argmin(costs))]


# ── data collection ─────────────────────────────────────────────────────────
def collect(seeds, n=8, kinds=('swap', 'maze'), max_t=40.0, driver='mppi_prior'):
    """Roll out `driver` (expert-ish or learner), label every visited state with the expert."""
    X, Y = [], []
    orig = sim.command
    base = orig

    def rec(ctrl, i, pos, vel, goals, world, rng):
        if np.linalg.norm(goals[i] - pos[i]) >= 0.25:
            f, Rm = features(i, pos, vel, goals, world)
            lab = expert_label(i, pos, vel, goals, world, base)
            X.append(f)
            Y.append(Rm @ lab)
        return base(driver, i, pos, vel, goals, world, rng) if driver != 'mppi_learned' else command_learned(i, pos, vel, goals, world, rng)
    sim.command = rec
    try:
        for kind in kinds:
            for sd in seeds:
                sim.simulate(kind, 'x', n, sd, max_t)
    finally:
        sim.command = orig
    return np.array(X), np.array(Y, np.float32)


# ── training ────────────────────────────────────────────────────────────────
def train(X, Y, epochs=60):
    import torch
    torch.set_num_threads(1)
    torch.manual_seed(0)
    mu, sd = X.mean(0), X.std(0) + 1e-6
    Xt = torch.tensor((X - mu) / sd)
    Yt = torch.tensor(Y)
    perm = torch.randperm(len(Xt))
    nv = len(Xt) // 10
    va, tr = perm[:nv], perm[nv:]
    net = torch.nn.Sequential(torch.nn.Linear(X.shape[1], 64), torch.nn.ReLU(),
                              torch.nn.Linear(64, 64), torch.nn.ReLU(), torch.nn.Linear(64, 2))
    opt = torch.optim.Adam(net.parameters(), 2e-3)
    for ep in range(epochs):
        p = tr[torch.randperm(len(tr))]
        for b in range(0, len(p), 256):
            idx = p[b:b + 256]
            loss = torch.nn.functional.smooth_l1_loss(net(Xt[idx]), Yt[idx])
            opt.zero_grad()
            loss.backward()
            opt.step()
        if ep % 10 == 9 or ep == epochs - 1:
            with torch.no_grad():
                print(f'  ep{ep+1} val_mse={torch.mean((net(Xt[va]) - Yt[va]) ** 2).item():.4f}', flush=True)
    W = [(m.weight.detach().numpy(), m.bias.detach().numpy()) for m in net if isinstance(m, torch.nn.Linear)]
    os.makedirs(os.path.dirname(MODEL), exist_ok=True)
    np.savez(MODEL, mu=mu, sd=sd, **{f'w{k}': w for k, (w, _) in enumerate(W)}, **{f'b{k}': b for k, (_, b) in enumerate(W)})


_NET = None


def _load():
    global _NET
    if _NET is None:
        _NET = dict(np.load(MODEL))
    return _NET


def learned_prior(i, pos, vel, goals, world):
    """Net output (world-frame velocity) to use as MPPI prior mean."""
    m = _load()
    f, Rm = features(i, pos, vel, goals, world)
    h = (f - m['mu']) / m['sd']
    h = np.maximum(m['w0'] @ h + m['b0'], 0)
    h = np.maximum(m['w1'] @ h + m['b1'], 0)
    out = m['w2'] @ h + m['b2']
    sp = np.linalg.norm(out)
    if sp > sim.VMAX:
        out = out * sim.VMAX / sp
    return Rm.T @ out


def command_learned(i, pos, vel, goals, world, rng, sigma=0.55):
    g = goals[i]
    if np.linalg.norm(g - pos[i]) < 0.25:
        return np.zeros(2)
    pref = sim.VMAX * world.pref_dir(pos[i], g)
    return sim.mppi_velocity(i, pos, vel, g, world, pref, rng, prior=learned_prior(i, pos, vel, goals, world), sigma=sigma)


def command_learned_yield(i, pos, vel, goals, world, rng, sigma=0.25):
    """Learned prior + the same hard yield-side override as sim.command's mppi_prior."""
    g = goals[i]
    if np.linalg.norm(g - pos[i]) < 0.25:
        return np.zeros(2)
    pref = sim.VMAX * world.pref_dir(pos[i], g)
    prior = learned_prior(i, pos, vel, goals, world)
    yf, side = yield_info(i, pos, vel)
    if yf:
        prior = 0.6 * sim.VMAX * side
    return sim.mppi_velocity(i, pos, vel, g, world, pref, rng, prior=prior, sigma=sigma)


def install():
    """Plug 'mppi_learned' into sim.command (module-level monkeypatch)."""
    orig = sim.command

    def cmd(ctrl, i, pos, vel, goals, world, rng):
        if ctrl == 'mppi_learned':
            return command_learned(i, pos, vel, goals, world, rng)
        if ctrl == 'mppi_learned_lo':       # lower sampling noise around the learned mean
            return command_learned(i, pos, vel, goals, world, rng, sigma=0.25)
        if ctrl == 'mppi_learned_yield':    # low sigma + hard yield-side override
            return command_learned_yield(i, pos, vel, goals, world, rng)
        return orig(ctrl, i, pos, vel, goals, world, rng)
    sim.command = cmd


def evaluate(seeds, n=8, kinds=('swap', 'maze'), ctrls=('vo', 'mppi_prior', 'mppi_learned', 'mppi_learned_lo', 'mppi_learned_yield'), max_t=60.0):
    install()
    rows = []
    for kind in kinds:
        for c in ctrls:
            t0 = time.time()
            rs = [sim.simulate(kind, c, n, sd, max_t) for sd in seeds]
            r = dict(kind=kind, ctrl=c, succ=100 * np.mean([x['success'] for x in rs]),
                     mk=np.mean([x['makespan'] for x in rs if x['success']] or [np.nan]),
                     delay=np.mean([x['delay'] for x in rs]), coll=np.mean([x['collisions'] for x in rs]))
            rows.append(r)
            print(f"{kind:5s} {c:13s} succ={r['succ']:5.1f} mk={r['mk']:6.1f} delay={r['delay']:6.1f} coll={r['coll']:.2f} ({time.time()-t0:.0f}s)", flush=True)
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['all', 'eval'], default='all', nargs='?')
    ap.add_argument('--train-seeds', type=int, default=8)
    ap.add_argument('--eval-seeds', type=int, default=10)
    ap.add_argument('--rounds', type=int, default=3)
    ap.add_argument('--epochs', type=int, default=60)
    a = ap.parse_args()
    global _NET
    ts = list(range(100, 100 + a.train_seeds))      # disjoint from eval seeds 0..N-1
    es = list(range(a.eval_seeds))
    log = []
    if a.cmd == 'all':
        t0 = time.time()
        X, Y = collect(ts)
        print(f'round0 (expert-driven): {len(X)} samples in {time.time()-t0:.0f}s', flush=True)
        train(X, Y, a.epochs)
        _NET = None
        log.append(len(X))
        for r in range(1, a.rounds + 1):
            t0 = time.time()
            Xn, Yn = collect(ts, driver='mppi_learned')
            X = np.concatenate([X, Xn])
            Y = np.concatenate([Y, Yn])
            print(f'DAgger round {r}: +{len(Xn)} learner-visited states relabelled ({len(X)} total) in {time.time()-t0:.0f}s', flush=True)
            train(X, Y, a.epochs)
            _NET = None
            log.append(len(X))
        nsamp = log
    else:
        nsamp = []
    rows = evaluate(es)
    md = [f'Learned MPPI prior v2 (MLP {NFEAT}-64-64-2). Expert = best-of-{N_BEST} mppi_prior samples + vo + pref (+yield side) '
          f'chosen by 2 s lookahead cost; features include yield flag/side + 4-dir wall clearance; '
          f'DAgger {a.rounds} rounds (dataset sizes {nsamp}); train seeds {ts[0]}-{ts[-1]}; '
          f'eval n=8, held-out seeds 0-{a.eval_seeds-1}, max_t=60s\n',
          '| scenario | controller | success % | makespan s | delay s | collision pairs |', '|---|---|---|---|---|---|']
    md += [f"| {r['kind']} | {r['ctrl']} | {r['succ']:.0f} | {r['mk']:.1f} | {r['delay']:.1f} | {r['coll']:.2f} |" for r in rows]
    md += ['', 'Previous version (v1, MLP 21-64-64-2 imitating mppi_prior output, 43625 samples, no yield flag), same eval:', '',
           '| scenario | controller | success % | makespan s | delay s | collision pairs |', '|---|---|---|---|---|---|',
           '| swap | mppi_learned (v1) | 100 | 22.4 | 9.1 | 2.00 |', '| maze | mppi_learned (v1) | 50 | 48.0 | 10.1 | 0.20 |']
    os.makedirs(sim.OUT, exist_ok=True)
    open(REPORT, 'w').write('\n'.join(md) + '\n')
    print('wrote', REPORT)


if __name__ == '__main__':
    main()
