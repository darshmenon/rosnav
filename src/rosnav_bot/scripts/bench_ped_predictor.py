#!/usr/bin/env python3
"""bench_ped_predictor.py — inference latency of the learned pedestrian predictor (CPU vs GPU)."""
import argparse, time
import torch
import train_ped_predictor as tp

ap = argparse.ArgumentParser(); ap.add_argument('--weights', default='outputs/ped_predictor_n24.pt')
a = ap.parse_args()
for dev in ['cpu'] + (['cuda'] if torch.cuda.is_available() else []):
    net = tp.SocialTransformer().to(dev)
    net.load_state_dict(torch.load(a.weights, map_location=dev)); net.eval()
    for n in (4, 12, 24):
        x = torch.randn(1, tp.NMAX, tp.OBS, 2, device=dev); m = torch.zeros(1, tp.NMAX, dtype=torch.bool, device=dev); m[0, :n] = True
        with torch.no_grad():
            for _ in range(20): net(x, m)
            if dev == 'cuda': torch.cuda.synchronize()
            ts = []
            for _ in range(200):
                t0 = time.perf_counter(); net(x, m)
                if dev == 'cuda': torch.cuda.synchronize()
                ts.append((time.perf_counter() - t0) * 1e3)
        ts.sort()
        print(f'{dev:4s} agents={n:2d}  p50={ts[100]:.2f} ms  p95={ts[190]:.2f} ms')
