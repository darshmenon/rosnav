#!/usr/bin/env python3
"""
pulse_rates.py — read per-topic rates from ros2_pulse logs. No ROS in here.

ros2_pulse (apt: ros-humble-ros2-pulse) is an LD_PRELOAD probe that appends one JSON line per
window to a log per process (ROS_TOPIC_STATS_FORMAT=jsonl). This module tails those logs and
returns the newest rate seen for each topic, so a monitor does not need its own subscription
(and message deserialisation) per topic. It is optional: with no fresh logs `read()` returns {}
and the caller keeps its own subscriptions.

    rates = PulseRates().read()      # {'/robot1/odom': 49.6, '/robot1/scan': 10.0, ...}
"""
from __future__ import annotations

import glob
import json
import os
import tempfile
import time

_TAIL_BYTES = 64 * 1024        # logs rotate at 10 MiB: only the last few windows matter
_MIN_WINDOW_S = 1.0            # the atexit flush is a sub-period sliver, its rate is one sample


def default_glob() -> str:
    return os.path.join(os.environ.get('TMPDIR') or tempfile.gettempdir(), 'topic_freq.*.log')


def _last_windows(path: str, n: int = 3):
    """Newest `n` complete JSON windows of one log, newest last."""
    try:
        with open(path, 'rb') as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - _TAIL_BYTES))
            data = f.read().decode('utf-8', 'replace')
    except OSError:
        return []
    out = []
    for line in data.splitlines()[-40:]:
        line = line.strip()
        if not line.startswith('{'):
            continue                    # text-format log, or a cut first line
        try:
            out.append(json.loads(line))
        except ValueError:
            continue
    return out[-n:]


class PulseRates:
    def __init__(self, pattern: str = '', max_age_s: float = 15.0):
        self.pattern = pattern or default_glob()
        self.max_age_s = max_age_s

    def read(self, now: float | None = None) -> dict[str, float]:
        """Topic -> Hz (the largest of publish/receive rate over all fresh logs). Logs of dead
        processes stop being rewritten and age out after `max_age_s`."""
        now = time.time() if now is None else now
        rates: dict[str, float] = {}
        for path in glob.glob(self.pattern):
            try:
                if now - os.path.getmtime(path) > self.max_age_s:
                    continue
            except OSError:
                continue
            # newest full-length window only: an older one would keep a stopped topic "alive"
            win = next((w for w in reversed(_last_windows(path))
                        if float(w.get('window_s') or 0.0) >= _MIN_WINDOW_S), None)
            for t in (win or {}).get('topics') or []:
                name = t.get('topic')
                if not name:
                    continue
                hz = max(float(t.get('pub_inter_hz') or 0.0), float(t.get('pub_intra_hz') or 0.0),
                         float(t.get('recv_inter_hz') or 0.0), float(t.get('recv_intra_hz') or 0.0))
                rates[name] = max(rates.get(name, 0.0), hz)
        return rates

    def active(self, now: float | None = None) -> bool:
        return bool(self.read(now))
