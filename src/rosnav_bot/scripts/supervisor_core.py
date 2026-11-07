#!/usr/bin/env python3
"""
supervisor_core.py - pure decision logic for rosnav_supervisor.py (no ROS imports, unit-tested).

Idea (from EasyNav's recovery design): every robot's health is a standard diagnostic, and a
*separate* component decides on a mitigation. Here the diagnostic is `rosnav_doctor.diagnose()`
findings; the mitigation policy lives in `Supervisor.step()`.

Mitigations are deliberately mild. A lifecycle RESET+STARTUP of Nav2 deadlocked in a real TF freeze
(see tf_freeze_watchdog.py), so the strongest action here is clearing costmaps. Anything the
supervisor cannot fix safely is reported only.

Actions returned by step(): ('clear_costmaps', robot) | ('log', robot, text)
Every decision (including "held back by cooldown") is recorded in `timeline` so the TUI/RViz can
show *why* the supervisor did or did not act.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

LEVEL_NUM = {'OK': 0, 'WARN': 1, 'ERROR': 2}


@dataclass
class RobotState:
    level: str = 'OK'
    findings: list = field(default_factory=list)
    bad_since: float | None = None       # when the current non-OK streak began
    last_action: float = -1e9
    actions: int = 0


class Supervisor:
    def __init__(self, persist_s: float = 5.0, cooldown_s: float = 30.0, max_actions: int = 8,
                 timeline_len: int = 50):
        self.persist_s = persist_s       # a fault must last this long before acting (ignores blips)
        self.cooldown_s = cooldown_s
        self.max_actions = max_actions   # per robot; 0 = unlimited
        self.robots: dict[str, RobotState] = {}
        self.timeline: deque = deque(maxlen=timeline_len)

    def _note(self, now, robot, text):
        self.timeline.append({'t': round(now, 1), 'robot': robot, 'event': text})

    def step(self, now: float, report: dict) -> list[tuple]:
        """report: {robot: [(level, msg), ...]} (rosnav_doctor.diagnose output). Returns actions."""
        actions: list[tuple] = []
        for robot, findings in report.items():
            st = self.robots.setdefault(robot, RobotState())
            level = max((lv for lv, _ in findings), key=LEVEL_NUM.get, default='OK')
            if level != st.level:
                self._note(now, robot, f'{st.level} -> {level}: ' + '; '.join(m for _, m in findings)[:160])
            st.level, st.findings = level, findings
            if level == 'OK':
                st.bad_since = None
                continue
            if st.bad_since is None:
                st.bad_since = now
            if level != 'ERROR' or now - st.bad_since < self.persist_s:
                continue
            msgs = ' '.join(m for lv, m in findings if lv == 'ERROR')
            # map->odom stale while odometry itself is fine = the frozen-TF symptom: costmaps hold a
            # dead transform. Clearing them is cheap and safe; anything else is reported only.
            if 'map->odom' in msgs and 'odom->base_link' not in msgs and 'Nav2 lifecycle not active' not in msgs:
                if now - st.last_action < self.cooldown_s:
                    continue
                if self.max_actions and st.actions >= self.max_actions:
                    if st.actions == self.max_actions:
                        st.actions += 1
                        self._note(now, robot, f'gave up after {self.max_actions} mitigations')
                    continue
                st.last_action, st.actions = now, st.actions + 1
                self._note(now, robot, f'MITIGATE clear_costmaps (#{st.actions}): map->odom stale '
                                       f'{now - st.bad_since:.0f}s')
                actions.append(('clear_costmaps', robot))
            elif now - st.last_action >= self.cooldown_s:
                st.last_action = now
                self._note(now, robot, 'no safe automatic fix, operator needed: ' + msgs[:120])
                actions.append(('log', robot, msgs))
        return actions

    def snapshot(self) -> dict:
        return {'robots': {r: {'level': s.level, 'findings': s.findings, 'mitigations': s.actions}
                           for r, s in self.robots.items()},
                'timeline': list(self.timeline)}


def to_diagnostic_fields(robot: str, level: str, findings: list, metrics: dict) -> tuple[int, str, list]:
    """(level byte 0/1/2, message, [(key, value)]) for a diagnostic_msgs/DiagnosticStatus."""
    msg = '; '.join(m for _, m in findings) or 'healthy'
    kv = [(k, '' if v is None else str(round(v, 2) if isinstance(v, float) else v)) for k, v in metrics.items()]
    return LEVEL_NUM[level], msg, kv
