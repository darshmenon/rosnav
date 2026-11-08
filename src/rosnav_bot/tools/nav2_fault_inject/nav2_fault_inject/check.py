"""`expect:` assertions: did nav2_doctor notice each fault, and did the stack recover?

Reports come from `nav2_doctor --json` (one object per line: {time, ok, findings[{level, code, msg}]}).
"""
import json
import shlex
import subprocess
import sys
import threading
import time

DEFAULT_WITHIN = 8.0          # seconds after the pause in which the expected codes must show up
DEFAULT_CLEAR_WITHIN = 15.0   # seconds after the resume in which the doctor must report no ERROR


def normalize(expect):
    """expect mapping -> {detect: [codes], within, clear, clear_within, forbid: [codes]}."""
    e = dict(expect or {})
    unknown = set(e) - {'detect', 'within', 'clear', 'clear_within', 'forbid'}
    if unknown:
        raise ValueError(f'unknown expect key(s) {sorted(unknown)}; use detect, within, clear, clear_within, forbid')

    def codes(v):
        v = [] if v is None else ([v] if isinstance(v, str) else list(v))
        if not all(isinstance(c, str) for c in v):
            raise ValueError(f'codes must be strings: {v!r}')
        return v
    try:
        within = float(e.get('within', DEFAULT_WITHIN))
        clear_within = float(e.get('clear_within', DEFAULT_CLEAR_WITHIN))
    except (TypeError, ValueError) as err:
        raise ValueError(f'within / clear_within must be numbers: {expect!r}') from err
    return {'detect': codes(e.get('detect')), 'within': within, 'clear': bool(e.get('clear', True)),
            'clear_within': clear_within, 'forbid': codes(e.get('forbid'))}


def evaluate(fault, pause_ts, resume_ts, reports):
    """Check one fault. reports = [{'time', 'ok', 'codes'}]. Returns {node, passed, checks[{name, ok, detail}]}."""
    x = normalize(fault.get('expect'))
    checks = []
    if pause_ts is None or resume_ts is None:
        return {'node': fault['node'], 'passed': False,
                'checks': [{'name': 'run', 'ok': False, 'detail': 'fault was never injected / resumed'}]}
    win = [r for r in reports if pause_ts <= r['time'] <= pause_ts + x['within']]
    for code in x['detect']:
        hit = next((r for r in win if code in r['codes']), None)
        checks.append({'name': f'detect {code}', 'ok': hit is not None,
                       'detail': (f'seen {hit["time"] - pause_ts:.1f}s after the pause' if hit
                                  else f'not seen within {x["within"]:.0f}s of the pause')})
    if x['clear']:
        ok = next((r for r in reports if resume_ts <= r['time'] <= resume_ts + x['clear_within'] and r['ok']), None)
        checks.append({'name': 'clear', 'ok': ok is not None,
                       'detail': (f'healthy {ok["time"] - resume_ts:.1f}s after the resume' if ok
                                  else f'no ERROR-free report within {x["clear_within"]:.0f}s of the resume')})
    span = [r for r in reports if pause_ts <= r['time'] <= resume_ts + x['clear_within']]
    for code in x['forbid']:
        bad = next((r for r in span if code in r['codes']), None)
        checks.append({'name': f'forbid {code}', 'ok': bad is None,
                       'detail': 'never seen' if bad is None else f'seen {bad["time"] - pause_ts:.1f}s after the pause'})
    return {'node': fault['node'], 'passed': all(c['ok'] for c in checks), 'checks': checks}


def parse_report(line):
    """One doctor JSON line -> {'time', 'ok', 'codes'} or None for anything that is not a report."""
    line = line.strip()
    if not line.startswith('{'):
        return None
    try:
        d = json.loads(line)
        return {'time': float(d['time']), 'ok': bool(d['ok']), 'codes': {f['code'] for f in d['findings']}}
    except (ValueError, KeyError, TypeError):
        return None


class DoctorWatcher:
    """Runs a doctor command and collects its JSON reports on a thread."""

    def __init__(self, cmd=None, ns=''):
        if cmd:
            self.cmd = shlex.split(cmd)
        else:
            self.cmd = [sys.executable, '-m', 'nav2_doctor.doctor', '--json', '--period', '1', '--startup-grace', '0']
            if ns:
                self.cmd += ['--ns', ns]
        self.reports, self._proc, self._thr, self.stderr = [], None, None, ''

    def start(self):
        self._proc = subprocess.Popen(self.cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self._thr = threading.Thread(target=self._read, daemon=True)
        self._thr.start()

    def _read(self):
        for line in self._proc.stdout:
            r = parse_report(line)
            if r:
                self.reports.append(r)

    def stop(self):
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        if self._thr:
            self._thr.join(2)
        try:
            self.stderr = self._proc.stderr.read()[-400:] if self._proc else ''
        except (OSError, ValueError):
            pass

    def wait_for_first(self, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.reports:
                return True
            if self._proc.poll() is not None:
                return bool(self.reports)
            time.sleep(0.1)
        return bool(self.reports)


def format_verdicts(verdicts, faults):
    lines = []
    for v, f in zip(verdicts, faults):
        lines.append(f'{"PASS" if v["passed"] else "FAIL"}  {f["node"]} @{f["at"]:g}s for {f["for"]:g}s')
        for c in v['checks']:
            lines.append(f'       {"ok  " if c["ok"] else "FAIL"} {c["name"]}: {c["detail"]}')
    return '\n'.join(lines)
