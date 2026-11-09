"""run --check end to end: a real throw-away victim process plus a stand-in doctor that reports from its state."""
import json
import os
import subprocess
import sys
import time

import nav2_fault_inject

_DOM = 200 + os.getpid() % 25   # per-process ROS domain: never collides with a sim on a shared machine
os.environ['ROS_LOCALHOST_ONLY'] = '1'

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(nav2_fault_inject.__file__)))   # works in the repo and when vendored

DOCTOR = r'''
import json, sys, time
pid = int(sys.argv[1]); code = sys.argv[2]
while True:
    stopped = open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()[0] == "T"
    f = [{"level": "ERROR", "code": code, "msg": "x"}] if stopped else []
    print(json.dumps({"time": time.time(), "ns": "", "ok": not f, "findings": f}), flush=True)
    time.sleep(0.2)
'''


def run_check(tmp_path, domain, expect_code, doc_code='SCAN_STALE'):
    env = dict(os.environ, ROS_DOMAIN_ID=domain, PYTHONPATH=ROOT)
    victim = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)', '--ros-args', '-r', '__node:=victim'], env=env)
    try:
        time.sleep(0.4)
        scen = tmp_path / 's.yaml'
        scen.write_text(f'faults:\n - {{at: 0.5, node: victim, for: 1.5, expect: {{detect: [{expect_code}], within: 3, clear_within: 3}}}}\n')
        doc = tmp_path / 'doc.py'
        doc.write_text(DOCTOR)
        r = subprocess.run([sys.executable, '-m', 'nav2_fault_inject.cli', '--domain', domain, 'run', str(scen), '--check',
                            '--doctor-cmd', f'{sys.executable} {doc} {victim.pid} {doc_code}'],
                           cwd=ROOT, env=env, capture_output=True, text=True, timeout=60)
        return r, victim
    finally:
        victim.kill()
        victim.wait()


def test_check_passes_when_doctor_notices_and_stack_recovers(tmp_path):
    r, _ = run_check(tmp_path, str(_DOM), 'SCAN_STALE')
    assert r.returncode == 0, r.stdout + r.stderr
    assert 'PASS' in r.stdout and 'detect SCAN_STALE' in r.stdout


def test_check_fails_when_expected_code_never_appears(tmp_path):
    r, _ = run_check(tmp_path, str(_DOM + 1), 'ODOM_STALE')
    assert r.returncode == 1, r.stdout + r.stderr
    assert 'FAIL' in r.stdout and 'not seen' in r.stdout


def test_check_rejects_bad_expect_before_injecting(tmp_path):
    scen = tmp_path / 's.yaml'
    scen.write_text('faults:\n - {at: 0, node: x, for: 1, expect: {detects: [A]}}\n')
    r = subprocess.run([sys.executable, '-m', 'nav2_fault_inject.cli', 'run', str(scen), '--check'],
                       cwd=ROOT, env=dict(os.environ, PYTHONPATH=ROOT), capture_output=True, text=True, timeout=30)
    assert r.returncode == 2 and 'unknown expect' in r.stderr
