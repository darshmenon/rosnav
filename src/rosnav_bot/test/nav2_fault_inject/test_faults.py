import os, signal, subprocess, sys, time
import pytest
ROOT = os.path.join(os.path.dirname(__file__), '..')
sys.path.insert(0, ROOT)
from nav2_fault_inject import faults
from nav2_fault_inject.faults import Injector, Proc, find, hang, list_procs, parse_scenario, run_scenario
_DOM = 200 + os.getpid() % 25   # per-process ROS domain: never collides with a sim on a shared machine
os.environ['ROS_LOCALHOST_ONLY'] = '1'


def fake_proc(root, pid, args, domain='5'):
    d = root / str(pid); d.mkdir()
    (d / 'cmdline').write_bytes(b'\0'.join(a.encode() for a in args) + b'\0')
    (d / 'environ').write_bytes(f'ROS_DOMAIN_ID={domain}\0HOME=/x\0'.encode())


@pytest.fixture
def proc(tmp_path):
    fake_proc(tmp_path, 101, ['/usr/bin/x', '--ros-args', '-r', '__node:=controller_server'])
    fake_proc(tmp_path, 102, ['/usr/bin/x', '--ros-args', '-r', '__node:=bridge'])
    fake_proc(tmp_path, 103, ['/usr/bin/x', '--ros-args', '-r', '__node:=controller_server'], domain='9')
    fake_proc(tmp_path, 104, ['/usr/bin/bash', '-c', 'sleep 9'])                  # not a ROS node
    fake_proc(tmp_path, 105, ['/opt/ros/humble/lib/nav2_controller/planner_server', '--ros-args', '--params-file', '/t'])  # default name
    fake_proc(tmp_path, 106, ['/usr/bin/planner_server', '--other'])      # same basename, not a ROS node
    (tmp_path / 'self').mkdir()                                                    # non-numeric entry
    return str(tmp_path)


def test_list_only_our_domain(proc):
    assert [(p.pid, p.node) for p in list_procs('5', proc)] == [(102, 'bridge'), (101, 'controller_server'), (105, 'planner_server')]


def test_other_domain_never_returned(proc):
    assert [p.pid for p in find('controller_server', '9', proc)] == [103]
    assert 103 not in [p.pid for p in find('controller_server', '5', proc)]


def test_find_strips_leading_slash_and_is_exact(proc):
    assert [p.pid for p in find('/controller_server', '5', proc)] == [101]
    assert find('controller', '5', proc) == []                     # substring must not match


def test_never_lists_itself(tmp_path):
    fake_proc(tmp_path, os.getpid(), ['x', '__node:=me'], domain='5')
    assert list_procs('5', str(tmp_path)) == []


def test_pause_resume_signals_and_idempotence():
    calls = []
    inj = Injector(kill=lambda pid, sig: calls.append((pid, sig)), log=lambda *_: None)
    p = Proc(7, 'n', '5')
    inj.pause([p]); inj.pause([p]); inj.resume([p]); inj.resume([p])
    assert calls == [(7, signal.SIGSTOP), (7, signal.SIGCONT)]
    assert [e['action'] for e in inj.timeline] == ['PAUSE', 'RESUME']


def test_close_resumes_everything_paused():
    calls = []
    inj = Injector(kill=lambda pid, sig: calls.append((pid, sig)), log=lambda *_: None)
    inj.pause([Proc(1, 'a', ''), Proc(2, 'b', '')]); inj.close(); inj.close()
    assert sorted(c for c in calls if c[1] == signal.SIGCONT) == [(1, signal.SIGCONT), (2, signal.SIGCONT)]
    assert not inj.paused


def test_resume_survives_dead_process():
    def kill(pid, sig):
        if sig == signal.SIGCONT:
            raise ProcessLookupError
    inj = Injector(kill=kill, log=lambda *_: None)
    inj.pause([Proc(1, 'a', '')]); inj.close()
    assert not inj.paused


def test_dry_run_sends_nothing():
    calls = []
    inj = Injector(kill=lambda *a: calls.append(a), log=lambda *_: None, dry_run=True)
    inj.pause([Proc(1, 'a', '')]); inj.close()
    assert calls == [] and len(inj.timeline) == 2


def test_hang_unknown_node_raises(proc):
    with pytest.raises(LookupError, match='no process for node "nope"'):
        hang('nope', 1, '5', proc_root=proc)


def test_hang_resumes_even_if_sleep_is_interrupted(proc):
    calls = []
    inj = Injector(kill=lambda pid, sig: calls.append((pid, sig)), log=lambda *_: None)

    def boom(_):
        raise KeyboardInterrupt
    with pytest.raises(KeyboardInterrupt):
        hang('bridge', 5, '5', inj=inj, sleep=boom, proc_root=proc)
    assert (102, signal.SIGCONT) in calls


def test_parse_scenario_sorts_and_validates():
    assert parse_scenario('faults:\n - {at: 9, node: b, for: 2}\n - {at: 1, node: a, for: 3}\n') == [(1.0, 'a', 3.0), (9.0, 'b', 2.0)]
    for bad in ('- {at: 1, node: a}', '- {at: -1, node: a, for: 2}', '- {at: 1, node: a, for: 0}', '- {at: x, node: a, for: 1}'):
        with pytest.raises(ValueError):
            parse_scenario(bad)


def test_scenario_order_and_overlap(proc):
    calls, clock = [], [0.0]
    inj = Injector(kill=lambda pid, sig: calls.append((pid, sig.name)), log=lambda *_: None)
    sleeps = []

    def sleep(d):
        sleeps.append(d); clock[0] += d
    run_scenario([(1.0, 'bridge', 4.0), (2.0, 'controller_server', 1.0)], '5', inj, now=lambda: clock[0], sleep=sleep, proc_root=proc)
    assert calls == [(102, 'SIGSTOP'), (101, 'SIGSTOP'), (101, 'SIGCONT'), (102, 'SIGCONT')]
    assert sum(sleeps) == pytest.approx(5.0)


def test_scenario_aborts_before_injecting_if_node_missing(proc):
    calls = []
    inj = Injector(kill=lambda *a: calls.append(a), log=lambda *_: None)
    with pytest.raises(LookupError):
        run_scenario([(0, 'bridge', 1), (0, 'ghost', 1)], '5', inj, proc_root=proc)
    assert calls == []


def test_scenario_cleans_up_on_error(proc):
    calls = []
    inj = Injector(kill=lambda pid, sig: calls.append((pid, sig.name)), log=lambda *_: None)

    def sleep(d):
        if d > 0 and any(s == 'SIGSTOP' for _, s in calls):
            raise RuntimeError('boom')
    with pytest.raises(RuntimeError):
        run_scenario([(0, 'bridge', 5)], '5', inj, now=lambda: 0.0, sleep=sleep, proc_root=proc)
    assert calls[-1] == (102, 'SIGCONT')


# ---- real process: actually freezes and thaws -------------------------------
def state(pid):
    return open(f'/proc/{pid}/stat').read().rsplit(')', 1)[1].split()[0]


def test_real_process_is_frozen_then_resumed(tmp_path):
    out = tmp_path / 'tick'
    code = f"import time\nwhile True:\n    open({str(out)!r},'a').write('x'); time.sleep(0.05)"
    env = dict(os.environ, ROS_DOMAIN_ID=str(_DOM + 2))
    child = subprocess.Popen([sys.executable, '-c', code, '--ros-args', '-r', '__node:=victim'], env=env)
    try:
        time.sleep(0.5)
        os.environ['ROS_DOMAIN_ID'] = str(_DOM + 2)
        cli = subprocess.Popen([sys.executable, '-m', 'nav2_fault_inject.cli', 'hang', 'victim', '--for', '1.5'],
                               cwd=ROOT, env=env, stdout=subprocess.PIPE, text=True)
        time.sleep(0.7)
        assert state(child.pid) == 'T'                       # stopped
        n1 = out.stat().st_size; time.sleep(0.4)
        assert out.stat().st_size == n1                      # no progress while frozen
        cli.wait(timeout=10)
        time.sleep(0.4)
        assert state(child.pid) in 'SR' and out.stat().st_size > n1   # thawed, working again
        assert 'PAUSE' in cli.stdout.read() and cli.returncode == 0
    finally:
        child.kill(); child.wait()


def test_cli_ctrl_c_resumes(tmp_path):
    env = dict(os.environ, ROS_DOMAIN_ID=str(_DOM + 3))
    child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)', '__node:=victim2'], env=env)
    try:
        time.sleep(0.4)
        cli = subprocess.Popen([sys.executable, '-m', 'nav2_fault_inject.cli', 'hang', 'victim2', '--for', '30'], cwd=ROOT, env=env)
        time.sleep(0.8)
        assert state(child.pid) == 'T'
        cli.send_signal(signal.SIGINT); cli.wait(timeout=10)
        time.sleep(0.3)
        assert state(child.pid) != 'T'                       # never left frozen
    finally:
        child.kill(); child.wait()


def test_cli_unknown_node_exit_2():
    p = subprocess.run([sys.executable, '-m', 'nav2_fault_inject.cli', 'hang', 'nonexistent_node', '--for', '1'],
                       cwd=ROOT, capture_output=True, text=True, env=dict(os.environ, ROS_DOMAIN_ID=str(_DOM + 4)))
    assert p.returncode == 2 and 'no process for node' in p.stderr


def test_version():
    p = subprocess.run([sys.executable, '-m', 'nav2_fault_inject.cli', '--version'], cwd=ROOT, capture_output=True, text=True)
    assert p.stdout.startswith('nav2_fault_inject ')


def test_versions_match():
    import re
    from nav2_fault_inject import __version__
    pkg = re.search(r'<version>(.*?)</version>', open(os.path.join(ROOT, 'package.xml')).read()).group(1)
    setup = re.search(r"version='(.*?)'", open(os.path.join(ROOT, 'setup.py')).read()).group(1)
    assert __version__ == pkg == setup


def test_default_named_node_found_by_exe(proc):
    assert [p.pid for p in find('planner_server', '5', proc)] == [105]


def test_non_ros_process_with_same_basename_never_matched(proc):
    assert 106 not in [p.pid for p in list_procs('5', proc)]
    assert 106 not in [p.pid for p in find('planner_server', '5', proc)]


def test_exe_alias_matches_renamed_node(tmp_path):
    fake_proc(tmp_path, 201, ['/x/parameter_bridge', '--ros-args', '-r', '__node:=ros_gz_bridge'])
    assert [p.pid for p in find('parameter_bridge', '5', str(tmp_path))] == [201]
    assert [p.pid for p in find('ros_gz_bridge', '5', str(tmp_path))] == [201]


def test_log_lines_carry_wall_clock_time():
    lines = []
    inj = Injector(kill=lambda *a: None, log=lines.append)
    inj.pause([Proc(1, 'a', '')]); inj.close()
    import re
    assert all(re.match(r'\d\d:\d\d:\d\d \[ *[\d.]+s\] (PAUSE|RESUME)', l) for l in lines) and len(lines) == 2
    assert all(re.fullmatch(r'\d\d:\d\d:\d\d', e['wall']) for e in inj.timeline)
