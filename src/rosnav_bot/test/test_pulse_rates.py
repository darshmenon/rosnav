#!/usr/bin/env python3
"""scripts/pulse_rates.py: tail ros2_pulse jsonl logs, no ROS and no ros2_pulse needed."""
import importlib.util
import json
import os
import sys
import time

_SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'pulse_rates.py')
_spec = importlib.util.spec_from_file_location('pulse_rates', _SCRIPT)
pulse_rates = importlib.util.module_from_spec(_spec)
sys.modules['pulse_rates'] = pulse_rates
_spec.loader.exec_module(pulse_rates)


def _win(topics, window_s=5.0):
    return json.dumps({'ts_ns': '1', 'window_s': window_s, 'topics': topics, 'nodes': [], 'warns': []})


def _log(tmp_path, name, *lines, age=0.0):
    p = tmp_path / name
    p.write_text('\n'.join(lines) + '\n')
    if age:
        t = time.time() - age
        os.utime(p, (t, t))
    return p


def _reader(tmp_path, **kw):
    return pulse_rates.PulseRates(str(tmp_path / 'topic_freq.*.log'), **kw)


def test_no_logs_means_empty_so_caller_falls_back(tmp_path):
    r = _reader(tmp_path)
    assert r.read() == {}
    assert not r.active()


def test_reads_pub_and_recv_rates(tmp_path):
    _log(tmp_path, 'topic_freq.1.log',
         _win([{'topic': '/robot1/odom', 'pub_inter_hz': 49.6}, {'topic': '/robot1/scan', 'recv_inter_hz': 10.0}]))
    rates = _reader(tmp_path).read()
    assert rates['/robot1/odom'] == 49.6 and rates['/robot1/scan'] == 10.0


def test_largest_rate_across_processes_wins(tmp_path):
    _log(tmp_path, 'topic_freq.1.log', _win([{'topic': '/robot1/scan', 'pub_inter_hz': 10.0}]))
    _log(tmp_path, 'topic_freq.2.log', _win([{'topic': '/robot1/scan', 'recv_inter_hz': 17.5}]))
    assert _reader(tmp_path).read()['/robot1/scan'] == 17.5


def test_newest_window_wins_and_stopped_topic_disappears(tmp_path):
    _log(tmp_path, 'topic_freq.1.log',
         _win([{'topic': '/robot1/odom', 'pub_inter_hz': 50.0}]),
         _win([{'topic': '/robot1/scan', 'pub_inter_hz': 10.0}]))
    rates = _reader(tmp_path).read()
    assert '/robot1/odom' not in rates                 # odom stopped: absent from the newest window
    assert rates['/robot1/scan'] == 10.0


def test_flush_sliver_window_is_ignored(tmp_path):
    _log(tmp_path, 'topic_freq.1.log',
         _win([{'topic': '/robot1/odom', 'pub_inter_hz': 50.0}], window_s=5.0),
         _win([{'topic': '/robot1/odom', 'pub_inter_hz': 3.0}], window_s=0.1))
    assert _reader(tmp_path).read()['/robot1/odom'] == 50.0


def test_log_of_dead_process_ages_out(tmp_path):
    _log(tmp_path, 'topic_freq.1.log', _win([{'topic': '/robot1/odom', 'pub_inter_hz': 50.0}]), age=60.0)
    assert _reader(tmp_path, max_age_s=15.0).read() == {}


def test_text_format_and_garbage_lines_are_skipped(tmp_path):
    _log(tmp_path, 'topic_freq.1.log', 'TOPIC /scan 20.000000', 'not json {', '{"cut": ')
    assert _reader(tmp_path).read() == {}


def test_only_the_tail_of_a_big_log_is_read(tmp_path):
    big = [_win([{'topic': '/old', 'pub_inter_hz': 1.0}])] * 5000
    _log(tmp_path, 'topic_freq.1.log', *big, _win([{'topic': '/robot1/odom', 'pub_inter_hz': 50.0}]))
    assert _reader(tmp_path).read() == {'/robot1/odom': 50.0}
