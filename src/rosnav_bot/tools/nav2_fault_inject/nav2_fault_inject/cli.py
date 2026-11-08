#!/usr/bin/env python3
"""nav2_fault_inject - pause nodes of a running ROS 2 stack to test that it notices and recovers."""
import argparse
import json
import signal
import sys
import time

from . import __version__
from .check import DoctorWatcher, evaluate, format_verdicts, normalize
from .faults import Injector, hang, list_procs, parse_scenario_full, run_scenario


def main(argv=None):
    ap = argparse.ArgumentParser(prog='nav2_fault_inject', description=__doc__)
    ap.add_argument('--version', action='version', version=f'nav2_fault_inject {__version__}')
    ap.add_argument('--domain', help='ROS_DOMAIN_ID to act in (default: this shell\'s)')
    ap.add_argument('--dry-run', action='store_true', help='show what would be paused, touch nothing')
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('list', help='node processes that can be targeted')
    h = sub.add_parser('hang', help='pause one node for N seconds, then resume it')
    h.add_argument('node'); h.add_argument('--for', dest='secs', type=float, required=True)
    r = sub.add_parser('run', help='run a timed scenario file')
    r.add_argument('scenario'); r.add_argument('--timeline', help='write the timeline as JSON')
    r.add_argument('--check', action='store_true', help='run nav2_doctor alongside and assert each fault\'s `expect:`; exit 1 on any FAIL')
    r.add_argument('--doctor-cmd', help='command emitting nav2_doctor --json lines (default: python -m nav2_doctor.doctor)')
    r.add_argument('--ns', default='', help='namespace passed to the doctor')
    a = ap.parse_args(argv)

    if a.cmd == 'list':
        procs = list_procs(a.domain)
        for p in procs:
            print(f'{p.pid:8d}  {p.node}' + (f'  (exe {p.exe})' if p.exe and p.exe != p.node else ''))
        print(f'{len(procs)} node process(es) in ROS_DOMAIN_ID={p.domain if procs else a.domain or "(unset)"}')
        return 0

    inj = Injector(dry_run=a.dry_run)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        if a.cmd == 'hang':
            hang(a.node, a.secs, a.domain, inj)
        else:
            faults = parse_scenario_full(open(a.scenario).read())
            for f in faults:
                normalize(f['expect'])                       # fail on bad expect before injecting
            tuples = [(f['at'], f['node'], f['for']) for f in faults]
            if not a.check:
                tl = run_scenario(tuples, a.domain, inj)
            else:
                import os
                if a.domain is not None:
                    os.environ['ROS_DOMAIN_ID'] = str(a.domain)
                w, marks = DoctorWatcher(a.doctor_cmd, a.ns), []
                w.start()
                try:
                    if not w.wait_for_first(20):
                        w.stop()
                        print(f'error: doctor produced no report ({" ".join(w.cmd[:3])}...): {w.stderr.strip() or "no output"}\n'
                              'install nav2_doctor (pip install nav2_doctor / colcon) or pass --doctor-cmd', file=sys.stderr)
                        return 2
                    tl = run_scenario(tuples, a.domain, inj, marks=marks)
                    time.sleep(max([normalize(f['expect'])['clear_within'] for f in faults] + [0]) + 1)
                finally:
                    w.stop()
                verdicts = [evaluate(f, m.get('pause_ts'), m.get('resume_ts'), w.reports) for f, m in zip(faults, marks)]
                print(format_verdicts(verdicts, faults))
                rc = 0 if all(v['passed'] for v in verdicts) else 1
            if a.timeline:
                json.dump(tl, open(a.timeline, 'w'), indent=2)
            if a.check:
                return rc
    except LookupError as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    except ValueError as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print('interrupted - resuming everything', file=sys.stderr)
        return 130
    finally:
        inj.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
