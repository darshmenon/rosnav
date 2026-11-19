"""rosnav_check CLI: preflight | merge | eval. Degrades gracefully when a sibling tool is missing."""
import argparse
import contextlib
import io
import json
import os
import subprocess
import sys

from . import __version__
from .core import Item, load, normalize, render, report

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS_DIR = os.path.dirname(os.path.dirname(HERE))      # .../tools
FUSION_EVAL = os.path.expanduser('~/fusion_tools/eval_traj.sh')
EXTRA_PATHS = [TOOLS_DIR + '/lio_doctor', TOOLS_DIR + '/slam_doctor', TOOLS_DIR + '/nav2_doctor', os.path.expanduser('~/nav2_doctor')]
LIVE = {'lio_doctor': 'lio_doctor.doctor', 'slam_doctor': 'slam_doctor.doctor', 'nav2_doctor': 'nav2_doctor.doctor'}


def tool_path(tool):
    """Directory that makes `import <tool>` work, or None."""
    for d in EXTRA_PATHS:
        if os.path.isdir(os.path.join(d, tool)):
            return d
    return None


def run_lio_cloud(spec):
    """Offline: lio_doctor --check-cloud spec --json, in-process. Returns ([Item], note)."""
    d = tool_path('lio_doctor')
    if not d:
        return [], 'lio_doctor not found'
    sys.path.append(d)
    try:
        from lio_doctor.doctor import check_cloud_main
        from lio_doctor.core import HINTS
    except Exception as e:   # noqa: BLE001 - any import failure means "tool unavailable"
        return [], f'lio_doctor import failed: {e}'
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        check_cloud_main(spec, True)
    doc = json.loads(buf.getvalue().strip().splitlines()[-1])
    items = normalize(doc, 'lio_doctor')
    for i in items:
        i.hint = i.hint or HINTS.get(i.code, '')
    for name, r in doc.get('readiness', {}).items():
        if r['verdict'] != 'READY':
            items.append(Item('lio_doctor', f'READY_{name}', 'ERROR' if r['verdict'] == 'BLOCKED' else 'WARN',
                              f"{name} {r['verdict']}: {'; '.join(map(str, r['reasons']))}"))
    return items, ''


def run_live(tool, secs, extra=()):
    """Live (needs ROS + running stack): `python -m <tool>.doctor --once N --json`; take the last JSON line."""
    d = tool_path(tool)
    if not d:
        return [], f'{tool} not found'
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([d, os.environ.get('PYTHONPATH', '')]))
    flag = ['--duration', str(secs), '--once'] if tool == 'lio_doctor' else ['--once', str(secs)]
    try:
        p = subprocess.run([sys.executable, '-m', LIVE[tool], *flag, '--json', *extra], env=env, capture_output=True, text=True,
                           timeout=secs + 30)
    except (OSError, subprocess.TimeoutExpired) as e:
        return [], f'{tool} failed to run: {e}'
    lines = [x for x in p.stdout.splitlines() if x.startswith('{')]
    if not lines:
        return [], f'{tool} produced no JSON (ROS not sourced?): {p.stderr.strip()[-120:]}'
    return normalize(json.loads(lines[-1]), tool), ''


def cmd_preflight(a):
    items, notes = [], []
    if a.cloud_spec:
        got, note = run_lio_cloud(a.cloud_spec)
        items += got
        notes.append(note)
    if a.live:
        for t in ('lio_doctor', 'slam_doctor', 'nav2_doctor'):
            got, note = run_live(t, a.live)
            items += got
            notes.append(note)
    if not a.cloud_spec and not a.live:
        notes.append('nothing to run: pass --cloud-spec spec.json (offline) and/or --live SECS (needs ROS + running stack)')
    for n in filter(None, notes):
        items.append(Item('rosnav_check', 'TOOL_UNAVAILABLE', 'WARN', n))
    return emit(items, a)


def cmd_merge(a):
    items = []
    for p in a.files:
        try:
            items += load(p)
        except (OSError, ValueError) as e:
            items.append(Item('rosnav_check', 'BAD_INPUT', 'ERROR', f'{p}: {e}'))
    return emit(items, a)


def emit(items, a):
    rep = report(items)
    print(json.dumps(rep, indent=2) if a.json else render(rep))
    return rep['exit_code']


def cmd_eval(a):
    if not os.path.isfile(FUSION_EVAL):
        print(f'eval_traj.sh not found at {FUSION_EVAL}')
        return 2
    return subprocess.call(['bash', FUSION_EVAL, a.reference, *a.estimates])


def main(argv=None):
    ap = argparse.ArgumentParser(prog='rosnav_check', description=__doc__)
    ap.add_argument('--version', action='version', version=f'rosnav_check {__version__}')
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('preflight', help='run available doctors and print one merged report')
    p.add_argument('--cloud-spec', help='PointCloud2 layout JSON for lio_doctor --check-cloud (offline)')
    p.add_argument('--live', type=float, metavar='SECS', help='also run each doctor live for SECS (needs ROS)')
    p.add_argument('--json', action='store_true')
    p.set_defaults(fn=cmd_preflight)
    m = sub.add_parser('merge', help='merge findings JSON files (name them lio*/slam*/nav2* or use tool=path)')
    m.add_argument('files', nargs='+')
    m.add_argument('--json', action='store_true')
    m.set_defaults(fn=cmd_merge)
    e = sub.add_parser('eval', help='APE/RPE via ~/fusion_tools/eval_traj.sh (read-only use)')
    e.add_argument('reference')
    e.add_argument('estimates', nargs='+')
    e.set_defaults(fn=cmd_eval)
    a = ap.parse_args(argv)
    return a.fn(a)


if __name__ == '__main__':
    sys.exit(main())
