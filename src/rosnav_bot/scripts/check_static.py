#!/usr/bin/env python3
"""check_static - fast checks that catch a broken robot/sim before any simulator starts.

  src/rosnav_bot/scripts/check_static.py          # all checks
  src/rosnav_bot/scripts/check_static.py --only urdf yaml

Checks (each prints OK/FAIL/SKIP, exit code 1 on any FAIL):
  python  every script and launch file compiles
  yaml    every config/*.yaml and maps/*.yaml parses
  urdf    every urdf/*.urdf.xacro renders with xacro, is well-formed, and every joint
          references links that exist, with exactly one root link
  worlds  every worlds/*.world and *.sdf is well-formed XML
  launch  every launch file imports and builds a LaunchDescription (skipped without launch_ros)
"""
import argparse
import importlib.util
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
results = []


def record(group, name, status, detail=''):
    results.append((group, name, status, detail))
    mark = {'OK': 'ok  ', 'FAIL': 'FAIL', 'SKIP': 'skip'}[status]
    if status != 'OK' or os.environ.get('VERBOSE'):
        print(f'[{mark}] {group}: {name} {detail}')


def files(sub, *exts):
    d = os.path.join(PKG, sub)
    if not os.path.isdir(d):
        return []
    return sorted(os.path.join(d, f) for f in os.listdir(d) if f.endswith(exts))


def check_python():
    for path in files('scripts', '.py') + files('launch', '.py'):
        try:
            with open(path, 'rb') as f:
                compile(f.read(), path, 'exec')      # syntax check only, writes no .pyc
            record('python', os.path.basename(path), 'OK')
        except SyntaxError as e:
            record('python', os.path.basename(path), 'FAIL', f'line {e.lineno}: {e.msg}')


def check_yaml():
    try:
        import yaml
    except ImportError:
        record('yaml', 'pyyaml', 'SKIP', 'python3-yaml not installed')
        return
    for path in files('config', '.yaml', '.yml') + files('maps', '.yaml'):
        try:
            with open(path) as f:
                yaml.safe_load(f)
            record('yaml', os.path.basename(path), 'OK')
        except Exception as e:  # noqa: BLE001
            record('yaml', os.path.basename(path), 'FAIL', str(e).splitlines()[0][:120])


def check_urdf():
    for path in files('urdf', '.urdf.xacro'):
        name = os.path.basename(path)
        try:
            out = subprocess.run(['xacro', path], capture_output=True, text=True, timeout=60)
        except FileNotFoundError:
            record('urdf', name, 'SKIP', 'xacro not installed')
            continue
        if out.returncode != 0:
            record('urdf', name, 'FAIL', out.stderr.strip().splitlines()[-1][:120] if out.stderr.strip() else 'xacro failed')
            continue
        try:
            root = ET.fromstring(out.stdout)
        except ET.ParseError as e:
            record('urdf', name, 'FAIL', f'not well-formed: {e}')
            continue
        links = {l.get('name') for l in root.iter('link')}
        problems, children = [], set()
        for j in root.iter('joint'):
            p, c = j.find('parent'), j.find('child')
            if p is None or c is None:
                problems.append(f'joint {j.get("name")} missing parent/child')
                continue
            for end in (p.get('link'), c.get('link')):
                if end not in links:
                    problems.append(f'joint {j.get("name")} references missing link {end}')
            children.add(c.get('link'))
        roots = links - children
        if len(roots) != 1:
            problems.append(f'expected 1 root link, found {sorted(roots)}')
        if problems:
            record('urdf', name, 'FAIL', '; '.join(problems)[:200])
        else:
            record('urdf', name, 'OK', f'({len(links)} links)')


def check_worlds():
    for path in files('worlds', '.world', '.sdf'):
        try:
            ET.parse(path)
            record('worlds', os.path.basename(path), 'OK')
        except ET.ParseError as e:
            record('worlds', os.path.basename(path), 'FAIL', str(e))


def check_launch():
    try:
        import launch_ros  # noqa: F401
    except ImportError:
        record('launch', 'launch_ros', 'SKIP', 'ros-<distro>-launch-ros not installed')
        return
    sys.path.insert(0, os.path.join(PKG, 'launch'))
    for path in files('launch', '.py'):
        name = os.path.basename(path)
        if name.startswith('_'):
            continue
        try:
            spec = importlib.util.spec_from_file_location('chk_' + name.replace('.', '_'), path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            gld = getattr(mod, 'generate_launch_description', None)
            if gld is None:
                record('launch', name, 'FAIL', 'no generate_launch_description()')
                continue
            gld()
            record('launch', name, 'OK')
        except Exception as e:  # noqa: BLE001
            msg = f'{type(e).__name__}: {str(e)[:100]}'
            # a dependency that is simply not installed (slam_toolbox, nav2_common, ...) is an
            # environment gap, not a launch-file bug: skip it, but fail on every other error
            if type(e).__name__ == 'PackageNotFoundError' or (
                    isinstance(e, ModuleNotFoundError) and not str(getattr(e, 'name', '')).startswith('rosnav')):
                record('launch', name, 'SKIP', f'missing dependency ({msg})')
            else:
                record('launch', name, 'FAIL', msg)


CHECKS = {'python': check_python, 'yaml': check_yaml, 'urdf': check_urdf,
          'worlds': check_worlds, 'launch': check_launch}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--only', nargs='*', choices=sorted(CHECKS), help='run only these groups')
    args = ap.parse_args()
    for g in (args.only or CHECKS):
        CHECKS[g]()
    for g in (args.only or CHECKS):
        sel = [r for r in results if r[0] == g]
        print(f'{g:7s} {sum(r[2] == "OK" for r in sel):3d} ok  {sum(r[2] == "FAIL" for r in sel):3d} fail  '
              f'{sum(r[2] == "SKIP" for r in sel):3d} skip')
    failed = [r for r in results if r[2] == 'FAIL']
    print('STATIC CHECKS PASSED' if not failed else f'STATIC CHECKS FAILED ({len(failed)})')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
