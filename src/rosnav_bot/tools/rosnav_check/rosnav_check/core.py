"""ROS-free core: common finding schema, merge, severity sort, exit code, cross-tool root-cause rules."""
import json
import os
from dataclasses import dataclass

SEVERITY = {'ERROR': 3, 'WARN': 2, 'INFO': 1, 'OK': 0}
TOOLS = ('lio_doctor', 'slam_doctor', 'nav2_doctor')


@dataclass
class Item:
    tool: str
    code: str
    severity: str
    msg: str = ''
    hint: str = ''

    def as_dict(self):
        return dict(self.__dict__)


# (rule id, {tool: codes}, need, message). A rule fires when every tool group in `needs` has >=1 matching code
# (a group is a (tool, codes) pair; same tool may appear twice for "A and B" within one tool).
CLOUD_BAD = ('STAMP_AT_END', 'STAMP_UNKNOWN', 'CLOUD_NO_TIME', 'CLOUD_NO_RING', 'CLOUD_FIELD_OVERRUN', 'CLOUD_FIELD_OVERLAP',
             'CLOUD_UNALIGNED', 'CLOUD_STEP_UNALIGNED', 'CLOUD_DATA_SIZE', 'CLOUD_RING_TYPE', 'CLOUD_TIME_TYPE', 'CLOUD_NO_XYZ',
             'CLOUD_ENDIAN')
RULES = [
    ('CLOUD_UPSTREAM', [('lio_doctor', CLOUD_BAD), ('slam_doctor', ('DRIFT_HIGH', 'NO_MAP_ODOM', 'BIG_JUMP', 'JUMP_STORM', 'MAP_STALL'))],
     'likely cloud stamp/field problem upstream of SLAM: fix the PointCloud2 (lio_doctor --check-cloud) before tuning SLAM'),
    ('CLOUD_UPSTREAM_LIO', [('lio_doctor', CLOUD_BAD), ('lio_doctor', ('POSE_JUMP', 'YAW_JUMP', 'DRIFT_HIGH', 'ODOM_NONE', 'STAMP_GAP'))],
     'the cloud problem is the likely cause of the odometry misbehaving: fix the cloud layout/stamp first'),
    ('IMU_CLOCK', [('lio_doctor', ('IMU_TIME_OFFSET',)), ('lio_doctor', ('POSE_JUMP', 'YAW_JUMP', 'DRIFT_HIGH'))],
     'IMU/odometry clock offset is the likely cause of the pose/heading jumps: fix the offset before touching extrinsics'),
    ('IMU_YAW', [('lio_doctor', ('IMU_YAW_OFFSET',)), ('lio_doctor', ('POSE_JUMP', 'YAW_JUMP', 'DRIFT_HIGH'))],
     'constant IMU yaw offset vs odometry is the likely cause of the jumps/divergence: relay /imu with fixed orientation (imu_yaw_relay.py), not extrinsicRPY'),
    ('SLAM_TF_ROOT', [('nav2_doctor', ('TF_FREEZE', 'TF_EDGE_STALE', 'NO_MAP_ODOM')), ('slam_doctor', ('NO_MAP_ODOM', 'TRACKING_LOST'))],
     'Nav2 TF trouble is a symptom: SLAM is not publishing map->odom; fix SLAM first'),
    ('TF_FREEZE_FILTERS', [('nav2_doctor', ('TF_FREEZE',)), ('nav2_doctor', ('FILTER_NO_INFO', 'FILTER_NO_MASK'))],
     'TF freeze with inert costmap filters: filters without info/mask topics stall Nav2 (strip inert filters from the params)'),
    ('MAP_SOURCE_STALL', [('slam_doctor', ('MAP_STALL', 'MAP_NONE')), ('nav2_doctor', ('MAP_STALL', 'COSTMAP_STALE'))],
     'map source stalled: Nav2 is starved by the SLAM map, not misconfigured'),
    ('MAP_JUMPS_HURT_NAV', [('slam_doctor', ('BIG_JUMP', 'JUMP_STORM')), ('nav2_doctor', ('NO_PROGRESS', 'GOAL_FLAPPING', 'RECOVERY_LOOP'))],
     'SLAM map corrections are destabilising Nav2 planning: clear costmaps after jumps and raise loop-closure thresholds'),
    ('CLOCK_MIX', [('lio_doctor', ('STAMP_BACKWARDS',)), ('nav2_doctor', ('CLOCK_SKEW',))],
     'sim-time / wall-time mix across nodes: set use_sim_time consistently'),
]


def normalize(doc, tool=''):
    """Accept a doctor --json object ({'findings': [...]}), a bare list, or lio's offline output; return [Item]."""
    if isinstance(doc, dict):
        tool = doc.get('tool', tool) or tool
        rows = doc.get('findings', [])
    else:
        rows = doc
    out = []
    for r in rows:
        sev = str(r.get('severity', r.get('level', 'INFO'))).upper()
        out.append(Item(r.get('tool', tool) or 'unknown', r.get('code', '?'), sev if sev in SEVERITY else 'INFO',
                        r.get('msg', ''), r.get('hint') or ''))
    return out


def guess_tool(path):
    base = os.path.basename(path).lower()
    for t in TOOLS:
        if t in base or t.split('_')[0] in base:
            return t
    return 'unknown'


def load(path):
    """`tool=path` forces the tool name; otherwise it comes from the JSON 'tool' key, then the file name."""
    tool = ''
    if '=' in path and not os.path.exists(path):
        tool, path = path.split('=', 1)
    with open(path) as fh:
        return normalize(json.load(fh), tool or guess_tool(path))


def merge(items):
    """Drop duplicates, drop OK rows when anything else exists, sort worst first."""
    seen, out = set(), []
    for i in items:
        k = (i.tool, i.code, i.msg)
        if k not in seen:
            seen.add(k)
            out.append(i)
    if any(i.severity != 'OK' for i in out):
        out = [i for i in out if i.severity != 'OK']
    return sorted(out, key=lambda i: (-SEVERITY[i.severity], i.tool, i.code))


def correlate(items):
    """Fire cross-tool rules. Returns [{'rule', 'msg', 'evidence': ['tool:CODE', ...]}] (a rule needs >=1 hit per group)."""
    present = {(i.tool, i.code) for i in items if i.severity in ('ERROR', 'WARN')}
    hits = []
    for rid, groups, msg in RULES:
        ev, ok = [], True
        for tool, codes in groups:
            got = [f'{tool}:{c}' for c in codes if (tool, c) in present]
            if not got:
                ok = False
                break
            ev.extend(g for g in got[:2] if g not in ev)
        if ok:
            hits.append({'rule': rid, 'msg': msg, 'evidence': ev})
    return hits


def exit_code(items):
    worst = max((SEVERITY[i.severity] for i in items), default=0)
    return 2 if worst >= 3 else 1 if worst == 2 else 0


def report(items):
    items = merge(items)
    return {'ok': exit_code(items) == 0, 'exit_code': exit_code(items), 'findings': [i.as_dict() for i in items],
            'root_causes': correlate(items), 'tools': sorted({i.tool for i in items})}


def render(rep):
    lines = []
    for rc in rep['root_causes']:
        lines.append(f"ROOT CAUSE [{rc['rule']}] {rc['msg']}  <- {', '.join(rc['evidence'])}")
    for f in rep['findings']:
        lines.append(f"{f['severity']:<5} {f['tool']}:{f['code']} {f['msg']}" + (f"\n        hint: {f['hint']}" if f['hint'] else ''))
    if not rep['findings']:
        lines.append('OK no findings')
    lines.append(f"exit {rep['exit_code']} ({'clean' if rep['ok'] else 'findings'})")
    return '\n'.join(lines)
