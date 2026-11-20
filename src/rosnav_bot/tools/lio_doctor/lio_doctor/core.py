"""Pure lidar(-inertial) odometry health logic (no ROS): feed samples, ask for findings."""
import math
import struct
from collections import deque
from dataclasses import dataclass, field

ODOM_RATE_FRAC = 0.8        # measured rate below this fraction of the expected rate = ODOM_RATE_LOW
GAP_FACTOR = 3.0            # a stamp step this many times the median step = a stamp gap
JUMP_SLACK_M = 0.05         # pose-step tolerance on top of v_max*dt
RATE_WINDOW_S = 10.0
SILENT_S = 2.0              # odom silent this long = ODOM_NONE / lost
IMU_RATIO_MIN = 5.0         # IMU should run at least this many times faster than the lidar
IMU_HZ_MIN = 50.0
OFFSET_WARN_S = 0.02        # |IMU-vs-odom time offset| above this is worth a warning
OFFSET_CORR_MIN = 0.5
DRIFT_PCT = 5.0             # end error / path length over a window
DRIFT_MIN_PATH_M = 2.0
STAMP_END_TOL = 0.02
YAW_MAX_GAP_S = 0.05        # max stamp gap when matching an IMU sample to an odometry sample
YAW_OFFSET_DEG = 5.0        # |mean(imu_yaw - odom_yaw)| above this = IMU_YAW_OFFSET
YAW_NOISE_DEG = 5.0         # std of the same above this = IMU_YAW_NOISY
YAW_MIN_MATCHES = 10

# sensor_msgs/PointField datatype -> (size, struct code)
DTYPES = {1: (1, 'b'), 2: (1, 'B'), 3: (2, 'h'), 4: (2, 'H'), 5: (4, 'i'), 6: (4, 'I'), 7: (4, 'f'), 8: (8, 'd')}
U16, F32 = 4, 7
TIME_NAMES = ('time', 't', 'timestamp', 'time_offset')


@dataclass
class Finding:
    level: str   # ERROR / WARN / INFO / OK
    code: str
    msg: str


@dataclass
class CloudInfo:
    """The layout of one PointCloud2, without the payload. fields = [(name, offset, datatype, count), ...]."""
    fields: list
    point_step: int
    width: int = 0
    height: int = 1
    row_step: int = 0
    data_len: int = 0
    is_bigendian: bool = False
    stamp: float = 0.0

    @classmethod
    def from_dict(cls, d):
        return cls(fields=[tuple(f) for f in d['fields']], point_step=d['point_step'], width=d.get('width', 0),
                   height=d.get('height', 1), row_step=d.get('row_step', 0), data_len=d.get('data_len', 0),
                   is_bigendian=d.get('is_bigendian', False), stamp=d.get('stamp', 0.0))


HINTS = {
    'CLOUD_STALE': 'the cloud stopped: check the lidar bridge and any adapter node (lidar_ring_time_adapter) between it and the estimator',
    'IMU_STALE': 'the IMU stopped: check the IMU bridge and any relay node (imu_yaw_relay) the estimator reads through',
    'ODOM_NONE': 'no odometry: is the LIO node running, and is its topic/QoS (best-effort?) what you subscribed to?',
    'ODOM_RATE_LOW': 'odometry slower than the sensor: CPU starved or scans dropped; lower the voxel size or the cloud rate',
    'STAMP_BACKWARDS': 'odometry stamps go back in time: two publishers, or a sim-time / wall-time mix',
    'STAMP_GAP': 'holes in the odometry stamps: the estimator dropped scans (bad per-point time, waiting for IMU, or overload)',
    'POSE_JUMP': 'pose stepped faster than the robot can move: bad scan match or IMU time offset; check deskew and lidar-IMU extrinsics',
    'YAW_JUMP': 'heading stepped faster than the robot can turn: wrong IMU orientation or gyro axis/sign',
    'CLOUD_NONE': 'no PointCloud2 seen: check the topic name and QoS',
    'CLOUD_NO_XYZ': 'cloud lacks float32 x/y/z: every estimator needs them',
    'CLOUD_FIELD_OVERRUN': 'a field ends past point_step: the writer used a larger record than it declared; fix point_step or the offsets',
    'CLOUD_FIELD_OVERLAP': 'two fields share bytes: fix the field offsets in the publisher',
    'CLOUD_UNALIGNED': 'a field offset is not a multiple of its size: pad so each field is naturally aligned (LIO-SAM reads garbage otherwise)',
    'CLOUD_STEP_UNALIGNED': 'point_step is not a multiple of 4: pad the record to an aligned size (e.g. 24)',
    'CLOUD_DATA_SIZE': 'data length != row_step*height (or row_step != width*point_step): record size written differs from point_step',
    'CLOUD_ENDIAN': 'big-endian cloud: LIO nodes assume little-endian',
    'CLOUD_RING_TYPE': 'ring must be uint16 (LIO-SAM reads it as u16): change the field datatype to UINT16',
    'CLOUD_TIME_TYPE': 'per-point time must be float32 seconds: change the field datatype to FLOAT32',
    'CLOUD_NO_RING': 'no ring field: LIO-SAM cannot run; add ring (row index) in an adapter',
    'CLOUD_NO_TIME': 'no per-point time: LIO-SAM cannot run and deskew is off for the others; add time (seconds from sweep start)',
    'STAMP_AT_END': 'cloud stamp is at the sweep END but estimators expect the START: shift the stamp by -sweep period (stamp_shift_s)',
    'STAMP_UNKNOWN': 'cannot tell whether the stamp is sweep start or end: publish per-point time so it can be checked',
    'IMU_NONE': 'no IMU data: LIO needs one; check the topic and that the bridge/driver publishes it',
    'IMU_RATE_LOW': 'IMU is too slow relative to the lidar: raise the IMU rate (200 Hz+ typical)',
    'IMU_YAW_OFFSET': 'IMU orientation yaw differs from odometry by a constant: relay /imu with the orientation rotated by the opposite yaw '
                      '(scripts/imu_yaw_relay.py, param yaw_deg); acc/gyro stay untouched. extrinsicRPY is NOT the fix, it also rotates acc/gyro',
    'IMU_YAW_NOISY': 'IMU yaw vs odometry yaw scatters: noisy or drifting magnetometer/orientation filter; LIO-SAM deskew and initial guess suffer',
    'IMU_TIME_OFFSET': 'IMU and odometry clocks are offset: set the estimator time-offset parameter or fix the driver stamps',
    'TF_MISSING': 'required frame not in TF: check static transforms and frame_id names (and namespace prefix)',
    'DRIFT_HIGH': 'drift against the reference is high: check extrinsics, deskew, IMU noise settings, and the stamp offset',
}


# ---- point cloud layout ----
def _fsize(f):
    return DTYPES.get(f[2], (0, ''))[0] * max(1, f[3])


def check_cloud_fields(info):
    """Validate the PointCloud2 layout: the bug class where a writer's record size and the declared one disagree."""
    out = []
    byname = {f[0]: f for f in info.fields}
    if not all(n in byname and byname[n][2] == F32 for n in 'xyz'):
        out.append(Finding('ERROR', 'CLOUD_NO_XYZ', 'x/y/z float32 fields missing or wrong type'))
    if info.is_bigendian:
        out.append(Finding('ERROR', 'CLOUD_ENDIAN', 'is_bigendian is set'))
    if info.point_step % 4:
        out.append(Finding('WARN', 'CLOUD_STEP_UNALIGNED', f'point_step {info.point_step} is not a multiple of 4'))
    spans = []
    for f in info.fields:
        size = _fsize(f)
        if f[1] + size > info.point_step:
            out.append(Finding('ERROR', 'CLOUD_FIELD_OVERRUN', f'field {f[0]!r} ends at byte {f[1] + size} but point_step is {info.point_step}'))
        elem = DTYPES.get(f[2], (0, ''))[0]
        if elem and f[1] % elem:
            out.append(Finding('ERROR', 'CLOUD_UNALIGNED', f'field {f[0]!r} at offset {f[1]} is not a multiple of its size {elem}'))
        spans.append((f[1], f[1] + size, f[0]))
    spans.sort()
    for a, b in zip(spans, spans[1:]):
        if b[0] < a[1]:
            out.append(Finding('ERROR', 'CLOUD_FIELD_OVERLAP', f'fields {a[2]!r} and {b[2]!r} overlap'))
    if info.data_len:
        if info.height * info.row_step != info.data_len or (info.width and info.row_step != info.width * info.point_step):
            out.append(Finding('ERROR', 'CLOUD_DATA_SIZE', f'data is {info.data_len} B, width*point_step*height is '
                                                           f'{info.width * info.point_step * info.height} B'
                                                           + (f' (record size looks like {info.data_len // (info.width * info.height)} B)'
                                                              if info.width * info.height and info.data_len % (info.width * info.height) == 0 else '')))
    if 'ring' in byname and byname['ring'][2] != U16:
        out.append(Finding('ERROR', 'CLOUD_RING_TYPE', f'ring has datatype {byname["ring"][2]}, needs UINT16 (4)'))
    tname = next((n for n in TIME_NAMES if n in byname), None)
    if tname and byname[tname][2] != F32:
        out.append(Finding('WARN', 'CLOUD_TIME_TYPE', f'{tname} has datatype {byname[tname][2]}, expected FLOAT32 (7)'))
    return out


def time_field(info):
    return next((f for f in info.fields if f[0] in TIME_NAMES), None)


def field_range(data, info, name, max_points=2000):
    """(min, max) of a numeric field over up to max_points points, read with the DECLARED layout; None if unreadable."""
    f = next((x for x in info.fields if x[0] == name), None)
    if f is None or f[2] not in DTYPES or not info.point_step:
        return None
    code = DTYPES[f[2]][1]
    n = len(data) // info.point_step
    step = max(1, n // max_points)
    end = '>' if info.is_bigendian else '<'
    vals = []
    for i in range(0, n, step):
        o = i * info.point_step + f[1]
        if o + DTYPES[f[2]][0] > len(data):
            break
        vals.append(struct.unpack_from(end + code, data, o)[0])
    vals = [v for v in vals if math.isfinite(v)]
    return (min(vals), max(vals)) if vals else None


def stamp_position(point_time_range, period):
    """'start' / 'end' / 'unknown': where the header stamp sits in the sweep, from per-point times relative to it."""
    if point_time_range is None or not period:
        return 'unknown'
    lo, hi = point_time_range
    tol = STAMP_END_TOL + 0.1 * period
    if lo >= -tol and hi > 0.5 * period:
        return 'start'
    if hi <= tol and lo < -0.5 * period:
        return 'end'
    return 'unknown'


# ---- IMU heading vs odometry ----
def quat_yaw(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def imu_yaw_offset(imu, odom, max_gap=YAW_MAX_GAP_S):
    """imu, odom: lists of (stamp, (x, y, z, w)). Nearest-stamp match (within max_gap), then the circular mean and
    circular std (deg) of imu_yaw - odom_yaw. Returns (mean_deg, std_deg, n_matches) or None if nothing matched."""
    if not imu or not odom:
        return None
    od = sorted(odom, key=lambda p: p[0])
    ts = [p[0] for p in od]
    diffs = []
    for t, q in imu:
        lo, hi = 0, len(ts)
        while lo < hi:
            m = (lo + hi) // 2
            if ts[m] < t:
                lo = m + 1
            else:
                hi = m
        cand = [i for i in (lo - 1, lo) if 0 <= i < len(ts)]
        i = min(cand, key=lambda k: abs(ts[k] - t))
        if abs(ts[i] - t) <= max_gap:
            diffs.append(_wrap(quat_yaw(*q) - quat_yaw(*od[i][1])))
    if not diffs:
        return None
    c = sum(math.cos(d) for d in diffs) / len(diffs)
    s = sum(math.sin(d) for d in diffs) / len(diffs)
    r = min(1.0, math.hypot(c, s))
    std = math.sqrt(-2.0 * math.log(r)) if r > 1e-12 else math.pi
    return math.degrees(math.atan2(s, c)), math.degrees(std), len(diffs)


def check_imu_yaw(imu, odom, max_gap=YAW_MAX_GAP_S):
    """Findings for the IMU-orientation-vs-odometry heading check. imu=None means no orientation in the IMU data."""
    r = imu_yaw_offset(imu, odom, max_gap) if imu and odom else None
    if r is None or r[2] < YAW_MIN_MATCHES:
        n = 0 if r is None else r[2]
        why = 'no IMU orientation' if not imu else 'no reference orientation' if not odom else f'only {n} stamp-matched samples'
        return [Finding('INFO', 'IMU_YAW_SKIPPED', f'IMU heading check skipped: {why}')]
    mean, std, n = r
    if abs(mean) > YAW_OFFSET_DEG:
        return [Finding('ERROR', 'IMU_YAW_OFFSET', f'IMU yaw is {mean:+.1f} deg off odometry yaw (std {std:.1f} deg, {n} matches); '
                        f'fix: imu_yaw_relay yaw_deg:={-mean:+.1f} (add to the relay\'s current yaw if this was measured on its output)')]
    if std > YAW_NOISE_DEG:
        return [Finding('WARN', 'IMU_YAW_NOISY', f'IMU yaw vs odometry yaw std {std:.1f} deg (mean {mean:+.1f} deg, {n} matches)')]
    return [Finding('INFO', 'IMU_YAW_OK', f'IMU yaw agrees with odometry: mean {mean:+.1f} deg, std {std:.1f} deg ({n} matches)')]


# ---- preflight ----
ESTIMATORS = ('KISS-ICP', 'RKO-LIO', 'LIO-SAM')


def readiness(info, stamp_pos='unknown', imu_hz=None, imu_orientation=None, imu_yaw_offset_deg=None):
    """Which of KISS-ICP / RKO-LIO / LIO-SAM can run on this cloud (+ IMU). {name: (verdict, [reasons])}.
    verdict: READY / DEGRADED (runs with a documented option, e.g. deskew off) / BLOCKED."""
    names = {f[0]: f for f in info.fields}
    layout_errors = [x for x in check_cloud_fields(info) if x.level == 'ERROR']
    tf = time_field(info)
    time_ok = tf is not None and tf[2] == F32
    ring_ok = 'ring' in names and names['ring'][2] == U16
    res = {}
    # KISS-ICP: xyz only; per-point time only improves deskew
    r, v = [], 'READY'
    if layout_errors:
        v = 'BLOCKED'
        r += [f'{x.code}: {x.msg}' for x in layout_errors]
    elif not time_ok:
        v = 'DEGRADED'
        r.append('no float32 per-point time: runs without deskew')
    res['KISS-ICP'] = (v, r)
    # RKO-LIO: xyz + IMU; per-point time needed for deskew (else deskew:=false)
    r, v = [], 'READY'
    if layout_errors:
        v = 'BLOCKED'
        r += [f'{x.code}: {x.msg}' for x in layout_errors]
    else:
        if imu_hz is None or imu_hz <= 0:
            v = 'BLOCKED'
            r.append('no IMU seen: RKO-LIO is lidar-inertial')
        if not time_ok and v != 'BLOCKED':
            v = 'DEGRADED'
            r.append('no float32 per-point time: needs deskew:=false (default deskew drops every scan)')
    res['RKO-LIO'] = (v, r)
    # LIO-SAM: xyz + ring u16 + time f4, stamp at sweep start, IMU with orientation
    r, v = [], 'READY'
    if layout_errors:
        v = 'BLOCKED'
        r += [f'{x.code}: {x.msg}' for x in layout_errors]
    if not ring_ok:
        v = 'BLOCKED'
        r.append('ring uint16 missing or wrong type')
    if not time_ok:
        v = 'BLOCKED'
        r.append('time float32 missing or wrong type')
    if stamp_pos == 'end':
        v = 'BLOCKED'
        r.append('cloud stamp is at sweep end: LIO-SAM waits for IMU that does not exist yet')
    elif stamp_pos == 'unknown' and v == 'READY':
        v = 'DEGRADED'
        r.append('cannot verify the stamp is at sweep start')
    if imu_hz is None or imu_hz <= 0:
        v = 'BLOCKED'
        r.append('no IMU seen')
    elif imu_orientation is False:
        v = 'BLOCKED' if v != 'BLOCKED' else v
        r.append('IMU has no orientation (LIO-SAM needs a 9-axis IMU)')
    if imu_yaw_offset_deg is not None and abs(imu_yaw_offset_deg) > YAW_OFFSET_DEG:
        if v == 'READY':
            v = 'DEGRADED'
        r.append(f'IMU_YAW_OFFSET: IMU yaw is {imu_yaw_offset_deg:+.1f} deg off odometry: LIO-SAM initial guess/deskew diverge; '
                 'relay /imu with corrected orientation (imu_yaw_relay.py)')
    res['LIO-SAM'] = (v, r)
    return res


# ---- helpers ----
def _median(v):
    s = sorted(v)
    n = len(s)
    return 0.0 if not n else s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _interp(t, ts, vs):
    if t <= ts[0]:
        return vs[0]
    if t >= ts[-1]:
        return vs[-1]
    lo, hi = 0, len(ts) - 1
    while hi - lo > 1:
        m = (lo + hi) // 2
        if ts[m] <= t:
            lo = m
        else:
            hi = m
    w = (t - ts[lo]) / (ts[hi] - ts[lo])
    return vs[lo] + w * (vs[hi] - vs[lo])


def estimate_time_offset(imu_t, imu_w, odom_t, odom_yaw, max_off=0.3, dt=0.01):
    """Cross-correlate gyro-z against odometry yaw rate (idea from fusion_tools/imu_time_offset.py).
    Returns (offset_s, correlation); positive = odom stamps lag the IMU. None if there is too little data."""
    if len(imu_t) < 20 or len(odom_t) < 20:
        return None
    yaw, prev = [], odom_yaw[0]
    acc = 0.0
    for y in odom_yaw:
        acc += _wrap(y - prev)
        prev = y
        yaw.append(acc)
    rt = [0.5 * (odom_t[i] + odom_t[i + 1]) for i in range(len(odom_t) - 1)]
    rw = [(yaw[i + 1] - yaw[i]) / (odom_t[i + 1] - odom_t[i]) if odom_t[i + 1] > odom_t[i] else 0.0 for i in range(len(rt))]
    t0, t1 = max(imu_t[0], rt[0]) + max_off, min(imu_t[-1], rt[-1]) - max_off
    if t1 - t0 < 1.0:
        return None
    grid = [t0 + k * dt for k in range(int((t1 - t0) / dt))]
    a = [_interp(t, imu_t, imu_w) for t in grid]
    ma = sum(a) / len(a)
    a = [x - ma for x in a]
    na = math.sqrt(sum(x * x for x in a)) or 1e-12
    best, best_off = -2.0, 0.0
    for k in range(-int(max_off / dt), int(max_off / dt) + 1):
        b = [_interp(t + k * dt, rt, rw) for t in grid]
        mb = sum(b) / len(b)
        b = [x - mb for x in b]
        c = sum(x * y for x, y in zip(a, b)) / (na * (math.sqrt(sum(y * y for y in b)) or 1e-12))
        if c > best:
            best, best_off = c, k * dt
    return best_off, best


def align_drift(est, ref):
    """est, ref: lists of (x, y) at the same instants. Rigid 2D alignment (Umeyama, no scale) then
    (rmse, end_error, path_length)."""
    n = len(est)
    mex, mey = sum(p[0] for p in est) / n, sum(p[1] for p in est) / n
    mrx, mry = sum(p[0] for p in ref) / n, sum(p[1] for p in ref) / n
    sxx = sxy = 0.0
    for (ex, ey), (rx, ry) in zip(est, ref):
        ex, ey, rx, ry = ex - mex, ey - mey, rx - mrx, ry - mry
        sxx += ex * rx + ey * ry
        sxy += ex * ry - ey * rx
    th = math.atan2(sxy, sxx)
    c, s = math.cos(th), math.sin(th)
    err = []
    for (ex, ey), (rx, ry) in zip(est, ref):
        px = c * (ex - mex) - s * (ey - mey) + mrx
        py = s * (ex - mex) + c * (ey - mey) + mry
        err.append(math.hypot(px - rx, py - ry))
    path = sum(math.hypot(ref[i + 1][0] - ref[i][0], ref[i + 1][1] - ref[i][1]) for i in range(n - 1))
    return math.sqrt(sum(e * e for e in err) / n), err[-1], path


# ---- the live doctor ----
@dataclass
class LioDoctor:
    expected_hz: float = 10.0
    v_max: float = 2.0            # m/s the robot can really do
    w_max: float = 3.0            # rad/s
    frames: tuple = ()            # frames that must exist in TF
    drift_window_s: float = 30.0
    odom_arrivals: deque = field(default_factory=deque)
    stamps: deque = field(default_factory=lambda: deque(maxlen=400))
    last: tuple = None            # (stamp, x, y, yaw)
    events: list = field(default_factory=list)
    cloud_info: CloudInfo = None
    cloud_range: tuple = None
    cloud_arrivals: deque = field(default_factory=lambda: deque(maxlen=50))
    imu_arrivals: deque = field(default_factory=lambda: deque(maxlen=400))
    imu_has_orientation: bool = None
    imu_series: deque = field(default_factory=lambda: deque(maxlen=3000))   # (stamp, gyro_z)
    odom_series: deque = field(default_factory=lambda: deque(maxlen=3000))  # (stamp, yaw)
    tf_seen: set = field(default_factory=set)
    track: deque = field(default_factory=lambda: deque(maxlen=4000))        # (stamp, x, y)
    ref_track: deque = field(default_factory=lambda: deque(maxlen=4000))
    imu_quats: deque = field(default_factory=lambda: deque(maxlen=3000))    # (stamp, (x, y, z, w))
    ref_quats: deque = field(default_factory=lambda: deque(maxlen=3000))

    # inputs
    def odom(self, stamp, x, y, yaw, now):
        self.odom_arrivals.append(now)
        while self.odom_arrivals and now - self.odom_arrivals[0] > RATE_WINDOW_S:
            self.odom_arrivals.popleft()
        if self.last is not None:
            dt = stamp - self.last[0]
            if dt <= 0:
                self.events.append({'t': now, 'kind': 'STAMP_BACKWARDS', 'dt': round(dt, 4)})
            else:
                med = _median([b - a for a, b in zip(self.stamps, list(self.stamps)[1:])]) if len(self.stamps) > 3 else 0.0
                if med and dt > GAP_FACTOR * med:
                    self.events.append({'t': now, 'kind': 'STAMP_GAP', 'dt': round(dt, 3), 'median': round(med, 3)})
                step = math.hypot(x - self.last[1], y - self.last[2])
                if step > self.v_max * dt + JUMP_SLACK_M:
                    self.events.append({'t': now, 'kind': 'POSE_JUMP', 'step': round(step, 3), 'dt': round(dt, 3)})
                turn = abs(_wrap(yaw - self.last[3]))
                if turn > self.w_max * dt + 0.1:
                    self.events.append({'t': now, 'kind': 'YAW_JUMP', 'turn': round(turn, 3), 'dt': round(dt, 3)})
        if self.last is None or stamp > self.last[0]:
            self.stamps.append(stamp)
            self.last = (stamp, x, y, yaw)
            self.odom_series.append((stamp, yaw))
            self.track.append((stamp, x, y))

    def ref_odom(self, stamp, x, y, quat=None):
        self.ref_track.append((stamp, x, y))
        if quat is not None:
            self.ref_quats.append((stamp, tuple(quat)))

    def cloud(self, info, now, point_time_range=None):
        self.cloud_info, self.cloud_range = info, point_time_range
        self.cloud_arrivals.append(now)

    def imu(self, stamp, now, gyro_z=0.0, has_orientation=None, quat=None):
        if quat is not None and has_orientation is not False:
            self.imu_quats.append((stamp, tuple(quat)))
        self.imu_arrivals.append(now)
        self.imu_series.append((stamp, gyro_z))
        if has_orientation is not None:
            self.imu_has_orientation = has_orientation

    def tf(self, frames):
        self.tf_seen |= set(frames)

    # derived
    @staticmethod
    def _hz(arr):
        a = list(arr)
        return (len(a) - 1) / (a[-1] - a[0]) if len(a) >= 3 and a[-1] > a[0] else None

    def odom_hz(self):
        return self._hz(self.odom_arrivals)

    def cloud_hz(self):
        return self._hz(self.cloud_arrivals)

    def imu_hz(self):
        return self._hz(self.imu_arrivals)

    def stamp_pos(self):
        hz = self.cloud_hz()
        return stamp_position(self.cloud_range, 1.0 / hz if hz else 0.1)

    def drift(self):
        """(rmse, end_err, path) over the last drift_window_s of the estimate vs the reference, or None."""
        if len(self.track) < 5 or len(self.ref_track) < 5:
            return None
        rt = [p[0] for p in self.ref_track]
        t_end = min(self.track[-1][0], rt[-1])
        pts = [p for p in self.track if t_end - self.drift_window_s <= p[0] <= t_end and rt[0] <= p[0]]
        if len(pts) < 5:
            return None
        rx, ry = [p[1] for p in self.ref_track], [p[2] for p in self.ref_track]
        est = [(p[1], p[2]) for p in pts]
        ref = [(_interp(p[0], rt, rx), _interp(p[0], rt, ry)) for p in pts]
        return align_drift(est, ref)

    def diagnose(self, now, recent_s=10.0):
        f = []
        hz = self.odom_hz()
        if self.last is None or (self.odom_arrivals and now - self.odom_arrivals[-1] > SILENT_S):
            f.append(Finding('ERROR', 'ODOM_NONE', 'no odometry received' if self.last is None else f'odometry silent for {now - self.odom_arrivals[-1]:.1f}s'))
        elif hz is not None and hz < ODOM_RATE_FRAC * self.expected_hz:
            f.append(Finding('WARN', 'ODOM_RATE_LOW', f'odometry {hz:.1f} Hz, expected {self.expected_hz:.1f}'))
        for kind, lvl in (('STAMP_BACKWARDS', 'ERROR'), ('STAMP_GAP', 'WARN'), ('POSE_JUMP', 'ERROR'), ('YAW_JUMP', 'WARN')):
            ev = [e for e in self.events if e['kind'] == kind and now - e['t'] <= recent_s]
            if ev:
                key = {'STAMP_BACKWARDS': ('dt', 'dt {:.3f}s'), 'STAMP_GAP': ('dt', 'largest {:.2f}s'),
                       'POSE_JUMP': ('step', 'largest step {:.2f}m'), 'YAW_JUMP': ('turn', 'largest turn {:.2f}rad')}[kind]
                detail = key[1].format(max(abs(e[key[0]]) for e in ev))
                f.append(Finding(lvl, kind, f'{len(ev)}x in {recent_s:.0f}s, {detail}'))
        if self.cloud_info is None:
            f.append(Finding('WARN', 'CLOUD_NONE', 'no PointCloud2 received'))
        elif self.cloud_arrivals and now - self.cloud_arrivals[-1] > SILENT_S:
            f.append(Finding('ERROR', 'CLOUD_STALE', f'no PointCloud2 for {now - self.cloud_arrivals[-1]:.1f}s (was {self.cloud_hz() or 0:.1f} Hz)'))
        else:
            f += [x for x in check_cloud_fields(self.cloud_info)]
            pos = self.stamp_pos()
            if pos == 'end':
                f.append(Finding('ERROR', 'STAMP_AT_END', 'per-point times are all negative: the header stamp is the sweep END'))
            elif pos == 'unknown' and self.cloud_range is None:
                f.append(Finding('INFO', 'STAMP_UNKNOWN', 'no per-point time to check the stamp against'))
        ihz = self.imu_hz()
        if ihz is None:
            f.append(Finding('WARN', 'IMU_NONE', 'no IMU data received'))
        elif self.imu_arrivals and now - self.imu_arrivals[-1] > SILENT_S:
            f.append(Finding('ERROR', 'IMU_STALE', f'no IMU for {now - self.imu_arrivals[-1]:.1f}s (was {ihz:.0f} Hz)'))
        else:
            chz = self.cloud_hz()
            if ihz < IMU_HZ_MIN or (chz and ihz < IMU_RATIO_MIN * chz):
                f.append(Finding('WARN', 'IMU_RATE_LOW', f'IMU {ihz:.0f} Hz vs lidar {chz or 0:.1f} Hz'))
            r = estimate_time_offset([p[0] for p in self.imu_series], [p[1] for p in self.imu_series],
                                     [p[0] for p in self.odom_series], [p[1] for p in self.odom_series])
            if r and abs(r[0]) > OFFSET_WARN_S and r[1] >= OFFSET_CORR_MIN:
                f.append(Finding('WARN', 'IMU_TIME_OFFSET', f'odom lags IMU by {r[0]:+.3f}s (correlation {r[1]:.2f})'))
        if ihz is not None and self.imu_has_orientation is False:
            f.append(Finding('INFO', 'IMU_YAW_SKIPPED', 'IMU heading check skipped: no IMU orientation'))
        elif ihz is not None and self.imu_quats:
            if not self.ref_quats:
                f.append(Finding('INFO', 'IMU_YAW_SKIPPED', 'IMU heading check skipped: no reference orientation (--ref-odom)'))
            else:
                f += check_imu_yaw(list(self.imu_quats), list(self.ref_quats))
        if self.frames:
            missing = [x for x in self.frames if x not in self.tf_seen]
            if missing:
                f.append(Finding('ERROR', 'TF_MISSING', 'frames not in TF: ' + ', '.join(missing)))
        d = self.drift()
        if d and d[2] >= DRIFT_MIN_PATH_M and 100.0 * d[1] / d[2] > DRIFT_PCT:
            f.append(Finding('WARN', 'DRIFT_HIGH', f'end error {d[1]:.2f}m over {d[2]:.1f}m ({100.0 * d[1] / d[2]:.1f}%), rmse {d[0]:.2f}m'))
        if not any(x.level in ('ERROR', 'WARN') for x in f):
            f.append(Finding('OK', 'HEALTHY', 'lidar odometry looks healthy'))
        return f

    def score(self, now):
        """0-100 health score: ERROR -40, WARN -10 each, floor 0."""
        return max(0, 100 - sum(40 if x.level == 'ERROR' else 10 if x.level == 'WARN' else 0 for x in self.diagnose(now)))

    def imu_yaw_deg(self):
        """Measured IMU-vs-reference yaw offset in degrees, or None if it cannot be measured."""
        r = imu_yaw_offset(list(self.imu_quats), list(self.ref_quats))
        return r[0] if r and r[2] >= YAW_MIN_MATCHES else None

    def summary(self, now):
        d = self.drift()
        return {'score': self.score(now), 'odom_hz': self.odom_hz(), 'cloud_hz': self.cloud_hz(), 'imu_hz': self.imu_hz(),
                'stamp_position': self.stamp_pos(), 'drift_end_err_m': None if d is None else round(d[1], 3)}


def health_overlay(findings, summary):
    """(text, (r, g, b)) for an RViz text marker: score, rates, then one line per ERROR/WARN finding. Colour by worst level."""
    bad = [x for x in findings if x.level in ('ERROR', 'WARN')]
    lines = [f"LIO health {summary['score']}/100"]
    rates = [f'{n} {summary[k]:.1f} Hz' for n, k in (('odom', 'odom_hz'), ('cloud', 'cloud_hz'), ('imu', 'imu_hz')) if summary.get(k)]
    if rates:
        lines.append('  '.join(rates))
    lines += [f'{x.level} {x.code}' for x in bad[:6]] + ([f'+{len(bad) - 6} more'] if len(bad) > 6 else [])
    color = (1.0, 0.2, 0.2) if any(x.level == 'ERROR' for x in bad) else (1.0, 0.8, 0.1) if bad else (0.3, 1.0, 0.4)
    return '\n'.join(lines), color
