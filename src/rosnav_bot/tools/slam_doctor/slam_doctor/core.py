"""Pure SLAM-health logic (no ROS): feed it samples with a timestamp, ask for findings."""
import math
from collections import deque
from dataclasses import dataclass

JUMP_M = 0.10            # a map->odom step bigger than this is a map correction (loop closure / scan-match jump)
JUMP_DEG = 2.0
BIG_JUMP_M = 0.50        # corrections this large make Nav2 plans and costmaps visibly jump
STORM_WINDOW_S = 60.0
STORM_COUNT = 4          # this many corrections in the window = periodic jumping (e.g. slam_toolbox every ~10 s)
GAP_S = 1.0              # map->odom silent this long = tracker stopped publishing
FLAP_WINDOW_S = 30.0
FLAP_COUNT = 3
LOST_S = 5.0             # silent this long = tracking lost, not a blip
STALL_S = 20.0           # no new free cells for this long ...
STALL_TRAVEL_M = 1.0     # ... while the robot travelled this far = map not growing
DRIFT_RATIO = 0.10       # metres of map correction per metre travelled
DRIFT_MIN_TRAVEL_M = 5.0
FEATURE_FRAMES = 5       # consecutive bad frames before a visual warning


@dataclass
class Finding:
    level: str   # ERROR / WARN / INFO / OK
    code: str
    msg: str


HINTS = {
    'TRACKING_LOST': 'move back to a textured, well-lit area; check camera topic rate; look for tracker reset in the SLAM log',
    'TRACKING_FLAP': 'tracker keeps dropping and recovering: slow down, add texture/light, check sync of image and depth',
    'JUMP_STORM': 'map corrections arrive in bursts: raise loop-closure / scan-match thresholds',
    'BIG_JUMP': 'large map correction: expect plan and costmap jumps; clear costmaps after it (costmap_ghost_clear)',
    'MAP_STALL': 'map stopped growing while the robot moves: check /map publisher, SLAM node queue, and tracking state',
    'MAP_NONE': 'no /map seen: is the SLAM node running and publishing in this namespace?',
    'NO_MAP_ODOM': 'SLAM publishes no map->odom: check the SLAM node and its frame names / namespace',
    'DRIFT_HIGH': 'odometry is drifting: check wheel radius/track width, IMU fusion, slip on this surface',
    'DARK': 'turn on lights or raise exposure', 'OVEREXPOSED': 'lower exposure / avoid facing the light',
    'BLURRED': 'slow down or raise shutter speed; blur kills feature tracking',
    'FEATURE_POOR': 'few corners in view (blank wall/floor): a feature-based SLAM will lose tracking here',
}


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class SlamDoctor:
    def __init__(self, need_images=False):
        self.need_images = need_images
        self.last_mo = None            # (x, y, yaw) of the previous map->odom sample
        self.last_mo_t = None
        self.events = []               # loop-closure / correction timeline: {t, kind, dist, deg}
        self.gaps = deque()            # times a map->odom silence ended (tracker resumed)
        self.gap_open = False
        self.odom_xy = None
        self.travel = 0.0              # metres of odometry travelled
        self.corrected = 0.0           # metres of map->odom correction
        self.map_seen = False
        self.free = None
        self.free_changed_t = None
        self.travel_at_change = 0.0
        self.frames = deque(maxlen=FEATURE_FRAMES)
        self.last_frame = None

    # ---- inputs ----
    def map_odom(self, x, y, yaw, now):
        if self.last_mo is not None:
            if now - self.last_mo_t > GAP_S:
                self.gaps.append(now)
                self.events.append({'t': now, 'kind': 'resumed', 'dist': 0.0, 'deg': 0.0,
                                    'silent_s': round(now - self.last_mo_t, 2)})
            d = math.hypot(x - self.last_mo[0], y - self.last_mo[1])
            a = abs(math.degrees(_wrap(yaw - self.last_mo[2])))
            if d > JUMP_M or a > JUMP_DEG:
                self.corrected += d
                self.events.append({'t': now, 'kind': 'correction', 'dist': round(d, 3), 'deg': round(a, 2)})
        self.last_mo, self.last_mo_t = (x, y, yaw), now

    def odom(self, x, y, now):
        if self.odom_xy is not None:
            self.travel += math.hypot(x - self.odom_xy[0], y - self.odom_xy[1])
        self.odom_xy = (x, y)

    def map_stats(self, free, occupied, unknown, now):
        self.map_seen = True
        if self.free is None or free != self.free:
            self.free_changed_t, self.travel_at_change = now, self.travel
        self.free = free

    def frame(self, stats, now):
        from .features import frame_problems
        self.last_frame = stats
        self.frames.append(set(frame_problems(stats)))

    # ---- verdict ----
    def corrections(self, now, window=STORM_WINDOW_S):
        return [e for e in self.events if e['kind'] == 'correction' and now - e['t'] <= window]

    def diagnose(self, now):
        f = []
        if self.last_mo is None:
            f.append(Finding('WARN', 'NO_MAP_ODOM', 'no map->odom seen yet'))
        elif now - self.last_mo_t > LOST_S:
            f.append(Finding('ERROR', 'TRACKING_LOST', f'map->odom silent for {now - self.last_mo_t:.1f}s'))
        if not self.map_seen:
            f.append(Finding('WARN', 'MAP_NONE', 'no /map received'))
        while self.gaps and now - self.gaps[0] > FLAP_WINDOW_S:
            self.gaps.popleft()
        if len(self.gaps) >= FLAP_COUNT:
            f.append(Finding('WARN', 'TRACKING_FLAP',
                             f'tracker stopped and resumed {len(self.gaps)}x in {FLAP_WINDOW_S:.0f}s'))
        recent = self.corrections(now)
        if len(recent) >= STORM_COUNT:
            f.append(Finding('WARN', 'JUMP_STORM', f'{len(recent)} map corrections in {STORM_WINDOW_S:.0f}s, '
                                                   f'largest {max(e["dist"] for e in recent):.2f}m'))
        big = [e for e in self.corrections(now, 10.0) if e['dist'] >= BIG_JUMP_M]
        if big:
            f.append(Finding('WARN', 'BIG_JUMP', f'map jumped {big[-1]["dist"]:.2f}m / {big[-1]["deg"]:.1f}deg '
                                                 f'{now - big[-1]["t"]:.0f}s ago'))
        if (self.free_changed_t is not None and now - self.free_changed_t >= STALL_S
                and self.travel - self.travel_at_change >= STALL_TRAVEL_M):
            f.append(Finding('ERROR', 'MAP_STALL', f'no new free cells for {now - self.free_changed_t:.0f}s while the '
                                                   f'robot moved {self.travel - self.travel_at_change:.1f}m'))
        if self.travel >= DRIFT_MIN_TRAVEL_M and self.corrected / self.travel > DRIFT_RATIO:
            f.append(Finding('WARN', 'DRIFT_HIGH', f'{self.corrected:.1f}m of map correction over {self.travel:.1f}m '
                                                   f'travelled ({self.corrected / self.travel:.2f} m/m)'))
        if len(self.frames) == FEATURE_FRAMES:
            for code in sorted(set.intersection(*self.frames)):
                f.append(Finding('WARN', code, f'{FEATURE_FRAMES} consecutive frames: ' +
                                 (f'{self.last_frame["keypoints"]} keypoints, sharpness {self.last_frame["sharpness"]:.0f}, '
                                  f'mean {self.last_frame["mean"]:.0f}' if self.last_frame else '')))
        if not any(x.level in ('ERROR', 'WARN') for x in f):
            f.append(Finding('OK', 'HEALTHY', 'SLAM looks healthy'))
        return f

    def score(self, now):
        """0-100 health score: ERROR -40, WARN -10 each, floor 0."""
        s = 100
        for x in self.diagnose(now):
            s -= 40 if x.level == 'ERROR' else 10 if x.level == 'WARN' else 0
        return max(0, s)

    def summary(self, now):
        c = [e for e in self.events if e['kind'] == 'correction']
        return {'score': self.score(now), 'travel_m': round(self.travel, 2), 'corrections': len(c),
                'largest_correction_m': round(max((e['dist'] for e in c), default=0.0), 3),
                'tracker_resumes': sum(1 for e in self.events if e['kind'] == 'resumed')}
