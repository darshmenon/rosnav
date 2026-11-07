import importlib.util
import math
import pathlib
import sys
import types

# leader_follower imports rclpy/nav2 at module level; stub them so the pure Trail logic is testable anywhere
for m in ['visualization_msgs', 'visualization_msgs.msg', 'nav_msgs', 'nav_msgs.msg', 'rclpy', 'rclpy.action', 'rclpy.node', 'geometry_msgs', 'geometry_msgs.msg', 'nav2_msgs', 'nav2_msgs.action',
          'tf2_ros']:
    sys.modules.setdefault(m, types.ModuleType(m))
sys.modules['rclpy.node'].Node = object
sys.modules['geometry_msgs.msg'].Point = sys.modules['nav_msgs.msg'].Path = object
sys.modules['visualization_msgs.msg'].Marker = sys.modules['visualization_msgs.msg'].MarkerArray = object
sys.modules['rclpy.action'].ActionClient = object
sys.modules['geometry_msgs.msg'].PoseStamped = sys.modules['geometry_msgs.msg'].Twist = object
sys.modules['nav2_msgs.action'].NavigateToPose = object
sys.modules['tf2_ros'].Buffer = sys.modules['tf2_ros'].TransformListener = object
spec = importlib.util.spec_from_file_location(
    'leader_follower', pathlib.Path(__file__).resolve().parents[1] / 'scripts' / 'leader_follower.py')
lf = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lf)


def straight(n=30, step=0.1):
    t = lf.Trail(step)
    for i in range(n):
        t.add(i * step, 0.0)
    return t


def test_none_until_long_enough():
    t = straight(5)
    assert t.point_behind(1.0) is None


def test_point_behind_on_straight_line():
    x, y, yaw = straight().point_behind(1.0)
    assert math.isclose(x, 2.9 - 1.0, abs_tol=1e-6) and math.isclose(y, 0.0)
    assert math.isclose(yaw, 0.0, abs_tol=1e-6)  # direction of travel (east)


def test_follows_corner_not_straight_offset():
    t = lf.Trail(0.1)
    for i in range(21):
        t.add(i * 0.1, 0.0)           # east 2 m
    for i in range(1, 21):
        t.add(2.0, i * 0.1)           # then north 2 m (leader head at (2, 2))
    x, y, _ = t.point_behind(3.0)     # 3 m back along the trail: 2 m north leg + 1 m of east leg
    assert math.isclose(x, 1.0, abs_tol=0.11) and math.isclose(y, 0.0, abs_tol=0.11)


def test_trim_bounds_memory():
    t = straight(200)
    t.trim(3.0)
    assert len(t.pts) < 40 and t.point_behind(2.5) is not None


def test_adaptive_spacing_grows_and_caps():
    assert lf.adaptive_spacing(1.0, 0.5, 0.0) == 1.0
    assert math.isclose(lf.adaptive_spacing(1.0, 0.5, 0.4), 1.2)
    assert lf.adaptive_spacing(1.0, 0.5, 100.0) == 3.0
    assert lf.adaptive_spacing(1.0, 0.5, -1.0) == 1.0


def test_se2_apply_translation_and_rotation():
    x, y, yaw = lf.se2_apply(1.0, 2.0, math.pi / 2, 1.0, 0.0, 0.0)   # odom point (1,0) in a frame rotated 90 deg at (1,2)
    assert math.isclose(x, 1.0, abs_tol=1e-9) and math.isclose(y, 3.0, abs_tol=1e-9)
    assert math.isclose(yaw, math.pi / 2, abs_tol=1e-9)


def test_localization_jump_moves_trail_consistently():
    t = straight()                                    # odom-frame trail
    a = lf.se2_apply(0, 0, 0, *t.point_behind(1.0)[:2])
    b = lf.se2_apply(5, -3, 0, *t.point_behind(1.0)[:2])    # AMCL correction of (5, -3)
    assert math.isclose(b[0] - a[0], 5.0) and math.isclose(b[1] - a[1], -3.0)


def test_project_arclen_lookahead():
    t = straight(30)                       # x = 0 .. 2.9
    i = t.project(1.0, 0.05)
    assert math.isclose(t.pts[i][0], 1.0, abs_tol=0.06)
    assert math.isclose(t.arclen_from(i), 2.9 - t.pts[i][0], abs_tol=1e-6)
    assert math.isclose(t.lookahead(i, 0.5)[0], t.pts[i][0] + 0.5, abs_tol=1e-6)
    assert t.lookahead(i, 99.0) == t.pts[-1]


def test_gap_speed_feedforward_feedback_and_hold():
    assert math.isclose(lf.gap_speed(0.2, 0.0, 0.8, 0.35), 0.2)          # on slot: match leader speed
    assert lf.gap_speed(0.2, 1.0, 0.8, 0.35) == 0.35                      # far behind: catch up, capped
    assert lf.gap_speed(0.2, -0.5, 0.8, 0.35) == 0.0                      # too close: stop
    assert math.isclose(lf.gap_speed(0.2, -0.1, 0.8, 0.35), 0.12)         # slightly close: ease off


def test_pursuit_cmd_steers_and_turns_in_place():
    v, w = lf.pursuit_cmd(0, 0, 0, 1.0, 0.0, 0.2)
    assert v == 0.2 and abs(w) < 1e-9                                      # straight ahead
    v, w = lf.pursuit_cmd(0, 0, 0, 1.0, 0.5, 0.2)
    assert w > 0                                                           # target to the left -> turn left
    v, w = lf.pursuit_cmd(0, 0, 0, -1.0, 0.1, 0.2)
    assert v == 0.0 and w > 0                                              # target behind -> rotate in place


def test_gap_color_bands():
    assert lf.gap_color(0.05) == (0.1, 0.9, 0.2)      # green: within 15 cm
    assert lf.gap_color(-0.3)[0] == 1.0 and lf.gap_color(-0.3)[1] > 0.5   # yellow
    assert lf.gap_color(1.0)[1] < 0.3                  # red


def test_formation_slots_shapes():
    assert lf.formation_slots('line', 3, 1.0) == [(0.0, 1.0), (0.0, -1.0), (0.0, 2.0)]
    w = lf.formation_slots('wedge', 4, 1.0)
    assert all(b > 0 for b, _ in w) and w[0][1] > 0 > w[1][1]                    # behind, alternating sides
    d = lf.formation_slots('diamond', 3, 1.0)
    assert d[0][1] > 0 > d[1][1] and d[2] == (1.2 * 2, 0.0) or d[2][1] == 0.0    # left, right, rear on axis
    c = lf.formation_slots('circle', 4, 1.0)
    assert all(math.isclose(math.hypot(b, l), 1.5) for b, l in c)               # all on a ring of 1.5 s
    assert len({(round(b, 3), round(l, 3)) for b, l in c}) == 4                  # distinct slots
    assert len(lf.formation_slots('echelon', 5, 1.0)) == 5


def test_slot_world_is_rotation_correct():
    # heading east: back = -x, left = +y
    x_, y_ = lf.slot_world(0, 0, 0.0, 1.0, 0.0); assert math.isclose(x_, -1) and math.isclose(y_, 0, abs_tol=1e-9)
    x_, y_ = lf.slot_world(0, 0, 0.0, 0.0, 1.0); assert math.isclose(x_, 0, abs_tol=1e-9) and math.isclose(y_, 1)
    # heading north: back = -y, left = -x   (the sign that was wrong before)
    x_, y_ = lf.slot_world(0, 0, math.pi / 2, 1.0, 0.0); assert math.isclose(x_, 0, abs_tol=1e-9) and math.isclose(y_, -1)
    x_, y_ = lf.slot_world(0, 0, math.pi / 2, 0.0, 1.0); assert math.isclose(x_, -1) and math.isclose(y_, 0, abs_tol=1e-9)
    # slot distance to the leader never depends on heading
    assert math.isclose(math.hypot(*lf.slot_world(2, 3, 1.1, 0.7, -0.4)[:2]) if False else math.hypot(
        lf.slot_world(2, 3, 1.1, 0.7, -0.4)[0] - 2, lf.slot_world(2, 3, 1.1, 0.7, -0.4)[1] - 3), math.hypot(0.7, 0.4))


def test_keep_clear_pushes_glide_off_the_leader():
    assert lf.keep_clear((2.0, 0.0), 0.8) == (2.0, 0.0)                       # already clear: untouched
    assert math.isclose(math.hypot(*lf.keep_clear((0.1, 0.1), 0.8)), 0.8)     # inside: pushed out to r_min
    assert lf.keep_clear((0.0, 0.0), 0.8) == (0.8, 0.0)                       # on the leader: to its rear
