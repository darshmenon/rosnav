#!/usr/bin/env python3
"""
cbs_planner.py — Conflict-Based Search (CBS) multi-robot path coordination.

Computes a joint, conflict-free set of grid paths for a batch of robots —
guaranteed collision-free by construction — and dispatches each robot's path
as a dense waypoint sequence via Nav2's NavigateThroughPoses action. Waypoints
are spaced one planning-grid cell apart, so planner_server has very little
room to deviate between them — in practice it tracks the CBS route closely
while still getting Nav2's own local replanning/recovery as a safety net.

This complements, rather than replaces, `priority_collision_avoidance.py`:
that node is a reactive runtime yield rule (fast, but a bad priority order
can deadlock two robots in a corridor). CBS is an upfront, offline joint
plan — slower to compute, but conflict-free whenever a solution exists.
Local reactive avoidance (Nav2's costmap, or priority_collision_avoidance.py)
is still the right safety net for obstacles CBS never saw coming.

Algorithm
─────────
  1. Downsample /map into a coarse planning grid (`cell_size`), eroded by
     `robot_radius` for wall clearance.
  2. Low level: per-robot time-expanded A* (state = cell + timestep, with a
     "wait" action) under a set of forbidden (cell, time) / (edge, time)
     constraints.
  3. High level: start with zero constraints, find each robot's independent
     shortest path, then repeatedly find the first vertex or edge conflict
     between any two robots' paths and branch — one child forbids robot A
     from that cell/edge at that timestep, the other forbids robot B.
     Re-run only the constrained robot's low-level search in each child.
     This is a best-first search (branch and bound) over the constraint
     tree, bounded by `max_cbs_nodes`.
  4. Once a node's paths have no remaining conflict, convert each robot's
     cell path to a PoseStamped waypoint list and send it to
     `/<ns>/navigate_through_poses` (bt_navigator).

Parameters
──────────
  robots_json         JSON list [{"ns": "robot1", "goal_x": 2.0, "goal_y": 1.0}, ...]
  map_topic           (default: /map)
  map_frame           (default: map)
  base_frame          robot TF frame, namespaced per robot (default: base_link)
  cell_size           planning grid cell size, metres   (default: 0.4)
  robot_radius        wall clearance erosion, metres     (default: 0.3)
  max_planning_cells  auto-coarsen cell_size above this  (default: 4000)
  max_horizon         time steps per robot search         (default: 200)
  max_cbs_nodes       high-level constraint-tree budget   (default: 300)
  time_per_step       seconds per grid cell of travel      (default: 0.6)
  behavior_tree       bt_navigator BT xml path, '' = default
  auto_plan           plan once automatically on first map (default: true)

Usage
─────
  # A JSON value with spaces/quotes trips rcl's CLI YAML parser when passed
  # inline via -p, so use a params file instead:
  #   cat > cbs_params.yaml <<'EOF'
  #   cbs_planner:
  #     ros__parameters:
  #       robots_json: '[{"ns": "robot1", "goal_x": 3.0, "goal_y": 1.0},
  #                      {"ns": "robot2", "goal_x": -3.0, "goal_y": 1.0}]'
  #   EOF
  ros2 run rosnav_bot cbs_planner.py --ros-args --params-file cbs_params.yaml

  # Re-trigger a fresh joint plan at any time (e.g. after new goals):
  ros2 topic pub --once /cbs_planner/plan std_msgs/msg/Empty {}
"""

import heapq
import itertools
import json
import math
import time

import numpy as np
import rclpy
import tf2_ros
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateThroughPoses
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Empty

_MAP_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

try:
    from scipy.ndimage import binary_erosion as _scipy_erosion
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


# ── Grid helpers (pure functions, no ROS) ────────────────────────────────────

def _world_to_cell(x: float, y: float, ox: float, oy: float, cell_size: float):
    return (int((y - oy) // cell_size), int((x - ox) // cell_size))


def _cell_to_world(r: int, c: int, ox: float, oy: float, cell_size: float):
    return (ox + (c + 0.5) * cell_size, oy + (r + 0.5) * cell_size)


def _manhattan(a, b) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _build_planning_grid(msg: OccupancyGrid, cell_size: float, robot_radius: float,
                          max_cells: int, logger=None):
    """Erode /map by robot_radius, then downsample to a coarse planning grid
    (a coarse cell is free only if every fine cell inside it is free)."""
    info = msg.info
    res  = info.resolution
    w, h = info.width, info.height
    ox   = info.origin.position.x
    oy   = info.origin.position.y

    grid = np.array(msg.data, dtype=np.int8).reshape((h, w))
    free = (grid == 0)

    r_cells = max(1, int(math.ceil(robot_radius / res)))
    struct  = np.ones((2 * r_cells + 1, 2 * r_cells + 1), dtype=bool)
    if HAS_SCIPY:
        eroded = _scipy_erosion(free, structure=struct)
    else:
        from numpy.lib.stride_tricks import sliding_window_view
        s   = 2 * r_cells + 1
        pad = r_cells
        padded_free = np.pad(free, pad, constant_values=False)
        eroded = sliding_window_view(padded_free, (s, s)).all(axis=(-2, -1))

    k = max(1, int(round(cell_size / res)))
    rows = max(1, h // k)
    cols = max(1, w // k)
    while rows * cols > max_cells and k < max(h, w):
        k += 1
        rows = max(1, h // k)
        cols = max(1, w // k)
    actual_cell_size = k * res
    if logger and actual_cell_size > cell_size * 1.01:
        logger(f'[cbs] cell_size auto-increased {cell_size:.2f}m -> {actual_cell_size:.2f}m '
               f'to keep planning grid <= {max_cells} cells')

    planning_free = np.zeros((rows, cols), dtype=bool)
    for R in range(rows):
        for C in range(cols):
            block = eroded[R * k:(R + 1) * k, C * k:(C + 1) * k]
            planning_free[R, C] = bool(block.all()) if block.size else False

    return planning_free, rows, cols, ox, oy, actual_cell_size


# ── Low-level: time-expanded A* for one agent ────────────────────────────────

_MOVES = [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)]  # wait, S, N, E, W


def _low_level_astar(free, rows: int, cols: int, start, goal,
                      vertex_c: set, edge_c: set, max_horizon: int):
    """State = (row, col, t). vertex_c = {(r,c,t)}, edge_c = {(r1,c1,r2,c2,t)}
    meaning 'cannot move (r1,c1)->(r2,c2) between t and t+1'."""
    goal_hold_t = max((t for (r, c, t) in vertex_c if (r, c) == goal), default=-1)

    start_state = (start[0], start[1], 0)
    open_heap = [(_manhattan(start, goal), 0, start[0], start[1], 0)]
    came_from = {}
    g_score   = {start_state: 0}
    visited   = set()

    while open_heap:
        _, g, r, c, t = heapq.heappop(open_heap)
        if (r, c, t) in visited:
            continue
        visited.add((r, c, t))

        if (r, c) == goal and t > goal_hold_t:
            path = [(r, c)]
            cur  = (r, c, t)
            while cur in came_from:
                cur = came_from[cur]
                path.append((cur[0], cur[1]))
            path.reverse()
            return path

        if t >= max_horizon:
            continue

        for dr, dc in _MOVES:
            nr, nc, nt = r + dr, c + dc, t + 1
            if not (0 <= nr < rows and 0 <= nc < cols) or not free[nr][nc]:
                continue
            if (nr, nc, nt) in vertex_c:
                continue
            if (dr, dc) != (0, 0) and (r, c, nr, nc, t) in edge_c:
                continue

            ng    = g + 1
            state = (nr, nc, nt)
            if ng < g_score.get(state, float('inf')):
                g_score[state]   = ng
                came_from[state] = (r, c, t)
                heapq.heappush(open_heap, (ng + _manhattan((nr, nc), goal), ng, nr, nc, nt))

    return None


# ── High level: conflict detection + constraint-tree search ─────────────────

def _pad(path: list, length: int) -> list:
    return path if len(path) >= length else path + [path[-1]] * (length - len(path))


def _first_conflict(paths: dict):
    agents  = list(paths.keys())
    max_len = max(len(p) for p in paths.values())
    padded  = {a: _pad(p, max_len) for a, p in paths.items()}

    for t in range(max_len):
        seen = {}
        for a in agents:
            cell = padded[a][t]
            if cell in seen:
                return ('vertex', seen[cell], a, cell, t)
            seen[cell] = a
        if t + 1 < max_len:
            for i in range(len(agents)):
                for j in range(i + 1, len(agents)):
                    a, b = agents[i], agents[j]
                    if (padded[a][t] == padded[b][t + 1] and padded[a][t + 1] == padded[b][t]
                            and padded[a][t] != padded[a][t + 1]):
                        return ('edge', a, b, padded[a][t], padded[a][t + 1], t)
    return None


def _cbs_solve(free, rows: int, cols: int, agents: dict,
               max_horizon: int, max_nodes: int, logger=None):
    """agents = {name: {'start': (r,c), 'goal': (r,c)}}.  Returns
    (paths_by_agent, nodes_expanded) or (None, nodes_expanded) on failure."""
    counter = itertools.count()

    def solve_agent(name, constraints):
        c = constraints[name]
        return _low_level_astar(free, rows, cols, agents[name]['start'], agents[name]['goal'],
                                 c['vertex'], c['edge'], max_horizon)

    root_constraints = {a: {'vertex': set(), 'edge': set()} for a in agents}
    root_paths = {}
    for a in agents:
        p = solve_agent(a, root_constraints)
        if p is None:
            if logger:
                logger(f'[cbs] agent {a} has no path to its goal at all — aborting')
            return None, 0
        root_paths[a] = p
    root_cost = sum(len(p) - 1 for p in root_paths.values())

    open_heap = [(root_cost, next(counter), root_constraints, root_paths)]
    nodes = 0

    while open_heap and nodes < max_nodes:
        cost, _, constraints, paths = heapq.heappop(open_heap)
        nodes += 1

        conflict = _first_conflict(paths)
        if conflict is None:
            if logger:
                logger(f'[cbs] conflict-free solution found after {nodes} node(s), cost={cost}')
            return paths, nodes

        if conflict[0] == 'vertex':
            _, a1, a2, cell, t = conflict
            branches = [(a1, ('vertex', cell, t)), (a2, ('vertex', cell, t))]
        else:
            _, a1, a2, cell1, cell2, t = conflict
            branches = [(a1, ('edge', cell1, cell2, t)), (a2, ('edge', cell2, cell1, t))]

        for agent, cons in branches:
            new_constraints = {a: {'vertex': set(v['vertex']), 'edge': set(v['edge'])}
                                for a, v in constraints.items()}
            if cons[0] == 'vertex':
                _, cell, t = cons
                new_constraints[agent]['vertex'].add((cell[0], cell[1], t))
            else:
                _, c1, c2, t = cons
                new_constraints[agent]['edge'].add((c1[0], c1[1], c2[0], c2[1], t))

            new_path = solve_agent(agent, new_constraints)
            if new_path is None:
                continue

            new_paths = dict(paths)
            new_paths[agent] = new_path
            new_cost = sum(len(p) - 1 for p in new_paths.values())
            heapq.heappush(open_heap, (new_cost, next(counter), new_constraints, new_paths))

    if logger:
        logger(f'[cbs] gave up after {nodes} node(s) — no conflict-free plan within budget')
    return None, nodes


# ── ROS node ──────────────────────────────────────────────────────────────────

def _make_pose(x: float, y: float, yaw: float, stamp, frame: str) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp    = stamp
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


class CBSPlanner(Node):
    def __init__(self):
        super().__init__('cbs_planner')

        self.declare_parameter(
            'robots_json', '[]',
            ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('map_topic',          '/map')
        self.declare_parameter('map_frame',          'map')
        self.declare_parameter('base_frame',         'base_link')
        self.declare_parameter('cell_size',          0.4)
        self.declare_parameter('robot_radius',       0.3)
        self.declare_parameter('max_planning_cells', 4000)
        self.declare_parameter('max_horizon',        200)
        self.declare_parameter('max_cbs_nodes',      300)
        self.declare_parameter('time_per_step',      0.6)
        self.declare_parameter('behavior_tree',      '')
        self.declare_parameter('auto_plan',          True)

        raw = self.get_parameter('robots_json').value
        if isinstance(raw, list):
            # CLI YAML parsing reads e.g. `-p robots_json:=[]` as an empty
            # list rather than the string '[]' the launch-file form uses.
            self._agents_cfg = raw
        else:
            try:
                self._agents_cfg = json.loads(raw) if raw else []
            except json.JSONDecodeError as e:
                self.get_logger().error(f'Bad robots_json: {e}')
                self._agents_cfg = []

        self._map_topic     = self.get_parameter('map_topic').value
        self._map_frame     = self.get_parameter('map_frame').value
        self._base_frame    = self.get_parameter('base_frame').value
        self._cell_size     = self.get_parameter('cell_size').value
        self._robot_radius  = self.get_parameter('robot_radius').value
        self._max_cells     = self.get_parameter('max_planning_cells').value
        self._max_horizon   = self.get_parameter('max_horizon').value
        self._max_nodes     = self.get_parameter('max_cbs_nodes').value
        self._time_per_step = self.get_parameter('time_per_step').value
        self._behavior_tree = self.get_parameter('behavior_tree').value
        self._auto_plan     = self.get_parameter('auto_plan').value

        self._map: OccupancyGrid | None = None
        self._planned    = False
        self._plan_timer = None

        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)

        self._nav_clients: dict[str, ActionClient] = {}

        self.create_subscription(OccupancyGrid, self._map_topic, self._map_cb, _MAP_QOS)
        self.create_subscription(Empty, '/cbs_planner/plan', self._trigger_cb, 10)

        self.get_logger().info(
            f'CBSPlanner ready — {len(self._agents_cfg)} robot(s) configured, '
            f'cell_size={self._cell_size}m  waiting for map on {self._map_topic} …')

    # ── Triggers ─────────────────────────────────────────────────────────────

    def _map_cb(self, msg: OccupancyGrid):
        self._map = msg
        if self._auto_plan and not self._planned:
            self._planned = True
            self.get_logger().info(
                f'[cbs] map received: {msg.info.width}x{msg.info.height} '
                f'res={msg.info.resolution:.3f}m — planning in 1s (letting TF settle)')
            self._plan_timer = self.create_timer(1.0, self._auto_plan_cb)

    def _auto_plan_cb(self):
        self._plan_timer.cancel()
        self._plan_once()

    def _trigger_cb(self, _):
        self.get_logger().info('[cbs] manual replan triggered')
        self._plan_once()

    # ── Planning ─────────────────────────────────────────────────────────────

    def _plan_once(self):
        if self._map is None:
            self.get_logger().warn('[cbs] no map yet — cannot plan')
            return
        if not self._agents_cfg:
            self.get_logger().warn('[cbs] robots_json is empty — nothing to plan')
            return

        log = self.get_logger().info
        free, rows, cols, ox, oy, cs = _build_planning_grid(
            self._map, self._cell_size, self._robot_radius, self._max_cells, logger=log)
        log(f'[cbs] planning grid {rows}x{cols} cells @ {cs:.2f}m  '
            f'({int(free.sum())}/{rows * cols} free)')

        agents = {}
        for cfg in self._agents_cfg:
            ns   = cfg['ns']
            base = f'{ns}/{self._base_frame}' if ns else self._base_frame
            try:
                tf = self._tf_buf.lookup_transform(
                    self._map_frame, base, rclpy.time.Time(), timeout=Duration(seconds=2.0))
            except Exception as e:
                self.get_logger().error(f'[cbs] TF lookup failed for {ns or "(default)"}: {e}')
                return

            sx = tf.transform.translation.x
            sy = tf.transform.translation.y
            start = _world_to_cell(sx, sy, ox, oy, cs)
            goal  = _world_to_cell(float(cfg['goal_x']), float(cfg['goal_y']), ox, oy, cs)

            if not (0 <= start[0] < rows and 0 <= start[1] < cols) or not free[start[0]][start[1]]:
                self.get_logger().error(f'[cbs] {ns}: start cell {start} out of bounds/occupied')
                return
            if not (0 <= goal[0] < rows and 0 <= goal[1] < cols) or not free[goal[0]][goal[1]]:
                self.get_logger().error(f'[cbs] {ns}: goal cell {goal} out of bounds/occupied')
                return

            agents[ns] = {'start': start, 'goal': goal}
            log(f'[cbs] {ns}: start={start} goal={goal}  '
                f'(world start=({sx:.2f},{sy:.2f}))')

        t0 = time.time()
        paths, nodes = _cbs_solve(free, rows, cols, agents, self._max_horizon, self._max_nodes,
                                   logger=log)
        dt = time.time() - t0

        if paths is None:
            self.get_logger().error(
                f'[cbs] FAILED to find a conflict-free plan for {len(agents)} robot(s) '
                f'after {nodes} node(s), {dt:.2f}s — no goals sent')
            return

        log(f'[cbs] solved for {len(agents)} robot(s) in {nodes} node(s), {dt:.2f}s')
        for ns, cellpath in paths.items():
            self._dispatch(ns, cellpath, ox, oy, cs)

    # ── Dispatch via NavigateThroughPoses ────────────────────────────────────

    def _dispatch(self, ns: str, cellpath: list, ox: float, oy: float, cs: float):
        now = self.get_clock().now().to_msg()

        poses = []
        last_yaw = 0.0
        for i, (r, c) in enumerate(cellpath):
            x, y = _cell_to_world(r, c, ox, oy, cs)
            if i + 1 < len(cellpath) and cellpath[i + 1] != (r, c):
                nr, nc = cellpath[i + 1]
                nx, ny = _cell_to_world(nr, nc, ox, oy, cs)
                last_yaw = math.atan2(ny - y, nx - x)
            poses.append(_make_pose(x, y, last_yaw, now, self._map_frame))

        action_topic = f'/{ns}/navigate_through_poses' if ns else '/navigate_through_poses'
        client = self._nav_clients.get(ns)
        if client is None:
            client = ActionClient(self, NavigateThroughPoses, action_topic)
            self._nav_clients[ns] = client

        self.get_logger().info(
            f'[cbs] {ns}: dispatching {len(poses)}-waypoint route '
            f'(~{(len(cellpath) - 1) * self._time_per_step:.1f}s) to {action_topic}')

        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'[cbs] {ns}: NavigateThroughPoses server not available at {action_topic}')
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        goal.behavior_tree = self._behavior_tree

        future = client.send_goal_async(goal)
        future.add_done_callback(lambda f, n=ns: self._goal_resp_cb(f, n))

    def _goal_resp_cb(self, future, ns: str):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(f'[cbs] {ns}: NavigateThroughPoses goal REJECTED')
            return
        self.get_logger().info(f'[cbs] {ns}: NavigateThroughPoses goal accepted')
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f, n=ns: self._result_cb(f, n))

    def _result_cb(self, future, ns: str):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'[cbs] {ns}: NavigateThroughPoses SUCCEEDED')
        else:
            self.get_logger().warn(f'[cbs] {ns}: NavigateThroughPoses finished with status={status}')


def main(args=None):
    rclpy.init(args=args)
    node = CBSPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
