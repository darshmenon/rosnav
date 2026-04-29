#!/usr/bin/env python3
"""
task_allocator.py — Multi-robot task allocation with Hungarian optimal assignment.

Maintains a shared task queue.  When one or more robots become idle, the
allocator solves an optimal assignment problem (Hungarian / Munkres algorithm)
that minimises total travel distance across all idle robots × pending tasks.

With only one idle robot, this reduces to nearest-task selection.
With N idle robots and M≥N tasks, it finds the globally optimal batch assignment.

Topics
──────
  /task_queue/add    (std_msgs/String JSON in)   — add a task to the queue
  /task_queue/state  (std_msgs/String JSON out)  — queue + assignment state at 1Hz
  /mission/state     (std_msgs/String JSON in)   — robot states from mission_server
  /mission/execute   (std_msgs/String JSON out)  — goto commands to mission_server
  /<ns>/odom         (nav_msgs/Odometry in)      — robot position fallback

Task JSON format (add)
──────────────────────
  {"x": 2.0, "y": 1.5, "yaw": 0}                  — single pose task
  {"x": 2.0, "y": 1.5, "yaw": 0, "id": "dock_A"}  — with optional label

Fleet discovery
───────────────
  Robots are detected from live /*/cmd_vel topics (same as fleet_manager).
  Or pass explicit robots: --ros-args -p robots:=robot1,robot2,robot3

Usage
─────
  # Start allocator daemon:
  ros2 run diff_drive_robot task_allocator.py

  # Add tasks to queue:
  ros2 run diff_drive_robot task_allocator.py add 2.0 1.5 0
  ros2 run diff_drive_robot task_allocator.py add 3.5 -1.0 90 dock_B

  # Check queue state:
  ros2 run diff_drive_robot task_allocator.py status

  # Clear queue:
  ros2 run diff_drive_robot task_allocator.py clear

  # Or via fleet_manager:
  ros2 run diff_drive_robot fleet_manager.py tasks add 2.0 1.5 0
  ros2 run diff_drive_robot fleet_manager.py tasks status
"""

import json
import sys
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from nav_msgs.msg import Odometry
from std_msgs.msg import String


# ── Pure-Python Hungarian algorithm (O(n³)) ──────────────────────────────────
# Finds the minimum-cost assignment for an NxM cost matrix (N ≤ M).
# Returns a list of (row, col) pairs — one per row — giving the optimal match.

def _hungarian(cost: list[list[float]]) -> list[tuple[int, int]]:
    n = len(cost)
    m = len(cost[0])
    INF = float('inf')

    # Pad to square if needed
    if n < m:
        cost = [row[:] for row in cost] + [[INF] * m] * (m - n)
    size = len(cost)

    u = [0.0] * (size + 1)
    v = [0.0] * (size + 1)
    p = [0] * (size + 1)   # p[j] = row assigned to column j (1-indexed)
    way = [0] * (size + 1)

    for i in range(1, size + 1):
        p[0] = i
        j0 = 0
        minv = [INF] * (size + 1)
        used = [False] * (size + 1)
        while True:
            used[j0] = True
            i0, delta, j1 = p[j0], INF, -1
            for j in range(1, size + 1):
                if not used[j]:
                    cur = cost[i0 - 1][j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta, j1 = minv[j], j
            for j in range(size + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while j0:
            p[j0] = p[way[j0]]
            j0 = way[j0]

    # Extract assignments for original n rows only
    result = []
    for j in range(1, size + 1):
        r = p[j] - 1  # 0-indexed row
        c = j - 1     # 0-indexed col
        if r < n and c < m and cost[r][c] < INF:
            result.append((r, c))
    return result

PENDING  = 'pending'
ASSIGNED = 'assigned'
DONE     = 'done'
FAILED   = 'failed'


class TaskAllocator(Node):
    def __init__(self):
        super().__init__('task_allocator')

        self.declare_parameter('robots', '')          # comma-sep list; '' = auto-discover
        self.declare_parameter('max_retries', 3)

        robots_param = self.get_parameter('robots').value
        self._explicit_robots = (
            [r.strip() for r in robots_param.split(',') if r.strip()]
            if robots_param else []
        )
        self._max_retries = int(self.get_parameter('max_retries').value)

        self._lock         = threading.Lock()
        self._tasks: list[dict]        = []   # [{id, x, y, yaw, label, status, robot}]
        self._robot_states: dict[str, str]  = {}   # ns → mission state string
        self._robot_poses:  dict[str, tuple] = {}  # ns → (x, y)
        self._odom_subs:    dict[str, object] = {}

        # Subscriptions
        self.create_subscription(String, '/task_queue/add',   self._add_cb,     10)
        self.create_subscription(String, '/task_queue/clear', self._clear_cb,   10)
        self.create_subscription(String, '/mission/state',    self._mission_cb, 10)

        # Publishers
        self._state_pub   = self.create_publisher(String, '/task_queue/state',   10)
        self._mission_pub = self.create_publisher(String, '/mission/execute',    10)

        # Discovery + allocation loop at 2 Hz
        self.create_timer(0.5, self._tick)

        self.get_logger().info('TaskAllocator ready.')

    # ── Task intake ───────────────────────────────────────────────────────────

    def _add_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Bad task JSON: {e}')
            return

        task = {
            'id':     data.get('id', str(uuid.uuid4())[:8]),
            'x':      float(data['x']),
            'y':      float(data['y']),
            'yaw':    float(data.get('yaw', 0)),
            'label':  data.get('label', ''),
            'status': PENDING,
            'robot':  '',
            'retries': 0,
        }
        with self._lock:
            self._tasks.append(task)
        self.get_logger().info(
            f'Task added: id={task["id"]}  '
            f'({task["x"]:.2f}, {task["y"]:.2f}, {task["yaw"]:.0f}°)  '
            f'label={task["label"] or "—"}')

    def _clear_cb(self, _):
        with self._lock:
            cleared = len([t for t in self._tasks if t['status'] == PENDING])
            self._tasks = [t for t in self._tasks if t['status'] == ASSIGNED]
        self.get_logger().info(f'Cleared {cleared} pending tasks.')

    def _mission_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        robot = data.get('robot', '')
        state = data.get('state', '')
        if not robot:
            return

        with self._lock:
            prev = self._robot_states.get(robot, '')
            self._robot_states[robot] = state

            # Reconcile any assigned task when a mission finishes or is interrupted.
            if prev == 'NAVIGATING' and state in ('DONE', 'FAILED', 'IDLE'):
                for t in self._tasks:
                    if t['robot'] == robot and t['status'] == ASSIGNED:
                        if state == 'DONE':
                            t['status'] = DONE
                            self.get_logger().info(
                                f'Task {t["id"]} marked {DONE} (robot={robot})')
                        elif state == 'FAILED':
                            t['retries'] += 1
                            if t['retries'] >= self._max_retries:
                                t['status'] = FAILED
                                self.get_logger().warn(
                                    f'Task {t["id"]} marked {FAILED} after '
                                    f'{t["retries"]} attempt(s)')
                            else:
                                t['status'] = PENDING
                                self.get_logger().warn(
                                    f'Task {t["id"]} re-queued after mission failure '
                                    f'(attempt {t["retries"]}/{self._max_retries})')
                            t['robot'] = ''
                        else:
                            t['status'] = PENDING
                            t['robot'] = ''
                            self.get_logger().info(
                                f'Task {t["id"]} returned to queue after cancel/idle')

    # ── Allocation tick ───────────────────────────────────────────────────────

    def _tick(self):
        self._discover_robots()
        self._allocate()
        self._publish_state()

    def _discover_robots(self):
        if self._explicit_robots:
            robots = self._explicit_robots
        else:
            topics = self.get_topic_names_and_types()
            robots = sorted({
                t.split('/')[1]
                for t, _ in topics
                if t.count('/') >= 2 and t.endswith('/cmd_vel')
            })

        for ns in robots:
            if ns not in self._robot_states:
                self._robot_states[ns] = 'IDLE'
            if ns not in self._odom_subs:
                self._odom_subs[ns] = self.create_subscription(
                    Odometry, f'/{ns}/odom',
                    lambda msg, n=ns: self._odom_cb(msg, n), 10)

    def _odom_cb(self, msg: Odometry, ns: str):
        with self._lock:
            self._robot_poses[ns] = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            )

    def _allocate(self):
        with self._lock:
            pending = [t for t in self._tasks if t['status'] == PENDING]
            if not pending:
                return

            idle_robots = [
                ns for ns, st in self._robot_states.items()
                if st in ('IDLE', 'DONE', 'FAILED')
                and not any(t['robot'] == ns and t['status'] == ASSIGNED
                            for t in self._tasks)
            ]
            if not idle_robots:
                return

            # Build cost matrix: rows = robots, cols = tasks (Euclidean distance)
            cost = []
            for ns in idle_robots:
                rx, ry = self._robot_poses.get(ns, (0.0, 0.0))
                row = [((t['x'] - rx) ** 2 + (t['y'] - ry) ** 2) ** 0.5
                       for t in pending]
                cost.append(row)

            # Hungarian assignment minimises total distance
            assignments = _hungarian(cost)

            for robot_idx, task_idx in assignments:
                ns   = idle_robots[robot_idx]
                best = pending[task_idx]
                best['status'] = ASSIGNED
                best['robot']  = ns

                payload = {
                    'type':      'goto',
                    'robot':     ns,
                    'pose':      [best['x'], best['y'], best['yaw']],
                    'waypoints': [],
                }
                msg      = String()
                msg.data = json.dumps(payload)
                self._mission_pub.publish(msg)
                self.get_logger().info(
                    f'[hungarian] Assigned task {best["id"]} → {ns}  '
                    f'({best["x"]:.2f}, {best["y"]:.2f})')

    # ── State publisher ───────────────────────────────────────────────────────

    def _publish_state(self):
        with self._lock:
            payload = {
                'tasks':        self._tasks,
                'robot_states': self._robot_states,
            }
        msg      = String()
        msg.data = json.dumps(payload)
        self._state_pub.publish(msg)


# ── CLI helpers ────────────────────────────────────────────────────────────────

def _send(node: Node, topic: str, payload: dict):
    pub = node.create_publisher(String, topic, 10)
    time.sleep(0.5)
    msg      = String()
    msg.data = json.dumps(payload)
    pub.publish(msg)
    time.sleep(0.2)


def _status(node: Node):
    received = [None]

    def _cb(msg):
        received[0] = json.loads(msg.data)

    node.create_subscription(String, '/task_queue/state', _cb, 10)
    spin = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin.start()
    deadline = time.time() + 3.0
    while received[0] is None and time.time() < deadline:
        time.sleep(0.05)

    if received[0]:
        s = received[0]
        tasks = s.get('tasks', [])
        bots  = s.get('robot_states', {})
        print('\n── Task Queue ────────────────────────────────────')
        if tasks:
            for t in tasks:
                robot = f' → {t["robot"]}' if t['robot'] else ''
                label = f' [{t["label"]}]' if t['label'] else ''
                retry = f' retries={t["retries"]}' if t.get('retries') else ''
                print(f'  {t["id"]}  ({t["x"]:.2f},{t["y"]:.2f},{t["yaw"]:.0f}°)'
                      f'{label}  {t["status"]}{robot}{retry}')
        else:
            print('  (empty)')
        print('\n── Robot States ──────────────────────────────────')
        for ns, st in (bots.items() if bots else []):
            print(f'  {ns}: {st}')
        print('──────────────────────────────────────────────────\n')
    else:
        print('No task_allocator found (is it running?)')


def _usage():
    print(__doc__)
    sys.exit(0)


def main():
    argv = remove_ros_args(args=sys.argv)[1:]

    if not argv or argv[0] == '--daemon':
        rclpy.init()
        node = TaskAllocator()
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
        return

    rclpy.init()
    node = Node('task_allocator_cli')

    cmd = argv[0].lower()

    if cmd == 'status':
        _status(node)

    elif cmd == 'add':
        if len(argv) < 4:
            print('Usage: task_allocator.py add <x> <y> <yaw_deg> [label]')
            sys.exit(1)
        payload = {
            'x': float(argv[1]), 'y': float(argv[2]),
            'yaw': float(argv[3]),
            'label': argv[4] if len(argv) > 4 else '',
        }
        _send(node, '/task_queue/add', payload)
        print(f'Task queued: {payload}')

    elif cmd == 'clear':
        _send(node, '/task_queue/clear', {})
        print('Queue cleared.')

    else:
        _usage()

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
