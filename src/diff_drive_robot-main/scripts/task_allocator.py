#!/usr/bin/env python3
"""
task_allocator.py — Multi-robot task allocation via nearest-idle-robot auction.

Maintains a shared task queue.  When a robot becomes idle (mission_server
reports DONE/FAILED/IDLE), the allocator assigns it the nearest pending task
by Euclidean distance to the robot's current map position (from TF or odom).

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
import math
import sys
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

PENDING  = 'pending'
ASSIGNED = 'assigned'
DONE     = 'done'


class TaskAllocator(Node):
    def __init__(self):
        super().__init__('task_allocator')

        self.declare_parameter('robots', '')          # comma-sep list; '' = auto-discover

        robots_param = self.get_parameter('robots').value
        self._explicit_robots = (
            [r.strip() for r in robots_param.split(',') if r.strip()]
            if robots_param else []
        )

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

            # If robot just finished (DONE/FAILED/IDLE), mark its task done
            if prev == 'NAVIGATING' and state in ('DONE', 'FAILED', 'IDLE'):
                for t in self._tasks:
                    if t['robot'] == robot and t['status'] == ASSIGNED:
                        t['status'] = DONE
                        self.get_logger().info(
                            f'Task {t["id"]} marked {DONE} (robot={robot}, nav={state})')

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

            for robot in idle_robots:
                if not pending:
                    break

                rx, ry = self._robot_poses.get(robot, (0.0, 0.0))

                # Pick nearest pending task to this robot
                best = min(pending,
                           key=lambda t: (t['x'] - rx) ** 2 + (t['y'] - ry) ** 2)
                best['status'] = ASSIGNED
                best['robot']  = robot
                pending.remove(best)

                payload = {
                    'type':      'goto',
                    'robot':     robot,
                    'pose':      [best['x'], best['y'], best['yaw']],
                    'waypoints': [],
                }
                msg       = String()
                msg.data  = json.dumps(payload)
                self._mission_pub.publish(msg)
                self.get_logger().info(
                    f'Assigned task {best["id"]} → {robot}  '
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
                print(f'  {t["id"]}  ({t["x"]:.2f},{t["y"]:.2f},{t["yaw"]:.0f}°)'
                      f'{label}  {t["status"]}{robot}')
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
    argv = sys.argv[1:]

    if not argv or argv[0] == '--daemon':
        rclpy.init()
        node = TaskAllocator()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
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
    rclpy.shutdown()


if __name__ == '__main__':
    main()
