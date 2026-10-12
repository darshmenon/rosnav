#!/usr/bin/env python3
"""
slam_posegraph_autosave.py — periodically save slam_toolbox's pose graph so a crashed or
stalled exploration run can be continued instead of restarted from scratch.

Calls /slam_toolbox/serialize_map every `period_sec`, overwriting <path>.posegraph and
<path>.data in place. Resume later by starting slam_toolbox with map_file_name:=<path>
(and a map_start_pose / map_start_at_dock) — the occupancy grid is rebuilt from the graph.

Parameters:
  path        (str,   '')    output prefix (no extension); empty = node does nothing
  period_sec  (float, 60.0)  save interval
  service     (str, '/slam_toolbox/serialize_map')
"""

import os
import time

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SerializePoseGraph


class PoseGraphAutosave(Node):
    def __init__(self):
        super().__init__('slam_posegraph_autosave')
        self.declare_parameter('path', '')
        self.declare_parameter('period_sec', 60.0)
        self.declare_parameter('service', '/slam_toolbox/serialize_map')
        self._path = str(self.get_parameter('path').value).strip()
        period = float(self.get_parameter('period_sec').value)
        if not self._path:
            self.get_logger().warn('path is empty — pose-graph autosave disabled')
            return
        os.makedirs(os.path.dirname(os.path.abspath(self._path)), exist_ok=True)
        self._cli = self.create_client(SerializePoseGraph, str(self.get_parameter('service').value))
        self._busy = False
        self._call_t = 0.0
        self._saves = 0
        self.create_timer(period, self._save)
        self.get_logger().info(f'saving pose graph to {self._path}.posegraph every {period:.0f}s')

    def _save(self):
        if self._busy:
            # slam_toolbox may have been killed/restarted (slam_stall_watchdog) with a call pending:
            # that future never completes, so give up on it after a while instead of waiting forever.
            if time.monotonic() - self._call_t < 45.0:
                self.get_logger().warn('previous save still running — skipping this cycle')
                return
            self.get_logger().warn('previous save never finished (slam_toolbox restarted?) — retrying')
            self._busy = False
        if not self._cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('slam_toolbox serialize_map service not available yet')
            return
        self._busy = True
        self._call_t = time.monotonic()
        req = SerializePoseGraph.Request()
        req.filename = self._path
        self._cli.call_async(req).add_done_callback(self._done)

    def _done(self, fut):
        self._busy = False   # (an abandoned call that completes late is harmless)
        res = fut.result()
        if res is not None and res.result == 0:
            self._saves += 1
            size = os.path.getsize(self._path + '.posegraph') if os.path.exists(self._path + '.posegraph') else 0
            self.get_logger().info(f'pose graph saved (#{self._saves}, {size/1e6:.1f} MB)')
        else:
            self.get_logger().error(f'serialize_map failed: {res}')


def main():
    rclpy.init()
    node = PoseGraphAutosave()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
