#!/usr/bin/env python3
"""
Geofence Manager for Nav2 — monitors robot pose and publishes alerts
when the robot enters a forbidden polygon zone.

Topics published:
  /geofence/breach        (std_msgs/Bool)   — True when inside a forbidden zone
  /geofence/current_zone  (std_msgs/String) — name of the current zone

Parameters:
  geofence_file (str) — path to a YAML file defining zones (see config/geofences.yaml)
"""

import os

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String


def _point_in_polygon(x: float, y: float, polygon: list) -> bool:
    """Ray-casting point-in-polygon test (no external deps)."""
    inside = False
    px, py = polygon[-1]
    for cx, cy in polygon:
        if ((cy > y) != (py > y)) and (x < (px - cx) * (y - cy) / (py - cy) + cx):
            inside = not inside
        px, py = cx, cy
    return inside


class GeofenceManager(Node):
    def __init__(self):
        super().__init__("geofence_manager")

        self.declare_parameter("geofence_file", "")
        geofence_file = self.get_parameter("geofence_file").value
        self.zones = self._load_zones(geofence_file)

        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._pose_callback, 10
        )
        self.alert_pub = self.create_publisher(Bool, "/geofence/breach", 10)
        self.zone_pub = self.create_publisher(String, "/geofence/current_zone", 10)

        self.get_logger().info(
            f"GeofenceManager ready — {len(self.zones)} zone(s) loaded"
        )

    def _load_zones(self, path: str) -> list:
        if not path or not os.path.exists(path):
            self.get_logger().warn("No geofence file provided — running with no zones")
            return []
        with open(path) as f:
            data = yaml.safe_load(f)
        zones = []
        for z in data.get("zones", []):
            pts = [(p["x"], p["y"]) for p in z["polygon"]]
            zones.append(
                {
                    "name": z["name"],
                    "polygon": pts,
                    "allowed": z.get("allowed", True),
                }
            )
        return zones

    def _pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        breached = False
        zone_name = "outside"

        for zone in self.zones:
            if _point_in_polygon(x, y, zone["polygon"]):
                zone_name = zone["name"]
                if not zone["allowed"]:
                    breached = True
                    self.get_logger().warn(
                        f'Geofence breach: robot entered forbidden zone "{zone_name}"'
                    )
                break

        alert = Bool()
        alert.data = breached
        self.alert_pub.publish(alert)

        zname = String()
        zname.data = zone_name
        self.zone_pub.publish(zname)


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
