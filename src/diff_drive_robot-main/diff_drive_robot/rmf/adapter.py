"""Assemble and run the Open-RMF fleet adapter from RmfFleetConfig."""

from __future__ import annotations

import sys
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry
import rmf_adapter.battery as battery
import rmf_adapter.plan as plan

from diff_drive_robot.rmf.config import RmfFleetConfig, load_rmf_config
from diff_drive_robot.rmf.docking import get_docking_plugin
from diff_drive_robot.rmf.graph_builder import build_graph
from diff_drive_robot.rmf.robot_command import Nav2RobotCommand, yaw_from_quat


def _accept_all(json_desc):
    confirmation = adpt.fleet_update_handle.Confirmation()
    confirmation.accept()
    print(f'[rmf] accepted task request: {json_desc}')
    return confirmation


def run_adapter(
    cfg: RmfFleetConfig,
    robot_namespaces: List[str],
    fleet_name: Optional[str] = None,
    map_name: Optional[str] = None,
) -> None:
    """Build graph, register robots, spin until shutdown."""
    if fleet_name:
        cfg.fleet_name = fleet_name
    if map_name:
        cfg.map_name = map_name

    if not robot_namespaces:
        print('[rmf] no robot namespaces given, exiting')
        sys.exit(1)
    if not cfg.locations:
        print(f'[rmf] no locations loaded from {cfg.locations_file!r}, exiting')
        sys.exit(1)
    if cfg.charger not in cfg.locations:
        print(f'[rmf] warning: charger {cfg.charger!r} missing from locations')

    rclpy.init()
    try:
        adpt.init_rclcpp()
    except RuntimeError:
        pass

    rmf_graph, index_of = build_graph(cfg)
    docking = get_docking_plugin(cfg.docking_plugin, args=cfg.docking_args)
    print(f'[rmf] docking plugin: {cfg.docking_plugin} ({type(docking).__name__})')

    profile = traits.Profile(geometry.make_final_convex_circle(cfg.robot_radius))
    robot_traits = traits.VehicleTraits(
        linear=traits.Limits(*cfg.linear_limits),
        angular=traits.Limits(*cfg.angular_limits),
        profile=profile)

    adapter = adpt.Adapter.make('rmf_fleet_adapter')
    fleet = adapter.add_fleet(cfg.fleet_name, robot_traits, rmf_graph)

    if cfg.accept_patrol:
        fleet.consider_patrol_requests(_accept_all)
    if cfg.accept_composed:
        fleet.consider_composed_requests(_accept_all)
    if cfg.accept_delivery:
        fleet.consider_delivery_requests(_accept_all, _accept_all)

    battery_sys = battery.BatterySystem.make(*cfg.battery_voltage_capacity_charging)
    mech_sys = battery.MechanicalSystem.make(*cfg.mechanical_system)
    motion_sink = battery.SimpleMotionPowerSink(battery_sys, mech_sys)
    ambient_sink = battery.SimpleDevicePowerSink(
        battery_sys, battery.PowerSystem.make(cfg.ambient_power_w))
    tool_sink = battery.SimpleDevicePowerSink(
        battery_sys, battery.PowerSystem.make(cfg.tool_power_w))

    ok = fleet.set_task_planner_params(
        battery_sys, motion_sink, ambient_sink, tool_sink,
        cfg.recharge_threshold, cfg.recharge_soc, cfg.account_for_drain)
    if not ok:
        print('[rmf] set_task_planner_params failed, exiting')
        sys.exit(1)

    cmd_node = Node(
        'rmf_fleet_adapter_cmd',
        parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, cfg.use_sim_time),
        ],
    )
    commands = []

    for ns in robot_namespaces:
        cmd = Nav2RobotCommand(cmd_node, ns, rmf_graph, cfg, docking)
        commands.append(cmd)

        pose = cmd.latest_start()
        starts = []
        if pose is not None:
            x = pose.pose.pose.position.x
            y = pose.pose.pose.position.y
            yaw = yaw_from_quat(pose.pose.pose.orientation)
            starts = plan.compute_plan_starts(
                rmf_graph, cfg.map_name, [[x], [y], [yaw]], adapter.now()) or []
            if not starts:
                print(f'[rmf] {ns}: could not place ({x:.2f},{y:.2f}) on the nav graph '
                      f'— falling back to "origin"')
        else:
            print(f'[rmf] {ns}: no /{ns}/amcl_pose received in time — falling back to "origin"')

        if not starts:
            origin_idx = index_of.get('origin', 0)
            starts = [plan.Start(adapter.now(), origin_idx, 0.0)]

        def handle_cb(updater, cmd=cmd, charger=cfg.charger):
            cmd.updater = updater
            updater.update_battery_soc(1.0)
            if charger in index_of:
                updater.set_charger_waypoint(index_of[charger])

        fleet.add_robot(cmd, ns, profile, starts, handle_cb)
        print(f'[rmf] registered {ns} with fleet {cfg.fleet_name!r}')

    adapter.start()
    print(f'[rmf] fleet {cfg.fleet_name!r} live: {robot_namespaces} on map {cfg.map_name!r} '
          f'(config={cfg.config_path})')

    try:
        rclpy.spin(cmd_node)
    except KeyboardInterrupt:
        pass
    finally:
        adapter.stop()
        cmd_node.destroy_node()
        rclpy.shutdown()


def main(argv=None) -> None:
    import argparse
    parser = argparse.ArgumentParser(
        description='Open-RMF fleet adapter (config-driven). '
                    'Edit config/rmf_fleet.yaml to customize.')
    parser.add_argument(
        '--config', default=None,
        help='Path to rmf_fleet.yaml (default: share/diff_drive_robot/config/rmf_fleet.yaml)')
    parser.add_argument('--fleet-name', default=None, help='Override fleet_name from config')
    parser.add_argument('--map-name', default=None, help='Override map_name from config')
    parser.add_argument(
        '--robots', default='robot1,robot2',
        help='Comma-separated robot namespaces')
    parser.add_argument(
        '--docking', default=None,
        help='Override docking.plugin (aruco|noop|module:Class)')
    args, _ = parser.parse_known_args(argv)

    cfg = load_rmf_config(args.config)
    if args.docking:
        cfg.docking_plugin = args.docking

    robots = [r.strip() for r in args.robots.split(',') if r.strip()]
    run_adapter(cfg, robots, fleet_name=args.fleet_name, map_name=args.map_name)
