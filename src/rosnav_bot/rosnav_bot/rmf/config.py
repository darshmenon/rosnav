"""Load Open-RMF fleet settings from YAML (config/rmf_fleet.yaml)."""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

PKG = 'rosnav_bot'


def package_share_dir() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(PKG)
    except Exception:
        return os.path.join(
            os.path.expanduser('~'), 'rosnav', 'src', 'rosnav_bot')


def _resolve(path: str, base_dir: Optional[str] = None) -> str:
    if os.path.isabs(path):
        return path
    bases = []
    if base_dir:
        bases.append(base_dir)
    share = package_share_dir()
    bases.append(os.path.join(share, 'config'))
    bases.append(share)
    bases.append(os.path.join(os.path.expanduser('~'), 'rosnav'))
    for b in bases:
        candidate = os.path.join(b, path)
        if os.path.isfile(candidate):
            return candidate
    return os.path.join(bases[0], path) if bases else path


@dataclass
class RmfFleetConfig:
    fleet_name: str = 'rosnav_fleet'
    map_name: str = 'hospital'
    charger: str = 'charging_dock'
    locations_file: str = 'locations.yaml'
    lanes: List[Tuple[str, str]] = field(default_factory=list)
    locations: Dict[str, List[float]] = field(default_factory=dict)

    robot_radius: float = 0.22
    linear_limits: Tuple[float, float] = (0.5, 2.5)
    angular_limits: Tuple[float, float] = (2.5, 3.2)

    docking_plugin: str = 'aruco'
    docking_args: List[str] = field(default_factory=list)

    goal_timeout_sec: float = 5.0
    max_retries: int = 3
    pose_wait_sec: float = 5.0
    use_sim_time: bool = True

    battery_voltage_capacity_charging: Tuple[float, float, float] = (24.0, 40.0, 8.8)
    mechanical_mass_inertia: Tuple[float, float] = (70.0, 40.0)
    ambient_power_w: float = 20.0
    tool_power_w: float = 10.0
    recharge_threshold: float = 0.2
    recharge_soc: float = 1.0
    account_for_drain: bool = False

    accept_patrol: bool = True
    accept_composed: bool = True
    accept_delivery: bool = True

    config_path: str = ''

    @property
    def mechanical_system(self) -> Tuple[float, float, float]:
        m, i = self.mechanical_mass_inertia
        return (m, i, self.robot_radius)


def _as_pair(value: Any, default: Tuple[float, float]) -> Tuple[float, float]:
    if not value or len(value) < 2:
        return default
    return (float(value[0]), float(value[1]))


def _as_triple(value: Any, default: Tuple[float, float, float]) -> Tuple[float, float, float]:
    if not value or len(value) < 3:
        return default
    return (float(value[0]), float(value[1]), float(value[2]))


def load_locations(path: str) -> Dict[str, List[float]]:
    if not HAS_YAML or not os.path.isfile(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    locs = data.get('locations', {}) or {}
    return {str(k): list(v) for k, v in locs.items()}


def load_rmf_config(path: Optional[str] = None) -> RmfFleetConfig:
    """Load ``rmf_fleet.yaml`` (and the locations file it references)."""
    if not HAS_YAML:
        raise RuntimeError('PyYAML is required to load rmf_fleet.yaml')

    if path is None:
        path = _resolve('rmf_fleet.yaml')
    else:
        path = _resolve(path) if not os.path.isabs(path) else path

    if not os.path.isfile(path):
        raise FileNotFoundError(f'rmf config not found: {path}')

    with open(path) as f:
        raw = yaml.safe_load(f) or {}

    cfg_dir = os.path.dirname(os.path.abspath(path))
    vehicle = raw.get('vehicle', {}) or {}
    docking = raw.get('docking', {}) or {}
    nav = raw.get('nav', {}) or {}
    battery = raw.get('battery', {}) or {}
    tasks = raw.get('tasks', {}) or {}

    lanes_raw = raw.get('lanes', []) or []
    lanes: List[Tuple[str, str]] = []
    for entry in lanes_raw:
        if isinstance(entry, (list, tuple)) and len(entry) >= 2:
            lanes.append((str(entry[0]), str(entry[1])))

    locations_file = str(raw.get('locations_file', 'locations.yaml'))
    locations_path = _resolve(locations_file, base_dir=cfg_dir)
    locations = load_locations(locations_path)

    radius = float(vehicle.get('radius', 0.22))
    cfg = RmfFleetConfig(
        fleet_name=str(raw.get('fleet_name', 'rosnav_fleet')),
        map_name=str(raw.get('map_name', 'hospital')),
        charger=str(raw.get('charger', 'charging_dock')),
        locations_file=locations_path,
        lanes=lanes,
        locations=locations,
        robot_radius=radius,
        linear_limits=_as_pair(vehicle.get('linear'), (0.5, 2.5)),
        angular_limits=_as_pair(vehicle.get('angular'), (2.5, 3.2)),
        docking_plugin=str(docking.get('plugin', 'aruco')),
        docking_args=[str(a) for a in (docking.get('args') or [])],
        goal_timeout_sec=float(nav.get('goal_timeout_sec', 5.0)),
        max_retries=int(nav.get('max_retries', 3)),
        pose_wait_sec=float(nav.get('pose_wait_sec', 5.0)),
        use_sim_time=bool(nav.get('use_sim_time', True)),
        battery_voltage_capacity_charging=_as_triple(
            battery.get('voltage_capacity_charging'), (24.0, 40.0, 8.8)),
        mechanical_mass_inertia=_as_pair(battery.get('mechanical'), (70.0, 40.0)),
        ambient_power_w=float(battery.get('ambient_power_w', 20.0)),
        tool_power_w=float(battery.get('tool_power_w', 10.0)),
        recharge_threshold=float(battery.get('recharge_threshold', 0.2)),
        recharge_soc=float(battery.get('recharge_soc', 1.0)),
        account_for_drain=bool(battery.get('account_for_drain', False)),
        accept_patrol=bool(tasks.get('accept_patrol', True)),
        accept_composed=bool(tasks.get('accept_composed', True)),
        accept_delivery=bool(tasks.get('accept_delivery', True)),
        config_path=path,
    )
    return cfg
