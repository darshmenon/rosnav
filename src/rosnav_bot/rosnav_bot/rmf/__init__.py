"""
Modular Open-RMF integration for rosnav_bot.

Layout
──────
  config.py         Load config/rmf_fleet.yaml + locations.yaml
  graph_builder.py  Build the RMF nav graph from locations + lanes
  docking.py        Pluggable docking backends (aruco / noop / custom)
  robot_command.py  Nav2 ↔ RMF RobotCommandHandle
  adapter.py        Assemble Adapter + fleet + robots

New users typically only edit ``config/rmf_fleet.yaml`` and
``config/locations.yaml``. To add a custom dock:

  1. Subclass ``DockingPlugin`` in your own module
  2. Set ``docking.plugin: my_pkg.my_dock:MyDock`` in rmf_fleet.yaml
"""

from rosnav_bot.rmf.config import RmfFleetConfig, load_rmf_config
from rosnav_bot.rmf.docking import DockingPlugin, get_docking_plugin

__all__ = [
    'RmfFleetConfig',
    'load_rmf_config',
    'DockingPlugin',
    'get_docking_plugin',
]
