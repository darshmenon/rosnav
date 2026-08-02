#!/usr/bin/env python3
"""
rmf_fleet_adapter.py — thin entry point for the modular Open-RMF adapter.

Customize for new maps / docks / robots via:
  config/rmf_fleet.yaml
  config/locations.yaml

Optional overrides:
  --config PATH
  --docking aruco|noop|module.path:Class
  --robots robot1,robot2
"""

from diff_drive_robot.rmf.adapter import main

if __name__ == '__main__':
    main()
