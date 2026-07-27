#!/usr/bin/env python3
"""Compat alias for map_merge_known.py (slam_mode:=multi)."""
import pathlib
import runpy

runpy.run_path(
    str(pathlib.Path(__file__).with_name('map_merge_known.py')),
    run_name='__main__',
)
