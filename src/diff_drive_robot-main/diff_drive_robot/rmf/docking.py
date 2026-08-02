"""
Pluggable docking backends for the RMF fleet adapter.

Built-ins
─────────
  aruco  — ``ros2 run diff_drive_robot aruco_dock.py``
  noop   — immediately succeed (bring-up / no camera)

Custom plugins
──────────────
Set ``docking.plugin: my_package.my_module:MyClass`` in ``rmf_fleet.yaml``.
``MyClass`` must subclass ``DockingPlugin`` (or duck-type the same methods).
"""

from __future__ import annotations

import importlib
import signal
import subprocess
import threading
from abc import ABC, abstractmethod
from typing import Callable, Dict, List, Optional, Type

PKG = 'diff_drive_robot'

FinishedCb = Callable[[], None]
FailedCb = Callable[[str], None]


class DockingPlugin(ABC):
    """Interface for final-approach docking used by ``Nav2RobotCommand.dock``."""

    def __init__(self, args: Optional[List[str]] = None):
        self.args = list(args or [])

    @abstractmethod
    def start(
        self,
        namespace: str,
        dock_name: str,
        on_success: FinishedCb,
        on_failure: FailedCb,
        log=None,
    ) -> None:
        """Begin docking asynchronously; invoke exactly one of the callbacks."""

    def cancel(self) -> None:
        """Stop any in-flight docking attempt (optional for plugins)."""


class NoopDockingPlugin(DockingPlugin):
    """Confirm dock immediately — useful when bringing up RMF without a camera."""

    def start(self, namespace, dock_name, on_success, on_failure, log=None):
        if log:
            log.info(f'[{namespace}] noop dock({dock_name}) — confirming')
        on_success()


class ArucoDockingPlugin(DockingPlugin):
    """Hand off to ``aruco_dock.py`` for camera-guided final approach."""

    def __init__(self, args: Optional[List[str]] = None):
        super().__init__(args=args)
        self._lock = threading.Lock()
        self._proc: Optional[subprocess.Popen] = None
        self._cancelled = False
        self._cb_fired = False
        self._generation = 0

    def cancel(self) -> None:
        with self._lock:
            self._cancelled = True
            self._generation += 1
            proc = self._proc
        if proc is not None and proc.poll() is None:
            try:
                proc.send_signal(signal.SIGINT)
            except ProcessLookupError:
                return
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                try:
                    proc.kill()
                except ProcessLookupError:
                    pass

    def start(self, namespace, dock_name, on_success, on_failure, log=None):
        # Abort any previous attempt before starting a new one.
        self.cancel()
        with self._lock:
            self._cancelled = False
            self._cb_fired = False
            self._proc = None
            self._generation += 1
            generation = self._generation

        if log:
            log.info(f'[{namespace}] aruco dock({dock_name}) — starting visual approach')

        def _fire_once(ok: bool, detail: str = ''):
            with self._lock:
                if generation != self._generation or self._cb_fired:
                    return
                self._cb_fired = True
                cancelled = self._cancelled
            # Cancelled by stop()/interrupt — caller already knows; do not re-signal.
            if cancelled:
                if log:
                    log.warning(f'[{namespace}] visual dock cancelled')
                return
            if ok:
                on_success()
            else:
                on_failure(detail or 'visual dock failed')

        def _run():
            cmd = ['ros2', 'run', PKG, 'aruco_dock.py', '--ros-args']
            if namespace:
                cmd += ['-p', f'namespace:={namespace}']
            if dock_name:
                cmd += ['-p', f'dock_name:={dock_name}']
            cmd += ['-p', 'docks_file:=docks.yaml', '-p', 'prefer_restage:=true']
            if self.args:
                # User args are already ros-arg fragments (e.g. -p key:=val).
                cmd += list(self.args)
            try:
                proc = subprocess.Popen(cmd)
                with self._lock:
                    if generation != self._generation:
                        proc.send_signal(signal.SIGINT)
                        return
                    self._proc = proc
                    cancelled = self._cancelled
                if cancelled:
                    self.cancel()
                    _fire_once(False, 'visual dock cancelled')
                    return

                returncode = proc.wait()
                with self._lock:
                    stale = generation != self._generation
                    cancelled = self._cancelled
                    if self._proc is proc:
                        self._proc = None

                if stale or cancelled:
                    _fire_once(False, 'visual dock cancelled')
                elif returncode == 0:
                    if log:
                        log.info(f'[{namespace}] visual dock complete')
                    _fire_once(True)
                else:
                    _fire_once(False, f'visual dock failed (exit {returncode})')
            except Exception as e:
                with self._lock:
                    if self._proc is not None and generation == self._generation:
                        self._proc = None
                _fire_once(False, f'visual dock error: {e}')

        threading.Thread(target=_run, daemon=True).start()


_BUILTIN: Dict[str, Type[DockingPlugin]] = {
    'aruco': ArucoDockingPlugin,
    'noop': NoopDockingPlugin,
}


def register_docking_plugin(name: str, cls: Type[DockingPlugin]) -> None:
    """Register a built-in alias (e.g. from a user's bringup script)."""
    _BUILTIN[name] = cls


def _load_class(spec: str) -> Type[DockingPlugin]:
    """Load ``module.path:ClassName``."""
    if ':' not in spec:
        raise ValueError(
            f'docking plugin {spec!r} must be a built-in name '
            f'({", ".join(sorted(_BUILTIN))}) or module.path:ClassName')
    module_name, _, class_name = spec.partition(':')
    mod = importlib.import_module(module_name)
    cls = getattr(mod, class_name)
    return cls


def get_docking_plugin(name: str, args: Optional[List[str]] = None) -> DockingPlugin:
    """Resolve ``name`` to a ``DockingPlugin`` instance."""
    key = (name or 'aruco').strip()
    if key in _BUILTIN:
        return _BUILTIN[key](args=args)
    cls = _load_class(key)
    return cls(args=args)
