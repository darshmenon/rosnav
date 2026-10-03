#!/usr/bin/env python3
"""
fleet_gui.py — Tkinter-based Fleet Control Dashboard.

Launch:
  ros2 run rosnav_bot fleet_gui.py

Features
────────
• Live list of active robots (auto-refreshes every 2 s)
• Select a robot → click-to-send navigation goals on map canvas
• Virtual joystick pad (drag to steer) for teleoperation
• Live camera feed for the selected robot
• Spawn a new robot at a given (x, y) in Gazebo
• Save the current SLAM map
• Map canvas shows the /map OccupancyGrid
• Status bar with SLAM / Nav2 / map information
"""

import math
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog

import cv2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image

PKG = 'rosnav_bot'
REFRESH_MS = 2000     # robot list refresh interval
MAP_W = 400           # canvas width in pixels
MAP_H = 400           # canvas height in pixels
CAM_W = 320           # camera view width in pixels
CAM_H = 240           # camera view height in pixels
JOY_SIZE = 150         # joystick pad diameter in pixels
JOY_KNOB_R = 12        # joystick knob radius in pixels
MAX_LIN = 0.5          # m/s at full joystick deflection
MAX_ANG = 2.0          # rad/s at full joystick deflection


# ─────────────────────────────────────────────────────────────────────────────
class FleetGuiNode(Node):
    def __init__(self):
        super().__init__('fleet_gui')
        self._pubs: dict[str, any] = {}
        self._map: OccupancyGrid | None = None
        self._bridge = CvBridge()
        self._image_subs: dict[str, any] = {}
        self._latest_image: dict[str, any] = {}   # ns -> bgr8 numpy frame

        qos = QoSProfile(depth=1,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, qos)

    def _map_cb(self, msg):
        self._map = msg

    def ensure_image_sub(self, ns: str):
        """Lazily subscribe to a robot's camera the first time it's viewed."""
        if ns in self._image_subs:
            return

        def _cb(msg, ns=ns):
            try:
                self._latest_image[ns] = self._bridge.imgmsg_to_cv2(
                    msg, desired_encoding='bgr8')
            except Exception as exc:
                self.get_logger().warn(f'camera decode failed for {ns}: {exc}')

        self._image_subs[ns] = self.create_subscription(
            Image, f'/{ns}/camera/image_raw', _cb, 5)

    def _pub(self, ns: str):
        if ns not in self._pubs:
            self._pubs[ns] = self.create_publisher(
                Twist, f'/{ns}/cmd_vel', 10)
        return self._pubs[ns]

    def send_vel(self, ns: str, lin: float, ang: float):
        msg = Twist()
        msg.linear.x  = lin
        msg.angular.z = ang
        self._pub(ns).publish(msg)

    def send_goal(self, ns: str, x: float, y: float, yaw: float = 0.0):
        client = ActionClient(self, NavigateToPose, f'/{ns}/navigate_to_pose')
        if not client.wait_for_server(timeout_sec=3.0):
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        client.send_goal_async(goal)
        return True

    def active_robots(self) -> list[str]:
        topics = self.get_topic_names_and_types()
        return sorted({
            t.split('/')[1]
            for t, _ in topics
            if t.count('/') >= 2 and t.endswith('/cmd_vel')
        })

    def slam_running(self) -> bool:
        return any('slam' in n.lower() for n in self.get_node_names())

    def nav2_running(self) -> bool:
        return any('bt_navigator' in n for n in self.get_node_names())


# ─────────────────────────────────────────────────────────────────────────────
class FleetGUI:
    def __init__(self, root: tk.Tk, node: FleetGuiNode):
        self.root = root
        self.node = node
        self._selected_ns: str | None = None
        self._click_goal_mode = False

        root.title('Fleet Control Dashboard')
        root.resizable(True, True)
        self._build_ui()
        self._refresh_robots()
        self._refresh_map()
        self._refresh_camera()
        self._start_watchdog()

    # ── UI construction ───────────────────────────────────────────────────────
    def _build_ui(self):
        root = self.root

        # ── Top bar ──
        top = tk.Frame(root, bg='#1e1e2e')
        top.pack(fill=tk.X)
        tk.Label(top, text='Fleet Control Dashboard',
                 font=('Helvetica', 14, 'bold'),
                 fg='#cdd6f4', bg='#1e1e2e').pack(side=tk.LEFT, padx=10, pady=6)

        self._status_var = tk.StringVar(value='Connecting …')
        tk.Label(top, textvariable=self._status_var,
                 fg='#a6e3a1', bg='#1e1e2e', font=('Courier', 10)).pack(
            side=tk.RIGHT, padx=10)

        # ── Main pane ──
        main = tk.Frame(root)
        main.pack(fill=tk.BOTH, expand=True)

        # Left panel
        left = tk.Frame(main, width=220, bg='#181825')
        left.pack(side=tk.LEFT, fill=tk.Y, padx=4, pady=4)

        tk.Label(left, text='Active Robots', font=('Helvetica', 11, 'bold'),
                 fg='#cba6f7', bg='#181825').pack(pady=(8, 2))

        self._robot_lb = tk.Listbox(left, height=10, width=18,
                                    bg='#1e1e2e', fg='#cdd6f4',
                                    selectbackground='#7f849c',
                                    font=('Courier', 11))
        self._robot_lb.pack(padx=6, pady=4)
        self._robot_lb.bind('<<ListboxSelect>>', self._on_robot_select)

        tk.Button(left, text='Refresh robots', command=self._refresh_robots,
                  bg='#313244', fg='#cdd6f4').pack(pady=2, fill=tk.X, padx=6)

        ttk.Separator(left, orient='horizontal').pack(fill=tk.X, pady=6, padx=4)

        # Goal click toggle
        self._goal_btn = tk.Button(left, text='Click map → Send goal',
                                   command=self._toggle_goal_mode,
                                   bg='#45475a', fg='#cdd6f4')
        self._goal_btn.pack(pady=2, fill=tk.X, padx=6)

        # Direct goal entry
        goal_frame = tk.LabelFrame(left, text='Goal (x, y)',
                                   bg='#181825', fg='#89b4fa')
        goal_frame.pack(fill=tk.X, padx=6, pady=4)
        self._gx = ttk.Entry(goal_frame, width=7)
        self._gx.insert(0, '0.0')
        self._gx.grid(row=0, column=0, padx=2, pady=2)
        self._gy = ttk.Entry(goal_frame, width=7)
        self._gy.insert(0, '0.0')
        self._gy.grid(row=0, column=1, padx=2, pady=2)
        tk.Button(goal_frame, text='Go!', command=self._send_goal_entry,
                  bg='#a6e3a1', fg='#1e1e2e', font=('Helvetica', 9, 'bold')
                  ).grid(row=1, column=0, columnspan=2, pady=2, sticky='ew')

        ttk.Separator(left, orient='horizontal').pack(fill=tk.X, pady=6, padx=4)

        # Teleop joystick
        tel_frame = tk.LabelFrame(left, text='Teleop',
                                  bg='#181825', fg='#89b4fa')
        tel_frame.pack(fill=tk.X, padx=6, pady=4)

        self._joy_vel_var = tk.StringVar(value='lin 0.00 m/s | ang 0.00 rad/s')
        tk.Label(tel_frame, textvariable=self._joy_vel_var,
                 bg='#181825', fg='#cdd6f4', font=('Courier', 8)).pack(pady=(4, 2))

        self._joy_canvas = tk.Canvas(tel_frame, width=JOY_SIZE, height=JOY_SIZE,
                                     bg='#11111b', highlightthickness=0,
                                     cursor='hand2')
        self._joy_canvas.pack(pady=2)
        self._joy_center = (JOY_SIZE / 2, JOY_SIZE / 2)
        self._joy_radius = JOY_SIZE / 2 - JOY_KNOB_R
        self._joy_canvas.create_oval(
            2, 2, JOY_SIZE - 2, JOY_SIZE - 2, outline='#45475a', width=2)
        self._joy_canvas.create_line(
            JOY_SIZE / 2, 2, JOY_SIZE / 2, JOY_SIZE - 2, fill='#313244')
        self._joy_canvas.create_line(
            2, JOY_SIZE / 2, JOY_SIZE - 2, JOY_SIZE / 2, fill='#313244')
        cx, cy = self._joy_center
        self._joy_knob = self._joy_canvas.create_oval(
            cx - JOY_KNOB_R, cy - JOY_KNOB_R, cx + JOY_KNOB_R, cy + JOY_KNOB_R,
            fill='#89b4fa', outline='')
        self._joy_canvas.bind('<Button-1>', self._on_joy_drag)
        self._joy_canvas.bind('<B1-Motion>', self._on_joy_drag)
        self._joy_canvas.bind('<ButtonRelease-1>', self._on_joy_release)

        tk.Button(tel_frame, text='STOP', command=self._stop_robot,
                  bg='#f38ba8', fg='#1e1e2e',
                  font=('Helvetica', 10, 'bold')).pack(fill=tk.X, padx=4, pady=(4, 2))

        ttk.Separator(left, orient='horizontal').pack(fill=tk.X, pady=6, padx=4)

        # Spawn + save
        tk.Button(left, text='Spawn new robot …',
                  command=self._spawn_dialog,
                  bg='#313244', fg='#cdd6f4').pack(pady=2, fill=tk.X, padx=6)
        tk.Button(left, text='Save map …',
                  command=self._save_map_dialog,
                  bg='#313244', fg='#cdd6f4').pack(pady=2, fill=tk.X, padx=6)

        # Right: camera feed + map canvas
        right = tk.Frame(main)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4, pady=4)

        tk.Label(right, text='Camera', font=('Helvetica', 10, 'bold'),
                 fg='#89b4fa').pack()
        self._cam_canvas = tk.Canvas(right, width=CAM_W, height=CAM_H,
                                     bg='#11111b', highlightthickness=0)
        self._cam_canvas.pack()
        self._cam_photo = None   # keep a reference so Tk doesn't GC it

        tk.Label(right, text='/map', font=('Helvetica', 10, 'bold'),
                 fg='#89b4fa').pack(pady=(8, 0))
        self._canvas = tk.Canvas(right, width=MAP_W, height=MAP_H,
                                 bg='#313244', cursor='crosshair')
        self._canvas.pack(fill=tk.BOTH, expand=True)
        self._canvas.bind('<Button-1>', self._on_map_click)

        self._map_label = tk.Label(right, text='Waiting for /map …',
                                   font=('Courier', 8), fg='#6c7086')
        self._map_label.pack()

    # ── Robot list ────────────────────────────────────────────────────────────
    def _refresh_robots(self):
        robots = self.node.active_robots()
        self._robot_lb.delete(0, tk.END)
        for r in robots:
            self._robot_lb.insert(tk.END, r)
        if self._selected_ns and self._selected_ns in robots:
            idx = robots.index(self._selected_ns)
            self._robot_lb.selection_set(idx)

        slam = self.node.slam_running()
        nav  = self.node.nav2_running()
        has_map = self.node._map is not None
        self._status_var.set(
            f'Robots: {len(robots)}  SLAM:{"✓" if slam else "✗"}'
            f'  Nav2:{"✓" if nav else "✗"}  Map:{"✓" if has_map else "✗"}')
        self.root.after(REFRESH_MS, self._refresh_robots)

    def _on_robot_select(self, _event=None):
        sel = self._robot_lb.curselection()
        if sel:
            self._selected_ns = self._robot_lb.get(sel[0])
            self.node.ensure_image_sub(self._selected_ns)

    # ── Map canvas ────────────────────────────────────────────────────────────
    def _refresh_map(self):
        msg = self.node._map
        if msg:
            self._draw_map(msg)
        self.root.after(1000, self._refresh_map)

    def _draw_map(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        res   = msg.info.resolution
        data  = msg.data
        c = self._canvas
        cw, ch = int(c.winfo_width()) or MAP_W, int(c.winfo_height()) or MAP_H
        c.delete('map')

        # Scale factor
        sx = cw / w
        sy = ch / h
        s  = min(sx, sy)

        import tkinter.font as tkfont  # noqa: PLC0415

        # Draw cells
        for row in range(h):
            for col in range(w):
                val = data[row * w + col]
                if val == -1:
                    color = '#45475a'    # unknown — grey
                elif val == 0:
                    color = '#cdd6f4'    # free — light
                else:
                    color = '#1e1e2e'    # occupied — dark
                x0 = col * s
                y0 = (h - row - 1) * s   # flip Y
                c.create_rectangle(x0, y0, x0 + s, y0 + s,
                                   fill=color, outline='', tags='map')

        self._map_label.config(
            text=f'Map: {w}×{h} cells @ {res:.3f} m/cell  |  '
                 f'origin ({msg.info.origin.position.x:.1f}, '
                 f'{msg.info.origin.position.y:.1f})')

    def _world_to_canvas(self, wx: float, wy: float) -> tuple[float, float]:
        """Convert world (m) → canvas (px)."""
        msg = self.node._map
        if not msg:
            return (MAP_W / 2, MAP_H / 2)
        w, h = msg.info.width, msg.info.height
        res  = msg.info.resolution
        ox   = msg.info.origin.position.x
        oy   = msg.info.origin.position.y
        c    = self._canvas
        cw   = int(c.winfo_width()) or MAP_W
        ch   = int(c.winfo_height()) or MAP_H
        s    = min(cw / w, ch / h)
        cx   = (wx - ox) / res * s
        cy   = ch - (wy - oy) / res * s
        return (cx, cy)

    def _canvas_to_world(self, cx: float, cy: float) -> tuple[float, float]:
        """Convert canvas (px) → world (m)."""
        msg = self.node._map
        if not msg:
            return (cx / 10.0, -cy / 10.0)
        w, h = msg.info.width, msg.info.height
        res  = msg.info.resolution
        ox   = msg.info.origin.position.x
        oy   = msg.info.origin.position.y
        c    = self._canvas
        cw   = int(c.winfo_width()) or MAP_W
        ch   = int(c.winfo_height()) or MAP_H
        s    = min(cw / w, ch / h)
        wx   = cx / s * res + ox
        wy   = (ch - cy) / s * res + oy
        return (wx, wy)

    # ── Goal handling ─────────────────────────────────────────────────────────
    def _toggle_goal_mode(self):
        self._click_goal_mode = not self._click_goal_mode
        if self._click_goal_mode:
            self._goal_btn.config(bg='#a6e3a1', fg='#1e1e2e',
                                  text='Click map → ACTIVE (click to cancel)')
        else:
            self._goal_btn.config(bg='#45475a', fg='#cdd6f4',
                                  text='Click map → Send goal')

    def _on_map_click(self, event):
        if not self._click_goal_mode:
            return
        if not self._selected_ns:
            messagebox.showwarning('No robot', 'Select a robot first.')
            return
        wx, wy = self._canvas_to_world(event.x, event.y)
        self._gx.delete(0, tk.END); self._gx.insert(0, f'{wx:.2f}')
        self._gy.delete(0, tk.END); self._gy.insert(0, f'{wy:.2f}')
        self._dispatch_goal(wx, wy)
        # Draw marker
        c = self._canvas
        r = 6
        c.delete('goal_marker')
        c.create_oval(event.x - r, event.y - r, event.x + r, event.y + r,
                      fill='#f38ba8', outline='white', width=2, tags='goal_marker')

    def _send_goal_entry(self):
        if not self._selected_ns:
            messagebox.showwarning('No robot', 'Select a robot first.')
            return
        try:
            x = float(self._gx.get())
            y = float(self._gy.get())
        except ValueError:
            messagebox.showerror('Error', 'Invalid x or y value.')
            return
        self._dispatch_goal(x, y)

    def _dispatch_goal(self, x: float, y: float):
        ns = self._selected_ns
        ok = self.node.send_goal(ns, x, y)
        if ok:
            self._status_var.set(f'Goal sent to {ns} → ({x:.2f}, {y:.2f})')
        else:
            messagebox.showwarning('Nav2 not ready',
                                   f'navigate_to_pose not available for {ns}.\n'
                                   'Is Nav2 running?')

    # ── Teleop (virtual joystick) ────────────────────────────────────────────
    _last_vel_t = 0.0
    _joy_lin = 0.0
    _joy_ang = 0.0

    def _on_joy_drag(self, event):
        cx, cy = self._joy_center
        dx, dy = event.x - cx, event.y - cy
        dist = math.hypot(dx, dy)
        if dist > self._joy_radius:
            dx *= self._joy_radius / dist
            dy *= self._joy_radius / dist
        self._joy_canvas.coords(
            self._joy_knob,
            cx + dx - JOY_KNOB_R, cy + dy - JOY_KNOB_R,
            cx + dx + JOY_KNOB_R, cy + dy + JOY_KNOB_R)

        # Up = forward (-dy), right = turn right (+dx → negative ang, standard ccw-positive)
        self._joy_lin = -dy / self._joy_radius * MAX_LIN
        self._joy_ang = -dx / self._joy_radius * MAX_ANG
        self._joy_vel_var.set(
            f'lin {self._joy_lin:+.2f} m/s | ang {self._joy_ang:+.2f} rad/s')

        if self._selected_ns:
            self.node.send_vel(self._selected_ns, self._joy_lin, self._joy_ang)
        self._last_vel_t = time.time()

    def _on_joy_release(self, _event=None):
        cx, cy = self._joy_center
        self._joy_canvas.coords(
            self._joy_knob,
            cx - JOY_KNOB_R, cy - JOY_KNOB_R,
            cx + JOY_KNOB_R, cy + JOY_KNOB_R)
        self._joy_lin = self._joy_ang = 0.0
        self._joy_vel_var.set('lin 0.00 m/s | ang 0.00 rad/s')
        if self._selected_ns:
            self.node.send_vel(self._selected_ns, 0.0, 0.0)
        self._last_vel_t = time.time()

    def _stop_robot(self):
        self._on_joy_release()

    def _start_watchdog(self):
        """Stop robot if the joystick knob hasn't moved for 0.5 s."""
        def wd():
            while True:
                time.sleep(0.1)
                ns = self._selected_ns
                if ns and time.time() - self._last_vel_t > 0.5:
                    if self._joy_lin != 0 or self._joy_ang != 0:
                        self.node.send_vel(ns, 0.0, 0.0)
                        self._joy_lin = self._joy_ang = 0.0

        threading.Thread(target=wd, daemon=True).start()

    # ── Camera feed ───────────────────────────────────────────────────────────
    def _refresh_camera(self):
        ns = self._selected_ns
        frame = self.node._latest_image.get(ns) if ns else None
        if frame is not None:
            small = cv2.resize(frame, (CAM_W, CAM_H))
            rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
            header = f'P6 {CAM_W} {CAM_H} 255\n'.encode()
            self._cam_photo = tk.PhotoImage(
                data=header + rgb.tobytes(), format='PPM')
            self._cam_canvas.delete('cam')
            self._cam_canvas.create_image(
                0, 0, anchor='nw', image=self._cam_photo, tags='cam')
        else:
            self._cam_canvas.delete('cam')
            msg = 'Select a robot' if not ns else 'No camera feed'
            self._cam_canvas.create_text(
                CAM_W / 2, CAM_H / 2, text=msg, fill='#6c7086', tags='cam')
        self.root.after(100, self._refresh_camera)

    # ── Spawn dialog ──────────────────────────────────────────────────────────
    def _spawn_dialog(self):
        dlg = tk.Toplevel(self.root)
        dlg.title('Spawn New Robot')
        dlg.resizable(False, False)

        tk.Label(dlg, text='Namespace (e.g. robot3)').grid(row=0, column=0, padx=8, pady=4)
        ns_var = ttk.Entry(dlg, width=12); ns_var.grid(row=0, column=1, padx=8)
        ns_var.insert(0, 'robot3')

        tk.Label(dlg, text='X (m)').grid(row=1, column=0)
        x_var = ttk.Entry(dlg, width=8); x_var.grid(row=1, column=1)
        x_var.insert(0, '0.0')

        tk.Label(dlg, text='Y (m)').grid(row=2, column=0)
        y_var = ttk.Entry(dlg, width=8); y_var.grid(row=2, column=1)
        y_var.insert(0, '0.0')

        def _spawn():
            ns = ns_var.get().strip()
            try:
                x, y = float(x_var.get()), float(y_var.get())
            except ValueError:
                messagebox.showerror('Error', 'Invalid x/y.')
                return
            dlg.destroy()
            self._status_var.set(f'Spawning {ns} …')
            threading.Thread(target=self._do_spawn, args=(ns, x, y),
                             daemon=True).start()

        tk.Button(dlg, text='Spawn', command=_spawn,
                  bg='#a6e3a1', fg='#1e1e2e').grid(
            row=3, column=0, columnspan=2, pady=8, padx=8, sticky='ew')

    def _do_spawn(self, ns: str, x: float, y: float):
        subprocess.run([
            'ros2', 'run', PKG, 'fleet_manager.py', 'add', ns, str(x), str(y)
        ])
        self.root.after(100, lambda:
            self._status_var.set(f'{ns} spawn complete.'))

    # ── Save map dialog ───────────────────────────────────────────────────────
    def _save_map_dialog(self):
        path = simpledialog.askstring(
            'Save Map',
            'Save prefix (e.g. src/rosnav_bot/maps/map_maze):',
            initialvalue='src/rosnav_bot/maps/map_fleet')
        if not path:
            return
        self._status_var.set(f'Saving map to {path} …')
        threading.Thread(target=self._do_save_map, args=(path,),
                         daemon=True).start()

    def _do_save_map(self, path: str):
        r = subprocess.run(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', path],
            capture_output=True, text=True)
        if r.returncode == 0:
            self.root.after(100, lambda:
                self._status_var.set(f'Map saved: {path}.yaml'))
        else:
            self.root.after(100, lambda:
                messagebox.showerror('Save failed', r.stderr.strip()))


# ─────────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = FleetGuiNode()

    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    root.configure(bg='#1e1e2e')

    app = FleetGUI(root, node)

    def _on_close():
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', _on_close)
    root.mainloop()


if __name__ == '__main__':
    main()
