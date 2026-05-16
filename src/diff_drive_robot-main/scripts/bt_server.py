#!/usr/bin/env python3
"""
BT Editor server — serves bt_editor/index.html and handles save/load.

  ros2 run diff_drive_robot bt_server.py
  # open http://localhost:8080 in browser

Endpoints:
  GET  /           → serves index.html
  POST /save?name= → writes XML to config/bt/<name>.xml
  GET  /load?name= → returns config/bt/<name>.xml content
  GET  /list       → JSON list of .xml files in config/bt/
"""

import json
import os
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

import rclpy
from rclpy.node import Node


class BTEditorServer(Node):
    def __init__(self):
        super().__init__('bt_editor_server')
        self.declare_parameter('port', 8080)

        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('diff_drive_robot')
        except Exception:
            share = os.path.join(
                os.path.expanduser('~'), 'rosnav', 'src', 'diff_drive_robot-main')

        self._bt_dir   = os.path.join(share, 'config', 'bt')
        self._html_path = os.path.join(share, 'bt_editor', 'index.html')

        port = self.get_parameter('port').value
        server = HTTPServer(('', port), self._make_handler())
        t = threading.Thread(target=server.serve_forever, daemon=True)
        t.start()
        self.get_logger().info(f'BT Editor → http://localhost:{port}')

    def _make_handler(self):
        bt_dir    = self._bt_dir
        html_path = self._html_path
        logger    = self.get_logger()

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                pass  # suppress default access log

            def _cors(self):
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')

            def do_OPTIONS(self):
                self.send_response(200)
                self._cors()
                self.end_headers()

            def do_GET(self):
                parsed = urlparse(self.path)
                qs     = parse_qs(parsed.query)
                path   = parsed.path

                if path in ('/', '/index.html'):
                    if not os.path.isfile(html_path):
                        self._err(404, 'bt_editor/index.html not found')
                        return
                    with open(html_path, 'rb') as f:
                        data = f.read()
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self._cors()
                    self.end_headers()
                    self.wfile.write(data)

                elif path == '/list':
                    if not os.path.isdir(bt_dir):
                        files = []
                    else:
                        files = [f for f in os.listdir(bt_dir) if f.endswith('.xml')]
                    self._json(files)

                elif path == '/load':
                    name = qs.get('name', [''])[0]
                    name = os.path.basename(name)         # no path traversal
                    fpath = os.path.join(bt_dir, name if name.endswith('.xml') else name + '.xml')
                    if not os.path.isfile(fpath):
                        self._err(404, f'{name}.xml not found in config/bt/')
                        return
                    with open(fpath, 'rb') as f:
                        data = f.read()
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/xml; charset=utf-8')
                    self._cors()
                    self.end_headers()
                    self.wfile.write(data)

                else:
                    self._err(404, 'not found')

            def do_POST(self):
                parsed = urlparse(self.path)
                qs     = parse_qs(parsed.query)
                path   = parsed.path

                if path == '/save':
                    name = qs.get('name', ['custom_bt'])[0]
                    name = os.path.basename(name)         # no path traversal
                    if not name.endswith('.xml'):
                        name += '.xml'
                    length = int(self.headers.get('Content-Length', 0))
                    body   = self.rfile.read(length)
                    os.makedirs(bt_dir, exist_ok=True)
                    fpath = os.path.join(bt_dir, name)
                    with open(fpath, 'wb') as f:
                        f.write(body)
                    logger.info(f'Saved BT → {fpath}')
                    self._json({'path': fpath, 'name': name})
                else:
                    self._err(404, 'not found')

            def _json(self, data):
                body = json.dumps(data).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self._cors()
                self.end_headers()
                self.wfile.write(body)

            def _err(self, code, msg):
                body = json.dumps({'error': msg}).encode()
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self._cors()
                self.end_headers()
                self.wfile.write(body)

        return Handler


def main(args=None):
    rclpy.init(args=args)
    node = BTEditorServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
