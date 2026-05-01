#!/usr/bin/env python3
"""
Block Painting Helper — Web UI Server

Serves:
  GET /            → main control panel (paint buttons + arm mode toggle)
  GET /humanrequest → human-facing paint request display

Usage:
  python3 bph_ui_server.py           # default port 8080
  python3 bph_ui_server.py 9000      # custom port
"""

import sys
import os
from http.server import HTTPServer, BaseHTTPRequestHandler

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
STATIC_DIR = os.path.dirname(os.path.abspath(__file__))


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self._serve_file('index.html')
        elif self.path.rstrip('/') == '/humanrequest':
            self._serve_file('humanrequest.html')
        else:
            self.send_response(404)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'404 Not Found')

    def _serve_file(self, name):
        path = os.path.join(STATIC_DIR, name)
        try:
            with open(path, 'rb') as f:
                data = f.read()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        except FileNotFoundError:
            self.send_response(404)
            self.end_headers()

    def log_message(self, fmt, *args):
        print(f'[bph_ui] {self.address_string()} - {fmt % args}')


if __name__ == '__main__':
    server = HTTPServer(('0.0.0.0', PORT), Handler)
    print(f'Block Painting Helper UI running at http://localhost:{PORT}/')
    print(f'Human request display:           http://localhost:{PORT}/humanrequest')
    print('Press Ctrl-C to stop.')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
