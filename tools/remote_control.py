#!/usr/bin/env python3
"""Local browser remote for AX simulation; ROS stays inside Ubuntu."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import argparse
import json
import math
from pathlib import Path
import queue
import secrets
import signal
import threading
import time

from keyboard_teleop import KeyboardTeleop, JOINT_LABELS, rclpy, SignalHandlerOptions


def validate_command(data):
    if not isinstance(data, dict):
        raise ValueError('Invalid command')
    action = data.get('action')
    value = data.get('value')
    if action == 'drive' and value in ('w', 's', 'a', 'd'):
        pass
    elif action == 'robot' and value in ('robot_1', 'robot_2'):
        pass
    elif action == 'joint' and type(value) is int and 0 <= value < 5:
        pass
    elif action in ('speed', 'angle', 'jog') and type(value) is int and value in (-1, 1):
        pass
    elif action == 'stop':
        value = None
    else:
        raise ValueError('Invalid command or value')
    if type(data.get('seq')) is not int or data['seq'] < 1:
        raise ValueError('Invalid sequence')
    if not isinstance(data.get('client'), str) or not 1 <= len(data['client']) <= 64:
        raise ValueError('Invalid client')
    return action, value


class Remote:
    def __init__(self):
        self.commands = queue.Queue(maxsize=32)
        self.state = {}
        self.token = secrets.token_urlsafe(24)
        self.last_seen = time.monotonic()
        self.sequences = {}

    def apply(self, node, data, received):
        action, value = validate_command(data)
        client, seq = data['client'], data['seq']
        if seq <= self.sequences.get(client, 0):
            return
        self.sequences[client] = seq
        if action == 'drive':
            if time.monotonic() - received <= .2:
                node.key(ord(value))
        elif action == 'stop':
            node.stop()
        elif action == 'robot':
            node.stop()
            node.robot = value
            node.adjust_mode = 'drive'
        elif action == 'joint':
            node.key(ord('1') + value)
        elif action in ('speed', 'angle'):
            node.adjust_mode = 'drive' if action == 'speed' else 'arm'
            node.adjust(value)
        elif action == 'jog':
            node.adjust_mode = 'arm'
            node.jog(value)

    def update(self, node):
        self.state = {
            'robot': node.robot, 'joint': node.joint, 'labels': JOINT_LABELS,
            'linear': round(node.drive_scale * .1, 3),
            'angular': round(node.drive_scale * .2, 2),
            'step_deg': round(math.degrees(node.arm_step), 2),
            'angles_deg': [round(math.degrees(p), 2) for p in node.positions] if node.positions else [],
            'arm_busy': node.pending, 'notice': node.notice,
            'arm_ready': node.positions is not None and time.monotonic() - node.state_time < 1,
        }


def handler_for(remote):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, code, content, mime='application/json; charset=utf-8'):
            body = content.encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            if self.path == '/':
                page = Path(__file__).with_name('remote_control.html').read_text()
                self.reply(200, page.replace('__TOKEN__', remote.token), 'text/html; charset=utf-8')
            elif self.path == '/api/state':
                remote.last_seen = time.monotonic()
                self.reply(200, json.dumps(remote.state, ensure_ascii=False))
            else:
                self.reply(404, '{}')

        def do_POST(self):
            if self.path != '/api/command':
                return self.reply(404, '{}')
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 4096:
                    return self.reply(400, '{}')
                data = json.loads(self.rfile.read(size))
                if data.get('token') != remote.token:
                    return self.reply(403, '{}')
                validate_command(data)
                remote.commands.put_nowait((data, time.monotonic()))
                remote.last_seen = time.monotonic()
                self.reply(202, '{}')
            except (ValueError, TypeError, AttributeError):
                self.reply(400, '{}')
            except queue.Full:
                self.reply(429, '{}')

        def log_message(self, *args):
            pass
    return Handler


def stop_signal_handler(*_):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    remote = Remote()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), handler_for(remote))
    server.daemon_threads = True
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = KeyboardTeleop()
    node.notice = '준비 완료. 리모컨에서 로봇과 동작을 선택하세요.'
    signal.signal(signal.SIGTERM, stop_signal_handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'AX remote: http://localhost:{args.port} (local PC only)', flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.01)
            for _ in range(16):
                try:
                    data, received = remote.commands.get_nowait()
                except queue.Empty:
                    break
                remote.apply(node, data, received)
            if time.monotonic() - remote.last_seen > 1. and node.pending:
                node.stop()
            node.publish_drive()
            remote.update(node)
            time.sleep(.015)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_safely()
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
