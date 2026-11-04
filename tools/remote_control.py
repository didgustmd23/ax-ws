#!/usr/bin/env python3
# 브라우저 HTTP 요청을 큐로 받아 단일 ROS 루프에서 실행한다. HTTP 스레드는 ROS 객체를 직접 제어하지 않는다.
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


# 허용 명령·값·클라이언트 식별자·순서를 검사한다. bool은 Python에서 int의 하위형이므로 정수와 구분한다.
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
    elif action in ('set_speed', 'set_angle'):
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError('A finite numeric setting is required')
        minimum, maximum = (.025, 3.0) if action == 'set_speed' else (math.degrees(.01), 90.0)
        if not minimum <= value <= maximum:
            raise ValueError('Setting outside permitted range')
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
    # 대기열 크기를 제한하고 실행마다 새 토큰을 만든다. 브라우저를 새로고침하면 새 토큰을 받는다.
    def __init__(self):
        self.commands = queue.Queue(maxsize=32)
        self.state = {}
        self.token = secrets.token_urlsafe(24)
        self.last_seen = time.monotonic()
        self.sequences = {}

    # 단일 ROS 스레드에서 큐 명령을 실행한다. 늦게 도착한 이전 주행 명령은 새 정지를 덮어쓰지 못한다.
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
        elif action in ('set_speed', 'set_angle'):
            if action == 'set_speed':
                node.drive_scale = value / .1
                node.adjust_mode = 'drive'
                node.notice = f'주행 {value:.3f} m/s / 회전 {value * 2:.2f} rad/s'
            else:
                node.arm_step = math.radians(value)
                node.adjust_mode = 'arm'
                node.notice = f'다음 팔 이동 각도: {value:.3f}도'
            # 설정 변경은 이동 명령이 아니다. 진행 중인 팔 궤적은 바꾸지 않는다.
            node.velocity = (0., 0.)
            node.key_deadline = 0.
            node.publish_drive()

    # ROS 내부 상태를 브라우저용 복사본으로 만든다. 내부 각도 rad를 화면용 degree로 변환한다.
    def update(self, node):
        self.state = {
            'robot': node.robot, 'joint': node.joint, 'labels': JOINT_LABELS,
            'linear': round(node.drive_scale * .1, 3),
            'angular': round(node.drive_scale * .2, 2),
            'step_deg': round(math.degrees(node.arm_step), 3),
            'angles_deg': [round(math.degrees(p), 2) for p in node.positions] if node.positions else [],
            'arm_busy': node.pending, 'notice': node.notice,
            'arm_ready': node.positions is not None and time.monotonic() - node.state_time < 1,
        }


# HTTP 핸들러를 만든다. 서버 객체를 캡처하고 ROS 접근은 큐와 상태 복사본으로 제한한다.
def handler_for(remote):
    class Handler(BaseHTTPRequestHandler):
        # UTF-8 응답 길이를 바이트로 계산하고 캐시를 막아 이전 토큰/UI가 재사용되지 않게 한다.
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

        # 페이지와 상태 조회만 제공한다. 상태 요청 시각은 브라우저 연결 생존 확인에 사용한다.
        def do_GET(self):
            if self.path == '/':
                page = Path(__file__).with_name('remote_control.html').read_text()
                self.reply(200, page.replace('__TOKEN__', remote.token), 'text/html; charset=utf-8')
            elif self.path == '/api/state':
                remote.last_seen = time.monotonic()
                self.reply(200, json.dumps(remote.state, ensure_ascii=False))
            else:
                self.reply(404, '{}')

        # 토큰과 명령 형식을 확인한 후 큐에 넣는다. 202는 접수 의미이며 실제 동작 성공 응답은 아니다.
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

        # 반복 상태 조회가 실행 로그를 채우지 않도록 기본 HTTP 접근 로그를 생략한다.
        def log_message(self, *args):
            pass
    return Handler


# SIGTERM도 KeyboardInterrupt 경로로 합쳐 finally에서 정지 처리를 수행한다.
def stop_signal_handler(*_):
    raise KeyboardInterrupt


# 루프백 주소로만 서버를 열고 ROS 루프에서 명령을 실행한다. 브라우저 연결 만료 시 자기 팔 Action을 취소한다.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    remote = Remote()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), handler_for(remote))
    server.daemon_threads = True
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = KeyboardTeleop(max_drive_scale=30.0, max_arm_step=math.pi / 2)
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
