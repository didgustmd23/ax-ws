"""관제 화면 HTTP 서버(표시 전용). GET·HEAD만 받고 쓰기 경로가 없다(계약 8절).

- 경로 화이트리스트: /, /index.html, /static/<web 폴더 파일>, /api/state, /map/info.json, /map/map.pgm
- Host 헤더가 허용 이름(127.0.0.1·localhost·바인딩 주소 + 포트)이 아니면 403(DNS rebinding 대비)
- 다른 메서드는 405. 오류 본문은 고정 문구이며 예외 내용은 로그에만 남긴다. CORS 헤더를 보내지 않는다.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path

STATIC_TYPES = {'.html': 'text/html; charset=utf-8', '.css': 'text/css; charset=utf-8',
                '.js': 'text/javascript; charset=utf-8', '.svg': 'image/svg+xml'}
TEXT = 'text/plain; charset=utf-8'


# 허용 Host 이름 집합
def allowed_hosts(bind_address, port):
    return {f'127.0.0.1:{port}', f'localhost:{port}', f'{bind_address}:{port}'}


# /static/ 아래 경로 → web 폴더 안 실제 파일(밖으로 나가면 None)
def resolve_static(web_dir, relative):
    root = Path(web_dir).resolve()
    target = (root / relative).resolve()

    if root not in target.parents or not target.is_file():
        return None

    return target


# 요청 처리기 클래스 생성. state_fn은 화면 상태 dict를 돌려주는 함수, log_error는 오류 기록 함수
def make_handler(state_fn, web_dir, map_info, map_bytes, hosts, log_error):
    index_path = Path(web_dir) / 'index.html'

    class Handler(BaseHTTPRequestHandler):
        server_version = 'ax-operations-console'
        sys_version = ''

        # 요청마다 로그를 남기지 않는다
        def log_message(self, *args):
            pass

        # 응답 송신(HEAD는 본문 생략)
        def reply(self, status, body, content_type, head_only=False):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('X-Frame-Options', 'DENY')
            self.end_headers()

            if not head_only:
                self.wfile.write(body)

        # 쓰기 메서드 거절
        def refuse(self):
            self.reply(405, b'method not allowed: read-only display\n', TEXT)

        do_POST = do_PUT = do_DELETE = do_PATCH = do_OPTIONS = do_TRACE = do_CONNECT = refuse

        # 경로 → (본문, 형식) 또는 None
        def route(self, path):
            if path in ('/', '/index.html'):
                return index_path.read_bytes(), STATIC_TYPES['.html']

            if path == '/api/state':
                body = json.dumps(state_fn(), ensure_ascii=False).encode()
                return body, 'application/json; charset=utf-8'

            if path == '/map/info.json':
                return json.dumps(map_info).encode(), 'application/json'

            if path == '/map/map.pgm':
                return map_bytes, 'application/octet-stream'

            if path.startswith('/static/'):
                target = resolve_static(web_dir, path[len('/static/'):])
                if target is not None:
                    kind = STATIC_TYPES.get(target.suffix) or mimetypes.guess_type(target.name)[0]
                    return target.read_bytes(), kind or 'application/octet-stream'

            return None

        # GET·HEAD 공통 처리
        def serve(self, head_only):
            if self.headers.get('Host', '') not in hosts:
                return self.reply(403, b'forbidden host\n', TEXT, head_only)

            path = self.path.split('?', 1)[0]

            try:
                found = self.route(path)
            except Exception as error:  # 예외 내용은 응답에 넣지 않는다
                log_error(f'요청 처리 오류 {path}: {type(error).__name__}: {error}')
                return self.reply(500, b'internal error\n', TEXT, head_only)

            if found is None:
                return self.reply(404, b'not found\n', TEXT, head_only)

            self.reply(200, found[0], found[1], head_only)

        def do_GET(self):
            self.serve(False)

        def do_HEAD(self):
            self.serve(True)

    return Handler


# 서버 생성(시작은 호출자가 serve_forever로)
def make_server(bind_address, port, handler):
    server = ThreadingHTTPServer((bind_address, port), handler)
    server.daemon_threads = True
    return server
