"""관제 화면 HTTP 서버(console_http) 시험. ROS 없이 실제 소켓(127.0.0.1 임시 포트)으로 확인한다.

확인: GET·HEAD 200, 쓰기 메서드 405, 허용 밖 Host 403, 경로 이탈·미등록 경로 404,
상태 함수 예외 시 고정 본문 500(예외 내용 미노출·로그 기록), CORS 헤더 없음, 정적 파일 형식.
"""
import http.client
import json
from pathlib import Path
import sys
import threading
import unittest

ROOT = Path(__file__).resolve().parents[2]
PKG = ROOT / 'src' / 'ax_operations_console'
sys.path.insert(0, str(PKG))

from ax_operations_console import console_http as ch  # noqa: E402

WEB = PKG / 'web'
SECRET = 'secret-detail-should-not-leak'


class HttpTest(unittest.TestCase):
    """서버 하나를 띄워 요청한다."""

    # 서버 시작(포트 0 → 임시 포트)
    @classmethod
    def setUpClass(cls):
        cls.errors = []
        cls.fail_state = False

        def state():
            if cls.fail_state:
                raise RuntimeError(SECRET)
            return {'schema': 1, 'robots': []}

        cls.server = ch.make_server('127.0.0.1', 0, None)
        cls.port = cls.server.server_address[1]
        hosts = ch.allowed_hosts('127.0.0.1', cls.port)
        pgm = b'P5\n1 1\n255\n\x00'
        cls.server.RequestHandlerClass = ch.make_handler(state, WEB, {'width': 1}, pgm, hosts,
                                                         cls.errors.append)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    # 서버 정지
    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=5)

    # 요청 도우미 → (상태, 헤더, 본문)
    def request(self, method, path, host=None):
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=5)
        conn.putrequest(method, path, skip_host=True)
        conn.putheader('Host', f'127.0.0.1:{self.port}' if host is None else host)
        conn.endheaders()
        response = conn.getresponse()
        body = response.read()
        conn.close()
        return response.status, dict(response.getheaders()), body

    # 화이트리스트 경로 GET 200과 형식
    def test_get_routes(self):
        expect = {'/': 'text/html', '/index.html': 'text/html', '/api/state': 'application/json',
                  '/map/info.json': 'application/json', '/map/map.pgm': 'application/octet-stream',
                  '/static/app.js': 'text/javascript', '/static/console.css': 'text/css'}

        for path, kind in expect.items():
            with self.subTest(path=path):
                status, headers, body = self.request('GET', path)
                self.assertEqual(status, 200)
                self.assertTrue(headers['Content-Type'].startswith(kind))
                self.assertNotIn('Access-Control-Allow-Origin', headers)
                self.assertEqual(headers['Cache-Control'], 'no-store')

        body = self.request('GET', '/api/state')[2]
        self.assertEqual(json.loads(body), {'schema': 1, 'robots': []})

    # HEAD는 본문 없이 200
    def test_head(self):
        status, headers, body = self.request('HEAD', '/api/state')
        self.assertEqual((status, body), (200, b''))
        self.assertGreater(int(headers['Content-Length']), 0)

    # 쓰기 메서드 405
    def test_write_methods_refused(self):
        for method in ('POST', 'PUT', 'DELETE', 'PATCH', 'OPTIONS'):
            with self.subTest(method=method):
                self.assertEqual(self.request(method, '/api/state')[0], 405)

    # 허용 밖 Host 403(localhost·127.0.0.1은 허용)
    def test_host_check(self):
        self.assertEqual(self.request('GET', '/', host=f'localhost:{self.port}')[0], 200)

        for host in (f'evil.example:{self.port}', '127.0.0.1', f'127.0.0.1:{self.port + 1}', ''):
            with self.subTest(host=host):
                self.assertEqual(self.request('GET', '/api/state', host=host)[0], 403)

    # 경로 이탈·미등록 경로 404
    def test_path_escape(self):
        paths = ('/static/../ax_operations_console/console_node.py', '/static/%2e%2e/package.xml',
                 '/static/', '/static/nope.js', '/api', '/config/console_topics.yaml',
                 '/etc/passwd')

        for path in paths:
            with self.subTest(path=path):
                self.assertEqual(self.request('GET', path)[0], 404)

    # 상태 함수 예외: 고정 본문 500, 예외 내용은 응답에 없고 로그에만 있다
    def test_error_body_fixed(self):
        type(self).fail_state = True

        try:
            status, _, body = self.request('GET', '/api/state')
        finally:
            type(self).fail_state = False

        self.assertEqual((status, body), (500, b'internal error\n'))
        self.assertNotIn(SECRET.encode(), body)
        self.assertTrue(any(SECRET in line for line in self.errors))


if __name__ == '__main__':
    unittest.main()
