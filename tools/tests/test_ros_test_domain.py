# 시험 domain 공용 도우미(ros_test_domain.py)를 검사한다: 범위 배정, 사용 중·잠금 domain 건너뛰기, 잠금 해제.
# ROS 통신 없이 /proc 환경변수와 잠금 파일만 쓴다.
import os
from pathlib import Path
import subprocess
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import ros_test_domain  # noqa: E402


class AssignedRanges(unittest.TestCase):

    # 시험별 범위는 서로 겹치지 않고 팀 격리 범위(141~231) 밖이며, ROS domain 유효값(0~232) 안이다.
    def test_ranges_disjoint_and_outside_team(self):
        ranges = ros_test_domain.assigned_ranges()
        seen = {}

        for name, domains in ranges.items():
            for domain in domains:
                self.assertNotIn(domain, seen, f'{name}와 {seen.get(domain)}가 domain {domain}을 함께 씀')
                seen[domain] = name

        self.assertFalse(set(seen) & set(ros_test_domain.TEAM_ISOLATION))
        self.assertTrue(all(0 <= domain <= 232 for domain in seen))

    # 팀 격리 범위와 겹치는 범위는 거절한다.
    def test_reject_team_range(self):
        with self.assertRaises(ValueError):
            ros_test_domain.reserve_domain(range(139, 145))


class Reserve(unittest.TestCase):

    # 첫 후보를 다른 프로세스가 쓰면 다음 값, 잠금을 쥔 동안은 그다음 값, 잠금을 풀면 다시 쓸 수 있다.
    def test_skip_busy_and_locked(self):
        domains = ros_test_domain.STARTUP_DOMAINS
        first = list(domains)[os.getpid() % len(domains)]
        busy = subprocess.Popen(['sleep', '30'], env=dict(os.environ, ROS_DOMAIN_ID=str(first)))
        self.addCleanup(busy.wait)
        self.addCleanup(busy.kill)

        a, lock_a = ros_test_domain.reserve_domain(domains)
        b, lock_b = ros_test_domain.reserve_domain(domains)

        self.assertNotEqual(a, first)
        self.assertNotIn(b, (first, a))
        self.assertEqual(ros_test_domain.other_domain_users(first), [busy.pid])

        ros_test_domain.release(lock_a)
        c, lock_c = ros_test_domain.reserve_domain(domains)
        ros_test_domain.release(lock_b)
        ros_test_domain.release(lock_c)

        self.assertEqual(c, a)

    # 모두 사용 중이면 RuntimeError로 끝난다.
    def test_all_busy(self):
        domains = range(232, 233)
        domain, lock = ros_test_domain.reserve_domain(domains)

        try:
            with self.assertRaises(RuntimeError):
                ros_test_domain.reserve_domain(domains)
        finally:
            ros_test_domain.release(lock)

        self.assertEqual(domain, 232)


if __name__ == '__main__':
    unittest.main()
