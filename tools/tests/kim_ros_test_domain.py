"""Require a team-approved domain range before Kim's ROS integration tests."""
import os
import unittest

import ros_test_domain

_state = None


def setUpModule():
    global _state
    # 담당자와 합의한 범위만 명시합니다. 기본 domain이나 임의 범위를 사용하지 않습니다.
    configured = os.environ.get('AX_KIM_TEST_DOMAINS', '')
    if not configured:
        raise unittest.SkipTest('Set AX_KIM_TEST_DOMAINS=start-end after team approval')
    bounds = configured.split('-')
    if len(bounds) != 2:
        raise ValueError('AX_KIM_TEST_DOMAINS must be start-end')
    start, end = map(int, bounds)
    if not 1 <= start <= end < 100:
        raise ValueError('Kim test domains require an approved range within 1-99')
    domain, lock = ros_test_domain.reserve_domain(range(start, end + 1))
    # 잠금만으로 외부 노드까지 막을 수 없어 실제 그래프도 검사합니다.
    try:
        nodes = ros_test_domain.conflicting_nodes(domain, r'^/(?!ax_test_domain_probe$).+')
        services = ros_test_domain.conflicting_services(domain, r'^/ax/')
        if nodes or services:
            raise AssertionError(ros_test_domain.conflict_message(
                domain, nodes + services, '노드·서비스'))
    except BaseException:
        # 시작 실패 뒤 예약이 남아 다음 시험을 막지 않도록 잠금을 반환합니다.
        ros_test_domain.release(lock)
        raise
    previous = {key: os.environ.get(key) for key in (
        'ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')}
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    _state = {'domain': domain, 'lock': lock, 'previous': previous}


def tearDownModule():
    # 시험 전 환경을 복원하고 domain 예약을 해제합니다.
    global _state
    ros_test_domain.leave_module_domain(_state)
    _state = None
