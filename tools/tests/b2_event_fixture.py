# B2-L4~L7 사건 지표 시험용 합성 원시 기록(ROS 없음). 시험 모듈이 import해서 쓴다(시험 수집 대상 아님).
#
# 기본 시나리오(robot_2, 감독 세션 s1, 0~40 s, T_world_map 단위 변환):
# - 3.0 s 정지·OK에서 재위치 요청 → REINIT_REJECTED_NOT_NEEDED
# - 5~10 s 주행(0.3 m/s), 6.8~7.4 s health DEGRADED, 7.0 s 요청 → REINIT_REJECTED_NOT_STOPPED
# - 15.0 s AMCL lifecycle active 이탈, 15.4 s health LOST(지연 0.4 s)
# - 17.0 s 재위치 접수(attempt 1), 19.9 s SUCCEEDED, 20.0 s OK 복귀(오차 0)
import copy

SESSION = 's1'


# start~end(포함)를 step 간격으로 나눈 시각 목록(정수 배로 만들어 누적 오차를 피한다)
def times(start, end, step):
    count = int(round((end - start) / step))

    return [round(start + step * index, 6) for index in range(count + 1)]


# 기본 시나리오의 x 위치(5~10 s 동안 0.3 m/s로 이동)
def position(t):
    return 0.3 * (min(max(t, 5.0), 10.0) - 5.0)


# 기본 시나리오의 health 상태(state, valid)
def health_state(t):
    if 6.8 <= t < 7.4:
        return 'DEGRADED', True

    if 15.4 <= t < 20.0:
        return 'LOST', True

    return 'OK', True


# health 원시 기록 한 줄
def health_row(t, state, valid=True, session=SESSION, robot_id='robot_2'):
    return {'stamp': t, 'state': state, 'valid': valid, 'reason': '' if state == 'OK' else 'X',
            'session': session, 'robot_id': robot_id, 'recv_sim': t, 'recv_mono': 1000.0 + t}


# 감독 진단 한 줄(state·valid는 감독이 판단에 쓴 health)
def diag(kind, t, state='OK', valid=True, session=SESSION, **extra):
    return {'kind': kind, 'sim': t, 'mono': 1000.0 + t, 'session': session, 'state': state, 'valid': valid,
            'reinit_failure': None, **extra}


# 기본 시나리오 원시 기록 dict(collect_raw_evidence가 읽는 stream 이름과 같다)
def streams():
    health = [health_row(t, *health_state(t)) for t in times(0.0, 40.0, 0.2)]
    odom = [{'stamp': t, 'x': position(t), 'y': 0.0, 'yaw': 0.0, 'v': 0.3 if 5.0 <= t < 10.0 else 0.0, 'w': 0.0}
            for t in times(0.0, 40.0, 0.1)]
    tf = [{'stamp': t + 1.0, 'valid_time': t, 'tf_future_s': 1.0, 'x': 0.0, 'y': 0.0, 'yaw': 0.0}
          for t in times(0.0, 40.0, 0.1)]
    truth = [{'stamp': t, 'x': position(t), 'y': 0.0, 'yaw': 0.0} for t in times(0.0, 40.0, 0.05)]
    lifecycle = [{'start_state': 'active', 'goal_state': 'deactivating', 'recv_sim': 15.0, 'recv_mono': 1015.0}]
    actuation = [{'stamp': t, 'state': 4 if not 5.0 <= t < 10.0 else 1, 'valid': True,
                  'source_id': 'stub_actuation_state', 'robot_id': 'robot_2'} for t in times(0.0, 40.0, 0.1)]
    supervisor = [diag('tick', t, *health_state(t)) for t in times(0.0, 40.0, 0.2)]
    supervisor += [
        diag('reinit_request', 3.0, accepted=False, code='REINIT_REJECTED_NOT_NEEDED'),
        diag('reinit_request', 7.0, 'DEGRADED', accepted=False, code='REINIT_REJECTED_NOT_STOPPED'),
        diag('reinit_request', 17.0, 'LOST', accepted=True, code='REINIT_ACCEPTED', attempt=1),
        diag('reinit_finished', 19.9, 'LOST', result='SUCCEEDED', attempt=1, timeout=None),
    ]
    supervisor.sort(key=lambda row: row['sim'])

    return {'health': health, 'odom': odom, 'tf_map_odom': tf, 'truth': truth, 'amcl_lifecycle': lifecycle,
            'actuation': actuation, 'supervisor': supervisor}


JUMP_LIMITS = {'position_m': 0.3, 'yaw_rad': 0.3}


# 단위 T_world_map 행렬(평가용 실제 자세 = 지도 좌표)
def identity_world_map(check):
    return check.se2(0.0, 0.0, 0.0)


# 합성 기록으로 B2-L4~L7 근거를 만든다. changes(stream dict)를 받아 바꾼 뒤 계산한다.
def evidence(check, criteria, changes=None, source_problems=(), record_end=40.0, world_map='identity'):
    data = streams()

    if changes is not None:
        changes(data)

    matrix = identity_world_map(check) if world_map == 'identity' else world_map

    return check.event_evidence(data, matrix, criteria, JUMP_LIMITS, list(source_problems), record_end, 'robot_2')


# 합성 기록으로 B2-L4~L7 항목을 만든다.
def items(check, criteria, changes=None, **options):
    return check.event_items('robot_2', evidence(check, criteria, changes, **options), copy.deepcopy(criteria))
