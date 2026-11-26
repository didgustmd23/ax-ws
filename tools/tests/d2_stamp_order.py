"""D2 출력 stamp 단조 판정(순수 함수, ROS 없음). 시스템 시계 역행이 측정된 경우에만 감소 1회를 허용한다.

stamp는 ROS 시간(시스템 시계)이고 재발행 주기는 STEADY 시계다. WSL에서 시스템 시계가 약 32 s마다
1.2~1.75 s 역행하는 것을 실측했다(/tmp/ax_w3_logs probe_timer_clock). 그 순간 발행된 stamp는 앞
메시지보다 작아질 수 있다. 시험은 수신 측에서 offset = time.time() - time.monotonic()을 재서 역행이
실제로 있었을 때만, 그 크기 안에서 감소를 허용한다. 노드 결함(역행 없이 감소, 큰 감소, 여러 번 감소,
중복 stamp)은 계속 실패로 잡는다.
"""

# 역행 판정 임계(s). offset은 수신 처리 지연으로 수 ms 흔들리므로 그보다 크고, 실측 역행(1.2~1.75 s)보다
# 훨씬 작은 값으로 둔다
CLOCK_REGRESSION_THRESHOLD_S = 0.05

# 허용 감소 폭 여유(s): stamp 시각과 offset 측정 시각의 차이로 생기는 오차
STAMP_DECREASE_MARGIN_S = 0.05


# offset 열에서 가장 큰 감소량(s). 앞의 최댓값에서 뒤 값으로 떨어진 폭(없으면 0)
def clock_regression_s(offsets):
    peak = None
    drop = 0.0

    for value in offsets:
        if peak is None or value > peak:
            peak = value
        drop = max(drop, peak - value)

    return drop


# stamp(ns) 열 판정 → (통과 여부, 설명). offsets는 창 시작·수신마다·창 종료에 잰 offset(s)
def check_stamp_order(stamps_ns, offsets):
    if len(set(stamps_ns)) != len(stamps_ns):
        return False, f'중복 stamp: {stamps_ns}'

    decreases = [(stamps_ns[i] - stamps_ns[i + 1]) / 1e9
                 for i in range(len(stamps_ns) - 1) if stamps_ns[i + 1] < stamps_ns[i]]
    regression = clock_regression_s(offsets)

    # 역행이 없으면 엄격한 단조 증가
    if regression <= CLOCK_REGRESSION_THRESHOLD_S:
        if decreases:
            return False, (f'시스템 시계 역행 없음(offset Δ=-{regression:.3f} s)인데 stamp 감소 '
                           f'{[round(d, 3) for d in decreases]} s')
        return True, ''

    # 역행이 있으면 감소 1회만, 측정한 역행량 + 여유 이내만 허용
    if len(decreases) > 1:
        return False, (f'시스템 시계 역행 감지(offset Δ=-{regression:.3f} s)지만 stamp 감소 '
                       f'{len(decreases)}회: {[round(d, 3) for d in decreases]} s')

    if decreases and decreases[0] > regression + STAMP_DECREASE_MARGIN_S:
        return False, (f'stamp 감소 {decreases[0]:.3f} s가 측정 역행 {regression:.3f} s + '
                       f'{STAMP_DECREASE_MARGIN_S} s를 넘음')

    if decreases:
        return True, (f'시스템 시계 역행 감지: offset Δ=-{regression:.3f} s, '
                      f'stamp 감소 {decreases[0]:.3f} s 허용')

    return True, f'시스템 시계 역행 감지: offset Δ=-{regression:.3f} s, stamp 감소 없음'
