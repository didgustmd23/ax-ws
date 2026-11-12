#!/usr/bin/env python3
"""B2 지도 작성 주행의 회전 정렬(선택 모드) 계산·상태 전환. ROS·Gazebo 의존이 없어 단독 시험할 수 있다.

근거: log/b2_custom_warehouse/20261111_112945/c_align(C 조건 시험)의 align_logic.py를 일반화했다.
C 시험과 다른 점
- 각속도 상한을 고정 0.3이 아니라 호출부가 넘긴 실행 상한(route와 제어기 제한 중 작은 값)으로 쓴다.
- 방향이 비유한이면 실패, sim 시계가 되돌아가면 실패로 끝낸다(운영 도구의 입력 보호와 맞춤).

명령 법칙: ω = sign(e) · min(ω_max, max(ω_min, k·|e|)). |e| ≤ stop_band이면 0을 내고 정지 확인 뒤 e를 다시 계산하며,
정지 뒤 |e| ≤ final_tolerance일 때만 성공(직진 허가)이다. 아니면 최대 max_realign회 다시 정렬하고, 정렬 전체는
limit_sim_s(sim)·limit_wall_s(단조 벽시계) 안에 끝나야 한다. 실패 사유는 AlignFailed로 알린다.
"""
import math

# 경로 YAML rotation_align 항목의 키(모두 필요). 첫 적용값은 C 조건 실측값이다.
KEYS = ("gain_per_s", "omega_min_rad_s", "stop_band_rad", "final_tolerance_rad", "max_realign", "limit_sim_s",
        "limit_wall_s")


class AlignFailed(RuntimeError):
    pass


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# 경로의 rotation_align 항목을 검사해 dict로 돌려준다. 틀리면 ValueError. omega_max는 실행 상한(선택, 비교용).
def parse(item, omega_max=None):
    if not isinstance(item, dict) or set(item) != set(KEYS):
        raise ValueError("rotation_align은 " + "·".join(KEYS) + " 일곱 값만 가능")

    values = {}

    for key in KEYS:
        value = item[key]

        if key == "max_realign":
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError("rotation_align max_realign은 0 이상 정수")
            values[key] = value
            continue

        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"rotation_align {key}는 유한한 양수")

        values[key] = float(value)

    if values["stop_band_rad"] > values["final_tolerance_rad"]:
        raise ValueError("rotation_align stop_band_rad는 final_tolerance_rad 이하")

    if omega_max is not None and values["omega_min_rad_s"] > omega_max:
        raise ValueError("rotation_align omega_min_rad_s가 각속도 상한보다 큼")

    return values


# 목표 방향 오차 e에 대한 각속도 명령(정지 대역 안이면 0).
def omega_command(error, law, omega_max):
    if abs(error) <= law["stop_band_rad"]:
        return 0.0

    return math.copysign(min(omega_max, max(law["omega_min_rad_s"], law["gain_per_s"] * abs(error))), error)


# 정렬 절차. io는 heading()·now_sim()·now_wall()·rotate(w)(선속도 0 회전 명령 발행, 보호 검사 포함)·stop_confirm()
# (성공 시 반환, 실패 시 예외)·tick()을 제공한다. 결과 dict(passes 포함) 또는 AlignFailed.
def align(io, target, law, omega_max):
    sim0, wall0 = io.now_sim(), io.now_wall()
    last_sim = [sim0]
    passes = []

    def check_limits():
        now = io.now_sim()

        if now < last_sim[0] - 1e-9:
            raise AlignFailed("sim 시계 역행")

        last_sim[0] = now

        if now - sim0 > law["limit_sim_s"]:
            raise AlignFailed(f"정렬 sim 시간 {law['limit_sim_s']} s 초과")

        if io.now_wall() - wall0 > law["limit_wall_s"]:
            raise AlignFailed(f"정렬 벽시계 {law['limit_wall_s']} s 초과")

    def error():
        heading = io.heading()

        if not math.isfinite(heading):
            raise AlignFailed("odom 방향 비유한")

        return wrap(target - heading)

    for attempt in range(law["max_realign"] + 1):
        while True:
            check_limits()
            w = omega_command(error(), law, omega_max)

            if w == 0.0:
                break

            io.rotate(w)
            io.tick()

        try:
            io.stop_confirm()
        except AlignFailed:
            raise
        except Exception as exc:  # noqa: BLE001
            raise AlignFailed(f"정렬 정지 확인 실패: {exc}") from exc

        e = error()
        passes.append({"attempt": attempt + 1, "error_after_stop_rad": e, "sim_t": io.now_sim()})

        if abs(e) <= law["final_tolerance_rad"]:
            return {"aligned": True, "realigns": attempt, "final_error_rad": e, "sim_s": io.now_sim() - sim0,
                    "wall_s": io.now_wall() - wall0, "passes": passes}

        check_limits()

    raise AlignFailed(f"재정렬 {law['max_realign']}회 뒤에도 허용 밖(마지막 오차 {passes[-1]['error_after_stop_rad']:.5f} rad)")
