"""ROS 메시지 → 판정용 Sample 변환(ROS import 없음, 속성 이름으로만 읽는다).

화면에 필요한 요약 필드만 꺼낸다. 원본 메시지는 보관하지 않는다.
"""
import math

from .console_state import (ACTUATION_NAMES, enum_name, HEALTH_NAMES, RESULT_NAMES, SAFETY_NAMES,
                            Sample, TASK_NAMES, TRANSFER_NAMES)


# builtin_interfaces/Time → 초
def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


# StateMeta가 있는 메시지의 Sample 틀
def with_meta(meta, value):
    return Sample(value=value, stamp=stamp_s(meta.header.stamp), session=meta.source_session_id,
                  revision=int(meta.revision), valid=bool(meta.valid), reason=meta.validity_reason,
                  source_id=meta.source_id, schema=int(meta.schema_version))


# ActuationState: 상태·두 허가·차단 사유
def actuation(msg):
    return with_meta(msg.meta, {
        'state': enum_name(ACTUATION_NAMES, msg.state),
        'drive_allowed': bool(msg.drive_allowed),
        'arm_allowed': bool(msg.arm_allowed),
        'block_reasons': list(msg.block_reasons),
        'linear_speed': round(float(msg.linear_speed), 3),
    })


# SafetyState: 수준·사유·속도 상한
def safety(msg):
    return with_meta(msg.meta, {
        'level': enum_name(SAFETY_NAMES, msg.level),
        'reason': msg.reason_code,
        'max_linear_speed': round(float(msg.max_linear_speed), 3),
    })


# LocalizationHealth: 상태·위치 표준편차
def health(msg):
    return with_meta(msg.meta, {
        'state': enum_name(HEALTH_NAMES, msg.state),
        'position_std_m': round(float(msg.position_std_m), 3),
    })


# RobotTaskStatus: 로컬 단계·일시 정지·마지막 결과
def executor(msg):
    return with_meta(msg.meta, {
        'task_id': msg.execution.task_id,
        'local_phase': msg.local_phase,
        'paused': bool(msg.paused),
        'last_result': enum_name(RESULT_NAMES, msg.last_result.outcome),
    })


# PoseWithCovarianceStamped(map): 위치·yaw(표시 전용, meta 없음)
def pose(msg):
    q = msg.pose.pose.orientation
    p = msg.pose.pose.position
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    return Sample(value={'x': round(p.x, 3), 'y': round(p.y, 3), 'yaw': round(yaw, 4),
                         'frame': msg.header.frame_id},
                  stamp=stamp_s(msg.header.stamp))


# BatteryState: 잔량·장착(표시 전용, meta 없음). NaN은 값 없음으로 둔다
def battery(msg):
    percentage = float(msg.percentage)
    value = {'percentage': None if math.isnan(percentage) else round(percentage, 3),
             'present': bool(msg.present)}
    return Sample(value=value, stamp=stamp_s(msg.header.stamp))


# GroupTaskList: 작업별 상태·인계·역할 로봇
def tasks(msg):
    rows = [{
        'task_id': t.task_id,
        'state': enum_name(TASK_NAMES, t.state),
        'transfer': enum_name(TRANSFER_NAMES, t.transfer_state),
        'manipulator': t.manipulator_robot_id,
        'transporter': t.transporter_robot_id,
        'source': t.source_location_id,
        'destination': t.destination_location_id,
    } for t in msg.tasks]
    return with_meta(msg.meta, rows)


CONVERTERS = {'actuation': actuation, 'safety': safety, 'health': health, 'executor': executor,
              'pose': pose, 'battery': battery}
