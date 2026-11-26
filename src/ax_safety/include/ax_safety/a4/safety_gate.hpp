#pragma once
#include <vector>
#include <string>
#include <cstring>
// 이 헤더는 A4 주행 게이트의 순수 판정 부분이다.
// safety_gate.cpp가 ROS 메시지를 GateInput으로 정리하면, 여기서는 ROS 의존성 없이
// 신선도·허가·정지 상태와 요청 속도를 검사해 A2로 보낼 최종 속도를 계산한다.
#include <algorithm>
#include <cstdint>
#include <unordered_map>


#include <cmath>

namespace ax_safety {
// B4 SafetyState, A1/A2 상태, D2/D3 허가를 A4가 이해하는 입력 계약으로 묶는다.
// 하나라도 검증되지 않으면 evaluate_drive_gate는 기본값(0 속도·false)을 유지한다.


struct GateInput {
  bool enabled{false};
  bool safety_fresh{false};
  bool state_fresh{false};
  bool travel_pose_valid{false};
  bool operational_permits_valid{false};
  // operational_permits_valid is the existing explicit D2/D3 permit. This is
  // an additional A4 handoff interlock and must not weaken that permit.
  bool handoff_gate_valid{false};
  bool stop_required{true};
  bool allow_forward{false};
  bool allow_backward{false};
  bool allow_rotate{false};
  double max_linear{0};
  double max_angular{0};
};

struct HandoffTaskInput {
  std::string manipulator_robot_id;
  std::string transporter_robot_id;
  uint8_t transfer_state{0};
};

struct HandoffPeerInput {
  bool actuation_fresh{false};
  bool load_fresh{false};
};

constexpr uint8_t kTransferUnknown = 0;
constexpr uint8_t kTransferLocked = 2;
constexpr uint8_t kTransferInProgress = 3;

// DEC-19 handoff gate. A fresh list with no assignment for this robot has no
// handoff rule to apply. A missing/stale list or a session transition remains
// conservative and blocks, including for an idle/unassigned robot.
inline bool evaluate_handoff_gate(
  const std::string &robot_id, bool tasks_fresh, bool session_changed,
  const std::vector<HandoffTaskInput> &tasks,
  const std::unordered_map<std::string, HandoffPeerInput> &peers)
{
  if (!tasks_fresh || session_changed) return false;
  for (const auto &task : tasks) {
    const bool is_manipulator = task.manipulator_robot_id == robot_id;
    const bool is_transporter = task.transporter_robot_id == robot_id;
    if (!is_manipulator && !is_transporter) continue;
    if (task.transfer_state != kTransferUnknown &&
        task.transfer_state != kTransferLocked &&
        task.transfer_state != kTransferInProgress) continue;

    const std::string &peer_id = is_manipulator ? task.transporter_robot_id : task.manipulator_robot_id;
    const auto peer = peers.find(peer_id);
    if (peer == peers.end() || !peer->second.actuation_fresh ||
        (is_transporter && !peer->second.load_fresh)) return false;
    return false;  // Any locked/in-progress task blocks this robot.
  }
  return true;
}
// A4가 실제로 보낼 속도와 주행 허용 여부이다.
// drive_allowed가 false이면 호출자는 linear/angular를 사용하지 않고 정지 명령을 보내야 한다.


struct GateOutput {
  double linear{0};
  double angular{0};
  bool drive_allowed{false};
};
// 같은 stamp 표본의 내용 비교용 정규화 값: 이름(정렬)과 그 이름에 대응하는 원 값.
// 해시가 아니라 원 값을 비트 단위로 직접 비교한다(-0·NaN도 구분).
struct SampleContent {
  std::vector<std::string> names;
  std::vector<double> values;

  bool operator==(const SampleContent& o) const {
    if (names != o.names || values.size() != o.values.size()) return false;
    return values.empty() || std::memcmp(values.data(), o.values.data(), values.size() * sizeof(double)) == 0;
  }
};

// D-06 정지 근거: 피드백 채널 하나(odom 또는 joint_states)의 "연속 저속 구간"(ROS 의존 없음).
//
// 유지 시간은 wall tick이나 /clock 진행이 아니라 수용한 관측 표본의 stamp 진행으로만 늘어난다.
// - 수용(ACCEPTED): stamp가 유한·양수이고, 현재 ROS 시각(now_s) 이하이며, 이 채널 최신 수용 stamp보다 클 때만이다.
//   저속이면 구간 [start, end]의 end를 stamp로 늘린다. 직전 수용 stamp와의 간격이 max_gap_s를 넘으면
//   그 사이 움직임을 관측하지 못했으므로 구간을 끊고 이 표본부터 다시 시작한다.
//   저속이 아니면(이동·비유한·필수 관절 누락 등) 그 자리에서 구간을 버린다. 다음 tick 전에 저속 표본으로 덮여도
//   이동 사실이 사라지지 않는다.
// - 미래 stamp(stamp > now_s): 정지 근거로 쓰지 않고 최신 상태·신선도도 바꾸지 않는다(REJECTED).
//   저속이 아니면 이동 근거이므로 구간을 버린다. 보류하지 않으므로 시계가 그 시각에 닿아도 다시 쓰이지 않는다.
// - 같은 stamp 재수신: 정규화 내용이 같으면 무시한다(DUPLICATE). 정지 근거도, 신선도도 늘리지 않는다.
//   내용이 다르면(CONFLICT) 어느 쪽이 맞는지 알 수 없으므로 구간을 버린다.
// - 더 오래된 stamp(LATE, 메시지 순서 역전): 최신 상태를 덮지 않는다. 그 표본이 저속이 아니면 과거 움직임의 근거라
//   구간을 버린다(보수 처리. 그때의 사건을 복원했다는 뜻은 아니다). 저속이면 무시한다.
// - /clock 역행(리셋)은 호출자가 reset_all로 채널 전체를 비운다(LATE와 구분).
// 관측 사이의 실제 움직임은 측정하지 못한다. max_gap_s는 그 공백 허용의 상한이다.
struct LowChannel {
  enum Result { ACCEPTED = 0, DUPLICATE = 1, CONFLICT = 2, LATE = 3, REJECTED = 4 };

  bool has_latest{false};
  double latest_stamp{0.0};
  SampleContent latest_content;
  bool latest_low{false};
  bool has_interval{false};
  double start{0.0};
  double end{0.0};

  void reset_interval() { has_interval = false; }
  void reset_all() { has_latest = false; latest_low = false; has_interval = false; latest_content = SampleContent{}; }

  // 시각상 받아들일 수 없는 표본(미래·시계 무효·stamp 무효)
  Result reject(bool low) {
    if (!low) reset_interval();
    return REJECTED;
  }

  // stamp_s: 표본 header stamp(초), now_s: 현재 ROS 시각(초), low: 이 표본이 D-06 저속인지
  Result add(double stamp_s, double now_s, bool low, const SampleContent& content, double max_gap_s) {
    const bool clock_ok = std::isfinite(now_s) && now_s > 0.0;
    if (!clock_ok || !std::isfinite(stamp_s) || stamp_s <= 0.0 || stamp_s > now_s) return reject(low);

    if (has_latest && stamp_s < latest_stamp) {
      if (!low) reset_interval();
      return LATE;
    }

    if (has_latest && stamp_s == latest_stamp) {
      if (content == latest_content) return DUPLICATE;
      reset_interval();
      latest_low = false;
      return CONFLICT;
    }

    const double gap = has_latest ? stamp_s - latest_stamp : 0.0;
    has_latest = true;
    latest_stamp = stamp_s;
    latest_content = content;
    latest_low = low;

    if (!low) {
      reset_interval();
    } else if (!has_interval || gap > max_gap_s) {
      has_interval = true;
      start = end = stamp_s;
    } else {
      end = stamp_s;
    }

    return ACCEPTED;
  }
};

// 진단 전용(판정에 쓰지 않음): REJECTED 표본의 이유를 add와 같은 조건 순서로 분류한다.
// 0=시계 무효(now 비유한·0 이하), 1=stamp 무효(비유한·0 이하), 2=미래(stamp > now), 3=거부 아님
inline int reject_reason(double stamp_s, double now_s) {
  if (!std::isfinite(now_s) || now_s <= 0.0) return 0;
  if (!std::isfinite(stamp_s) || stamp_s <= 0.0) return 1;
  if (stamp_s > now_s) return 2;
  return 3;
}

// 진단 전용 채널 카운터: LowChannel::Result별 수와 REJECTED 이유별 수(판정에 쓰지 않음)
struct ChannelCounters {
  unsigned long long result[5]{0, 0, 0, 0, 0};
  unsigned long long reject[3]{0, 0, 0};

  void count(LowChannel::Result r, double stamp_s, double now_s) {
    result[r] += 1;
    if (r == LowChannel::REJECTED) {
      const int why = reject_reason(stamp_s, now_s);
      if (why < 3) reject[why] += 1;
    }
  }
};

// 한 채널 저속 구간 길이(초). 구간 끝은 현재 시각 now_s를 넘지 않게 자른다(수용 규칙상 넘지 않지만 판정에서도 막는다).
inline double low_span(const LowChannel& a, double now_s) {
  if (!a.has_interval || !std::isfinite(now_s)) return 0.0;
  const double hi = std::min(a.end, now_s);
  return hi > a.start ? hi - a.start : 0.0;
}

// 두 채널의 저속 구간이 함께 덮는 길이(초). 어느 쪽이든 구간이 없거나 겹치지 않으면 0.
// 서로 다른 주기의 odom·joint_states를 같은 stamp로 맞추지 않고, 각자 관측한 연속 저속 구간의 교집합만
// 정지 유지 근거로 쓴다. 교집합의 끝은 두 채널 중 더 이른 최신 저속 stamp이며 now_s를 넘지 않는다.
inline double common_low_span(const LowChannel& a, const LowChannel& b, double now_s) {
  if (!a.has_interval || !b.has_interval || !std::isfinite(now_s)) return 0.0;
  const double lo = std::max(a.start, b.start);
  const double hi = std::min({a.end, b.end, now_s});
  return hi > lo ? hi - lo : 0.0;
}

// 팔 정지 조건 역할. 정지 판정에만 쓰며 주행·팔 허가를 열지 않는다.
// - UNCONFIRMED: 역할 근거 없음(기본). 정지를 확인하지 않는다.
// - PRESENT: 팔 있음. 필수 팔 관절 목록(비어 있지 않음, 빈 이름·중복 없음)이 있어야 한다.
// - ABSENT: 팔 없음을 명시. 팔 정지 조건은 해당 없음, 차체 정지는 odom으로 확인한다.
// - INVALID: 알 수 없는 값이거나 PRESENT인데 목록이 잘못됨. 정지를 확인하지 않는다.
enum class ArmPresence { UNCONFIRMED, PRESENT, ABSENT, INVALID };

inline ArmPresence parse_arm_presence(const std::string& presence, const std::vector<std::string>& required) {
  if (presence == "unconfirmed") return ArmPresence::UNCONFIRMED;
  if (presence == "absent") return required.empty() ? ArmPresence::ABSENT : ArmPresence::INVALID;
  if (presence != "present" || required.empty()) return ArmPresence::INVALID;

  std::vector<std::string> sorted = required;
  std::sort(sorted.begin(), sorted.end());
  for (std::size_t i = 0; i < sorted.size(); ++i) {
    if (sorted[i].empty() || (i > 0 && sorted[i] == sorted[i - 1])) return ArmPresence::INVALID;
  }
  return ArmPresence::PRESENT;
}

// 안전 조건과 방향별 허용 플래그를 확인한 뒤 속도 상한을 적용한다.
// finite 검사와 음수 상한 검사는 NaN/잘못된 파라미터가 차단 경로를 우회하지 못하게 한다.


inline GateOutput evaluate_drive_gate(const GateInput& g, double linear, double angular) {
  GateOutput result;
  // 초기 result는 의도적으로 정지 상태다. 모든 안전 조건을 만족할 때만 아래에서 허용한다.
  if (!g.enabled || !g.safety_fresh || !g.state_fresh ||
      !g.travel_pose_valid || !g.operational_permits_valid || !g.handoff_gate_valid ||
      g.stop_required ||
      !std::isfinite(linear) || !std::isfinite(angular) ||
      !std::isfinite(g.max_linear) || !std::isfinite(g.max_angular) ||
      g.max_linear < 0 || g.max_angular < 0) return result;
  // 양·음 주행과 회전은 B4가 허용한 방향만 통과시킨다.


  if ((linear > 0 && !g.allow_forward) || (linear < 0 && !g.allow_backward) ||
      (angular != 0 && !g.allow_rotate)) return result;
  result.linear = std::clamp(linear, -g.max_linear, g.max_linear);
  result.angular = std::clamp(angular, -g.max_angular, g.max_angular);
  result.drive_allowed = true;
  return result;
}
}
