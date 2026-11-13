#pragma once
// 이 헤더는 A4 주행 게이트의 순수 판정 부분이다.
// safety_gate.cpp가 ROS 메시지를 GateInput으로 정리하면, 여기서는 ROS 의존성 없이
// 신선도·허가·정지 상태와 요청 속도를 검사해 A2로 보낼 최종 속도를 계산한다.
#include <algorithm>


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
  bool stop_required{true};
  bool allow_forward{false};
  bool allow_backward{false};
  bool allow_rotate{false};
  double max_linear{0};
  double max_angular{0};
};
// A4가 실제로 보낼 속도와 주행 허용 여부이다.
// drive_allowed가 false이면 호출자는 linear/angular를 사용하지 않고 정지 명령을 보내야 한다.


struct GateOutput {
  double linear{0};
  double angular{0};
  bool drive_allowed{false};
};
// 안전 조건과 방향별 허용 플래그를 확인한 뒤 속도 상한을 적용한다.
// finite 검사와 음수 상한 검사는 NaN/잘못된 파라미터가 차단 경로를 우회하지 못하게 한다.


inline GateOutput evaluate_drive_gate(const GateInput& g, double linear, double angular) {
  GateOutput result;
  // 초기 result는 의도적으로 정지 상태다. 모든 안전 조건을 만족할 때만 아래에서 허용한다.
  if (!g.enabled || !g.safety_fresh || !g.state_fresh ||
      !g.travel_pose_valid || !g.operational_permits_valid || g.stop_required ||
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
