#pragma once
#include <algorithm>
#include <cmath>

namespace ax_safety {
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
struct GateOutput {
  double linear{0};
  double angular{0};
  bool drive_allowed{false};
};
inline GateOutput evaluate_drive_gate(const GateInput& g, double linear, double angular) {
  GateOutput result;
  if (!g.enabled || !g.safety_fresh || !g.state_fresh ||
      !g.travel_pose_valid || !g.operational_permits_valid || g.stop_required ||
      !std::isfinite(linear) || !std::isfinite(angular) ||
      !std::isfinite(g.max_linear) || !std::isfinite(g.max_angular) ||
      g.max_linear < 0 || g.max_angular < 0) return result;
  if ((linear > 0 && !g.allow_forward) || (linear < 0 && !g.allow_backward) ||
      (angular != 0 && !g.allow_rotate)) return result;
  result.linear = std::clamp(linear, -g.max_linear, g.max_linear);
  result.angular = std::clamp(angular, -g.max_angular, g.max_angular);
  result.drive_allowed = true;
  return result;
}
}
