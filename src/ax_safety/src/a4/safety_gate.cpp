// A4 런타임 주행 게이트: Nav2 -> A4 -> A2 diff_drive_controller.
// 안전 미검증인 경우 기본값으로 모든 이동 명령을 0으로 차단한다.
#include "ax_safety/a4/safety_gate.hpp"

#include <ax_interfaces/msg/safety_state.hpp>
#include <ax_interfaces/msg/actuation_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <algorithm>
#include <cstring>
#include <limits>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using ax_interfaces::msg::SafetyState;
using ax_interfaces::msg::ActuationState;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::JointState;

class A4SafetyGate : public rclcpp::Node {
public:
  A4SafetyGate() : Node("a4_safety_gate") {
    // 안전 게이트는 필요한 검증값이 모두 준비되기 전까지 잠긴 상태로 시작한다.

    // 기본값은 잠금 상태이며, A1 자세·D2/D3 허가·시뮬레이션 시험 조건을 각각 확인해야 연다.
    robot_ = declare_parameter<std::string>("robot_id", "robot_1");
    enable_ = declare_parameter<bool>("enable_simulation_drive", false);

    // D2/D3의 작업·통로 허가 계약이 완전히 구현되어야 true로 설정 가능.
    permits_verified_ = declare_parameter<bool>("operational_permits_verified", false);
    pose_approved_ = declare_parameter<bool>("travel_envelope_verified", false);
    arm_approved_ = declare_parameter<bool>("arm_operation_verified", false);

    joint_names_ = declare_parameter<std::vector<std::string>>(
        "travel_joint_names", std::vector<std::string>{});
    joint_positions_ = declare_parameter<std::vector<double>>(
        "travel_joint_positions", std::vector<double>{});
    pose_tolerance_ = declare_parameter<double>("travel_pose_tolerance_rad", 0.04);

    max_linear_ = declare_parameter<double>("max_linear_m_s", 0.67);
    max_angular_ = declare_parameter<double>("max_angular_rad_s", 0.50);
    max_linear_accel_ = declare_parameter<double>("max_linear_accel_m_s2", 0.67);
    max_angular_accel_ = declare_parameter<double>("max_angular_accel_rad_s2", 0.20);

    // D-06 실제 정지 확인 기준은 명령 제한값이 아니라 피드백 기준값이다.
    // 정지 명령을 보냈다는 사실만으로 STOPPED를 판정하지 않는다.
    stop_linear_threshold_ = declare_parameter<double>("stop_linear_threshold_m_s", 0.01);
    stop_angular_threshold_ = declare_parameter<double>("stop_angular_threshold_rad_s", 0.02);
    stop_joint_threshold_ = declare_parameter<double>("stop_joint_threshold_rad_s", 0.01);
    stop_hold_s_ = declare_parameter<double>("stop_hold_s", 0.5);

    // 팔 정지 조건 역할과 필수 팔 관절(정지 판정 전용, 허가와 무관). 기본은 역할 미정 = 정지 미확인.
    // 값은 모델별 파일(config/stop_evidence_*.yaml)을 launch stop_evidence_file로 명시해 준다.
    const auto presence = declare_parameter<std::string>("stop_arm_presence", "unconfirmed");
    stop_arm_joints_ = declare_parameter<std::vector<std::string>>("stop_arm_joint_names", std::vector<std::string>{});
    arm_presence_ = ax_safety::parse_arm_presence(presence, stop_arm_joints_);
    if (arm_presence_ == ax_safety::ArmPresence::UNCONFIRMED)
      RCLCPP_WARN(get_logger(), "stop_arm_presence unconfirmed: STOPPED will not be confirmed.");
    if (arm_presence_ == ax_safety::ArmPresence::INVALID)
      RCLCPP_ERROR(get_logger(), "stop_arm_presence/stop_arm_joint_names invalid: STOPPED will not be confirmed.");
    // 진단 로그(기본 false). 판정·발행 내용은 바꾸지 않는다.
    diagnostics_ = declare_parameter<bool>("stop_diagnostics", false);
    session_ = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());

    const auto base = "/ax/" + robot_;

    // nav 입력은 A4가 받고, 출력은 A2 diff_drive_controller로 보낸다.
    auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto feedback_qos = rclcpp::SensorDataQoS();

    pub_cmd_ = create_publisher<TwistStamped>(base + "/diff_drive_controller/cmd_vel", cmd_qos);
    pub_state_ = create_publisher<ActuationState>(base + "/safety/actuation_state", state_qos);

    sub_safety_ = create_subscription<SafetyState>(base + "/safety/state", state_qos,
        [this](SafetyState::SharedPtr m) { safety_ = *m; received_safety_ = steady(); got_safety_ = true; });
    // 피드백은 표본마다 바로 정지 근거를 갱신한다(on_odom·on_joints). 수용한 표본만 상태·수신 시각을 바꾼다.
    sub_odom_ = create_subscription<Odometry>(base + "/diff_drive_controller/odom", feedback_qos,
        [this](Odometry::SharedPtr m) { on_odom(*m); });
    sub_joints_ = create_subscription<JointState>(base + "/joint_states", feedback_qos,
        [this](JointState::SharedPtr m) { on_joints(*m); });
    sub_cmd_ = create_subscription<TwistStamped>(base + "/nav/cmd_vel", cmd_qos,
        [this](TwistStamped::SharedPtr m) { on_command(*m); });

    timer_ = create_wall_timer(50ms, [this]() { tick(); });
    RCLCPP_WARN(get_logger(), "A4 gate started locked. A1 pose envelope and D2/D3 permits must be explicitly verified.");
  }
private:
  // 통신 수신 시각은 steady clock으로 기록해 시스템 시간 변경의 영향을 받지 않게 한다.
  // steady clock은 시스템 시간이 바뀌어도 통신 timeout을 일정하게 계산하게 한다.
  static std::chrono::steady_clock::time_point steady() { return std::chrono::steady_clock::now(); }

  static double elapsed(std::chrono::steady_clock::time_point t) {
    return std::chrono::duration<double>(steady()-t).count();
  }

  // 피드백 정지 근거의 시간 한계.
  // - 0.3 s(kFeedbackTtlS): 표본 나이·무수신 한계는 이 게이트의 기존 state_fresh 값(계약 D-05 구동 상태 만료 0.3 s)이다.
  //   같은 수치를 "연속 표본 간격 상한"(연속성)에도 쓰는데, 이 적용은 새 개발 규칙이다(실측·운영 승인 전).
  // - 미래 stamp(현재 ROS 시각보다 뒤)는 허용하지 않는다(정지 근거·신선도 모두).
  // - /clock 정체 3.0 s(벽시계): 계약 D-09 "/clock이 3 s 진행하지 않으면 PAUSED".
  static constexpr double kFeedbackTtlS = 0.3;
  static constexpr double kClockPauseS = 3.0;

  static double stamp_seconds(const builtin_interfaces::msg::Time &stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  // odom 표본 하나: D-06 선·각속도 저속 여부를 판정하고 채널 구간을 갱신한다.
  // 같은 stamp 비교 대상은 정지 판정에 쓰는 linear.x·angular.z와 frame_id·child_frame_id다.
  // pose·공분산 등 다른 필드는 비교하지 않는다. frame 이름 자체의 옳고 그름은 검증하지 않는다.
  void on_odom(const Odometry &m) {
    const auto &tw = m.twist.twist;
    const bool finite = std::isfinite(tw.linear.x) && std::isfinite(tw.angular.z);
    const bool low = finite && std::abs(tw.linear.x) <= stop_linear_threshold_ &&
                     std::abs(tw.angular.z) <= stop_angular_threshold_;
    ax_safety::SampleContent content{{m.header.frame_id, m.child_frame_id}, {tw.linear.x, tw.angular.z}};
    // 시계는 한 번만 읽어 판정과 진단 카운터에 같은 값을 쓴다
    const double stamp_s = stamp_seconds(m.header.stamp);
    const double now_s = now().seconds();
    const auto r = odom_channel_.add(stamp_s, now_s, low, content, kFeedbackTtlS);
    odom_counts_.count(r, stamp_s, now_s);

    if (r == ax_safety::LowChannel::ACCEPTED) { odom_ = m; received_odom_ = steady(); got_odom_ = true; }
  }

  // joint_states 표본 하나.
  // - 구조: 이름·위치·속도 길이 일치, 이름 중복 없음. 아니면 근거 상실(저속 아님).
  // - 팔 있음: 필수 팔 관절이 모두 있고 위치·속도가 유한하며 속도가 모두 기준 이하일 때만 저속.
  //   필수가 아닌 관절(바퀴 등) 속도는 팔 기준에 넣지 않는다(차체 정지는 odom으로 확인).
  // - 팔 없음: 팔 조건은 해당 없음. 구조가 맞으면 저속으로 둔다(정지 판정은 odom 구간만 쓴다).
  // - 역할 미정·무효: 저속으로 보지 않는다.
  // 같은 stamp 비교는 이름 정렬 후 (이름, 위치, 속도) 대응을 원 값으로 직접 비교한다(배열 순서만 다르면 같음).
  void on_joints(const JointState &m) {
    const std::size_t n = m.name.size();
    bool valid = n > 0 && m.position.size() == n && m.velocity.size() == n;

    std::vector<std::size_t> order(n);
    for (std::size_t i = 0; i < n; ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&m](std::size_t a, std::size_t b) { return m.name[a] < m.name[b]; });
    for (std::size_t k = 1; valid && k < n; ++k) valid = m.name[order[k]] != m.name[order[k - 1]];

    ax_safety::SampleContent content;
    for (std::size_t k = 0; k < n; ++k) {
      const std::size_t i = order[k];
      content.names.push_back(m.name[i]);
      content.values.push_back(i < m.position.size() ? m.position[i] : std::numeric_limits<double>::quiet_NaN());
      content.values.push_back(i < m.velocity.size() ? m.velocity[i] : std::numeric_limits<double>::quiet_NaN());
    }

    bool low = false;
    if (valid && arm_presence_ == ax_safety::ArmPresence::ABSENT) {
      low = true;
    } else if (valid && arm_presence_ == ax_safety::ArmPresence::PRESENT) {
      low = true;
      for (const auto &required : stop_arm_joints_) {
        const auto it = std::find(m.name.begin(), m.name.end(), required);
        if (it == m.name.end()) { low = false; break; }
        const auto i = static_cast<std::size_t>(it - m.name.begin());
        if (!std::isfinite(m.position[i]) || !std::isfinite(m.velocity[i]) ||
            std::abs(m.velocity[i]) > stop_joint_threshold_) { low = false; break; }
      }
    }

    const double stamp_s = stamp_seconds(m.header.stamp);
    const double now_s = now().seconds();
    const auto r = joint_channel_.add(stamp_s, now_s, low, content, kFeedbackTtlS);
    joint_counts_.count(r, stamp_s, now_s);
    if (r == ax_safety::LowChannel::ACCEPTED) { joints_ = m; received_joints_ = steady(); got_joints_ = true; }
  }

  // 필수 팔 관절의 최대 |속도|(보고용). 팔 없음·역할 미정이면 0, 필수 관절이 없거나 비유한이면 NaN.
  float required_joint_speed() const {
    if (arm_presence_ != ax_safety::ArmPresence::PRESENT) return 0.0f;
    float out = 0.0f;
    for (const auto &required : stop_arm_joints_) {
      const auto it = std::find(joints_.name.begin(), joints_.name.end(), required);
      const auto i = static_cast<std::size_t>(it - joints_.name.begin());
      if (it == joints_.name.end() || i >= joints_.velocity.size()) return std::numeric_limits<float>::quiet_NaN();
      out = std::max(out, static_cast<float>(std::abs(joints_.velocity[i])));
    }
    return out;
  }

  // 피드백 표본 나이: 기존 0.3 s 상한, 미래 stamp는 신선으로 보지 않는다(수용 규칙과 같게)
  bool feedback_stamp_fresh(const builtin_interfaces::msg::Time &stamp) const {
    if (stamp.sec == 0 && stamp.nanosec == 0) return false;
    double age = (now() - rclcpp::Time(stamp, RCL_ROS_TIME)).seconds();
    return std::isfinite(age) && age >= 0.0 && age <= kFeedbackTtlS;
  }

  // ROS timestamp는 메시지 자체의 시간 유효성을 확인할 때만 사용한다.
  bool stamp_fresh(const builtin_interfaces::msg::Time &stamp, double timeout) const {
    // zero timestamp와 미래/오래된 ROS timestamp는 현재 상태로 인정하지 않는다.
    if (stamp.sec == 0 && stamp.nanosec == 0) return false;
    double age = (now() - rclcpp::Time(stamp, RCL_ROS_TIME)).seconds();
    return std::isfinite(age) && age >= 0 && age <= timeout;
  }

  // 주행 허용 여부를 판단하려면 odom과 joint_states가 모두 최신이고 서로 일관되어야 한다.
  bool state_fresh() const {
    // odom과 joint state가 모두 최근 수신되며 finite이고, joint name/position 배열이 맞아야 한다.
    // received_*는 새 stamp로 수용한 표본에서만 갱신되므로, 같은 stamp 재수신은 무수신 만료를 늦추지 못한다.
    if (!got_odom_ || !got_joints_ || elapsed(received_odom_) > kFeedbackTtlS ||
        elapsed(received_joints_) > kFeedbackTtlS ||
        !feedback_stamp_fresh(odom_.header.stamp) ||
        !feedback_stamp_fresh(joints_.header.stamp)) return false;
    if (!std::isfinite(odom_.twist.twist.linear.x) || !std::isfinite(odom_.twist.twist.angular.z)) return false;
    // 관절 이름·위치·속도 배열이 모두 같은 관절을 가리켜야 보호 판단에 사용한다.
    if (joints_.name.empty() || joints_.position.size() != joints_.name.size() ||
        joints_.velocity.size() != joints_.name.size()) return false;
    for (auto v : joints_.velocity) if (!std::isfinite(v)) return false;
    // NaN·무한대 위치는 실제 관절 상태로 해석할 수 없으므로 즉시 무효 처리한다.
    for (auto p : joints_.position) if (!std::isfinite(p)) return false;
    return true;
  }

  // A1에서 승인한 주행 자세와 현재 관절 피드백이 일치하는지 확인한다.
  bool travel_pose_valid() const {
    // 주행 중 허용된 관절 자세가 실제 joint state와 tolerance 안에서 일치하는지 확인한다.
    if (!pose_approved_ || !state_fresh() || joint_names_.empty() ||
        joint_names_.size()!=joint_positions_.size() || pose_tolerance_ <= 0) return false;
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = std::find(joints_.name.begin(), joints_.name.end(), joint_names_[i]);
      if (it == joints_.name.end() || !std::isfinite(joint_positions_[i])) return false;
      const auto actual = joints_.position[static_cast<std::size_t>(it-joints_.name.begin())];
      if (!std::isfinite(actual) || std::abs(actual-joint_positions_[i]) > pose_tolerance_) return false;
    }
    return true;
  }

  // 각 입력 메시지를 순수 게이트 함수가 사용할 최소한의 안전 입력으로 변환한다.
  ax_safety::GateInput gates() const {
    // ROS 메시지들을 순수 게이트 함수의 입력 계약으로 변환한다.
    // 하나라도 stale/invalid이면 GateInput의 기본 차단값을 유지한다.
    ax_safety::GateInput g;
    g.enabled = enable_;
    g.state_fresh = state_fresh();
    g.travel_pose_valid = travel_pose_valid();
    g.operational_permits_valid = permits_verified_;
    g.max_linear = max_linear_;
    g.max_angular = max_angular_;
    if (!got_safety_ || elapsed(received_safety_) > 0.5 ||
        !safety_.meta.valid || safety_.meta.revision == 0 ||
        safety_.meta.schema_version != 1 || safety_.meta.source_id.empty() ||
        safety_.meta.source_session_id.empty() ||
        safety_.robot_id != robot_ || !stamp_fresh(safety_.meta.header.stamp, 0.5)) return g;
    if (!std::isfinite(safety_.max_linear_speed) || safety_.max_linear_speed < 0 ||
        !std::isfinite(safety_.max_angular_speed) || safety_.max_angular_speed < 0) return g;
    g.safety_fresh = true;
    g.stop_required = safety_.level >= SafetyState::STOP_REQUIRED;
    g.allow_forward = safety_.allow_forward;
    g.allow_backward = safety_.allow_backward;
    g.allow_rotate = safety_.allow_rotate;
    g.max_linear = std::min(max_linear_, static_cast<double>(safety_.max_linear_speed));
    g.max_angular = std::min(max_angular_, static_cast<double>(safety_.max_angular_speed));
    return g;
  }

  // 게이트가 허용한 속도만 A2 제어기로 전달하고, 다음 가속도 제한 계산을 위해 기억한다.
  void command(double linear, double angular) {
    // A4가 허용한 최종 속도만 A2 controller topic으로 전달하고 마지막 출력도 기록한다.
    TwistStamped msg;
    msg.header.stamp = now();
    msg.twist.linear.x = linear;
    msg.twist.angular.z = angular;
    pub_cmd_->publish(msg);
    previous_linear_ = linear; previous_angular_ = angular;
    previous_sent_ = steady();
  }

  // 안전 조건이 하나라도 충족되지 않으면 사용할 공통 정지 명령이다.
  void stop() { command(0,0); }


  void on_command(const TwistStamped &msg) {
    // 입력 timestamp·불필요한 축·B4/A1/D2 조건을 순서대로 확인한 뒤 제한된 명령만 보낸다.
    got_cmd_ = true; received_cmd_ = steady();
    if (!stamp_fresh(msg.header.stamp, 0.2) ||
        msg.twist.linear.y!=0 || msg.twist.linear.z!=0 ||
        msg.twist.angular.x!=0 || msg.twist.angular.y!=0) { stop(); return; }
    const auto output = ax_safety::evaluate_drive_gate(gates(),msg.twist.linear.x,msg.twist.angular.z);
    if (!output.drive_allowed) { stop(); return; }

    // A2 설정의 가속 상한을 초과하는 양의 명령 증가를 제한한다.
    double dt = std::clamp(elapsed(previous_sent_), 0.0, 0.2);
    auto bounded_linear = std::clamp(output.linear, previous_linear_-max_linear_accel_*dt,
                                   previous_linear_+max_linear_accel_*dt);
    auto bounded_angular = std::clamp(output.angular, previous_angular_-max_angular_accel_*dt,
                                    previous_angular_+max_angular_accel_*dt);
    if (!std::isfinite(bounded_linear) || !std::isfinite(bounded_angular)) { stop(); return; }
    command(bounded_linear,bounded_angular);
  }

  void tick() {
    // 명령·안전·상태의 신선도를 주기적으로 다시 확인해 통신이 끊기면 정지한다.
    auto gate = gates();
    bool cmd_fresh = got_cmd_ && elapsed(received_cmd_) <= 0.2;

    // stop_required와 미검증 조건은 새 명령이 없어도 계속 정지 출력을 유지해야 한다.
    if (!cmd_fresh || !gate.enabled || !gate.safety_fresh || !gate.state_fresh ||
        !gate.travel_pose_valid || !gate.operational_permits_valid || gate.stop_required) stop();

    // ActuationState는 다른 파트가 현재 주행 가능 여부와 차단 원인을 확인하는 공통 결과다.
    ActuationState m;
    m.robot_id = robot_;
    m.meta.header.stamp = now();
    m.meta.schema_version=1;
    m.meta.source_id="a4_safety_gate";
    m.meta.source_session_id=session_;
    m.meta.revision=++revision_;
    // 시간 유효성: 계약상 판정 주체는 A 공용 감독기(/ax/fleet/time_validity, D-09)지만 아직 발행자가 없다.
    // gate는 최소한으로 /clock 무효(0)·역행(리셋)·D-09 정체(3 s 벽시계 동안 미진행)만 직접 본다.
    // RTF 범위(0.5~1.5) 판정과 TimeValidity 소비는 하지 않는다(감독기 구현 뒤 연결).
    const double t_now = now().seconds();
    bool clock_ok = std::isfinite(t_now) && t_now > 0.0;
    if (clock_ok && has_prev_tick_time_ && t_now < prev_tick_time_) {
      // 시계 역행은 리셋으로 보고 채널 상태를 모두 비운다(이후 작은 stamp 표본을 '늦은 표본'으로 버리지 않게).
      odom_channel_.reset_all();
      joint_channel_.reset_all();
      clock_ok = false;
    }
    if (clock_ok && (!has_prev_tick_time_ || t_now > prev_tick_time_)) last_clock_advance_ = steady();
    if (clock_ok) { prev_tick_time_ = t_now; has_prev_tick_time_ = true; }
    if (clock_ok && elapsed(last_clock_advance_) > kClockPauseS) clock_ok = false;   // D-09 PAUSED

    // 정지 근거 전제가 깨지면 채널 구간을 버린다. 회복 뒤 이전 유지 시간을 재사용하지 않는다.
    if (!clock_ok || !gate.safety_fresh || !gate.state_fresh) {
      odom_channel_.reset_interval();
      joint_channel_.reset_interval();
    }

    m.meta.valid = clock_ok && gate.state_fresh && gate.safety_fresh;
    m.meta.validity_reason = m.meta.valid ? "" : (!clock_ok ? "CLOCK_INVALID" : "STATE_OR_SAFETY_STALE");
    m.drive_allowed = clock_ok && gate.enabled && gate.safety_fresh && gate.state_fresh &&
                      gate.travel_pose_valid && gate.operational_permits_valid && !gate.stop_required;
    m.arm_allowed = arm_approved_ && clock_ok && gate.enabled && gate.state_fresh &&
                    gate.safety_fresh && !gate.stop_required; // 이후 A3 lease 검사는 팔 서버 책임

    m.state = ActuationState::UNKNOWN;
    // 진단 스냅샷용(판정에는 아래 블록 안의 값만 쓴다)
    double diag_held = -1.0;
    bool diag_low = false;
    if (clock_ok && gate.state_fresh) {
      m.linear_speed = static_cast<float>(std::abs(odom_.twist.twist.linear.x));
      m.angular_speed = static_cast<float>(std::abs(odom_.twist.twist.angular.z));
      // 필수 팔 관절만 보고한다(바퀴 등 제외). 팔 없음이면 0.
      m.max_joint_speed = required_joint_speed();

      // 역할이 확인된 경우에만 정지를 확인한다(미정·무효면 STOPPED 없음).
      const bool role_ok = arm_presence_ == ax_safety::ArmPresence::PRESENT ||
                           arm_presence_ == ax_safety::ArmPresence::ABSENT;
      const bool arm_absent = arm_presence_ == ax_safety::ArmPresence::ABSENT;
      // 최신 수용 표본이 저속이어야 정지 진행으로 본다(팔 없음이면 odom만).
      const bool low = role_ok && odom_channel_.latest_low && (arm_absent || joint_channel_.latest_low);
      // D-06 유지 시간: 각 채널이 관측한 연속 저속 구간(팔 있음이면 두 채널 교집합) 길이, 끝은 현재 시각 이하.
      // wall tick 반복·같은 stamp 재수신·/clock 진행·미래 표본만으로는 늘지 않는다.
      const double held = arm_absent ? ax_safety::low_span(odom_channel_, t_now)
                                     : ax_safety::common_low_span(odom_channel_, joint_channel_, t_now);
      diag_held = held;
      diag_low = low;
      if (!low) m.state = m.drive_allowed ? ActuationState::RUNNING : ActuationState::STOPPING;
      else if (gate.safety_fresh && held >= stop_hold_s_) {
        // D-06은 정지 명령과 독립적으로 최신 odom·관절 피드백으로 확인한다.
        // 주행 잠금(drive_allowed=false)과 무관하게 정지 확인은 할 수 있고, STOPPED가 주행 허가를 대신하지 않는다.
        m.state = ActuationState::STOPPED;
      } else m.state=ActuationState::STOPPING;
    }

    // ActuationState는 왜 주행이 막혔는지 후속 A3/D4 관측자가 확인할 수 있도록 원인을 남긴다.
    if (!clock_ok) m.block_reasons.push_back("TIME_INVALID");   // 계약 D-12 구동 차단 사유
    if (!gate.safety_fresh) m.block_reasons.push_back("SAFETY_STOP");
    if (!gate.state_fresh) m.block_reasons.push_back("STATE_STALE");
    if (!gate.travel_pose_valid) m.block_reasons.push_back("TRAVEL_POSE_UNVERIFIED");
    if (!gate.operational_permits_valid) m.block_reasons.push_back("HANDOFF_LOCK");
    if (!gate.enabled) m.block_reasons.push_back("SIMULATION_DRIVE_DISABLED");
    pub_state_->publish(m);

    if (diagnostics_) diagnose(m, gate, clock_ok, t_now, diag_held, diag_low);
  }

  // 진단(기본 비활성, 판정 불변): STOPPED 진입·이탈 때 판정 근거 스냅샷, 1 s(벽시계)마다 카운터.
  // 판정이 끝난 뒤 이미 계산한 값만 기록한다. 무수신 나이(age)는 진단 시점에 따로 읽은 steady 값이다.
  // 출력은 노드 로그 한 줄 JSON(A4_STOP_DIAG). 공개 토픽·메시지를 추가하지 않는다.
  void diagnose(const ActuationState &m, const ax_safety::GateInput &gate, bool clock_ok, double t_now,
                double held, bool low) {
    const bool stopped = m.state == ActuationState::STOPPED;
    const bool transition = stopped != last_diag_stopped_;
    const bool periodic = elapsed(last_diag_counters_) >= 1.0;
    if (!transition && !periodic) return;

    const auto channel = [](const ax_safety::LowChannel &ch, const ax_safety::ChannelCounters &k) {
      std::ostringstream o;
      o << std::setprecision(12) << "{\"latest\":" << (ch.has_latest ? ch.latest_stamp : -1.0)
        << ",\"latest_low\":" << (ch.latest_low ? "true" : "false")
        << ",\"has_interval\":" << (ch.has_interval ? "true" : "false")
        << ",\"start\":" << (ch.has_interval ? ch.start : -1.0) << ",\"end\":" << (ch.has_interval ? ch.end : -1.0)
        << ",\"accepted\":" << k.result[0] << ",\"duplicate\":" << k.result[1] << ",\"conflict\":" << k.result[2]
        << ",\"late\":" << k.result[3] << ",\"rejected\":" << k.result[4]
        << ",\"rej_clock\":" << k.reject[0] << ",\"rej_stamp\":" << k.reject[1] << ",\"rej_future\":" << k.reject[2] << "}";
      return o.str();
    };
    const bool absent = arm_presence_ == ax_safety::ArmPresence::ABSENT;
    const char *presence = arm_presence_ == ax_safety::ArmPresence::PRESENT ? "present" :
                           absent ? "absent" : arm_presence_ == ax_safety::ArmPresence::UNCONFIRMED ? "unconfirmed" : "invalid";
    std::ostringstream o;
    o << std::setprecision(12) << "A4_STOP_DIAG {\"kind\":\"" << (transition ? (stopped ? "enter_stopped" : "leave_stopped") : "counters")
      << "\",\"now\":" << t_now << ",\"state\":" << static_cast<int>(m.state) << ",\"valid\":" << (m.meta.valid ? "true" : "false")
      << ",\"clock_ok\":" << (clock_ok ? "true" : "false") << ",\"state_fresh\":" << (gate.state_fresh ? "true" : "false")
      << ",\"safety_fresh\":" << (gate.safety_fresh ? "true" : "false") << ",\"low\":" << (low ? "true" : "false")
      << ",\"held\":" << held << ",\"hold_s\":" << stop_hold_s_ << ",\"presence\":\"" << presence
      << "\",\"joint_used_for_hold\":" << (absent ? "false" : "true") << ",\"required\":" << stop_arm_joints_.size()
      << ",\"odom\":" << channel(odom_channel_, odom_counts_) << ",\"joint\":" << channel(joint_channel_, joint_counts_)
      << ",\"age_odom\":" << (got_odom_ ? elapsed(received_odom_) : -1.0)
      << ",\"age_joint\":" << (got_joints_ ? elapsed(received_joints_) : -1.0)
      << ",\"age_safety\":" << (got_safety_ ? elapsed(received_safety_) : -1.0) << "}";
    RCLCPP_INFO(get_logger(), "%s", o.str().c_str());

    if (transition) last_diag_stopped_ = stopped;
    if (periodic) last_diag_counters_ = steady();
  }

  // 파라미터와 마지막으로 받은 안전·구동 피드백을 보관한다.
  std::string robot_,session_;
  bool enable_{false}, permits_verified_{false}, pose_approved_{false}, arm_approved_{false};
  bool got_safety_{false},got_odom_{false},got_joints_{false},got_cmd_{false};
  double max_linear_, max_angular_,max_linear_accel_,max_angular_accel_,pose_tolerance_;
  double stop_linear_threshold_{0.01}, stop_angular_threshold_{0.02},
      stop_joint_threshold_{0.01}, stop_hold_s_{0.5};
  double previous_linear_{0},previous_angular_{0};
  uint64_t revision_{0};
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  SafetyState safety_;
  Odometry odom_;
  JointState joints_;
  // D-06 표본 기반 정지 근거(채널별 연속 저속 구간), 기준 관절 집합, tick 시계 감시
  ax_safety::LowChannel odom_channel_, joint_channel_;
  // 진단 전용 카운터·상태(판정에 쓰지 않음)
  ax_safety::ChannelCounters odom_counts_, joint_counts_;
  bool diagnostics_{false}, last_diag_stopped_{false};
  std::chrono::steady_clock::time_point last_diag_counters_{steady()};
  std::vector<std::string> stop_arm_joints_;
  ax_safety::ArmPresence arm_presence_{ax_safety::ArmPresence::UNCONFIRMED};
  bool has_prev_tick_time_{false};
  double prev_tick_time_{0.0};
  std::chrono::steady_clock::time_point last_clock_advance_{steady()};
  std::chrono::steady_clock::time_point received_safety_{},received_odom_{},received_joints_{},
      received_cmd_{},previous_sent_{steady()};

  // 입력과 상태를 연결하는 ROS publisher/subscription/timer 핸들이다.
  rclcpp::Publisher<TwistStamped>::SharedPtr pub_cmd_;
  rclcpp::Publisher<ActuationState>::SharedPtr pub_state_;
  rclcpp::Subscription<SafetyState>::SharedPtr sub_safety_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<JointState>::SharedPtr sub_joints_;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char **argv) {
  // A4 게이트 하나만 ROS executor에 올려 입력·출력·상태 발행을 처리한다.
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<A4SafetyGate>());
  rclcpp::shutdown(); return 0;
}
