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
    session_ = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    const auto base = "/ax/" + robot_;
    // nav 입력은 A4가 받고, 출력은 A2 diff_drive_controller로 보낸다.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    pub_cmd_ = create_publisher<TwistStamped>(base + "/diff_drive_controller/cmd_vel", qos);
    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    pub_state_ = create_publisher<ActuationState>(base + "/safety/actuation_state", state_qos);
    sub_safety_ = create_subscription<SafetyState>(base + "/safety/state", qos,
        [this](SafetyState::SharedPtr m) { safety_ = *m; received_safety_ = steady(); got_safety_ = true; });
    sub_odom_ = create_subscription<Odometry>(base + "/diff_drive_controller/odom", qos,
        [this](Odometry::SharedPtr m) { odom_ = *m; received_odom_ = steady(); got_odom_ = true; });
    sub_joints_ = create_subscription<JointState>(base + "/joint_states", qos,
        [this](JointState::SharedPtr m) { joints_ = *m; received_joints_ = steady(); got_joints_ = true; });
    sub_cmd_ = create_subscription<TwistStamped>(base + "/nav/cmd_vel", qos,
        [this](TwistStamped::SharedPtr m) { on_command(*m); });
    timer_ = create_wall_timer(50ms, [this]() { tick(); });
    RCLCPP_WARN(get_logger(), "A4 gate started locked. A1 pose envelope and D2/D3 permits must be explicitly verified.");
  }
private:
  // steady clock은 시스템 시간이 바뀌어도 통신 timeout을 일정하게 계산하게 한다.
  static std::chrono::steady_clock::time_point steady() { return std::chrono::steady_clock::now(); }


  static double elapsed(std::chrono::steady_clock::time_point t) {
    return std::chrono::duration<double>(steady()-t).count();
  }
  bool stamp_fresh(const builtin_interfaces::msg::Time &stamp, double timeout) const {
    // zero timestamp와 미래/오래된 ROS timestamp는 현재 상태로 인정하지 않는다.
    if (stamp.sec == 0 && stamp.nanosec == 0) return false;
    double age = (now() - rclcpp::Time(stamp, RCL_ROS_TIME)).seconds();
    return std::isfinite(age) && age >= 0 && age <= timeout;
  }


  bool state_fresh() const {
    // odom과 joint state가 모두 최근 수신되며 finite이고, joint name/position 배열이 맞아야 한다.
    if (!got_odom_ || !got_joints_ || elapsed(received_odom_) > 0.3 ||
        elapsed(received_joints_) > 0.3 ||
        !stamp_fresh(odom_.header.stamp, 0.3) ||
        !stamp_fresh(joints_.header.stamp, 0.3)) return false;
    if (!std::isfinite(odom_.twist.twist.linear.x) || !std::isfinite(odom_.twist.twist.angular.z)) return false;
    for (auto v : joints_.velocity) if (!std::isfinite(v)) return false;
    return joints_.position.size() == joints_.name.size();
  }


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
        safety_.meta.schema_version != 1 || safety_.meta.source_session_id.empty() ||
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
    ActuationState m;
    m.robot_id = robot_;
    m.meta.header.stamp = now();
    m.meta.schema_version=1;
    m.meta.source_id="a4_safety_gate";
    m.meta.source_session_id=session_;
    m.meta.revision=++revision_;
    m.meta.valid=gate.state_fresh && gate.safety_fresh;
    m.meta.validity_reason=m.meta.valid ? "" : "STATE_OR_SAFETY_STALE";
    m.drive_allowed = gate.enabled && gate.safety_fresh && gate.state_fresh &&
                      gate.travel_pose_valid && gate.operational_permits_valid && !gate.stop_required;
    m.arm_allowed = arm_approved_ && gate.enabled && gate.state_fresh &&
                    gate.safety_fresh && !gate.stop_required; // 이후 A3 lease 검사는 팔 서버 책임
    m.state = ActuationState::UNKNOWN;
    if (gate.state_fresh) {
      m.linear_speed = static_cast<float>(std::abs(odom_.twist.twist.linear.x));
      m.angular_speed = static_cast<float>(std::abs(odom_.twist.twist.angular.z));
      for (auto x: joints_.velocity) m.max_joint_speed=std::max(m.max_joint_speed,static_cast<float>(std::abs(x)));
      bool low = m.linear_speed<=0.01f && m.angular_speed<=0.02f && m.max_joint_speed<=0.01f;
      if (low) {
        if (!stable_stop_since_) stable_stop_since_ = steady();
      } else stable_stop_since_.reset();
      if (!low) m.state = m.drive_allowed ? ActuationState::RUNNING : ActuationState::STOPPING;
      else if (stable_stop_since_ && elapsed(*stable_stop_since_)>=0.5) {
        // 정지 신호·차체 truth의 독립 검증(W2) 전에는 STOPPED를 선언하지 않는다.
        m.state = ActuationState::UNKNOWN;
      } else m.state=ActuationState::STOPPING;
    } else stable_stop_since_.reset();
    // ActuationState는 왜 주행이 막혔는지 후속 A3/D4 관측자가 확인할 수 있도록 원인을 남긴다.
    if (!gate.safety_fresh) m.block_reasons.push_back("SAFETY_STOP");
    if (!gate.state_fresh) m.block_reasons.push_back("STATE_STALE");
    if (!gate.travel_pose_valid) m.block_reasons.push_back("TRAVEL_POSE_UNVERIFIED");
    if (!gate.operational_permits_valid) m.block_reasons.push_back("HANDOFF_LOCK");
    if (!gate.enabled) m.block_reasons.push_back("SIMULATION_DRIVE_DISABLED");
    pub_state_->publish(m);
  }


  std::string robot_,session_;
  bool enable_{false}, permits_verified_{false}, pose_approved_{false}, arm_approved_{false};
  bool got_safety_{false},got_odom_{false},got_joints_{false},got_cmd_{false};
  double max_linear_, max_angular_,max_linear_accel_,max_angular_accel_,pose_tolerance_;
  double previous_linear_{0},previous_angular_{0};
  uint64_t revision_{0};
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  SafetyState safety_;
  Odometry odom_;
  JointState joints_;
  std::optional<std::chrono::steady_clock::time_point> stable_stop_since_;
  std::chrono::steady_clock::time_point received_safety_{},received_odom_{},received_joints_{},
      received_cmd_{},previous_sent_{steady()};
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
