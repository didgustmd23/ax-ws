#pragma once
// A4 보호 팔 서버의 선언부이다.
// C4/A3가 제출한 ProtectedArmTrajectory를 검증하고, A3 lease와 A4 안전 상태가
// 유효할 때만 A2 FollowJointTrajectory action으로 전달한다.
#include <ax_interfaces/action/protected_arm_trajectory.hpp>


#include <ax_interfaces/srv/check_command_lease.hpp>
#include <ax_interfaces/msg/safety_state.hpp>
#include <ax_interfaces/msg/actuation_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <vector>

class ProtectedArmServer : public rclcpp::Node {
public:
  // ROS action/service/topic을 만들고 기본적으로 비활성화된 보호 경로를 준비한다.


  using Action = ax_interfaces::action::ProtectedArmTrajectory;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  using Lease = ax_interfaces::srv::CheckCommandLease;
  using Controller = control_msgs::action::FollowJointTrajectory;
  using ControllerHandle = rclcpp_action::ClientGoalHandle<Controller>;
  ProtectedArmServer();
private:
  // 각 콜백은 action 요청의 수락·취소·실행과 안전 상태 갱신을 담당한다.


  rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID &,
                                     std::shared_ptr<const Action::Goal> goal);
  rclcpp_action::CancelResponse on_cancel(std::shared_ptr<Handle> handle);
  void on_accept(std::shared_ptr<Handle> handle);
  void on_safety(ax_interfaces::msg::SafetyState::SharedPtr msg);
  void on_actuation(ax_interfaces::msg::ActuationState::SharedPtr msg);
  void tick();
  // 목표의 authority, 관절 배열, 시간 순서, 위치·속도·가속도 한계를 검사한다.
  // 실패 이유는 action 결과와 로그에 남겨 후속 담당자가 원인을 구분할 수 있게 한다.


  bool verify_trajectory(const Action::Goal &goal, std::string &why) const;
  // SafetyState와 ActuationState가 같은 로봇·schema·session·시간 창을 만족하는지 확인한다.
  bool safety_ready() const;
  // A3 CheckCommandLease에 ACCEPT/CONTINUE 권한 확인을 요청한다.
  void ask_lease(uint8_t phase);
  // lease가 허용된 뒤에만 A2 팔 action을 시작한다.
  void start_controller();
  // 성공·취소·실패를 action에 기록하고 현재 실행 상태를 정리한다.
  void finish(bool success, const std::string& reason, bool canceled=false);
  // 동일한 물리 목표를 재전송해도 추적할 수 있도록 정규화된 목표의 digest를 만든다.
  std::string digest_goal(const Action::Goal &g) const;


  std::string robot_, action_path_, session_;
  bool enabled_{false}, pending_goal_{false}, cancel_requested_{false}, lease_pending_{false};
  double service_timeout_{0.45}, renewal_period_{0.5};
  std::vector<std::string> joint_names_;
  std::vector<double> mins_, maxs_, max_velocities_, max_accelerations_;
  ax_interfaces::msg::SafetyState safety_;
  ax_interfaces::msg::ActuationState actuation_;
  bool got_safety_{false}, got_actuation_{false};
  std::chrono::steady_clock::time_point safety_received_{}, actuation_received_{}, request_start_{},
      continue_start_{}, last_renew_{}, permission_deadline_{};
  // lease 응답과 A2 action future를 비동기로 기다리기 위한 실행 단계이다.
  enum class Phase {IDLE, WAIT_ACCEPT, WAIT_CONTROLLER, EXECUTING};
  Phase phase_{Phase::IDLE};
  std::string command_digest_, command_id_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp_action::Client<Controller>::SharedPtr controller_;
  rclcpp::Client<Lease>::SharedPtr lease_;
  rclcpp::Subscription<ax_interfaces::msg::SafetyState>::SharedPtr safety_sub_;
  rclcpp::Subscription<ax_interfaces::msg::ActuationState>::SharedPtr actuation_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<Handle> active_;
  std::shared_ptr<const Action::Goal> current_;
  ControllerHandle::SharedPtr controller_goal_;
  rclcpp::Client<Lease>::SharedFuture lease_future_;
  std::shared_future<ControllerHandle::SharedPtr> controller_future_;
  std::shared_future<ControllerHandle::WrappedResult> result_future_;
};
