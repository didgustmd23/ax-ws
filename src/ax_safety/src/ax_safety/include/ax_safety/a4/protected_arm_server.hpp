#pragma once
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
  using Action = ax_interfaces::action::ProtectedArmTrajectory;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  using Lease = ax_interfaces::srv::CheckCommandLease;
  using Controller = control_msgs::action::FollowJointTrajectory;
  using ControllerHandle = rclcpp_action::ClientGoalHandle<Controller>;
  ProtectedArmServer();
private:
  rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID &,
                                     std::shared_ptr<const Action::Goal> goal);
  rclcpp_action::CancelResponse on_cancel(std::shared_ptr<Handle> handle);
  void on_accept(std::shared_ptr<Handle> handle);
  void on_safety(ax_interfaces::msg::SafetyState::SharedPtr msg);
  void on_actuation(ax_interfaces::msg::ActuationState::SharedPtr msg);
  void tick();
  bool verify_trajectory(const Action::Goal &goal, std::string &why) const;
  bool safety_ready() const;
  void ask_lease(uint8_t phase);
  void start_controller();
  void finish(bool success, const std::string& reason, bool canceled=false);
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
