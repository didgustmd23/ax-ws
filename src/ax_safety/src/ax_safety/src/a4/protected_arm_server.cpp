// A4 보호 팔 Action: 검증된 범위 검사 + A3 명령 허가 확인 후 A2 Action 전달.
// 실제 A3가 없거나 상태가 불명확하면 실행하지 않는다.


#include "ax_safety/a4/protected_arm_server.hpp"
#include <ax_interfaces/msg/result_status.hpp>
#include <ax_interfaces/action/execute_local_task.hpp>
#include <openssl/sha.h>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <random>
#include <sstream>
#include <algorithm>
#include <cctype>

using namespace std::chrono_literals;
using namespace std::placeholders;
namespace {
using Clock = std::chrono::steady_clock;
// ROS callback 안에서 블로킹하지 않고 lease·controller 응답의 경과 시간을 측정한다.


double seconds(Clock::time_point t) {
  return std::chrono::duration<double>(Clock::now() - t).count();
}
// 매 lease 요청을 구분하기 위한 임시 request id를 만든다.


std::string unique_id() {
  std::random_device rd; std::mt19937_64 rng(rd());
  std::ostringstream out; out << std::hex << rng() << rng(); return out.str();
}
// C authority 목표에 필요한 16바이트 hex goal id 형식만 허용한다.


bool is_goal_id(const std::string &value) {
  if (value.size() != 32) return false;
  return std::all_of(value.begin(), value.end(), [](unsigned char c) {
    return std::isdigit(c) || (c >= 'a' && c <= 'f');
  });
}
}

ProtectedArmServer::ProtectedArmServer() : Node("a4_protected_arm_server") {
  // 기본은 비활성화이며, 관절 한계와 A3 lease 경로가 검증된 시험에서만 명시적으로 활성화한다.


  robot_ = declare_parameter<std::string>("robot_id", "robot_1");
  enabled_ = declare_parameter<bool>("enable_simulation_arm", false);
  joint_names_ = declare_parameter<std::vector<std::string>>(
      "arm_joint_names", std::vector<std::string>{});
  mins_ = declare_parameter<std::vector<double>>(
      "arm_position_min_rad", std::vector<double>{});
  maxs_ = declare_parameter<std::vector<double>>(
      "arm_position_max_rad", std::vector<double>{});
  max_velocities_ = declare_parameter<std::vector<double>>(
      "arm_max_velocity_rad_s", std::vector<double>{});
  max_accelerations_ = declare_parameter<std::vector<double>>(
      "arm_max_acceleration_rad_s2", std::vector<double>{});
  const auto base = "/ax/" + robot_;
  // A2 controller action, A3 lease service, A4 상태 토픽은 같은 robot namespace를 사용한다.


  action_path_ = declare_parameter<std::string>("controller_action",base+"/arm_controller/follow_joint_trajectory");
  controller_ = rclcpp_action::create_client<Controller>(this,action_path_);
  lease_ = create_client<Lease>(base+"/executor/lease/check_command");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto state_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
  safety_sub_ = create_subscription<ax_interfaces::msg::SafetyState>(base+"/safety/state",qos,
      std::bind(&ProtectedArmServer::on_safety,this,_1));
  actuation_sub_ = create_subscription<ax_interfaces::msg::ActuationState>(base+"/safety/actuation_state",state_qos,
      std::bind(&ProtectedArmServer::on_actuation,this,_1));
  server_ = rclcpp_action::create_server<Action>(this,base+"/arm/execute_trajectory",
      std::bind(&ProtectedArmServer::on_goal,this,_1,_2),
      std::bind(&ProtectedArmServer::on_cancel,this,_1),
      std::bind(&ProtectedArmServer::on_accept,this,_1));
  timer_ = create_wall_timer(20ms,std::bind(&ProtectedArmServer::tick,this));
  RCLCPP_WARN(get_logger(),"ProtectedArmTrajectory requires validated joint limits and A3 lease. Default disabled.");
}

void ProtectedArmServer::on_safety(ax_interfaces::msg::SafetyState::SharedPtr m) {
  // 수신 시각은 ROS timestamp와 별개로 기록해 통신 단절 timeout도 감지한다.


  safety_=*m; safety_received_=Clock::now(); got_safety_=true;
}


void ProtectedArmServer::on_actuation(ax_interfaces::msg::ActuationState::SharedPtr m) {
  actuation_=*m; actuation_received_=Clock::now(); got_actuation_=true;
}
bool ProtectedArmServer::safety_ready() const {
  // 안전·구동 상태는 메시지 내용뿐 아니라 robot id, schema, revision, session,
  // 수신 경과 시간과 ROS timestamp를 모두 통과해야 최신 권한으로 인정한다.


  if (!enabled_ || !got_safety_ || !got_actuation_ || seconds(safety_received_) > 0.5 ||
      seconds(actuation_received_) > 0.3 || !actuation_.meta.valid ||
      actuation_.meta.schema_version != 1 || !actuation_.meta.revision ||
      actuation_.meta.source_session_id.empty() || actuation_.robot_id!=robot_ ||
      !actuation_.arm_allowed ||
      !safety_.meta.valid || safety_.meta.schema_version!=1 || !safety_.meta.revision ||
      safety_.meta.source_session_id.empty() || safety_.robot_id!=robot_ ||
      safety_.level >= ax_interfaces::msg::SafetyState::STOP_REQUIRED) return false;
  const auto stamp = safety_.meta.header.stamp;
  const auto act_stamp = actuation_.meta.header.stamp;
  if ((stamp.sec==0 && stamp.nanosec==0) ||
      (act_stamp.sec==0 && act_stamp.nanosec==0)) return false;
  double age = (now()-rclcpp::Time(stamp,RCL_ROS_TIME)).seconds();
  double act_age = (now()-rclcpp::Time(act_stamp,RCL_ROS_TIME)).seconds();
  return std::isfinite(age) && age>=0 && age<=0.5 &&
         std::isfinite(act_age) && act_age>=0 && act_age<=0.3;
}
bool ProtectedArmServer::verify_trajectory(const Action::Goal &g, std::string &why) const {
  // 입력 vector 길이와 관절 계약을 먼저 맞춘 뒤 authority별 식별자 규칙을 검사한다.


  const std::size_t n = joint_names_.size();
  if (n==0 || n!=mins_.size() || n!=maxs_.size() || n!=max_velocities_.size() ||
      n!=max_accelerations_.size()) { why="ARM_LIMITS_NOT_VERIFIED"; return false; }
  if (g.request_id.empty() || g.execution_id.empty() ||
      (g.authority!=Action::Goal::AUTH_C_MANIPULATION &&
       g.authority!=Action::Goal::AUTH_A3_LOCAL_TASK)) {why="AUTHORITY_INVALID";return false;}
  if (g.authority==Action::Goal::AUTH_C_MANIPULATION &&
      (!is_goal_id(g.goal_id) || g.lease_generation==0)) {why="GOAL_MISMATCH";return false;}
  if (g.authority==Action::Goal::AUTH_A3_LOCAL_TASK &&
      (!g.goal_id.empty() || g.lease_generation!=0)) {why="AUTHORITY_INVALID";return false;}
  const auto &trajectory = g.trajectory;
  // A2 controller가 해석할 joint 이름·points·시간 순서를 고정해 비정상 trajectory를 막는다.


  if (trajectory.joint_names!=joint_names_ || trajectory.points.empty()) {
    why="JOINT_NAMES_OR_POINTS_INVALID";return false;
  }
  double previous_t=-1;
  for (std::size_t k=0;k<trajectory.points.size();++k) {
    const auto &p=trajectory.points[k];
    double t=p.time_from_start.sec+p.time_from_start.nanosec*1e-9;
    if (!std::isfinite(t) || t<=previous_t || t<=0 || p.positions.size()!=n ||
        (!p.velocities.empty() && p.velocities.size()!=n) ||
        (!p.accelerations.empty() && p.accelerations.size()!=n)) {
      why="TRAJECTORY_SHAPE_INVALID";return false;
    }
    for (std::size_t i=0;i<n;++i) {
      // 각 점의 위치/속도/가속도와 이전 점 사이의 추정 속도를 모두 한계 안에서 확인한다.


      if (!std::isfinite(p.positions[i]) || !std::isfinite(mins_[i]) ||
          !std::isfinite(maxs_[i]) || !std::isfinite(max_velocities_[i]) ||
          !std::isfinite(max_accelerations_[i]) ||
          mins_[i]>=maxs_[i] || max_velocities_[i]<=0 || max_accelerations_[i]<=0 ||
          p.positions[i]<mins_[i] || p.positions[i]>maxs_[i]) {
        why="JOINT_POSITION_LIMIT";return false;
      }
      if (!p.velocities.empty() && (!std::isfinite(p.velocities[i]) ||
                                    std::abs(p.velocities[i])>max_velocities_[i])) {
        why="JOINT_VELOCITY_LIMIT";return false;
      }
      if (!p.accelerations.empty() && (!std::isfinite(p.accelerations[i]) ||
                                       std::abs(p.accelerations[i])>max_accelerations_[i])) {
        why="JOINT_ACCELERATION_LIMIT";return false;
      }
      if (k>0) {
        const auto &q=trajectory.points[k-1];
        double estimated_velocity=(p.positions[i]-q.positions[i])/(t-previous_t);
        if (std::abs(estimated_velocity)>max_velocities_[i]) {
          why="JOINT_SEGMENT_SPEED_LIMIT";return false;
        }
      }
    }
    previous_t=t;
  }
  return true;
}
std::string ProtectedArmServer::digest_goal(const Action::Goal &g) const {
  // 동일 물리 명령을 동일 digest로 표현. 계약과 동일한 정규화 방식인지 A3와 최종 대조 필요.
  // 동일 목표 재전송을 식별할 수 있도록 authority와 모든 trajectory 값을 고정 순서로 직렬화한다.


  std::ostringstream out; out << std::setprecision(17);
  out << static_cast<int>(g.authority)<<'|'<<g.execution_id<<'|'<<g.goal_id<<'|'<<g.lease_generation<<'|';
  for (const auto &n:g.trajectory.joint_names) out << n << ',';
  for (const auto &p:g.trajectory.points) {
    out << p.time_from_start.sec << ':' << p.time_from_start.nanosec << '|';
    for (double v:p.positions) out << v << ',';
    out << '|'; for(double v:p.velocities) out << v << ',';
    out << '|'; for(double v:p.accelerations) out << v << ',';
  }
  auto data=out.str(); unsigned char md[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(data.data()),data.size(),md);
  std::ostringstream hex;
  for(unsigned char b:md) hex<<std::hex<<std::setw(2)<<std::setfill('0')<<static_cast<int>(b);
  return hex.str();
}
rclcpp_action::GoalResponse ProtectedArmServer::on_goal(
    const rclcpp_action::GoalUUID&,std::shared_ptr<const Action::Goal> g) {
  // 실행 중 목표·안전 상태·trajectory 중 하나라도 불충족이면 action 자체를 수락하지 않는다.


  std::string why;
  if (pending_goal_ || active_ || !safety_ready() || !verify_trajectory(*g,why)) {
    RCLCPP_WARN(get_logger(),"ProtectedArm rejected: %s",why.empty()?"SAFETY_OR_BUSY":why.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  pending_goal_=true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse ProtectedArmServer::on_cancel(std::shared_ptr<Handle> h) {
  // 현재 실행 중인 goal만 취소할 수 있게 해 다른 요청이 실행 상태를 바꾸지 못하게 한다.


  if (h==active_) {cancel_requested_=true;return rclcpp_action::CancelResponse::ACCEPT;}
  return rclcpp_action::CancelResponse::REJECT;
}
void ProtectedArmServer::on_accept(std::shared_ptr<Handle> h) {
  // 수락 직후에는 A2를 호출하지 않고 command digest를 만든 뒤 A3 ACCEPT lease부터 확인한다.


  pending_goal_=false; active_=h; current_=h->get_goal(); cancel_requested_=false;
  command_id_=current_->request_id;
  command_digest_=digest_goal(*current_);
  ask_lease(Lease::Request::ACCEPT);
  if (active_) phase_=Phase::WAIT_ACCEPT;
}
void ProtectedArmServer::ask_lease(uint8_t phase) {
  // A3 서버가 없거나 준비되지 않았으면 권한을 추측하지 않고 즉시 실패한다.


  if (!active_ || !lease_->service_is_ready()) { finish(false,"A3_LEASE_SERVER_UNAVAILABLE");return; }
  auto req=std::make_shared<Lease::Request>();
  req->request_id=unique_id(); req->robot_id=robot_; req->authority=current_->authority;
  req->execution_id=current_->execution_id; req->goal_id=current_->goal_id;
  req->lease_generation=current_->lease_generation;
  req->local_task_kind=current_->authority==Action::Goal::AUTH_A3_LOCAL_TASK ?
      ax_interfaces::action::ExecuteLocalTask::Goal::RETRACT_ARM : 0;
  req->phase=phase; req->command_source=Lease::Request::SOURCE_A4_ARM;
  req->command_id=command_id_; req->command_digest=command_digest_;
  request_start_=Clock::now();
  // 이후 tick이 service_timeout_과 lease valid_for_s를 비교할 수 있도록 요청 시각을 고정한다.


  lease_future_=lease_->async_send_request(req);
  lease_pending_=true;
}
void ProtectedArmServer::start_controller() {
  // lease가 허용된 경우에만 A2 FollowJointTrajectory action을 비동기로 시작한다.


  if (!controller_->action_server_is_ready()) {finish(false,"A2_CONTROLLER_UNAVAILABLE");return;}
  Controller::Goal goal;
  goal.trajectory=current_->trajectory;
  controller_future_=controller_->async_send_goal(goal);
  phase_=Phase::WAIT_CONTROLLER;
}
void ProtectedArmServer::finish(bool success,const std::string& reason,bool canceled) {
  // 모든 종료 경로는 controller 취소, action 결과 기록, 내부 포인터·phase 정리를 한 곳에서 수행한다.


  if (!active_) {phase_=Phase::IDLE; lease_pending_=false;return;}
  if (controller_goal_ && !success) controller_->async_cancel_goal(controller_goal_);
  auto result=std::make_shared<Action::Result>();
  result->status.outcome=success?ax_interfaces::msg::ResultStatus::SUCCEEDED:
      (canceled?ax_interfaces::msg::ResultStatus::CANCELED:ax_interfaces::msg::ResultStatus::FAILED);
  result->status.error_code=success?"":reason;
  result->status.detail=reason;
  if (success) active_->succeed(result);
  else if (canceled) active_->canceled(result);
  else active_->abort(result);
  active_.reset(); current_.reset(); controller_goal_.reset();
  phase_=Phase::IDLE; lease_pending_=false; cancel_requested_=false;
}
void ProtectedArmServer::tick() {
  // 20 ms timer에서 취소·안전 상태·lease·A2 future를 짧게 확인한다.
  // wait_for(0ms)를 사용해 ROS action callback을 블로킹하지 않는다.


  if (!active_) return;
  if (cancel_requested_) {finish(false,"CANCELED",true);return;}
  if (!safety_ready()) {finish(false,"SAFETY_STOP");return;}
  if (phase_==Phase::WAIT_ACCEPT) {
    if (!lease_pending_) return;
    // ACCEPT 응답이 늦으면 권한을 기다린 채 실행하지 않고 timeout으로 종료한다.


    if (seconds(request_start_)>service_timeout_) {finish(false,"A3_LEASE_TIMEOUT");return;}
    if (lease_future_.valid() && lease_future_.wait_for(0ms)==std::future_status::ready) {
      auto r=lease_future_.get(); lease_pending_=false;
      double age=seconds(request_start_);
      if (!r->allowed || r->valid_for_s<=0 || age>=r->valid_for_s || r->accept_record_id.empty()) {
        finish(false,r->reason_code.empty()?"LEASE_EXPIRED":r->reason_code);return;
      }
      permission_deadline_=request_start_+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(r->valid_for_s));
      last_renew_=Clock::now(); start_controller();
    }
    return;
  }
  if (phase_==Phase::WAIT_CONTROLLER) {
    // A2 action이 permission_deadline을 넘으면 controller가 늦게 시작해도 실행하지 않는다.


    if (Clock::now()>=permission_deadline_) {finish(false,"LEASE_EXPIRED");return;}
    if (controller_future_.valid() && controller_future_.wait_for(0ms)==std::future_status::ready) {
      controller_goal_=controller_future_.get();
      if (!controller_goal_) {finish(false,"CONTROLLER_GOAL_REJECTED");return;}
      // async_get_result resolves via future. Poll instead of blocking action callback.
      result_future_=controller_->async_get_result(controller_goal_);
      phase_=Phase::EXECUTING;
    }
    return;
  }
  if (phase_==Phase::EXECUTING) {
    // 실행 중에도 lease 만료를 재확인하고, 주기적으로 A3 CONTINUE를 받아 권한을 갱신한다.


    if (Clock::now()>=permission_deadline_) {finish(false,"LEASE_EXPIRED");return;}
    if (result_future_.valid() && result_future_.wait_for(0ms)==std::future_status::ready) {
      const auto wrapped=result_future_.get();
      bool succeeded=wrapped.code==rclcpp_action::ResultCode::SUCCEEDED &&
                     wrapped.result && wrapped.result->error_code==control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
      finish(succeeded,succeeded?"":"ARM_CONTROLLER_FAILED");return;
    }
    if (lease_pending_) {
      if (seconds(continue_start_)>service_timeout_) {finish(false,"A3_CONTINUE_TIMEOUT");return;}
      if (lease_future_.valid() && lease_future_.wait_for(0ms)==std::future_status::ready) {
        auto r=lease_future_.get();lease_pending_=false;
        if (!r->allowed || r->valid_for_s<=0 || seconds(request_start_)>=r->valid_for_s) {
          finish(false,r->reason_code.empty()?"LEASE_REVOKED":r->reason_code);return;
        }
        permission_deadline_=request_start_+
          std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(r->valid_for_s));
        last_renew_=Clock::now();
      }
    } else if (seconds(last_renew_)>=renewal_period_) {
      ask_lease(Lease::Request::CONTINUE);
      continue_start_=request_start_;
    }
  }
}
int main(int argc,char **argv) {
  // ROS executor가 action callback, 상태 구독, 20 ms 안전 tick을 처리한다.


  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<ProtectedArmServer>());
  rclcpp::shutdown();return 0;
}
