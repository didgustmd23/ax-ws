// 흡착 공구 부착·해제를 Gazebo에서 수행하는 모델 플러그인.
//
// 기본 DetachableJoint는 연결 대상을 불러올 때 고정해서, 실행 중에 정해지는 물체를
// 붙일 수 없다. 이 플러그인은 명령으로 받은 물체 이름에 고정 관절을 만들고 지운다.
// 진공·접촉 물리는 없으며 근접 조건을 만족하면 연결한다. 압력값은 만들지 않는다.
//
// 명령 토픽(gz.msgs.StringMsg): "attach <물체>" 또는 "release <물체>"
// 상태 토픽(gz.msgs.StringMsg): "<0|1> <물체 또는 ->" 뒤에 마지막 사건을 붙인다
// 사건: none, attached, released, not_found, too_far, busy, not_attached, child_removed
//
// 전체 흐름
//   공구 노드(suction_tool_node.py) → 브리지 → 명령 토픽 → OnCommand → pending_ 큐
//   → PreUpdate의 ProcessCommands → Attach·Release → 상태 토픽 → 브리지 → 공구 노드
//
// 수락 조건(lease, arm_allowed) 검사는 공구 노드가 맡는다. 이 플러그인은 근접 조건만 본다.
// 상태 문자열 예: "1 box_11 attached", "0 - too_far"
//
// 담당: A2. 팀원은 이 플러그인을 직접 호출하지 않고 tool_command 서비스를 쓴다.
//
// 사건 의미
//   none          아직 명령을 처리한 적 없음
//   attached      연결 성공, 또는 이미 같은 물체가 연결되어 있음
//   released      연결 해제 성공
//   not_found     그 이름의 Gazebo 모델이 없거나 링크가 없음
//   too_far       TCP와 물체 중심 거리가 max_attach_distance보다 큼
//   busy          다른 물체를 들고 있어 새 물체를 붙이지 않음
//   not_attached  들고 있지 않은 물체의 해제 요청
//   child_removed 연결 중이던 물체가 월드에서 삭제됨
// 사건은 마지막 결과만 남으며 현재 공구 노드는 앞의 두 필드만 쓴다.
// 그래서 실패 원인은 아직 attach_state로 전달되지 않는다.
//
// 용어
//   ECM    EntityComponentManager. Gazebo의 모든 물체·링크·관절 데이터를 담는 저장소
//   엔티티 ECM 안의 객체 번호. 모델·링크·관절이 각각 엔티티다
//   TCP    공구 끝 기준점. 여기서는 epick_tcp
//
// 공구나 모델이 바뀌면 launch의 SUCTION_PLUGIN에서 parent_link와 tcp_offset을 함께 고친다.

#include <algorithm>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <gz/msgs/stringmsg.pb.h>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace ax_control
{

// 상태 토픽 발행 주기. attach_state 20 Hz보다 짧게 둔다.
// 공구 노드는 0.3 s 동안 상태를 못 받으면 attach_state를 무효로 표시한다.
constexpr auto kStatePeriod = std::chrono::milliseconds(25);

// 로봇 모델에 붙는 시스템이다. launch가 URDF 끝에 <plugin>을 덧붙여 불러온다.
class SuctionAttachSystem
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  // SDF 설정을 읽고 명령·상태 토픽을 연결한다
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);

    // 값은 launch의 SUCTION_PLUGIN에서 온다. 누락 시 기본값이 쓰이므로 launch와 함께 확인한다.
    parent_link_name_ = sdf->Get<std::string>("parent_link");
    tcp_offset_ = sdf->Get<gz::math::Vector3d>("tcp_offset");
    max_attach_distance_ = sdf->Get<double>("max_attach_distance");

    const auto command_topic = sdf->Get<std::string>("command_topic");
    const auto state_topic = sdf->Get<std::string>("state_topic");

    // 이 시점에 링크가 아직 없을 수 있어 PreUpdate에서도 다시 찾는다.
    parent_link_ = model_.LinkByName(ecm, parent_link_name_);

    state_publisher_ = node_.Advertise<gz::msgs::StringMsg>(state_topic);
    node_.Subscribe(command_topic, &SuctionAttachSystem::OnCommand, this);
  }

  // 명령을 처리하고 연결 상태를 점검한 뒤 주기적으로 상태를 발행한다
  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (parent_link_ == gz::sim::kNullEntity) {
      parent_link_ = model_.LinkByName(ecm, parent_link_name_);
    }

    // 일시정지 중에는 물리가 돌지 않으므로 연결을 바꾸지 않는다. 상태 발행은 계속한다.
    if (!info.paused) {
      ProcessCommands(ecm);
      CheckChildExists(ecm);
    }

    PublishState(info);
  }

private:
  // 전송 스레드에서 받은 명령을 시뮬레이션 스레드로 넘긴다
  // 이 함수는 Gazebo 전송 스레드에서 호출되고 PreUpdate와 동시에 돌 수 있다.
  // ECM은 시뮬레이션 스레드에서만 만져야 하므로 여기서는 큐에 넣기만 한다.
  void OnCommand(const gz::msgs::StringMsg & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.push_back(message.data());
  }

  // 쌓인 명령을 순서대로 처리한다
  void ProcessCommands(gz::sim::EntityComponentManager & ecm)
  {
    std::deque<std::string> commands;

    // 잠금 시간을 줄이려고 큐를 통째로 꺼낸 뒤 잠금 밖에서 처리한다.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      commands.swap(pending_);
    }

    for (const auto & command : commands) {
      std::istringstream stream(command);
      std::string action;
      std::string object;
      stream >> action >> object;

      if (action == "attach") {
        Attach(ecm, object);
      } else if (action == "release") {
        Release(ecm, object);
      }
    }
  }

  // 근접 조건을 확인하고 공구와 물체를 고정 관절로 잇는다
  // 검사 순서: 이미 부착 중인지 → 물체가 있는지 → 거리가 조건 안인지 → 관절 생성
  // 실패해도 예외를 내지 않고 last_event_에 사건만 남긴다.
  void Attach(gz::sim::EntityComponentManager & ecm, const std::string & object)
  {
    // 이미 붙어 있으면 같은 물체는 성공으로, 다른 물체는 busy로 끝낸다. 한 번에 하나만 잡는다.
    if (attached_joint_ != gz::sim::kNullEntity) {
      last_event_ = object == attached_object_ ? "attached" : "busy";
      return;
    }

    const auto object_model = FindModel(ecm, object);

    if (object_model == gz::sim::kNullEntity || parent_link_ == gz::sim::kNullEntity) {
      last_event_ = "not_found";
      return;
    }

    const auto links = gz::sim::Model(object_model).Links(ecm);

    if (links.empty()) {
      last_event_ = "not_found";
      return;
    }

    // 시험 상자처럼 링크가 하나인 물체를 가정해 첫 링크를 쓴다.
    const auto child_link = links.front();

    // Gazebo가 고정 관절을 합쳐 epick_tcp 링크가 사라지므로 부모 링크에서 오프셋으로 계산한다.
    const gz::math::Pose3d tool_pose = gz::sim::worldPose(parent_link_, ecm);
    const auto tcp_position = tool_pose.Pos() + tool_pose.Rot().RotateVector(tcp_offset_);
    const auto object_position = gz::sim::worldPose(child_link, ecm).Pos();

    // 기준은 물체 표면이 아니라 링크 중심이다. 물체 크기가 달라지면 거리값도 함께 봐야 한다.
    if (tcp_position.Distance(object_position) > max_attach_distance_) {
      last_event_ = "too_far";
      return;
    }

    // 관절 엔티티를 만들면 Physics 시스템이 고정 연결을 생성한다.
    // 현재 상대 위치를 유지한 채 연결되므로 물체가 공구 쪽으로 끌려오지 않는다.
    // 연결 실패는 여기서 알 수 없다. 실제 추종은 check_tool_attach.py가 위치로 확인한다.
    attached_joint_ = ecm.CreateEntity();
    ecm.CreateComponent(
      attached_joint_,
      gz::sim::components::DetachableJoint({parent_link_, child_link, "fixed"}));

    attached_model_ = object_model;
    attached_object_ = object;
    last_event_ = "attached";
  }

  // 지정 물체가 연결된 경우에만 관절을 지운다
  // 다른 물체 이름으로 온 해제가 들고 있는 물체를 떨어뜨리지 않게 한다.
  void Release(gz::sim::EntityComponentManager & ecm, const std::string & object)
  {
    if (attached_joint_ == gz::sim::kNullEntity || object != attached_object_) {
      last_event_ = "not_attached";
      return;
    }

    ClearAttachment(ecm);
    last_event_ = "released";
  }

  // 연결된 물체가 월드에서 사라지면 부착 상태를 지운다
  // 없는 물체를 attached로 계속 보고하지 않기 위한 점검이다.
  void CheckChildExists(gz::sim::EntityComponentManager & ecm)
  {
    if (attached_joint_ == gz::sim::kNullEntity || ecm.HasEntity(attached_model_)) {
      return;
    }

    ClearAttachment(ecm);
    last_event_ = "child_removed";
  }

  // 관절 엔티티를 지우고 부착 기록을 비운다
  // 관절이 사라지면 물체는 중력으로 떨어진다. 놓는 위치 확인은 호출하는 쪽 책임이다.
  void ClearAttachment(gz::sim::EntityComponentManager & ecm)
  {
    ecm.RequestRemoveEntity(attached_joint_);

    attached_joint_ = gz::sim::kNullEntity;
    attached_model_ = gz::sim::kNullEntity;
    attached_object_.clear();
  }

  // 이름으로 월드의 모델을 찾는다
  // ToolCommand.object_id와 Gazebo 모델 이름이 같아야 찾을 수 있다. 입고·상자 생성 쪽과 맞출 것.
  gz::sim::Entity FindModel(
    gz::sim::EntityComponentManager & ecm,
    const std::string & name) const
  {
    return ecm.EntityByComponents(
      gz::sim::components::Name(name),
      gz::sim::components::Model());
  }

  // 현재 부착 상태를 상태 토픽으로 발행한다
  void PublishState(const gz::sim::UpdateInfo & info)
  {
    // 뒤의 조건은 시뮬레이션 리셋으로 시간이 되돌아간 경우에도 발행을 계속하기 위한 것이다.
    if (info.simTime - last_state_time_ < kStatePeriod &&
      info.simTime >= last_state_time_)
    {
      return;
    }

    last_state_time_ = info.simTime;

    // 물체가 없을 때 "-"를 넣어 공백으로 나눈 필드 개수를 일정하게 유지한다.
    gz::msgs::StringMsg message;
    const bool attached = attached_joint_ != gz::sim::kNullEntity;

    message.set_data(
      std::string(attached ? "1 " : "0 ") +
      (attached ? attached_object_ : "-") + " " + last_event_);

    state_publisher_.Publish(message);
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Entity parent_link_{gz::sim::kNullEntity};
  gz::sim::Entity attached_joint_{gz::sim::kNullEntity};
  gz::sim::Entity attached_model_{gz::sim::kNullEntity};

  std::string parent_link_name_;
  std::string attached_object_;
  std::string last_event_{"none"};

  gz::math::Vector3d tcp_offset_;
  double max_attach_distance_{0.0};

  // 음수로 시작해 첫 PreUpdate에서 바로 상태를 발행한다.
  std::chrono::steady_clock::duration last_state_time_{-kStatePeriod};

  gz::transport::Node node_;
  gz::transport::Node::Publisher state_publisher_;

  // mutex_는 전송 스레드와 시뮬레이션 스레드가 공유하는 pending_만 보호한다.
  std::mutex mutex_;
  std::deque<std::string> pending_;
};

}  // namespace ax_control

GZ_ADD_PLUGIN(
  ax_control::SuctionAttachSystem,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)
