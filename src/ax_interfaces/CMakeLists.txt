cmake_minimum_required(VERSION 3.8)
project(ax_interfaces)

# 공통 인터페이스 초안(K0). 타입 확정 전까지 schema_version으로 변경을 추적한다.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActuationState.msg"
  "msg/AgentDecision.msg"
  "msg/AgentPlan.msg"
  "msg/AgentProposal.msg"
  "msg/AgentReasoningState.msg"
  "msg/AgentReceiveState.msg"
  "msg/AgentReceiveStateList.msg"
  "msg/AssignmentCheckRequest.msg"
  "msg/AssignmentCheckResponse.msg"
  "msg/AttachState.msg"
  "msg/AttemptUsage.msg"
  "msg/BudgetGrant.msg"
  "msg/CandidatePreference.msg"
  "msg/CandidateSet.msg"
  "msg/CorridorObservation.msg"
  "msg/CorridorState.msg"
  "msg/CorridorStateList.msg"
  "msg/ExecutionLeaseGrant.msg"
  "msg/ExecutionLeaseRequest.msg"
  "msg/ExecutionLeaseRevoke.msg"
  "msg/ExecutionRef.msg"
  "msg/FailureEvent.msg"
  "msg/FleetSnapshot.msg"
  "msg/GraspCandidate.msg"
  "msg/GroupTaskList.msg"
  "msg/GroupTaskStatus.msg"
  "msg/HoldStatus.msg"
  "msg/HoldStatusList.msg"
  "msg/InventoryEntry.msg"
  "msg/InventorySnapshot.msg"
  "msg/LoadState.msg"
  "msg/LocalizationHealth.msg"
  "msg/OperationMode.msg"
  "msg/PlanCondition.msg"
  "msg/PlanStep.msg"
  "msg/ReceiptEvent.msg"
  "msg/ReceiptObservation.msg"
  "msg/RegisteredLocation.msg"
  "msg/ResultStatus.msg"
  "msg/RobotStopPose.msg"
  "msg/RobotSummary.msg"
  "msg/RobotTaskStatus.msg"
  "msg/SafetyState.msg"
  "msg/StateMeta.msg"
  "msg/TimeValidity.msg"
  "msg/ToolHoldState.msg"
  "srv/RecheckCondition.srv"
  "srv/RegisterTask.srv"
  "srv/RequestCorridorEntry.srv"
  "srv/SetOperationMode.srv"
  "srv/ToolCommand.srv"
  "action/ExecuteLocalTask.action"
  "action/ManipulateObject.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
