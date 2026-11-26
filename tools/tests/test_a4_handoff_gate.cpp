#include "ax_safety/a4/safety_gate.hpp"

#include <gtest/gtest.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace {

using ax_safety::HandoffPeerInput;
using ax_safety::HandoffTaskInput;

HandoffTaskInput task(const std::string &manipulator, const std::string &transporter,
                      uint8_t transfer_state)
{
  return {manipulator, transporter, transfer_state};
}

TEST(A4HandoffGate, MissingOrStaleListBlocksEvenWhenUnassigned)
{
  EXPECT_FALSE(ax_safety::evaluate_handoff_gate("robot_9", false, false, {}, {}));
  EXPECT_FALSE(ax_safety::evaluate_handoff_gate("robot_9", true, true, {}, {}));
}

TEST(A4HandoffGate, FreshListWithoutSelfAssignmentDoesNotApplyHandoffRule)
{
  const std::vector<HandoffTaskInput> tasks{
    task("robot_1", "robot_2", ax_safety::kTransferUnknown)};
  EXPECT_TRUE(ax_safety::evaluate_handoff_gate("robot_9", true, false, tasks, {}));
}

TEST(A4HandoffGate, AnySelfAssignmentInTransferBlocks)
{
  const std::unordered_map<std::string, HandoffPeerInput> fresh_peer{
    {"robot_2", {true, true}}};
  for (const auto state : {ax_safety::kTransferUnknown, ax_safety::kTransferLocked,
                           ax_safety::kTransferInProgress}) {
    EXPECT_FALSE(ax_safety::evaluate_handoff_gate(
      "robot_1", true, false, {task("robot_1", "robot_2", state)}, fresh_peer));
  }
}

TEST(A4HandoffGate, RelativeStateIsRequiredOnlyForTransferTask)
{
  const std::vector<HandoffTaskInput> unlocked{
    task("robot_1", "robot_2", 1)};
  EXPECT_TRUE(ax_safety::evaluate_handoff_gate("robot_1", true, false, unlocked, {}));

  const std::vector<HandoffTaskInput> transfer{
    task("robot_1", "robot_2", ax_safety::kTransferInProgress)};
  EXPECT_FALSE(ax_safety::evaluate_handoff_gate("robot_1", true, false, transfer, {}));
}

}  // namespace
