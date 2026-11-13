// g++ -std=c++17 -I src/ax_safety/include tools/tests/test_a4_safety_gate.cpp -o /tmp/a4_gate_test && /tmp/a4_gate_test
#include "ax_safety/a4/safety_gate.hpp"
#include <cassert>
#include <limits>
int main() {


  ax_safety::GateInput g;
  assert(!ax_safety::evaluate_drive_gate(g,0.2,0).drive_allowed);


  g.enabled=g.safety_fresh=g.state_fresh=g.travel_pose_valid=g.operational_permits_valid=true;
  g.stop_required=false;g.allow_forward=true;g.allow_backward=true;g.allow_rotate=true;
  g.max_linear=.67;g.max_angular=.50;
  assert(ax_safety::evaluate_drive_gate(g,.3,0).drive_allowed);


  assert(ax_safety::evaluate_drive_gate(g,3.0,0).linear==.67);
  g.allow_forward=false;
  assert(ax_safety::evaluate_drive_gate(g,.3,0).linear==0);
  g.allow_forward=true;g.operational_permits_valid=false;
  assert(!ax_safety::evaluate_drive_gate(g,.3,0).drive_allowed);
  g.operational_permits_valid=true;g.stop_required=true;
  assert(!ax_safety::evaluate_drive_gate(g,.3,0).drive_allowed);
  g.stop_required=false;
  assert(!ax_safety::evaluate_drive_gate(g,std::numeric_limits<double>::quiet_NaN(),0).drive_allowed);
}
