// g++ -std=c++17 -I src/ax_safety/include tools/tests/test_a4_safety_gate.cpp -o /tmp/a4_gate_test && /tmp/a4_gate_test
#include "ax_safety/a4/safety_gate.hpp"
#include <cassert>
#include <limits>
int main() {


  ax_safety::GateInput g;
  assert(!ax_safety::evaluate_drive_gate(g,0.2,0).drive_allowed);


  g.enabled=g.safety_fresh=g.state_fresh=g.travel_pose_valid=
    g.operational_permits_valid=g.handoff_gate_valid=true;
  g.stop_required=false;g.allow_forward=true;g.allow_backward=true;g.allow_rotate=true;
  g.max_linear=.67;g.max_angular=.50;
  assert(ax_safety::evaluate_drive_gate(g,.3,0).drive_allowed);


  assert(ax_safety::evaluate_drive_gate(g,3.0,0).linear==.67);
  g.allow_forward=false;
  assert(ax_safety::evaluate_drive_gate(g,.3,0).linear==0);

  // D-06 표본 기반 정지 근거(LowChannel·low_span·common_low_span). 시각은 표본 stamp(초), 연속 간격 상한 0.3 s.
  {
    using ax_safety::LowChannel;
    using ax_safety::SampleContent;
    const double gap = 0.3;
    const double far = 1e9;   // now_s: 판정과 무관하게 충분히 뒤인 현재 시각
    const SampleContent c0{{"a"}, {0.0}};
    const SampleContent c1{{"a"}, {1.0}};
    // 경계 직전·충족: 20 ms 간격 저속 표본, 0.48 s에는 미충족, 0.50 s에 충족(두 채널 같은 구간)
    LowChannel o, j;
    for (int i = 0; i <= 24; ++i) { o.add(10.0 + 0.02 * i, far, true, c0, gap); j.add(10.0 + 0.02 * i, far, true, c0, gap); }
    assert(ax_safety::common_low_span(o, j, far) < 0.5);
    o.add(10.5, far, true, c0, gap); j.add(10.5, far, true, c0, gap);
    assert(ax_safety::common_low_span(o, j, far) >= 0.5 - 1e-9);
    // 판정 구간 끝은 현재 시각을 넘지 않는다
    assert(ax_safety::common_low_span(o, j, 10.45) < 0.5);
    // tick 사이 high→low: 이동 표본이 들어오는 즉시 구간이 끊긴다
    o.add(10.52, far, false, c1, gap); o.add(10.54, far, true, c0, gap);
    assert(o.start == 10.54 && ax_safety::common_low_span(o, j, far) == 0.0);
    // 미래 stamp(현재 시각보다 뒤): 저속이면 무시(최신·구간 불변), 이동이면 구간을 버린다
    LowChannel f; f.add(1.0, 1.0, true, c0, gap);
    assert(f.add(1.05, 1.0, true, c0, gap) == LowChannel::REJECTED && f.latest_stamp == 1.0 && f.end == 1.0);
    for (int i = 0; i < 50; ++i) f.add(1.049, 1.0, true, c0, gap);
    assert(ax_safety::low_span(f, 1.0) == 0.0);
    assert(f.add(1.001, 1.0, false, c1, gap) == LowChannel::REJECTED && !f.has_interval);
    // 미래였던 stamp는 보류되지 않는다: 시계가 닿은 뒤 같은 stamp가 다시 와야 수용된다
    assert(f.add(1.05, 1.05, true, c0, gap) == LowChannel::ACCEPTED);
    // 같은 stamp 같은 내용 재수신은 근거를 늘리지 않는다
    LowChannel d; d.add(1.0, far, true, c0, gap);
    for (int i = 0; i < 100; ++i) assert(d.add(1.0, far, true, c0, gap) == LowChannel::DUPLICATE);
    assert(d.end - d.start == 0.0);
    // 같은 stamp 다른 내용(상충)은 구간을 버린다. -0과 0도 원 값 비트가 다르므로 상충으로 본다
    d.add(1.1, far, true, c0, gap);
    assert(d.add(1.1, far, true, c1, gap) == LowChannel::CONFLICT && !d.has_interval && !d.latest_low);
    LowChannel z; z.add(1.0, far, true, SampleContent{{"a"}, {0.0}}, gap);
    assert(z.add(1.0, far, true, SampleContent{{"a"}, {-0.0}}, gap) == LowChannel::CONFLICT);
    // 이름-값 대응: 이름이 다르면 값이 같아도 다른 내용
    assert(!(SampleContent{{"a", "b"}, {1.0, 2.0}} == SampleContent{{"a", "c"}, {1.0, 2.0}}));
    // 늦은(역행) 저속 표본은 최신을 덮지 않고 무시, 늦은 이동 표본은 구간을 버린다
    LowChannel l; l.add(2.0, far, true, c0, gap); l.add(2.2, far, true, c0, gap);
    assert(l.add(2.1, far, true, c0, gap) == LowChannel::LATE && l.latest_stamp == 2.2 && l.has_interval);
    assert(l.add(2.1, far, false, c1, gap) == LowChannel::LATE && l.latest_stamp == 2.2 && !l.has_interval);
    // 공백 초과(0.3 s 넘음)는 구간을 끊고, 공백 0.3 s 이하는 이어진다
    LowChannel g2; g2.add(5.0, far, true, c0, gap); g2.add(5.3, far, true, c0, gap);
    assert(g2.start == 5.0 && g2.end == 5.3);
    g2.add(5.61, far, true, c0, gap);
    assert(g2.start == 5.61);
    // 비유한·0 stamp, 무효 시계는 수용하지 않고, 저속 아님이면 구간을 버린다
    assert(g2.add(std::numeric_limits<double>::quiet_NaN(), far, true, c0, gap) == LowChannel::REJECTED && g2.has_interval);
    assert(g2.add(5.7, 0.0, true, c0, gap) == LowChannel::REJECTED && g2.has_interval);
    assert(g2.add(0.0, far, false, c1, gap) == LowChannel::REJECTED && !g2.has_interval);
    // 채널 주기·위상 차이: odom 20 ms, joint 30 ms(위상 7 ms) → 교집합으로 충족
    LowChannel po, pj;
    for (int i = 0; i <= 30; ++i) po.add(20.0 + 0.02 * i, far, true, c0, gap);
    for (int i = 0; i <= 20; ++i) pj.add(20.007 + 0.03 * i, far, true, c0, gap);
    assert(ax_safety::common_low_span(po, pj, far) >= 0.5 - 1e-9);
    pj.add(20.65, far, false, c1, gap);
    assert(ax_safety::common_low_span(po, pj, far) == 0.0);
    // reset_all 뒤에는 작은 stamp도 새 표본으로 받는다(시계 리셋), 이전 구간은 남지 않는다
    po.reset_all();
    assert(ax_safety::low_span(po, far) == 0.0);
    assert(po.add(1.0, far, true, c0, gap) == LowChannel::ACCEPTED);
  }

  // 진단 전용 카운터: 결과별·REJECTED 이유별 수(판정 함수와 같은 조건 순서)
  {
    using ax_safety::LowChannel;
    assert(ax_safety::reject_reason(1.0, 0.0) == 0);
    assert(ax_safety::reject_reason(std::numeric_limits<double>::quiet_NaN(), 1.0) == 1);
    assert(ax_safety::reject_reason(1.1, 1.0) == 2);
    assert(ax_safety::reject_reason(1.0, 1.0) == 3);
    const ax_safety::SampleContent c0{{"a"}, {0.0}};
    LowChannel ch;
    ax_safety::ChannelCounters k;
    const double seq[][2] = {{1.0, 1.0}, {1.0, 1.0}, {1.2, 1.1}, {0.9, 1.1}, {1.05, 1.1}, {1.1, 0.0}, {0.0, 1.2}};
    for (const auto &s : seq) k.count(ch.add(s[0], s[1], true, c0, 0.3), s[0], s[1]);
    // 1.0 수용, 1.0 중복, 1.2 미래, 0.9 늦음, 1.05 수용, 시계 무효, stamp 무효
    assert(k.result[LowChannel::ACCEPTED] == 2 && k.result[LowChannel::DUPLICATE] == 1);
    assert(k.result[LowChannel::LATE] == 1 && k.result[LowChannel::REJECTED] == 3);
    assert(k.reject[0] == 1 && k.reject[1] == 1 && k.reject[2] == 1);
  }

  // 팔 정지 조건 역할 해석: 기본 미정, 팔 있음은 비어 있지 않고 중복·빈 이름 없는 목록, 팔 없음은 목록 없음
  {
    using ax_safety::ArmPresence;
    using ax_safety::parse_arm_presence;
    const std::vector<std::string> none;
    assert(parse_arm_presence("unconfirmed", none) == ArmPresence::UNCONFIRMED);
    assert(parse_arm_presence("present", {"a", "b"}) == ArmPresence::PRESENT);
    assert(parse_arm_presence("present", none) == ArmPresence::INVALID);
    assert(parse_arm_presence("present", {"a", "a"}) == ArmPresence::INVALID);
    assert(parse_arm_presence("present", {"a", ""}) == ArmPresence::INVALID);
    assert(parse_arm_presence("absent", none) == ArmPresence::ABSENT);
    assert(parse_arm_presence("absent", {"a"}) == ArmPresence::INVALID);
    assert(parse_arm_presence("", none) == ArmPresence::INVALID);
    assert(parse_arm_presence("robot_2", none) == ArmPresence::INVALID);
  }

  g.allow_forward=true;g.operational_permits_valid=false;
  assert(!ax_safety::evaluate_drive_gate(g,.3,0).drive_allowed);
  g.operational_permits_valid=true;g.stop_required=true;
  assert(!ax_safety::evaluate_drive_gate(g,.3,0).drive_allowed);
  g.stop_required=false;
  assert(!ax_safety::evaluate_drive_gate(g,std::numeric_limits<double>::quiet_NaN(),0).drive_allowed);
}
