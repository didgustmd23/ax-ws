# AWS 창고 운영 지도 위치

`check_map_accuracy.py --adopt`가 채택한 지도를 여기에 둔다: `map.yaml`, `map.pgm`, `pose_graph.posegraph`, `pose_graph.data`, `map_info.yaml`(map_revision·해시·검증 결과).

현재 채택한 지도는 없다. 바퀴 마찰 후보(S005)를 적용한 임시 사본이나 충돌을 단순화한 월드(`light_collision`)에서 만든 지도는 운영 지도가 아니므로 채택하지 않는다.
운영 모델과 기준 월드가 확정되면 같은 도구로 다시 만들고, 정확도 기준을 정한 뒤 채택한다. 근거 자료는 [S005 바퀴 미끄럼 검증·공유 자료](../../../../docs/evidence/wheel_slip/)를 참고한다.
