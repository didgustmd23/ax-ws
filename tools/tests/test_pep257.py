# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path

from ament_pep257.main import main
import pytest

# 검사 범위: 이 파일 위치 기준 저장소 루트의 src/ax_task_executor.
# 옮기기 전 패키지 폴더에서 돌던 범위와 같은 패키지 코드다.
PACKAGE = Path(__file__).resolve().parents[2] / 'src' / 'ax_task_executor'


# ax_task_executor 패키지 코드의 docstring이 pep257 규칙을 따르는지 확인한다.
@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=[str(PACKAGE)])
    assert rc == 0, 'Found code style errors / warnings'
