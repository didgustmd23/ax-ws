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

from ament_copyright.main import main
import pytest

# 검사 범위: 이 파일 위치 기준 저장소 루트의 src/ax_task_executor.
# 옮기기 전 패키지 폴더에서 돌던 범위와 같은 패키지 코드다.
PACKAGE = Path(__file__).resolve().parents[2] / 'src' / 'ax_task_executor'


# Remove the `skip` decorator once the source file(s) have a copyright header
# ax_task_executor 패키지 코드의 저작권 머리말을 확인한다(원본과 같이 skip).
@pytest.mark.skip(reason='No copyright header has been placed in the generated source file.')
@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    rc = main(argv=[str(PACKAGE)])
    assert rc == 0, 'Found errors'
