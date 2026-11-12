# Copyright 2017 Open Source Robotics Foundation, Inc.
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

from ament_flake8.main import main_with_errors
import pytest

# 검사 범위: 이 파일 위치 기준 저장소 루트의 src/ax_task_executor.
# 옮기기 전 패키지 폴더에서 돌던 범위와 같은 패키지 코드다.
PACKAGE = Path(__file__).resolve().parents[2] / 'src' / 'ax_task_executor'


# ax_task_executor 패키지 코드에 flake8 위반이 없는지 확인한다.
@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[str(PACKAGE)])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
