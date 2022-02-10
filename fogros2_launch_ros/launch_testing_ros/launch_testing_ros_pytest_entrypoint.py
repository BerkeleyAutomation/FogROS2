# Copyright 2019 Open Source Robotics Foundation, Inc.
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

# Only import standard python modules here
# This module intentionally delays imports as late as possible to avoid
# importing downstream modules in upstream packages when built with a merged
# workspace.


def pytest_launch_collect_makemodule(path, parent, entrypoint):
    marks = getattr(entrypoint, 'pytestmark', [])
    if marks and any(m.name == 'rostest' for m in marks):
        from launch_testing_ros.pytest.hooks import LaunchROSTestModule
        return LaunchROSTestModule.from_parent(parent=parent, fspath=path)


def pytest_configure(config):
    config.addinivalue_line(
        'markers',
        'rostest: mark a generate_test_description function as a ROS launch test entrypoint'
    )
