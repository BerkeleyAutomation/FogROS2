# Copyright 2021 Open Source Robotics Foundation, Inc.
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

# imports needed for doctests
import launch
import launch.actions
import launch.conditions
import launch.substitutions

import pytest


@pytest.fixture(autouse=True)
def add_imports_to_doctest_namespace(doctest_namespace):
    doctest_namespace['launch'] = launch
    doctest_namespace['LaunchDescription'] = launch.LaunchDescription
    for subpackage in (
        launch.actions,
        launch.conditions,
        launch.substitutions,
    ):
        for x in subpackage.__all__:
            doctest_namespace[x] = getattr(subpackage, x)
