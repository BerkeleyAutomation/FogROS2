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

from pathlib import PurePath


def pytest_ignore_collect(path):
    # pytest doctest messes up when trying to import .launch.py packages, ignore them.
    # It also messes up when trying to import launch.logging.handlers due to conflicts with
    # logging.handlers, ignore that as well.
    return str(path).endswith((
        '.launch.py',
        str(PurePath('logging') / 'handlers.py'),
    ))
