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

"""Test for the FindPackage substitutions."""

from pathlib import Path

from launch import LaunchContext

from launch_ros.substitutions import FindPackagePrefix
from launch_ros.substitutions import FindPackageShare


def test_find_package_prefix():
    sub = FindPackagePrefix('launch_ros')
    context = LaunchContext()
    package_prefix = Path(sub.perform(context))
    package_xml_file = package_prefix / Path('share/launch_ros/package.xml')
    assert package_xml_file.is_file()


def test_find_package_share():
    sub = FindPackageShare('launch_ros')
    context = LaunchContext()
    package_prefix = Path(sub.perform(context))
    package_xml_file = package_prefix / Path('package.xml')
    assert package_xml_file.is_file()
