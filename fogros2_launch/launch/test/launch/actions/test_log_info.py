# Copyright 2020 Open Source Robotics Foundation, Inc.
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

"""Tests for the LogInfo action class."""

from launch import LaunchContext
from launch.actions import LogInfo
from launch.utilities import perform_substitutions


def test_log_info_constructors():
    """Test the constructors for LogInfo class."""
    LogInfo(msg='')
    LogInfo(msg='foo')
    LogInfo(msg=['foo', 'bar', 'baz'])


def test_log_info_methods():
    """Test the methods of the LogInfo class."""
    launch_context = LaunchContext()

    log_info = LogInfo(msg='')
    assert perform_substitutions(launch_context, log_info.msg) == ''

    log_info = LogInfo(msg='foo')
    assert perform_substitutions(launch_context, log_info.msg) == 'foo'

    log_info = LogInfo(msg=['foo', 'bar', 'baz'])
    assert perform_substitutions(launch_context, log_info.msg) == 'foobarbaz'


def test_log_info_execute():
    """Test the execute (or visit) of the LogInfo class."""
    log_info = LogInfo(msg='foo')
    launch_context = LaunchContext()
    assert log_info.visit(launch_context) is None
