# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Tests for the create_future() function."""

import asyncio

from launch.utilities import create_future


def test_create_future():
    """Test the create_future() function."""
    future_none_result = create_future(None)
    assert isinstance(future_none_result, asyncio.Future)
    future_event_loop_result = create_future(asyncio.get_event_loop())
    assert isinstance(future_event_loop_result, asyncio.Future)
