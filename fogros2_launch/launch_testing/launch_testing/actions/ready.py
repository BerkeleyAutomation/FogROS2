# Copyright 2019 Apex.AI, Inc.
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

import logging
from typing import List
from typing import Optional

from launch.action import Action
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity

_logger_ = logging.getLogger(__name__)


class ReadyToTest(Action):
    """Action that signals to launch_test that it's safe to start the tests."""

    def __init__(self):
        super().__init__()
        self._cb_list = []

    def _add_callback(self, callback):
        self._cb_list.append(callback)

    def execute(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        """Execute the action."""
        for cb in self._cb_list:
            try:
                cb()
            except Exception as e:
                _logger_.error(e)
