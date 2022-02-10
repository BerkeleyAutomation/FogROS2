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

import unittest

from launch_testing import ReadyAggregator


class TestReadyAggregator(unittest.TestCase):

    def setUp(self):
        self.called = 0

    def parent_ready_fn(self):
        self.called += 1

    def test_aggregate_one(self):
        dut = ReadyAggregator(self.parent_ready_fn, 1)

        self.assertEqual(self.called, 0)
        dut.ready_fn()
        self.assertEqual(self.called, 1)

        # Make sure subsequent calls don't trigger the parent function
        dut.ready_fn()
        self.assertEqual(self.called, 1)

    def test_aggregate_multiple(self):
        NUM_CALLS = 10  # Maybe make this random?  Probably not worth the effort

        dut = ReadyAggregator(self.parent_ready_fn, NUM_CALLS)

        self.assertEqual(self.called, 0)

        for _ in range(9):
            dut.ready_fn()

        self.assertEqual(self.called, 0)
        dut.ready_fn()
        self.assertEqual(self.called, 1)

        # Make sure subsequent calls don't trigger the parent function
        dut.ready_fn()
        self.assertEqual(self.called, 1)
