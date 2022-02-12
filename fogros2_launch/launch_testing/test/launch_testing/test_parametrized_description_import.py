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

import types
import unittest

import launch_testing
from launch_testing.loader import LoadTestsFromPythonModule


class TestModuleImport(unittest.TestCase):

    def setUp(self):
        class FakePreShutdownTests(unittest.TestCase):

            def test_1(self):
                pass  # pragma: no cover

            def test_2(self):
                pass  # pragma: no cover

        @launch_testing.post_shutdown_test()
        class FakePostShutdownTests(unittest.TestCase):

            def test_3(self):
                pass  # pragma: no cover

            def test_4(self):
                pass  # pragma: no cover

        self.test_module = types.ModuleType('test_module')
        self.test_module.FakePreShutdownTests = FakePreShutdownTests
        self.test_module.FakePostShutdownTests = FakePostShutdownTests

    def test_non_parametrized_test_description(self):

        def generate_test_description(ready_func):
            pass  # pragma: no cover

        self.test_module.generate_test_description = generate_test_description

        test_runs = LoadTestsFromPythonModule(self.test_module)

        assert len(test_runs) == 1

    def test_parametrized_test_description(self):

        @launch_testing.parametrize('arg_1', [1, 2, 3, 4, 5])
        def generate_test_description(ready_func, arg_1):
            pass  # pragma: no cover

        self.test_module.generate_test_description = generate_test_description

        test_runs = LoadTestsFromPythonModule(self.test_module)

        assert len(test_runs) == 5
