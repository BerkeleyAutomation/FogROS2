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

from launch_testing import post_shutdown_test
from launch_testing.loader import LoadTestsFromPythonModule
import pytest


def _source_test_loader(generate_test_description_fn,
                        pre_shutdown_tests=[],
                        post_shutdown_tests=[]):
    """
    Create a TestRun from python functions.

    The launch_testing API knows how to load tests from files on disk and from python modules.
    When we want to create a TestRun from a unit test, we usually have functions, not files
    and modules.
    Rather than refactoring launch_testing and python unittest, we leverage what we already have.
    This method stuffs test methods into unittest.TestCase classes, and then stuffs those
    classes into modules before loading them as a TestRun.

    :param: generate_test_description_fn A python function with a signature matching
        `generate_test_description`'s.
    :param: pre_shutdown_tests A list of test methods to run concurrently with the processes
        described by `generate_test_description`.
    :param: post_shutdown_tests A list of test methods to run after the launched processes
        have shut down.
    """
    test_module = types.ModuleType('test_module')
    test_module.generate_test_description = generate_test_description_fn

    if pre_shutdown_tests:
        class PreShutdownTestClass(unittest.TestCase):
            pass

        for test_func in pre_shutdown_tests:
            setattr(PreShutdownTestClass, test_func.__name__, test_func)

        setattr(test_module, 'PreShutdownTests', PreShutdownTestClass)

    if post_shutdown_tests:
        @post_shutdown_test()
        class PostShutdownTestClass(unittest.TestCase):
            pass

        for test_func in post_shutdown_tests:
            setattr(PostShutdownTestClass, test_func.__name__, test_func)

        setattr(test_module, 'PostShutdownTests', PostShutdownTestClass)

    return LoadTestsFromPythonModule(test_module)


@pytest.fixture
def source_test_loader():
    """Pytest fixture to be used by pytest-style tests."""
    return _source_test_loader


@pytest.fixture(scope='class')
def source_test_loader_class_fixture(request):
    """
    Pytest fixture to be used for old style unittest.TestCase classes.

    This will add the source_test_loader attribute onto the test class.

    Example:
    -------
        @pytest.mark.usefixtures('source_test_loader_class_fixture')
        class TestClass(unittest.TestCase):

            def test(self):
                test_runs = self.source_test_loader(  # <--Added by the fixture
                    . . .
                )

    """
    def wrapper(self, *args, **kwargs):
        return _source_test_loader(*args, **kwargs)

    request.cls.source_test_loader = wrapper
