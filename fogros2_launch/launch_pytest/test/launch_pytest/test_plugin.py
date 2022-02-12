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

from pathlib import Path
import shutil


def test_launch_fixture_is_not_a_launch_description(testdir):
    testdir.makepyfile("""\
import launch_pytest
import pytest

@launch_pytest.fixture
def launch_description():
    return object()

@pytest.mark.launch(fixture=launch_description)
def test_case():
    assert True
""")

    result = testdir.runpytest()
    try:
        result.assert_outcomes(errors=1)
    except TypeError:
        # Compatibility to pytest older than 6.0
        result.assert_outcomes(error=1)
    result.stdout.re_match_lines(['.*must be either a launch description.*'])


def test_launch_fixture_is_not_a_sequence_starting_with_a_ld(testdir):
    testdir.makepyfile("""\
import launch_pytest
import pytest

@launch_pytest.fixture
def launch_description():
    return [object, 'asd']

@pytest.mark.launch(fixture=launch_description)
def test_case():
    assert True
""")

    result = testdir.runpytest()

    try:
        result.assert_outcomes(errors=1)
    except TypeError:
        # Compatibility to pytest older than 6.0
        result.assert_outcomes(error=1)
    result.stdout.re_match_lines(['.*must be either a launch description.*'])


def test_multiple_ready_to_test_actions(testdir):
    testdir.makepyfile("""\
from launch import LaunchDescription
import launch_pytest
import pytest

@launch_pytest.fixture
def launch_description():
    return LaunchDescription(
        [launch_pytest.actions.ReadyToTest(), launch_pytest.actions.ReadyToTest()]
    )

@pytest.mark.launch(fixture=launch_description)
def test_case():
    assert True
""")

    result = testdir.runpytest()

    try:
        result.assert_outcomes(errors=1)
    except TypeError:
        # Compatibility to pytest older than 6.0
        result.assert_outcomes(error=1)
    result.stdout.re_match_lines(['.*only one ReadyToTest action.*'])


def test_generator_yields_twice(testdir):
    testdir.makepyfile("""\
from launch import LaunchDescription
import launch_pytest
import pytest

@launch_pytest.fixture
def launch_description():
    return LaunchDescription(
        [launch_pytest.actions.ReadyToTest()]
    )

@pytest.mark.launch(fixture=launch_description)
def test_case():
    assert True
    yield
    assert True
    yield
""")

    result = testdir.runpytest()

    result.assert_outcomes(failed=1)
    result.stdout.re_match_lines(['.*must stop iteration after yielding once.*'])


def test_generator_yields_twice_module_scope(testdir):
    testdir.makepyfile("""\
from launch import LaunchDescription
import launch_pytest
import pytest

@launch_pytest.fixture(scope='module')
def launch_description():
    return LaunchDescription(
        [launch_pytest.actions.ReadyToTest()]
    )

@pytest.mark.launch(fixture=launch_description)
def test_case():
    assert True
    yield
    assert True
    yield
""")

    result = testdir.runpytest()

    result.assert_outcomes(passed=1, failed=1)
    result.stdout.re_match_lines(['.*must stop iteration after yielding once.*'])


def test_asyncgenerator_yields_twice(testdir):
    testdir.makepyfile("""\
from launch import LaunchDescription
import launch_pytest
import pytest

@launch_pytest.fixture
def launch_description():
    return LaunchDescription(
        [launch_pytest.actions.ReadyToTest()]
    )

@pytest.mark.launch(fixture=launch_description)
async def test_case():
    assert True
    yield
    assert True
    yield
""")

    result = testdir.runpytest()

    result.assert_outcomes(failed=1)
    result.stdout.re_match_lines(['.*must stop iteration after yielding once.*'])


def test_asyncgenerator_yields_twice_module_scope(testdir):
    testdir.makepyfile("""\
from launch import LaunchDescription
import launch_pytest
import pytest

@launch_pytest.fixture(scope='module')
def launch_description():
    return LaunchDescription(
        [launch_pytest.actions.ReadyToTest()]
    )

@pytest.mark.launch(fixture=launch_description)
async def test_case():
    assert True
    yield
    assert True
    yield
""")

    result = testdir.runpytest()

    result.assert_outcomes(passed=1, failed=1)
    result.stdout.re_match_lines(['.*must stop iteration after yielding once.*'])


def test_fixture_kwarg_is_mandatory(LineMatcher, testdir):
    testdir.makepyfile("""\
from launch import LaunchDescription
import launch_pytest
import pytest

@launch_pytest.fixture
def launch_description():
    return LaunchDescription(
        [launch_pytest.actions.ReadyToTest()]
    )

@pytest.mark.launch
def test_case():
    pass
""")

    result = testdir.runpytest()

    result.assert_outcomes(skipped=1)
    # Warnings can appear in both stdout or stderr, depending on pytest capture mode.
    # Test for a match in any of both.
    LineMatcher(result.outlines + result.errlines).re_match_lines(
        ['.*"fixture" keyword argument is required .*'])


def test_generator_shutdown_kwarg_true(testdir):
    testdir.makepyfile("""\
from launch import LaunchDescription
import launch_pytest
import pytest

@launch_pytest.fixture
def launch_description():
    return LaunchDescription(
        [launch_pytest.actions.ReadyToTest()]
    )

@pytest.mark.launch(fixture=launch_description, shutdown=True)
def test_case():
    assert True
    yield
    assert True
""")

    result = testdir.runpytest()

    result.assert_outcomes(failed=1)
    result.stdout.re_match_lines(['.*generator or async generator.* shutdown=True.*'])


def test_async_generator_shutdown_kwarg_true(testdir):
    testdir.makepyfile("""\
from launch import LaunchDescription
import launch_pytest
import pytest

@launch_pytest.fixture
def launch_description():
    return LaunchDescription(
        [launch_pytest.actions.ReadyToTest()]
    )

@pytest.mark.launch(fixture=launch_description, shutdown=True)
async def test_case():
    assert True
    yield
    assert True
""")

    result = testdir.runpytest()

    result.assert_outcomes(failed=1)
    result.stdout.re_match_lines(['.*generator or async generator.* shutdown=True.*'])


def test_generator_with_pre_shutdown_failure(testdir):
    testdir.makepyfile("""\
from launch import LaunchDescription
import launch_pytest
import pytest

@launch_pytest.fixture(scope='module')
def launch_description():
    return LaunchDescription(
        [launch_pytest.actions.ReadyToTest()]
    )

@pytest.mark.launch(fixture=launch_description)
def test_case():
    assert False
    yield
    assert True
""")

    result = testdir.runpytest()

    result.assert_outcomes(failed=1, skipped=1)


def test_async_generator_with_pre_shutdown_failure(testdir):
    testdir.makepyfile("""\
from launch import LaunchDescription
import launch_pytest
import pytest

@launch_pytest.fixture(scope='module')
def launch_description():
    return LaunchDescription(
        [launch_pytest.actions.ReadyToTest()]
    )

@pytest.mark.launch(fixture=launch_description)
async def test_case():
    assert False
    yield
    assert True
""")

    result = testdir.runpytest()

    result.assert_outcomes(failed=1, skipped=1)


def test_examples(testdir):
    examples_dir = Path(__file__).parent / 'examples'
    for example in examples_dir.iterdir():
        # Ignoring `check_node_msgs.py` because we cannot depend on launch_ros here
        # as it creates a circular dependency between repositories.
        # We need to move that example elsewhere.
        if example.is_file() and example.name != 'check_node_msgs.py':
            copied_example = Path(testdir.copy_example(example))
            copied_example.rename(copied_example.parent / f'test_{copied_example.name}')
    shutil.copytree(examples_dir / 'executables', Path(str(testdir.tmpdir)) / 'executables')
    result = testdir.runpytest()
    result.assert_outcomes(passed=22)
