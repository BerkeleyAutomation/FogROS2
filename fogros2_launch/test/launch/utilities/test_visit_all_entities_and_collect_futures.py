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

"""Tests for the visit_all_entities_and_collect_futures() function."""

import asyncio

from launch import LaunchContext, LaunchDescriptionEntity
from launch.utilities import visit_all_entities_and_collect_futures


def test_visit_all_entities_and_collect_futures_with_future():
    """Test with an entity that has as future."""
    context = LaunchContext()

    class MockEntityDescriptionWithFuture(LaunchDescriptionEntity):

        def get_asyncio_future(self):
            return asyncio.Future()

    visit_with_future_result = visit_all_entities_and_collect_futures(
        MockEntityDescriptionWithFuture(),
        context
    )
    assert 1 == len(visit_with_future_result)
    assert 2 == len(visit_with_future_result[0])
    assert isinstance(visit_with_future_result[0][0], LaunchDescriptionEntity)
    assert isinstance(visit_with_future_result[0][0], MockEntityDescriptionWithFuture)
    assert isinstance(visit_with_future_result[0][1], asyncio.Future)


def test_visit_all_entities_and_collect_futures_no_future():
    """Test with an entity that has no future."""
    context = LaunchContext()

    class MockEntityDescriptionNoFuture(LaunchDescriptionEntity):
        pass

    visit_no_future_result = visit_all_entities_and_collect_futures(
        MockEntityDescriptionNoFuture(),
        context
    )
    assert 0 == len(visit_no_future_result)


def test_visit_all_entities_and_collect_futures_sub_entities():
    """Test with sub-entities."""
    context = LaunchContext()

    class MockEntityDescriptionWithFuture(LaunchDescriptionEntity):

        def get_asyncio_future(self):
            return asyncio.Future()

    class MockEntityDescriptionNoFuture(LaunchDescriptionEntity):
        pass

    class MockEntityDescriptionSubEntities(LaunchDescriptionEntity):

        def __init__(self, recurse_count):
            self.__recurse_count = recurse_count

        def get_asyncio_future(self):
            return asyncio.Future()

        def visit(self, context):
            # Recurse three times
            if self.__recurse_count < 3:
                return [MockEntityDescriptionWithFuture(),
                        MockEntityDescriptionNoFuture(),
                        MockEntityDescriptionSubEntities(self.__recurse_count + 1)]
            return None

    visit_sub_entities_result = visit_all_entities_and_collect_futures(
        MockEntityDescriptionSubEntities(0),
        context
    )
    assert 7 == len(visit_sub_entities_result)
    for i in range(0, len(visit_sub_entities_result)):
        future_pair = visit_sub_entities_result[i]
        assert 2 == len(future_pair)
        # Even entries should have type MockEntityDescriptionSubEntities
        if 0 == (i % 2):
            assert isinstance(future_pair[0], MockEntityDescriptionSubEntities)
        else:
            assert isinstance(future_pair[0], MockEntityDescriptionWithFuture)
        assert isinstance(future_pair[1], asyncio.Future)
