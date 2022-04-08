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

"""Tests for the class_tools_impl submodule."""

from launch.utilities.class_tools_impl import is_a, is_a_subclass, isclassinstance


def test_isclassinstance():
    """Test the isclassinstance function."""
    class MockClass:
        pass

    # Since Python3, everything is a class, so this means nothing (?)
    assert isclassinstance(0)
    assert isclassinstance(1.0)
    assert isclassinstance(complex(2.0))
    assert isclassinstance('foo')
    assert isclassinstance([])
    assert isclassinstance(())
    assert isclassinstance(range(6))
    assert isclassinstance(bytes(7))
    assert isclassinstance(bytearray())
    assert isclassinstance(memoryview(b'nine'))
    assert isclassinstance(set())
    assert isclassinstance(frozenset())
    assert isclassinstance({})
    assert isclassinstance(None)
    assert isclassinstance(MockClass())


def test_is_a():
    """Test the is_a function."""
    class MockParentClass:
        pass

    # Test primitives
    objects = [
        0,
        1.0,
        complex(2.0),
        'foo',
        [],
        (),
        range(6),
        bytes(7),
        bytearray(),
        memoryview(b'nine'),
        set(),
        frozenset(),
        {},
        MockParentClass(),
    ]
    types = [
        int,
        float,
        complex,
        str,
        list,
        tuple,
        range,
        bytes,
        bytearray,
        memoryview,
        set,
        frozenset,
        dict,
        MockParentClass,
    ]

    # Test primitives (+ one class) against each other
    assert len(objects) == len(types)
    for i in range(0, len(objects)):
        assert is_a(objects[i], types[i])
        for j in range(0, len(objects)):
            if j == i:
                continue
            assert not is_a(objects[i], types[j])

    class MockChildClass(MockParentClass):
        pass

    class MockNonRelativeClass:
        pass

    # Test with class inheritence
    child = MockChildClass()
    other = MockNonRelativeClass()
    assert is_a(child, MockChildClass)
    assert is_a(child, MockParentClass)
    assert not is_a(other, MockParentClass)
    assert not is_a(child, MockNonRelativeClass)


def test_is_a_subclass():
    """Test the is_a_subclass function."""
    class MockParentClass:
        pass

    class MockChildClass(MockParentClass):
        pass

    class MockGrandchildClass(MockChildClass):
        pass

    class MockNonRelativeClass:
        pass

    # Test with instances
    parent = MockParentClass()
    child = MockChildClass()
    grandchild = MockGrandchildClass()
    other = MockNonRelativeClass()
    assert is_a_subclass(1, int)
    assert is_a_subclass(child, MockParentClass)
    assert is_a_subclass(grandchild, MockParentClass)
    assert is_a_subclass(grandchild, MockChildClass)
    assert not is_a_subclass(1, float)
    assert not is_a_subclass(parent, MockNonRelativeClass)
    assert not is_a_subclass(child, MockNonRelativeClass)
    assert not is_a_subclass(grandchild, MockNonRelativeClass)
    assert not is_a_subclass(other, MockParentClass)
    assert not is_a_subclass(other, MockChildClass)
    assert not is_a_subclass(other, MockGrandchildClass)
    assert not is_a_subclass(parent, MockChildClass)
    assert not is_a_subclass(parent, MockGrandchildClass)
    assert not is_a_subclass(child, MockGrandchildClass)

    # Test with types
    assert is_a_subclass(int, int)
    assert is_a_subclass(MockChildClass, MockParentClass)
    assert is_a_subclass(MockGrandchildClass, MockParentClass)
    assert is_a_subclass(MockGrandchildClass, MockChildClass)
    assert not is_a_subclass(int, float)
    assert not is_a_subclass(MockParentClass, MockNonRelativeClass)
    assert not is_a_subclass(MockChildClass, MockNonRelativeClass)
    assert not is_a_subclass(MockGrandchildClass, MockNonRelativeClass)
    assert not is_a_subclass(MockNonRelativeClass, MockParentClass)
    assert not is_a_subclass(MockNonRelativeClass, MockChildClass)
    assert not is_a_subclass(MockNonRelativeClass, MockGrandchildClass)
    assert not is_a_subclass(MockParentClass, MockChildClass)
    assert not is_a_subclass(MockParentClass, MockGrandchildClass)
    assert not is_a_subclass(MockChildClass, MockGrandchildClass)
