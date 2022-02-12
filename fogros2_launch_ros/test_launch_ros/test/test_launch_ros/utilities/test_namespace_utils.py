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

from launch_ros.utilities import is_namespace_absolute
from launch_ros.utilities import is_root_namespace
from launch_ros.utilities import make_namespace_absolute
from launch_ros.utilities import prefix_namespace


def test_is_absolute():
    assert is_namespace_absolute('/')
    assert is_namespace_absolute('/asd')
    assert is_namespace_absolute('/asd/bsd')
    assert not is_namespace_absolute('')
    assert not is_namespace_absolute('asd')
    assert not is_namespace_absolute('asd/bsd')


def test_is_root():
    assert is_root_namespace('/')
    assert not is_root_namespace('/asd')
    assert not is_root_namespace('/asd/bsd')
    assert not is_root_namespace('')
    assert not is_root_namespace('asd')
    assert not is_root_namespace('asd/bsd')


def test_make_absolute():
    assert make_namespace_absolute('/') == '/'
    assert make_namespace_absolute('/asd') == '/asd'
    assert make_namespace_absolute('/asd/bsd') == '/asd/bsd'
    assert make_namespace_absolute('') == '/'
    assert make_namespace_absolute('asd') == '/asd'
    assert make_namespace_absolute('asd/bsd') == '/asd/bsd'


def test_prefix_namespace():
    assert prefix_namespace(None, None) is None

    assert prefix_namespace('asd', None) == 'asd'
    assert prefix_namespace('/asd', None) == '/asd'
    assert prefix_namespace('', None) == ''
    assert prefix_namespace('/', None) == '/'

    assert prefix_namespace(None, 'asd') == 'asd'
    assert prefix_namespace(None, '/asd') == '/asd'
    assert prefix_namespace(None, '') == ''
    assert prefix_namespace(None, '/') == '/'

    assert prefix_namespace('asd', 'bsd') == 'asd/bsd'
    assert prefix_namespace('asd', '/bsd') == '/bsd'
    assert prefix_namespace('/asd', 'bsd') == '/asd/bsd'
    assert prefix_namespace('/asd', '/bsd') == '/bsd'
    assert prefix_namespace('/', 'bsd') == '/bsd'
    assert prefix_namespace('', 'bsd') == 'bsd'
    assert prefix_namespace('asd', '') == 'asd'

    assert prefix_namespace('asd', 'bsd/') == 'asd/bsd'
    assert prefix_namespace('asd', '/bsd/') == '/bsd'
