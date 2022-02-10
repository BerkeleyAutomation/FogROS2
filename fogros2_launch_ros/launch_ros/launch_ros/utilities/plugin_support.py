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

from packaging.version import Version


class PluginException(Exception):
    """Base class for all exceptions within the plugin system."""

    pass


def satisfies_version(version, caret_range):
    assert caret_range.startswith('^'), 'Only supports caret ranges'
    extension_point_version = Version(version)
    extension_version = Version(caret_range[1:])
    next_extension_version = get_upper_bound_caret_version(
        extension_version)

    if extension_point_version < extension_version:
        raise PluginException(
            'Extension point is too old (%s), the extension requires '
            "'%s'" % (extension_point_version, extension_version))

    if extension_point_version >= next_extension_version:
        raise PluginException(
            'Extension point is newer (%s), than what the extension '
            "supports '%s'" % (extension_point_version, extension_version))


def get_upper_bound_caret_version(version):
    parts = version.base_version.split('.')
    if len(parts) < 2:
        parts += [0] * (2 - len(parts))
    major, minor = [int(p) for p in parts[:2]]
    if major > 0:
        major += 1
        minor = 0
    else:
        minor += 1
    return Version('%d.%d.0' % (major, minor))
