# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Module for the FindPackage substitution."""

from typing import Iterable
from typing import List
from typing import Text

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory

from launch.frontend import expose_substitution
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions


class FindPackage(Substitution):
    """
    Abstract base class for substitutions involving finding a package.

    Subclasses should implement the find() method.
    """

    def __init__(
        self,
        package: SomeSubstitutionsType,
    ) -> None:
        """Create a FindPackage substitution."""
        super().__init__()
        self.__package = normalize_to_list_of_substitutions(package)

    @classmethod
    def parse(cls, data: Iterable[SomeSubstitutionsType]):
        """Parse a FindPackage substitution."""
        if not data or len(data) != 1:
            raise AttributeError('find package substitutions expect 1 argument')
        kwargs = {'package': data[0]}
        return cls, kwargs

    @property
    def package(self) -> List[Substitution]:
        """Getter for package."""
        return self.__package

    def find(self, package_name: Text) -> Text:
        """
        Find a directory for a package.

        Called when the substitution is performed.

        :param: package_name The name of the package.
        :return: A directory related to the package.
        """
        raise NotImplementedError()

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        pkg_str = ' + '.join([sub.describe() for sub in self.package])
        return '{}(pkg={})'.format(self.__class__.__name__, pkg_str)

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by locating the package."""
        package = perform_substitutions(context, self.package)
        result = self.find(package)
        return result


@expose_substitution('find-pkg-prefix')
class FindPackagePrefix(FindPackage):
    """
    Substitution that tries to locate the package prefix of a ROS package.

    The ROS package is located using ament_index_python.

    :raise: ament_index_python.packages.PackageNotFoundError when package is
        not found during substitution.
    """

    def find(self, package_name: Text) -> Text:
        """Find the package prefix."""
        return get_package_prefix(package_name)


@expose_substitution('find-pkg-share')
class FindPackageShare(FindPackage):
    """
    Substitution that tries to locate the share directory of a ROS package.

    The directory is located using ament_index_python.

    :raise: ament_index_python.packages.PackageNotFoundError when package is
        not found during substitution.
    """

    def find(self, package_name: Text) -> Text:
        """Find the share directory of a package."""
        return get_package_share_directory(package_name)
