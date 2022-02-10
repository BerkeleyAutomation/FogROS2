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


def post_shutdown_test():
    """Decorate tests that are meant to run after process shutdown."""
    def decorator(test_item):
        if not isinstance(test_item, type):
            raise TypeError('postcondition_test should decorate test classes')
        test_item.__post_shutdown_test__ = True
        return test_item

    return decorator
