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

import functools
import inspect
import time
import unittest


def keep_alive(test_description):
    """Mark a test launch description to be kept alive after fixture processes' termination."""
    if not hasattr(test_description, '__markers__'):
        test_description.__markers__ = {}
    test_description.__markers__['keep_alive'] = True
    return test_description


def retry_on_failure(*, times, delay=None):
    """
    Mark a test case to be retried up to `times` on AssertionError.

    :param times: The number of times to rety the test.
    :param delay: The time to wait between retries, in seconds.
      A value of None will result in zero delay.
    """
    assert times > 0

    def _decorator(func):
        assert 'self' == list(inspect.signature(func).parameters)[0]

        @functools.wraps(func)
        def _wrapper(self, *args, **kwargs):
            n = times
            while n > 1:
                try:
                    ret = func(self, *args, **kwargs)
                    if isinstance(self, unittest.TestCase):
                        assert self._outcome.success
                    return ret
                except AssertionError:
                    self._outcome.errors.clear()
                    self._outcome.success = True
                    n -= 1
                if delay is not None:
                    time.sleep(delay)
            return func(self, *args, **kwargs)
        return _wrapper
    return _decorator
