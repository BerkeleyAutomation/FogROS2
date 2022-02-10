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


import time
import unittest


class FailResult(unittest.TestResult):
    """For test runs that fail when the DUT dies unexpectedly."""

    def __init__(self, test_run, message):
        super().__init__()
        for case in test_run.all_cases():
            self.addFailure(case, (Exception, Exception(message), None))
            self.testsRun += 1

    @property
    def testCases(self):
        return [failure[0] for failure in self.failures]

    @property
    def testTimes(self):
        """Get a dict of {test_case: elapsed_time}."""
        return {failure[0]: 0 for failure in self.failures}

    def wasSuccessful(self):
        return False


class SkipResult(unittest.TestResult):
    """For test runs with a skip decorator on the generate_test_description function."""

    def __init__(self, test_run, skip_reason=''):
        super().__init__()
        for case in test_run.all_cases():
            self.addSkip(case, skip_reason)
            self.testsRun += 1

    @property
    def testCases(self):
        return [skipped[0] for skipped in self.skipped]

    @property
    def testTimes(self):
        """Get a dict of {test_case: elapsed_time}."""
        return {skipped[0]: 0 for skipped in self.skipped}

    def wasSuccessful(self):
        return True


class TestResult(unittest.TextTestResult):
    """
    Subclass of unittest.TestResult that collects more information about the tests that ran.

    This class extends TestResult by recording all of the tests that ran, and by recording
    start and stop time for the individual test cases
    """

    def __init__(self, stream=None, descriptions=None, verbosity=None):
        self.__test_cases = {}
        super().__init__(stream, descriptions, verbosity)

    @property
    def testCases(self):
        return self.__test_cases.keys()

    @property
    def testTimes(self):
        """Get a dict of {test_case: elapsed_time}."""
        return {k: v['end'] - v['start'] for (k, v) in self.__test_cases.items()}

    def startTest(self, test):
        self.__test_cases[test] = {
            'start': time.time(),
            'end': 0
        }
        super().startTest(test)

    def stopTest(self, test):
        self.__test_cases[test]['end'] = time.time()
        super().stopTest(test)

    def append(self, results):
        self.__test_cases.update(results.__test_cases)

        self.failures += results.failures
        self.errors += results.errors
        self.testsRun += results.testsRun
        self.skipped += results.skipped
        self.expectedFailures += results.expectedFailures
        self.unexpectedSuccesses += results.unexpectedSuccesses
