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

import os
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

import ament_index_python
from launch_testing.junitxml import unittestResultsToXml
from launch_testing.test_result import FailResult
from launch_testing.test_result import SkipResult
from launch_testing.test_result import TestResult as TR
import pytest


class TestGoodXmlOutput(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # For performance, we'll run the test once and generate the XML output, then
        # have multiple test cases assert on it

        cls.tmpdir = tempfile.TemporaryDirectory()
        cls.xml_file = os.path.join(cls.tmpdir.name, 'junit.xml')

        path = os.path.join(
            ament_index_python.get_package_share_directory('launch_testing'),
            'examples',
            'good_proc_launch_test.py'
        )

        assert 0 == subprocess.run(
            args=[
                'launch_test',
                path,
                '--junit-xml', os.path.join(cls.tmpdir.name, 'junit.xml'),
                '--package-name', 'test_xml_output'
            ],
        ).returncode

    @classmethod
    def tearDownClass(cls):
        cls.tmpdir.cleanup()

    def test_pre_and_post(self):
        tree = ET.parse(self.xml_file)
        root = tree.getroot()

        self.assertEqual(len(root), 1)
        test_suite = root[0]

        # Expecting an element called '{package}.{test_base_name}.launch_tests' since this
        # was not parametrized
        self.assertEqual(
            test_suite.attrib['name'], 'test_xml_output.good_proc_launch_test.launch_tests'
        )

        # Drilling down a little further, we expect the class names to show up in the testcase
        # names
        case_names = [case.attrib['name'] for case in test_suite]
        self.assertIn('test_count_to_four', case_names)
        self.assertIn('test_full_output', case_names)


@pytest.mark.usefixtures('source_test_loader_class_fixture')
class TestXmlFunctions(unittest.TestCase):
    # This are closer to unit tests - just call the functions that generate XML

    def unit_test_result_factory(self, test_case_list):
        # Use the unittest library to run some fake test functions and generate a real TestResult
        # that we can serialize to XML

        class TestHost(unittest.TestCase):
            pass

        TestHost.__qualname__ = 'TestHost'  # as if it was defined in global scope

        for n, test_case in enumerate(test_case_list):
            setattr(TestHost, 'test_{}'.format(n), test_case)

        cases = unittest.TestLoader().loadTestsFromTestCase(TestHost)
        with open(os.devnull, 'w') as nullstream:
            runner = unittest.TextTestRunner(
                stream=nullstream,
                resultclass=TR
            )
            return runner.run(cases)

    def test_fail_results_serialize(self):

        def generate_test_description():
            raise Exception('This should never be invoked')  # pragma: no cover

        def test_fail_always(self):
            assert False  # pragma: no cover

        def test_pass_always(self):
            pass  # pragma: no cover

        test_runs = self.source_test_loader(
            generate_test_description,
            pre_shutdown_tests=[
                test_fail_always,
            ],
            post_shutdown_tests=[
                test_fail_always,
                test_pass_always
            ]
        )

        self.assertEqual(1, len(test_runs))  # Not a parametrized launch, so only 1 run

        xml_tree = unittestResultsToXml(
            name='fail_xml',
            test_results={
                'active_tests': FailResult(test_run=test_runs[0], message='Test Message')
            }
        )

        # Simple sanity check - see that there's a child element called active_tests
        child_names = [chld.attrib['name'] for chld in xml_tree.getroot()]
        self.assertEqual(set(child_names), {'active_tests'})

        # Make sure failures is non-zero, otherwise colcon test-result won't recognize this
        # as a failure
        self.assertGreater(int(xml_tree.getroot().get('failures')), 0)

    def test_skip_results_serialize(self):
        # This checks the case where all unit tests are skipped because of a skip
        # decorator on the generate_test_description function
        @unittest.skip('skip reason string')
        def generate_test_description():
            raise Exception('This should never be invoked')  # pragma: no cover

        def test_fail_always(self):
            assert False  # pragma: no cover

        def test_pass_always(self):
            pass  # pragma: no cover

        test_runs = self.source_test_loader(
            generate_test_description,
            pre_shutdown_tests=[
                test_fail_always,
            ],
            post_shutdown_tests=[
                test_fail_always,
                test_pass_always
            ]
        )

        self.assertEqual(1, len(test_runs))  # Not a parametrized launch, so only 1 run

        dut_xml = unittestResultsToXml(
            test_results={
                'run1': SkipResult(test_run=test_runs[0], skip_reason='skip message')
            }
        )

        # Make sure the message got into the 'skip' element
        testsuites_element = dut_xml.getroot()
        testsuite_element = testsuites_element.find('testsuite')

        self.assertEqual('0', testsuite_element.attrib['failures'])
        self.assertEqual('0', testsuite_element.attrib['errors'])
        self.assertEqual('3', testsuite_element.attrib['skipped'])
        self.assertEqual('3', testsuite_element.attrib['tests'])

        testcases = testsuite_element.findall('testcase')
        self.assertEqual(3, len(testcases))

        for testcase_element in testcases:
            skip_element = testcase_element.find('skipped')
            self.assertEqual('skip message', skip_element.attrib['message'])

    def test_multiple_test_results(self):
        xml_tree = unittestResultsToXml(
            name='multiple_launches',
            test_results={
                'launch_1': TR(None, True, 1),
                'launch_2': TR(None, True, 1),
                'launch_3': TR(None, True, 1),
            }
        )

        child_names = [chld.attrib['name'] for chld in xml_tree.getroot()]
        self.assertEqual(set(child_names), {'launch_1', 'launch_2', 'launch_3'})

    def test_result_that_ran(self):
        """
        Test we have output as a result of a test being run.

        The expected XML output for this test is:
        <testsuites>
          <testsuite name="run1" . . . >
            <testcase classname="TestHost" name="test_0" . . . />
          </testsuite>
        <testsuites>
        """
        # This mostly validates the test setup is good for the other tests
        dut_xml = unittestResultsToXml(
            test_results={
                'run1': self.unit_test_result_factory([
                    lambda self: None
                ])
            }
        )

        testsuites_element = dut_xml.getroot()
        testsuite_element = testsuites_element.find('testsuite')
        testcase_element = testsuite_element.find('testcase')

        # The bare minimum XML we require:
        self.assertEqual('0', testsuite_element.attrib['failures'])
        self.assertEqual('0', testsuite_element.attrib['errors'])
        self.assertEqual('1', testsuite_element.attrib['tests'])
        self.assertEqual('test_0', testcase_element.attrib['name'])
        # Expect a fully qualified class name
        self.assertEqual('test_xml_output.TestHost', testcase_element.attrib['classname'])

    def test_result_with_skipped_test(self):
        """
        Test we have output as a result of a skipped test.

        The expected XML output for this test is:
        <testsuites>
          <testsuite name="run1" skipped="1". . . >
            <testcase classname="TestHost" name="test_0" . . . >
              <skipped message="My reason is foo" . . . />
            </testcase>
          </testsuite>
        <testsuites>

        Notice the extra 'skipped' child-element of testcase.
        """
        @unittest.skip('My reason is foo')
        def test_that_is_skipped(self):
            pass  # pragma: no cover

        dut_xml = unittestResultsToXml(
            test_results={
                'run1': self.unit_test_result_factory([
                    test_that_is_skipped
                ])
            }
        )

        testsuites_element = dut_xml.getroot()
        testsuite_element = testsuites_element.find('testsuite')
        testcase_element = testsuite_element.find('testcase')
        skip_element = testcase_element.find('skipped')

        self.assertEqual('1', testsuite_element.attrib['skipped'])
        self.assertEqual('My reason is foo', skip_element.attrib['message'])

    def test_result_with_failure(self):
        """
        Test we have output as a result of failed test.

        The expected XML output for this test is:

        <testsuites>
          <testsuite name="run1" failures="1". . . >
            <testcase classname="TestHost" name="test_0" . . . >
              <failure message="assert 1 == 2" . . .>
              </failure>
            </testcase>
          </testsuite>
        <testsuites>

        Notice there's a failure message child-element of the testcase and
        a count of failed tests.
        """
        def test_that_fails(self):
            assert 1 == 2

        dut_xml = unittestResultsToXml(
            test_results={
                'run1': self.unit_test_result_factory([
                    test_that_fails
                ])
            }
        )

        testsuites_element = dut_xml.getroot()
        testsuite_element = testsuites_element.find('testsuite')
        testcase_element = testsuite_element.find('testcase')
        failure_element = testcase_element.find('failure')

        self.assertEqual('1', testsuite_element.attrib['failures'])
        self.assertIn('1 == 2', failure_element.attrib['message'])

    def test_result_with_error(self):
        """
        Test we have output as a result of an error in a test.

        The expected XML output for this test is:

        <testsuites>
          <testsuite name="run1" errors="1". . . >
            <testcase classname="TestHost" name="test_0" . . . >
              <error message="This is an error" . . .>
              </failure>
            </testcase>
          </testsuite>
        <testsuites>

        Python unittest treats exceptions other than AssertionError exceptions
        as 'errors' not failures so they get a different tag, and count.
        """
        def test_that_errors(self):
            raise Exception('This is an error')

        dut_xml = unittestResultsToXml(
            test_results={
                'run1': self.unit_test_result_factory([
                    test_that_errors
                ])
            }
        )

        testsuites_element = dut_xml.getroot()
        testsuite_element = testsuites_element.find('testsuite')
        testcase_element = testsuite_element.find('testcase')
        error_element = testcase_element.find('error')

        self.assertEqual('1', testsuite_element.attrib['errors'])
        self.assertIn('This is an error', error_element.attrib['message'])

    def test_with_multiple_results(self):

        def good_test(self):
            pass

        def error_test(self):
            raise Exception('I am an error')

        def fail_test(self):
            assert 1 == 2

        dut_xml = unittestResultsToXml(
            test_results={
                'run1': self.unit_test_result_factory([
                    good_test,
                    error_test,
                    fail_test,
                ])
            }
        )

        testsuites_element = dut_xml.getroot()
        testsuite_element = testsuites_element.find('testsuite')

        self.assertEqual('1', testsuite_element.attrib['failures'])
        self.assertEqual('1', testsuite_element.attrib['errors'])
        self.assertEqual('3', testsuite_element.attrib['tests'])
