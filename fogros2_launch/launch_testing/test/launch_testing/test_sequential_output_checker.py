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

import unittest

from launch_testing.asserts import SequentialTextChecker


class TestAssertSequentialStdout(unittest.TestCase):

    def setUp(self):
        self.to_check = [
            'output 10\n',
            'output 15\n',
            'output 20\n',
            'multi-line 1\nmulti-line 2\nmulti-line 3\n',
            'aaaaa bbbbb ccccc ddddd eeeee fffff ggggg hhhhh iiiii jjjjj\n',  # long line
            'xxxxx yyyyy\nsome dummy text\nzzzzz\n',  # mix of long line and multi-line
            'output 20\n',
            '!@#$%^&*()\n',  # Help find off by one errors in the substring logic
        ]

        self.dut = SequentialTextChecker(self.to_check)

    def test_good_sequential_output(self):

        for output in self.to_check:
            self.dut.assertInStdout(output)

        with self.assertRaises(AssertionError):
            # This should assert because we've already moved past 'output 10'
            self.dut.assertInStdout(self.to_check[0])

    def test_non_matching_output_does_not_advance_state(self):
        # Make sure we can match correct output even after failing to match something

        with self.assertRaises(AssertionError):
            self.dut.assertInStdout('bad output not found')

        self.test_good_sequential_output()

    def test_multi_line_find(self):
        self.dut.assertInStdout('multi-line 1')
        self.dut.assertInStdout('multi-line 2')
        self.dut.assertInStdout('multi-line 3')

        with self.assertRaises(AssertionError):
            self.dut.assertInStdout('multi-line 1')

    def test_long_line_find(self):
        self.dut.assertInStdout('ccccc')
        self.dut.assertInStdout('ddddd')
        self.dut.assertInStdout('eeeee')

        with self.assertRaises(AssertionError):
            self.dut.assertInStdout('aaaaa')

    def test_duplicates_advances_state(self):
        self.dut.assertInStdout('output 20')
        self.dut.assertInStdout('output 20')

        with self.assertRaises(AssertionError):
            self.dut.assertInStdout('multi-line 1')

    def test_individual_character_find(self):
        self.dut.assertInStdout('!')
        self.dut.assertInStdout('@')
        self.dut.assertInStdout('#')
        self.dut.assertInStdout('$')

        # Skip ahead
        self.dut.assertInStdout('*')
        self.dut.assertInStdout('(')

        # Check the same character
        with self.assertRaises(AssertionError):
            self.dut.assertInStdout('(')

    def test_mixed_multi_line(self):
        self.dut.assertInStdout('xxxxx')
        self.dut.assertInStdout('some dummy text')

        with self.assertRaises(AssertionError):
            self.dut.assertInStdout('yyyyy')

    # We want the error message to display the line we last matched, plus two lines before and
    # two lines after
    # The test cases below check that the error message contains useful information for the
    # following test cases:
    # 1. No matching has ocurred yet
    # 2. We just matched the first line of output so there are no lines before to print
    # 3. We just matched the last line of output so there are no lines after to print
    # 4. We just matched the second line of output so there is only one line before to print
    # 5. We just matched the second to last line of output so there's only one line after to print
    # 6. We match a line in the middle, so there are two lines before and after to print

    def test_checker_error_message_1(self):
        with self.assertRaises(AssertionError) as cm:
            self.dut.assertInStdout('I am the very model of a modern major general')

        print(cm.exception)

        expected = 'output 10\noutput 15\noutput 20'
        self.assertIn(
            expected,
            str(cm.exception)
        )
        self.assertEqual(
            expected,
            self.dut.get_nearby_lines()
        )

    def test_checker_error_message_2(self):
        # In this test, our current line is still the 0th line.
        with self.assertRaises(AssertionError) as cm:
            self.dut.assertInStdout('output')
            self.dut.assertInStdout('I am the very model of a modern major general')

        print(cm.exception)

        expected = 'output 10\noutput 15\noutput 20'

        self.assertIn(
            expected,
            str(cm.exception)
        )

        self.assertEqual(
            expected,
            self.dut.get_nearby_lines()
        )

    def test_checker_error_message_3(self):
        with self.assertRaises(AssertionError) as cm:
            self.dut.assertInStdout('!@#')
            self.dut.assertInStdout('I am the very model of a modern major general')

        print(cm.exception)

        expected = 'zzzzz\noutput 20\n!@#$%^&*()'

        self.assertIn(
            expected,
            str(cm.exception)
        )

        self.assertEqual(
            expected,
            self.dut.get_nearby_lines()
        )

    def test_checker_error_message_4(self):
        # Because we asserted on the whole first line.  Our current line is
        # now the 1st line, so we expect to see 0th, 1 (current), 2, and 3 (current + 2) in the
        # error message output
        with self.assertRaises(AssertionError) as cm:
            self.dut.assertInStdout('output 10')
            self.dut.assertInStdout('I am the very model of a modern major general')

        print(cm.exception)

        expected = 'output 10\noutput 15\noutput 20\nmulti-line 1'

        self.assertIn(
            expected,
            str(cm.exception)
        )

        self.assertEqual(
            expected,
            self.dut.get_nearby_lines()
        )

    def test_checker_error_message_4_1(self):
        # Match the middle of the 2nd line - should behave the same as test message_4 above
        with self.assertRaises(AssertionError) as cm:
            self.dut.assertInStdout('output')
            self.dut.assertInStdout('output')
            self.dut.assertInStdout('I am the very model of a modern major general')

        print(cm.exception)

        expected = 'output 10\noutput 15\noutput 20\nmulti-line 1'

        self.assertIn(
            expected,
            str(cm.exception)
        )

        self.assertEqual(
            expected,
            self.dut.get_nearby_lines()
        )

    def test_checker_error_message_5(self):
        with self.assertRaises(AssertionError) as cm:
            self.dut.assertInStdout('output 20')
            self.dut.assertInStdout('output 2')
            self.dut.assertInStdout('I am the very model of a modern major general')

        print(cm.exception)

        expected = 'some dummy text\nzzzzz\noutput 20\n!@#$%^&*()'

        self.assertIn(
            expected,
            str(cm.exception)
        )

        self.assertEqual(
            expected,
            self.dut.get_nearby_lines()
        )

    def test_checker_error_message_6(self):
        with self.assertRaises(AssertionError) as cm:
            self.dut.assertInStdout('multi')
            self.dut.assertInStdout('I am the very model of a modern major general')

        print(cm.exception)

        expected = 'output 15\noutput 20\nmulti-line 1\nmulti-line 2\nmulti-line 3'

        self.assertIn(
            expected,
            str(cm.exception)
        )

        self.assertEqual(
            expected,
            self.dut.get_nearby_lines()
        )
