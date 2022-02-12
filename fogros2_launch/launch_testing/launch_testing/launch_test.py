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

import argparse
import importlib.util
import logging
import os
import sys

from .junitxml import unittestResultsToXml
from .loader import LoadTestsFromPythonModule
from .print_arguments import print_arguments_of_launch_description
from .test_runner import LaunchTestRunner

_logger_ = logging.getLogger(__name__)


def _load_python_file_as_module(test_module_name, python_file_path):
    """Load a given Python launch file (by path) as a Python module."""
    spec = importlib.util.spec_from_file_location(test_module_name, python_file_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def add_arguments(parser):
    """Add arguments to the CLI parser."""
    parser.add_argument('launch_test_file', help='Path to the launch test.')
    parser.add_argument(
        '--package-name', action='store', default=None,
        help='Name of the package the test is in. Useful to aggregate xUnit reports.'
    )
    parser.add_argument(
        '-v', '--verbose', action='store_true', default=False, help='Run with verbose output'
    )
    parser.add_argument(
        '-s', '--show-args', '--show-arguments', action='store_true', default=False,
        help='Show arguments that may be given to the launch test.'
    )
    parser.add_argument(
        'launch_arguments', nargs='*',
        help="Arguments in '<name>:=<value>' format (for duplicates, last one wins)."
    )
    parser.add_argument(
        '--junit-xml', action='store', dest='xmlpath', default=None,
        help='Do write xUnit reports to specified path.'
    )


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Launch integration testing tool.'
    )
    add_arguments(parser)
    return parser, parser.parse_args()


def run(parser, args, test_runner_cls=LaunchTestRunner):
    # Load the test file as a module and make sure it has the required
    # components to run it as a launch test
    _logger_.debug("Loading tests from file '{}'".format(args.launch_test_file))
    if not os.path.isfile(args.launch_test_file):
        # Note to future reader: parser.error also exits as a side effect
        parser.error("Test file '{}' does not exist".format(args.launch_test_file))

    args.launch_test_file = os.path.abspath(args.launch_test_file)
    launch_test_file_basename = os.path.splitext(os.path.basename(args.launch_test_file))[0]
    if not args.package_name:
        args.package_name = launch_test_file_basename
    test_module = _load_python_file_as_module(args.package_name, args.launch_test_file)

    # This is a list of TestRun objects.  Each run corresponds to one launch.  There may be
    # multiple runs if the launch is parametrized
    test_runs = LoadTestsFromPythonModule(
        test_module, name='{}.{}.launch_tests'.format(
            args.package_name, launch_test_file_basename
        )
    )

    # The runner handles sequcing the launches
    runner = test_runner_cls(
        test_runs=test_runs,
        launch_file_arguments=args.launch_arguments,
        debug=args.verbose
    )

    _logger_.debug('Validating test configuration')

    runner.validate()

    if args.show_args:
        # TODO pete: Handle the case where different launch descriptions take different args?
        print_arguments_of_launch_description(
            launch_description=test_runs[0].get_launch_description()
        )
        return

    _logger_.debug('Running integration test')

    results = runner.run()

    _logger_.debug('Done running integration test')

    if args.xmlpath:
        xml_report = unittestResultsToXml(
            test_results=results, name='{}.{}'.format(
                args.package_name, launch_test_file_basename
            )
        )
        xml_report.write(args.xmlpath, encoding='utf-8', xml_declaration=True)

    # There will be one result for every test run (see above where we load the tests)
    if not all(result.wasSuccessful() for result in results.values()):
        return 1
    return 0


def main():
    logging.basicConfig()

    parser, args = parse_arguments()

    if args.verbose:
        _logger_.setLevel(logging.DEBUG)
        _logger_.debug('Running with verbose output')

    try:
        sys.exit(run(parser, args))
    except Exception as e:
        parser.error(e)
        sys.exit(1)


if __name__ == '__main__':
    main()
