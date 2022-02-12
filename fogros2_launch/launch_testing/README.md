# launch_testing

This tool is a framework for launch integration testing. For example:

  * The exit codes of all processes are available to the tests.
  * Tests can check that all processes shut down normally, or with specific exit codes.
  * Tests can fail when a process dies unexpectedly.
  * The stdout and stderr of all processes are available to the tests.
  * The command-line used to launch the processes are avilalbe to the tests.
  * Some tests run concurrently with the launch and can interact with the running processes.

## Quick start example

Start with the `launch_testing` example [`good_proc_launch_test.py`](test/launch_testing/examples/good_proc_launch_test.py).

Run the example by doing:

```sh
launch_test test/launch_testing/examples/good_proc_launch_test.py
```

`launch_test` will launch the nodes found in the `generate_test_description` function, run the tests from the `TestGoodProcess` class, shut down the launched nodes, and then run the tests from the `TestNodeOutput` class.

#### The Launch Description

```python
def generate_test_description():

    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=[path_to_process],
        ),

        # Start tests right away - no need to wait for anything in this example.
        # In a more complicated launch description, we might want this action happen
        # once some process starts or once some other event happens
        launch_testing.actions.ReadyToTest()
    ])
```

The `generate_test_description` function should return a `launch.LaunchDescription` object that launches the system to be tested.

The launch description needs to include a `ReadyToTest` action to signal to the test framework that it's safe to start the active tests.

In the above example, there is no need to delay the start of the tests so the `ReadyToTest` action is a peer to the process under test and will signal to the framework that it's safe to start around the same time the `ExecuteProcess` action is run.

In older style tests, a function called `ready_fn` is declared as an argument to `generate_test_description` and must be plumbed into the launch description with an `OpaqueFunction`.

```python
def generate_test_description(ready_fn):

    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=[path_to_process],
        ),

        # Start tests right away - no need to wait for anything in this example
        launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
    ])
```

#### Active Tests

Any classes that inherit from `unittest.TestCase` and not decorated with the `post_shutdown_test` descriptor will be run concurrently with the proccess under test.
These tests are expected to interact with the running processes in some way.

#### Post-Shutdown Tests

Any classes that inherit `from unittest.TestCase` that are decorated with the `post_shutdown_test` descriptor will be run after the launched processes have been shut down.
These tests have access to the exit codes and the stdout of all of the launched processes, as well as any data created as a side-effect of running the processes.

#### Exit Codes and Standard Out

The `launch_testing` framework automatically adds some member fields to each test case so that the tests can access process output and exit codes.

 - `self.proc_info` - a [ProcInfoHandler object](launch_testing/proc_info_handler.py)
 - `self.proc_output` - an [IoHandler object](launch_testing/io_handler.py)

These objects provide dictionary like access to information about the running processes.
They also contain methods that the active tests can use to wait for a process to exit or to wait for specific output.

## Assertions

The `launch_testing` framework automatically records all stdout from the launched processes as well as the exit codes from any processes that are launched.
This information is made available to the tests via the `proc_info` and `proc_output` object.
These objects can be used by one of several assert methods to check the output or exit codes of the process:

`launch_testing.asserts.assertInStdout(proc_output, msg, process, cmd_args=None, *, strict_proc_matching=True)`

Asserts that a message is found in the stdout of a particular process.

  - `msg`:

    The text to look for in the process standard out

  - `process`:

    Either the process name as a string, or a `launch.actions.ExecuteProcess` object that was used to start the process.
    Pass `None` or an empty string to search all processes.

  - `cmd_args`:

    When looking up processes by process by name, `cmd_args` can be used to disambiguate multiple processes with the same name.

  - `strict_proc_matching`:

    When looking up a process by name, `strict_proc_matching=True` will make it an error to match multiple processes.
    This prevents an assert from accidentally passing if the output came from a different process than the one the user was expecting.

`launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[EXIT_OK], process, cmd_args=None, *, strict_proc_matching=True)`

Asserts that the specified processes exited with a particular exit code.

  - `allowable_exit_codes`:

    A list of allowable exit codes.
    By default `EXIT_OK` (0) plus `EXIT_FORCED` (1) on Windows.
    Other exit codes provided are `EXIT_SIGINT` (130), `EXIT_SIGQUIT` (131), `EXIT_SIGKILL` (137) and `EXIT_SIGSEGV` (139).

  - The `process`, `cmd_args`, and `strict_proc_matching` arguments behave the same way as in `assertInStdout`.
    By default, assert on the exit codes of all processes.

`launch_testing.asserts.assertSequentialStdout(proc_output, process, cmd_args=None)`

Asserts that standard out was seen in a particular order.

  - `process` and `cmd_args`:

    These arguments are the same as in `assertInStdout` and `assertExitCodes`, however it is not possible to match multiple processes because there is no way to determine the order of stdout that came from multiple processes.

Returns a context manager that will check that a series of assertions happen in order.

As an example, consider:

```python
with assertSequentialStdout(self.proc_output, "proc_name") as cm:
    cm.assertInStdout("Loop 1")
    cm.assertInStdout("Loop 2")
    cm.assertInStdout("Loop 3")
```

#### Waiting for Output or Exit Codes

The `ActiveTests` can also call methods that wait for particular output or a particular process to exit or time out.
These asserts are methods on the `proc_output` and `proc_info` objects.

`proc_output.assertWaitFor(msg, process=None, cmd_args=None, *, strict_proc_matching=True, timeout=10)`

  - `msg`, `process`, `cmd_args`, and `strict_proc_matching`:

    These arguments work the same as in other assert methods.
    By default, this method waits on output from any process.

  - `timeout`:

    The amount of time to wait before raising an `AssertionError`.

`proc_info.assertWaitForShutdown(process, cmd_args=None, *, timeout=10)`

  - `process` and `cmd_args`:

    These arguments work the same as in other assertions, but it is not possible to wait on multiple processes to shut down.

  - `timeout`:

    The amount of time to wait before raising an `AssertionError`

## Arguments

`launch_test` uses launch arguments for tests too.

Arguments are declared in the launch description and can be accessed by the test via a `test_args` dictionary that's injected into the tests similar to `proc_info` and `proc_output`.

```sh
launch_test --show-args test/launch_testing/examples/args_launch_test.py
launch_test test/launch_testing/examples/args_launch_test.py dut_arg:=value
```

See the [launch_testing example with arguments](test/launch_testing/examples/args_launch_test.py) for further reference.

## Using CMake

To run launch tests from a CMakeLists.txt file, you'll need to declare a dependency on
`launch_testing_ament_cmake` in your `package.xml`.

Then, in the CMakeLists.txt file, add:

```cmake
find_package(launch_testing_ament_cmake)
add_launch_test(test/name_of_test.test.py)
```

Arguments can be passed to the tests via the CMake function, too:

```cmake
add_launch_test(
  test/test_with_args.test.py
  ARGS "arg1:=foo"
)
```

## Examples

### `hello_world_launch_test.py`

Usage:

```sh
launch_test test/launch_testing/examples/hello_world_launch_test.py
```

This test is a simple example on how to use the ``launch_testing``. 

It launches a process and asserts that it prints "hello_world" to ``stdout`` using ``proc_output.assertWaitFor()``.
Finally, it checks if the process exits normally (zero exit code).

The ``@launch_testing.markers.keep_alive`` decorator ensures that the launch process stays alive long enough for the tests to run.

### `good_proc_launch_test.py`

Usage:

```sh
launch_test test/launch_testing/examples/good_proc_launch_test.py
```

This test checks a process called good_proc.py (source found in the [example_processes folder](example_processes)).
good_proc.py is a simple python process that prints "Loop 1, Loop2, etc. every second until it's terminated with ctrl+c.
The test will launch the process, wait for a few loops to complete by monitoring stdout, then terminate the process
and run some post-shutdown checks.

The pre-shutdown tests check that "Loop 1, Loop 2, Loop 3, Loop 4"
are all printed to stdout.  Once this test finishes, the process under test is shut down

After shutdown, we run a similar test that checks more output, and also checks the
order of the output.  `test_out_of_order` demonstrates that the `assertSequentialStdout`
context manager is able to detect out of order stdout.

### `terminating_proc_launch_test.py`

Usage:

```sh
launch_test test/launch_testing/examples/terminating_proc_launch_test.py
```

This test checks proper functionality of the _terminating\_proc_ example (source found in the [example_processes folder](example_processes)).

### `args_launch_test.py`

Usage to view the arguments:

```sh
launch_test test/launch_testing/examples/args_launch_test.py --show-args
```

Usage to run the test:

```sh
launch_test test/launch_testing/examples/args_launch_test.py dut_arg:=hey
```

This example shows how to pass arguments into a launch test.
The arguments are made available in the launch description via a `launch.substitutions.LaunchConfiguration`.
The arguments are made available to the test cases via a `self.test_args` dictionary

This example will fail if no arguments are passed.

### `context_launch_test.py`

Usage:

```sh
launch_test test/launch_testing/examples/context_launch_test.py
```

This example shows how the `generate_test_description` function can return a tuple where the second
item is a dictionary of objects that will be injected into the individual test cases.
Tests that wish to use elements of the test context can add arguments with names matching the keys of the dictionary.
