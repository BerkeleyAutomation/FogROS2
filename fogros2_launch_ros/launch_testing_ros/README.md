# launch\_testing\_ros

## Examples

### `talker_listener_launch_test.py`

Usage:

```sh
launch_test test/examples/talker_listener_launch_test.py
```

This test launches the talker and listener example nodes from demo\_nodes\_py and interacts
with them via their ROS interfaces.  Remapping rules are used so that one of the tests can sit in
between the talker and the listener and change the data on the fly.

### `check_node_launch_test.py`

Usage:

```sh
launch_test test/examples/check_node_launch_test.py
```

There might be situations where nodes, once launched, take some time to actually start and we need to wait for the node to start to perform some action.
We can simulate this using ``launch.actions.TimerAction``. This example shows one way to detect when a node has been launched.
We delay the launch by 5 seconds, and wait for the node to start with a timeout of 8 seconds.

### `check_msgs_launch_test.py`

Usage:

```sh
launch_test test/examples/check_msgs_launch_test.py
```

Consider a problem statement where you need to launch a node and check if messages are published on a particular topic.
This example demonstrates how to do that, using a talker node.
It uses the ``Event`` object to end the test as soon as the first message is received on the chatter topic, with a timeout of 5 seconds.

### `set_param_launch_test.py`

Usage:

```sh
launch_test test/examples/set_param_launch_test.py
```

This example demonstrates how to launch a node, set a parameter in it and check if that was successful.

#### test\_fuzzy\_data
This test gives an example of what a test that fuzzes data might look like.  A ROS subscriber
and publisher pair encapsulated in a `DataRepublisher` object changes the string "Hello World" to
"Aloha World" as it travels between the talker and the listener.

#### test\_listener\_receives
This test publishes unique messages on the `/chatter` topic and asserts that the same messages
go to the stdout of the listener node

#### test\_talker\_transmits
This test subscribes to the remapped `/talker_chatter` topic and makes sure the talker node also
writes the data it's transmitting to stdout
