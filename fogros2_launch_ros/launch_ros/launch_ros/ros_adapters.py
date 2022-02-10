# Copyright 2018-2020 Open Source Robotics Foundation, Inc.
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

"""Module containing ROS specific adapters and their initialization."""

import os
import threading
from typing import List
from typing import Optional

import launch
import launch.events

import rclpy
from rclpy.executors import SingleThreadedExecutor


class ROSAdapter:
    """Wraps rclpy API to ease ROS node usage in `launch_ros` actions."""

    def __init__(
        self,
        *,
        argv: Optional[List[str]] = None,
        autostart: bool = True
    ):
        """
        Construct adapter.

        :param: argv List of global arguments for rclpy context initialization.
        :param: autostart Whether to start adapter on construction or not.
        """
        # Do not use `None` here, as `rclpy.init` will use `sys.argv` in that case.
        self.__argv = [] if argv is None else argv
        self.__ros_context = None
        self.__ros_node = None
        self.__ros_executor = None
        self.__is_running = False

        if autostart:
            self.start()

    def start(self):
        """Start ROS adapter."""
        if self.__is_running:
            raise RuntimeError('Cannot start a ROS adapter that is already running')
        self.__ros_context = rclpy.Context()
        rclpy.init(args=self.__argv, context=self.__ros_context)
        self.__ros_node = rclpy.create_node(
            'launch_ros_{}'.format(os.getpid()),
            context=self.__ros_context
        )
        self.__ros_executor = SingleThreadedExecutor(context=self.__ros_context)
        self.__is_running = True

        self.__ros_executor_thread = threading.Thread(target=self._run)
        self.__ros_executor_thread.start()

    def _run(self):
        try:
            self.__ros_executor.add_node(self.__ros_node)
            while self.__is_running:
                # TODO(wjwwood): switch this to `spin()` when it considers
                #   asynchronously added subscriptions.
                #   see: https://github.com/ros2/rclpy/issues/188
                self.__ros_executor.spin_once(timeout_sec=1.0)
        except KeyboardInterrupt:
            pass
        finally:
            self.__ros_executor.remove_node(self.__ros_node)

    def shutdown(self):
        """Shutdown ROS adapter."""
        if not self.__is_running:
            raise RuntimeError('Cannot shutdown a ROS adapter that is not running')
        self.__is_running = False
        self.__ros_executor_thread.join()
        self.__ros_node.destroy_node()
        rclpy.shutdown(context=self.__ros_context)

    @property
    def argv(self):
        return self.__argv

    @property
    def ros_context(self):
        return self.__ros_context

    @property
    def ros_node(self):
        return self.__ros_node

    @property
    def ros_executor(self):
        return self.__ros_executor


def get_ros_adapter(context: launch.LaunchContext):
    """
    Get the ROS adapter managed by the given launch context.

    If no adapter is found, one will be created.

    This function is reentrant but concurrent calls on the
    same `context` are not safe.
    """
    if not hasattr(context.locals, 'ros_adapter'):
        ros_adapter = ROSAdapter()
        context.extend_globals({'ros_adapter': ros_adapter})
        context.register_event_handler(launch.event_handlers.OnShutdown(
            on_shutdown=lambda *args, **kwargs: ros_adapter.shutdown()
        ))
    return context.locals.ros_adapter


def get_ros_node(context: launch.LaunchContext):
    """
    Get the ROS node managed by the given launch context.

    If no node is found, one will be created.

    This function is reentrant but concurrent calls on the
    same `context` are not safe.
    """
    return get_ros_adapter(context).ros_node
