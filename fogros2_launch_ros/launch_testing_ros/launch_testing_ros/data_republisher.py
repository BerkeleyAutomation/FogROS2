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


class DataRepublisher:
    """Republish mesasges with a transform function applied."""

    def __init__(self, node, listen_topic, publish_topic, msg_type, transform_fn):
        """
        Create a DataRepublisher.

        :param node: A rclpy node that will run the publisher and subscriber

        :param listen_topic: The topic to listen for incoming messages on

        :param publish_topic: The topic to republish messages on

        :param msg_type: The type of ROS msg to receive and republish

        :param transfor_fn: A function that takes one mesasge and returns a new message to
        republish or None to drop the message
        """
        self.__num_received = 0
        self.__republished_list = []

        self.__node = node
        self.__subscriber = node.create_subscription(
            msg_type,
            listen_topic,
            self.__cb,
            10
        )

        self.__publisher = node.create_publisher(
            msg_type,
            publish_topic,
            10
        )

        self.__transform_fn = transform_fn

    def shutdown(self):
        """Stop republishing messages."""
        self.__node.destroy_subscription(self.__subscriber)
        self.__node.destroy_publisher(self.__publisher)

    def get_num_received(self):
        """Get the number of messages received on the listen_topic."""
        return self.__num_received

    def get_num_republished(self):
        """
        Get the number of messages published on publish_topic.

        This may be lower than get_num_received if the transform_fn indicated a message
        should be dropped
        """
        return len(self.__republished_list)

    def get_republished(self):
        """Get a list of all of the transformed messages republished."""
        return self.__republished_list

    def __cb(self, msg):
        self.__num_received += 1
        repub = self.__transform_fn(msg)

        if repub:
            self.__republished_list.append(msg)
            self.__publisher.publish(repub)
