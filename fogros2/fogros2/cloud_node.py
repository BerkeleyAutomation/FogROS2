

from launch_ros.actions import Node


class CloudNode(Node):
    def __init__(self,
                 machine,
                 **kwargs
                 ):
        super().__init__(**kwargs)
        self.machine = machine

    def get_unique_id(self):
        return self.machine.get_name()