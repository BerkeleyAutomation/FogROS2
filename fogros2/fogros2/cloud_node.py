

from launch_ros.actions import Node


class CloudNode(Node):
    def __init__(self,
                 machine,
                 **kwargs
                 ):
        super().__init__(**kwargs)
        self.machine = machine

    def __getstate__(self):
        # workaround to make pickle not serializing self.machine 
        state = self.__dict__.copy()
        del state["machine"]
        return state

    def get_unique_id(self):
        return self.machine.get_name()