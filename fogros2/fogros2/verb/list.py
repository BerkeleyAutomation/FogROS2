import json
import os

from ros2cli.verb import VerbExtension
from fogros2.util import instance_dir

class ListVerb(VerbExtension):
    def main(self, *, args):
        fogros_working_dir = instance_dir()
        instances = os.listdir(fogros_working_dir)
        for instance in instances:
            print("======" + instance + "======")
            pwd = os.path.join(fogros_working_dir, instance)
            with open(os.path.join(pwd, "info")) as f:
                parsed = json.loads(f.read())
                print(json.dumps(parsed, indent=4))
