from ros2cli.verb import VerbExtension
import os
import json

class ListVerb(VerbExtension):
    def main(self, *, args):
        fogros_working_dir = "/tmp/fogros/"
        instances = os.listdir(fogros_working_dir)
        for instance in instances:
            print("======" +instance +"======")
            pwd = fogros_working_dir + instance
            with open(pwd + "/info") as f:
                parsed = json.loads(f.read())
                print(json.dumps(parsed, indent=4))