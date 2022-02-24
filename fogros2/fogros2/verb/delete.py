from ros2cli.verb import VerbExtension
import os
import json
from fogros2 import AWS
import shutil

class DeleteVerb(VerbExtension):

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '--all', '-A', action='store_true',  default=False,
            help="Delete All existing FogROS instances")
        parser.add_argument(
            '--name', '-n',  type=str, nargs=1,
            help="Select FogROS instance name to delete")

    def delete_instance(self, instance):
        pwd = self.fogros_working_dir + instance
        with open(pwd + "/info") as f:
            instance_info = json.loads(f.read())

        if instance_info["cloud_service_provider"] == "AWS":
            print("Terminating EC2 instance")
            AWS.delete(instance_info["ec2_instance_id"], instance_info["ec2_region"])

        print("Removing Instance Dir")
        shutil.rmtree(pwd)

        print(f"Delete {instance} successfully!")

    def main(self, *, args):
        self.fogros_working_dir = "/tmp/fogros/"

        if args.all == True:
            instances = os.listdir(self.fogros_working_dir)
            for instance in instances:
                print("======" +instance +"======")
                self.delete_instance(instance)
        else:
            self.delete_instance(args.name[0])

