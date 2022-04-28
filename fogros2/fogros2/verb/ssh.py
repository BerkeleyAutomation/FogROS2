import json
import os

from ros2cli.verb import VerbExtension
from fogros2.util import instance_dir

class SSHVerb(VerbExtension):
    def add_arguments(self, parser, cli_name):
        parser.add_argument("--name", "-n", type=str, nargs=1, help="Select FogROS instance name to connect")

        parser.add_argument(
            "--user", "-u", type=str, nargs="?", default="ubuntu", help="User name of the remote SSH instance"
        )

    def main(self, *, args):
        pwd = os.path.join(instance_dir(), args.name[0])
        with open(os.path.join(pwd, "info")) as f:
            instance_info = json.loads(f.read())

        ssh_key_path = os.path.join(pwd, [n for n in os.listdir(pwd) if n.endswith(".pem")][0])
        os.system(f"chmod 400 {ssh_key_path}")
        public_ip = instance_info["public_ip"]

        # execvp replaces the current process with the argument.  Here
        # we just open ssh directly.
        os.execvp("ssh", ("ssh", "-i", ssh_key_path, f"{args.user}@{public_ip}"))
