import json
import os

from ros2cli.verb import VerbExtension


class SSHVerb(VerbExtension):
    def add_arguments(self, parser, cli_name):
        parser.add_argument("--name", "-n", type=str, nargs=1, help="Select FogROS instance name to connect")

        parser.add_argument(
            "--user", "-u", type=str, nargs="?", default="ubuntu", help="User name of the remote SSH instance"
        )

    def main(self, *, args):
        self.fogros_working_dir = "/tmp/fogros/"
        pwd = self.fogros_working_dir + args.name[0]
        with open(pwd + "/info") as f:
            instance_info = json.loads(f.read())

        ssh_key_path = pwd + "/" + [n for n in os.listdir(pwd) if n.endswith(".pem")][0]
        os.system(f"chmod 400 {ssh_key_path}")
        public_ip = instance_info["public_ip"]

        import subprocess
        import sys
        import time

        input_buffer = sys.stdin  # a buffer to get the user input from
        output_buffer = sys.stdout  # a buffer to write rasa's output to
        ssh_cmd = f"ssh -i {ssh_key_path} {args.user}@{public_ip}"
        print(ssh_cmd)
        proc = subprocess.Popen(
            ssh_cmd,
            shell=True,
            stdin=subprocess.PIPE,
            stdout=output_buffer,
            stderr=output_buffer,
            universal_newlines=True,
        )

        while True:  # run a main loop
            time.sleep(0.5)  # give some time for `rasa` to forward its STDOUT
            print("$: ", end="", file=output_buffer, flush=True)  # print the input prompt
            print(input_buffer.readline(), file=proc.stdin, flush=True)  # forward the user input
