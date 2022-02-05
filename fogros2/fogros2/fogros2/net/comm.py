
import paramiko
from scp import SCPClient

class SCP_Client():
    def __init__(self, ip, ssh_key):
        self.ip = ip
        self.ssh_key = paramiko.RSAKey.from_private_key_file("/home/ubuntu/" + ssh_key + ".pem")
        self.ssh_client = paramiko.SSHClient()
        self.connect()


    def connect(self):
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh_client.connect(hostname=self.ip, username="ubuntu", pkey=private_key, look_for_keys=False)

    def send_file(self, src_path, dst_path):
        with SCPClient(ssh_client.get_transport()) as scp:
            scp.put(src_path, dst_path)

    def execute_cmd(self, cmd):
        stdin, stdout, stderr = ssh_client.exec_command(cmd, get_pty=True)
        for line in iter(stdout.readline, ""):
            print(line, end="")