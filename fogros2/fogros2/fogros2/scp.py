
import paramiko
from scp import SCPClient
import logging
from time import sleep

# ec2 console coloring
CRED = '\033[91m'
CEND = '\033[0m'

class SCP_Client():
    
    def __init__(self, ip, ssh_key_path):
        self.ip = ip
        self.ssh_key = paramiko.RSAKey.from_private_key_file(ssh_key_path)
        self.ssh_client = paramiko.SSHClient()
        self.logger = logging.getLogger(__name__)


    def connect(self, keep_trying = True):
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        while keep_trying:
            try:
                self.ssh_client.connect(hostname=self.ip, username="ubuntu", pkey=self.ssh_key, look_for_keys=False)
                keep_trying = False
            except Exception as e:
                self.logger.warn("Exception occured when connecting scp" + str(e) + ", retrying...")
                sleep(1)
        self.logger.info("SCP connection succeeds!")
                

    def send_file(self, src_path, dst_path):
        with SCPClient(self.ssh_client.get_transport()) as scp:
            scp.put(src_path, dst_path)

    def execute_cmd(self, cmd):
        stdin, stdout, stderr = self.ssh_client.exec_command(cmd, get_pty=True)
        for line in iter(stdout.readline, ""):
            print("ec2: " + CRED + line + CEND, end="")
        return stdin, stdout, stderr