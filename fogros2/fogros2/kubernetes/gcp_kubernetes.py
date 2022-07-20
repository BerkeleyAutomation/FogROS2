# Copyright 2022 The Regents of the University of California (Regents)
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
#
# Copyright ©2022. The Regents of the University of California (Regents).
# All Rights Reserved. Permission to use, copy, modify, and distribute this
# software and its documentation for educational, research, and not-for-profit
# purposes, without fee and without a signed licensing agreement, is hereby
# granted, provided that the above copyright notice, this paragraph and the
# following two paragraphs appear in all copies, modifications, and
# distributions. Contact The Office of Technology Licensing, UC Berkeley, 2150
# Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620, (510) 643-7201,
# otl@berkeley.edu, http://ipira.berkeley.edu/industry-info for commercial
# licensing opportunities. IN NO EVEpNT SHALL REGENTS BE LIABLE TO ANY PARTY
# FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES,
# INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE. REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY,
# PROVIDED HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
# MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

import json
import os

import subprocess
import time
import uuid
import tempfile

from ..util import extract_bash_column

from ..cloud_instance import CloudInstance


class GCPKubeInstance(CloudInstance):
    """Kubernetes Implementation of CloudInstance."""

    def __init__(
            self,
            container_image='ubuntu',
            zone="us-central1-a",
            mcpu=500,
            mb=200,
            **kwargs,
    ):
        super().__init__(**kwargs)
        self.cloud_service_provider = "GKE"

        id_ = str(uuid.uuid4())[0:8]
        self._name = f'fog-{id_}-{self._name}'

        self.zone = zone
        self.type = f'{mcpu}mx{mb}Mb'
        self.container_image = container_image

        self._mcpu = mcpu
        self._mmb = mb

        self._working_dir = os.path.join(self._working_dir_base, self._name)
        os.makedirs(self._working_dir, exist_ok=True)

        # after config
        self._ssh_key = None

        self.create()

    def create(self):
        self.logger.info(f"Creating new Kubernetes Pod with name {self._name}")
        self.create_compute_engine_instance()
        self.info(flush_to_disk=True)
        self.connect()
        self.install_ros()
        self.install_colcon()
        self.install_cloud_dependencies()
        self.push_ros_workspace()
        self.info(flush_to_disk=True)
        self._is_created = True

    def info(self, flush_to_disk=True):
        info_dict = super().info(flush_to_disk)
        info_dict["compute_region"] = self.zone
        info_dict["compute_instance_type"] = self.type
        info_dict["compute_instance_id"] = self._name
        if flush_to_disk:
            with open(os.path.join(self._working_dir, "info"), "w+") as f:
                json.dump(info_dict, f)
        return info_dict

    def create_service_pair(self, kube_wd: str, pub_key_path: str):
        ssh_config: dict = json.loads(open(f'{kube_wd}/ssh.json').read())
        vpn_config: dict = json.loads(open(f'{kube_wd}/vpn.json').read())
        pod_config: dict = json.loads(open(f'{kube_wd}/pod.json').read())

        # Configure the pod
        pod_config['spec']['containers'][0].image = self.container_image
        pod_config['spec']['containers'][0].name = self._name
        pod_config['spec']['containers'][0]['resources']['requests']['memory'] = str(self._mmb) + 'Mi'
        pod_config['spec']['containers'][0]['resources']['requests']['cpu'] = str(self._mcpu) + "m"

        pod_config['spec']['containers'][0]['resources']['limits']['memory'] = str(self._mmb) + "Mi"
        pod_config['spec']['containers'][0]['resources']['limits']['cpu'] = str(self._mcpu) + "m"

        pod_config['spec']['containers'][0].env.append({
            'name': "SSH_PUBKEY",
            'value': open(pub_key_path).read()
        })

        pod_config['metadata'] = {
            'labels': {
                'app': self._name
            }
        }

        # Configure SSH
        ssh_config['metadata']['name'] = f'{self._name}-ssh'
        ssh_config['selector']['app'] = self._name

        # Configure VPN
        vpn_config['metadata']['name'] = f'{self._name}-vpn'
        vpn_config['selector']['app'] = self._name

        ssh_tempfile = tempfile.NamedTemporaryFile()
        ssh_tempfile.write(json.dumps(ssh_config))
        vpn_tempfile = tempfile.NamedTemporaryFile()
        vpn_tempfile.write(json.dumps(vpn_config))
        pod_tempfile = tempfile.NamedTemporaryFile()
        pod_tempfile.write(json.dumps(pod_config))

        print("Creating SSH...")
        os.system(f"kubectl apply -f {ssh_tempfile.name}")
        print("Creating VPN...")
        os.system(f"kubectl apply -f {vpn_tempfile.name}")
        print("Creating Pod")
        os.system(f"kubectl apply -f {pod_tempfile.name}")

        ssh_tempfile.close()
        vpn_tempfile.close()
        pod_tempfile.close()

        # Wait until all services are live
        while True:
            if 'ContainerCreating' in \
                    subprocess.check_output(f'kubectl get pod {self._name}', shell=True).decode() or \
                    'pending' in \
                    subprocess.check_output(f'kubectl get service {vpn_config["metadata"]["name"]}', shell=True). \
                            decode() \
                    or 'pending' in \
                    subprocess.check_output(f'kubectl get service {ssh_config["metadata"]["name"]}', shell=True). \
                            decode():
                print("Some services still creating...")
                time.sleep(1)
            else:
                break

        # Setup ssh
        command = f'useradd "{self._username}" -m -s /bin/bash && ' \
                  f'mkdir "/home/{self._username}/.ssh" && ' \
                  f'echo "$SSH_PUBKEY" >> "/home/{self._username}/.ssh/authorized_keys" && ' \
                  f'chmod -R u=rwX "/home/{self._username}/.ssh" && ' \
                  f'chown -R "{self._username}:{self._username}" "/home/{self._username}/.ssh" && ' \
                  f'echo "{self._username} ALL=(ALL:ALL) NOPASSWD: ALL" > /etc/sudoers.d/shade && ' \
                  f'service ssh restart && ' \
                  'sleep infinity'

        print("Enabling SSH")
        subprocess.check_output(f'kubectl exec strategy -- /bin/bash -c "{command}"', shell=True)

        print("Extracting IPs")
        ssh_data = subprocess.check_output(f'kubectl get service {ssh_config["metadata"]["name"]}', shell=True).decode()
        vpn_data = subprocess.check_output(f'kubectl get service {vpn_config["metadata"]["name"]}', shell=True).decode()

        return extract_bash_column(ssh_data, 'EXTERNAL-IP'), extract_bash_column(vpn_data, 'EXTERNAL-IP')

    def create_compute_engine_instance(self):
        # Generate SSH keys
        user = subprocess.check_output('whoami', shell=True).decode().strip()
        kube_wd = f'/home/{user}/fog_ws/src/FogROS2/fogros2/fogros2/kubernetes'

        self._ssh_key = f'/home/{user}/.ssh/{self._name}'
        os.system(f"ssh-keygen -f {self._ssh_key} -q -N ''")

        ssh_ip, vpn_ip = self.create_service_pair(kube_wd, f'{self._ssh_key}.pub')

        self._ip = ssh_ip
        self._vpn_ip = vpn_ip

        self.logger.info(
            f"Created {self.type} instance named {self._name} "
            f"with id {self._name} and public IP address {self._ip} with VPN ip {self._vpn_ip}"
        )

        self._is_created = True
