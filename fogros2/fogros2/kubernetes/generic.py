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
# licensing opportunities. IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY
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
import textwrap

from ..util import extract_bash_column

from ..cloud_instance import CloudInstance

class KubeInstance(CloudInstance):
    """Generic Kubernetes CloudInstance"""

    def __init__(
        self,
        container_image="docker.io/njha/fogros2_base",
        zone="us-central1-a",
        mcpu=0,
        mb=0,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.cloud_service_provider = "ONPREM"

        id_ = str(uuid.uuid4())[0:8]
        self._name = f"fog-{id_}-{self._name}"

        self.zone = zone
        self.type = f"{mcpu}mx{mb}Mb"
        self.container_image = container_image

        self._mcpu = mcpu
        self._mmb = mb

        self._working_dir = os.path.join(self._working_dir_base, self._name)
        os.makedirs(self._working_dir, exist_ok=True)

        # after config
        self._ssh_key = None

        self.create()

    def create(self):
        self.logger.info(f"Creating new ROS node on Kubernetes with name {self._name}")
        self.create_compute_engine_instance()
        self.info(flush_to_disk=True)
        self.connect()
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
    
    def force_start_vpn(self):
        return False

    def create_service_pair(self, pub_key_path: str):
        # Instance Selector
        selector = {
            "edu.berkeley.autolab.fogros/instance": self._name,
        }

        # SSH Service
        ssh_config: dict = {
            "apiVersion": "v1",
            "kind": "Service",
            "metadata": {"name": f"{self._name}-ssh"},
            "spec": {
                "type": "LoadBalancer",
                "ports": [
                    {
                        "port": 22,
                        "targetPort": 22,
                        "name": "ssh",
                        "protocol": "TCP",
                    }
                ],
                "selector": selector,
            },
        }
        # VPN Service
        vpn_config: dict = {
            "apiVersion": "v1",
            "kind": "Service",
            "metadata": {
                "name": f"{self._name}-vpn",
            },
            "spec": {
                "type": "LoadBalancer",
                "ports": [
                    {
                        "port": 51820,
                        "targetPort": 51820,
                        "name": "wg",
                        "protocol": "UDP",
                    }
                ],
                "selector": selector,
            },
        }

        # Runner Pod
        pod_resources = {
            "memory": f"{self._mmb}Mi",
            "cpu": f"{self._mcpu}m",
        }
        pod_config: dict = {
            "apiVersion": "v1",
            "kind": "Pod",
            "metadata": {
                "name": self._name,
                "labels": selector,
            },
            "spec": {
                "restartPolicy": "Never",
                "containers": [
                    {
                        "name": self._name,
                        "image": self.container_image,
                        "imagePullPolicy": "Always",
                        "securityContext": {
                            "capabilities": {
                                "add": ["NET_ADMIN", "CAP_SYS_ADMIN"],
                            },
                            "privileged": True,
                        },
                        "resources": {
                            "requests": pod_resources,
                            "limits": pod_resources,
                        },
                        "env": [
                            {
                                "name": "SSH_PUBKEY",
                                "value": open(pub_key_path).read().strip(),
                            },
                        ],
                        "command": ["/bin/bash"],
                        "args": [
                            "-c",
                            textwrap.dedent(
                                """
                                echo $SSH_PUBKEY >> '/home/ubuntu/.ssh/authorized_keys' &&\\
                                chmod -R u=rwX '/home/ubuntu/.ssh' &&\\
                                chown -R 'ubuntu:ubuntu' '/home/ubuntu/.ssh' &&\\
                                service ssh restart &&\\
                                sleep infinity
                                """,
                            ),
                        ],
                    },
                ],
            },
        }

        # TODO: Use the Kubernetes API (pypy/kubernetes) instead of shelling out to kubectl.
        for config in [vpn_config, ssh_config, pod_config]:
            file = tempfile.NamedTemporaryFile()
            open(file.name, "w").write(json.dumps(config))
            self.logger.debug(
                f"Creating {config['kind']}/{config['metadata']['name']}..."
            )
            os.system(f"kubectl apply -f {file.name}")
            file.close()

        # Poll until services are live...
        while True:
            if (
                not "Running"
                in subprocess.check_output(
                    f"kubectl get pod {self._name}", shell=True
                ).decode()
                or "pending"
                in subprocess.check_output(
                    f'kubectl get service {ssh_config["metadata"]["name"]}', shell=True
                ).decode()
                or "pending"
                in subprocess.check_output(
                    f'kubectl get service {vpn_config["metadata"]["name"]}', shell=True
                ).decode()
            ):
                self.logger.info("Some services still creating...")
                time.sleep(5)
            else:
                break

        self.logger.debug("Extracting IPs")
        ssh_data = subprocess.check_output(
            f'kubectl get service {ssh_config["metadata"]["name"]}', shell=True
        ).decode()
        vpn_data = subprocess.check_output(
            f'kubectl get service {vpn_config["metadata"]["name"]}', shell=True
        ).decode()

        return extract_bash_column(ssh_data, "EXTERNAL-IP"), extract_bash_column(vpn_data, "EXTERNAL-IP")
    
    def create_compute_engine_instance(self):
        # Generate SSH keys
        self._ssh_key_path = os.path.expanduser(f"~/.ssh/{self._name}")
        os.system(f"ssh-keygen -f {self._ssh_key_path} -q -N ''")

        ssh_ip, vpn_ip = self.create_service_pair(f"{self._ssh_key_path}.pub")

        self._ip = ssh_ip
        self._vpn_ip = vpn_ip

        self._username = "ubuntu"
        self._is_created = True

        self.logger.info(
            f"Created {self.type} instance named {self._name} "
            f"with id {self._name} and public IP address {self._ip} with VPN IP {self._vpn_ip}"
        )
