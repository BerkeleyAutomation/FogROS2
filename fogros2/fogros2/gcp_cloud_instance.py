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
import uuid

from .cloud_instance import CloudInstance

from .util import extract_bash_column


class GCPCloudInstance(CloudInstance):
    """GCP Implementation of CloudInstance."""

    def __init__(
            self,
            project_id,
            ami_image='projects/ubuntu-os-cloud/global/images/ubuntu-2204-jammy-v20220712a',
            zone="us-central1-a",
            machine_type="e2-medium",
            disk_size=10,
            **kwargs,
    ):
        super().__init__(**kwargs)
        self.cloud_service_provider = "GCP"

        id_ = str(uuid.uuid4())[0:8]
        self._name = f'fog-{id_}-{self._name}'

        self.zone = zone
        self.type = machine_type
        self.compute_instance_disk_size = disk_size  # GB
        self.gcp_ami_image = ami_image

        self._working_dir = os.path.join(self._working_dir_base, self._name)
        os.makedirs(self._working_dir, exist_ok=True)

        self._project_id = project_id

        # after config
        self._ssh_key = None

        self.create()

    def create(self):
        self.logger.info(f"Creating new GCP Compute Engine instance with name {self._name}")
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
        info_dict["disk_size"] = self.compute_instance_disk_size
        info_dict["compute_instance_id"] = self._name
        if flush_to_disk:
            with open(os.path.join(self._working_dir, "info"), "w+") as f:
                json.dump(info_dict, f)
        return info_dict

    def create_compute_engine_instance(self):
        os.system(f'gcloud config set project {self._project_id}')

        result = subprocess\
            .check_output(
                    f'gcloud compute instances create {self._name} '
                    f'--project={self._project_id} --zone={self.zone} --machine-type={self.type} '
                    '--network-interface=network-tier=PREMIUM,subnet=default '
                    '--maintenance-policy=MIGRATE --provisioning-model=STANDARD '
                    '--scopes=https://www.googleapis.com/auth/devstorage.read_only,'
                    'https://www.googleapis.com/auth/logging.write,'
                    'https://www.googleapis.com/auth/monitoring.write,'
                    'https://www.googleapis.com/auth/servicecontrol,'
                    'https://www.googleapis.com/auth/service.management.readonly,'
                    'https://www.googleapis.com/auth/trace.append '
                    '--create-disk=auto-delete=yes,'
                    'boot=yes,'
                    f'device-name={self._name},'
                    f'image={self.gcp_ami_image},'
                    'mode=rw,'
                    f'size={self.compute_instance_disk_size},'
                    f'type=projects/{self._project_id}/zones/{self.zone}/diskTypes/pd-balanced '
                    '--no-shielded-secure-boot '
                    '--shielded-vtpm '
                    '--shielded-integrity-monitoring '
                    '--reservation-affinity=any', shell=True).decode()

        # Grab external IP
        ip = extract_bash_column(result, 'EXTERNAL_IP')

        # Verifies the response was an ip
        if len(ip.split('.')) != 4:
            raise Exception(f'Error creating instance: {ip}')

        self._ip = ip

        # Generate SSH keys
        os.system(f"printf '\n\n' | gcloud compute ssh {self._name} --zone {self.zone}")

        user = subprocess.check_output('whoami', shell=True).decode().strip()

        # Username
        self._username = (open(f'/home/{user}/.ssh/google_compute_engine.pub').
                          read()).split(' ')[-1].strip().split('@')[0]

        self._ssh_key_path = f'/home/{user}/.ssh/google_compute_engine'
        self._is_created = True

        self.logger.info(
            f"Created {self.type} instance named {self._name} "
            f"with id {self._name} and public IP address {self._ip}"
        )
