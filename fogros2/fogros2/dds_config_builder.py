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

import lsb_release

ubuntu_release = lsb_release.get_os_release()["RELEASE"]


class DDSConfigBuilder:
    def __init__(self, ip_addresses):
        """
        Construct new DDSConfigBuilder.

        @param:
            ip_addresses: a list of ip addresses of cloud instances/VPN peers
        """
        self.ip_addresses = ip_addresses
        self.config_save_path = None
        self.env_cmd = None

    def generate_config_file(self):
        pass


class CycloneConfigBuilder(DDSConfigBuilder):
    def __init__(self, ip_addresses):
        super().__init__(ip_addresses)
        self.config_save_path = "/tmp/cyclonedds.xml"
        self.env_cmd = (
            "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            "export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml"
        )

    def generate_config_file(self):
        if ubuntu_release == "20.04":
            interfaces = """
        <NetworkInterfaceAddress>wg0</NetworkInterfaceAddress>
            <AllowMulticast>false</AllowMulticast>
            """
        else:
            interfaces = """
            <Interfaces>
                <NetworkInterface name="wg0"/>
            </Interfaces>
            """

        xmlvals = (
            'xmlns="https://cdds.io/config" '
            'xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
            'xsi:schemaLocation="https://cdds.io/config '
            "https://raw.githubusercontent.com/eclipse-cyclonedds/"
            'cyclonedds/master/etc/cyclonedds.xsd"'
        )

        template = f"""
        <?xml version="1.0" encoding="UTF-8" ?>
        <CycloneDDS {xmlvals}>
            <Domain id="any">
                <General>{interfaces}</General>
                <Discovery>
                    <Peers>
                        <Peer address="10.0.0.1"/>
                        <Peer address="10.0.0.2"/>
                    </Peers>
                    <ParticipantIndex>auto</ParticipantIndex>
                </Discovery>
            </Domain>
        </CycloneDDS>
        """

        with open(self.config_save_path, "w+") as f:
            f.write(template)
