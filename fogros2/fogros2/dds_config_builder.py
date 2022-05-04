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

import lsb_release

ubuntu_release = lsb_release.get_os_release()["RELEASE"]


class DDSConfigBuilder:
    def __init__(self, ip_addresses):
        """
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
            "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml"
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

        template = f"""
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
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
