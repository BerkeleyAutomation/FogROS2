

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
        self.env_cmd = "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml"


    def generate_config_file(self):
        template = '''
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSch\
ema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercont\
ent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>wg0</NetworkInterfaceAddress>
            <AllowMulticast>false</AllowMulticast>
        </General>
        <Discovery>
            <Peers>
                <Peer address="10.0.0.2"/>
            </Peers>
            <ParticipantIndex>auto</ParticipantIndex>
        </Discovery>
    </Domain>
</CycloneDDS>
        '''
        with open(self.config_save_path, "w+") as f:
            f.write(template)

