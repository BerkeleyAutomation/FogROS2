import jmespath
import boto3
import configparser
import os


def get_all_instances():
    config = configparser.ConfigParser()
    config.read(os.path.join(os.path.expanduser('~'), '.aws/credentials'))
    ec2_client = boto3.client('ec2')
    regions = [region['RegionName']
                for region in ec2_client.describe_regions()['Regions']]

    result = []
    for profile in config.sections():
        for region in regions:
            current_session = boto3.Session(profile_name = profile, region_name = region)
            client = current_session.client('ec2')
            response = client.describe_instances()
            output = jmespath.search("Reservations[].Instances[].[NetworkInterfaces[0].OwnerId, InstanceId, InstanceType, \
                State.Name, Placement.AvailabilityZone, PrivateIpAddress, PublicIpAddress, KeyName, [Tags[?Key=='Name'].Value] [0][0]]", response)
            if output: result.append(output)
            
    return result

def get_running_instances():
    return [[instance for instance in region if "running" in instance] for region in get_all_instances()]
