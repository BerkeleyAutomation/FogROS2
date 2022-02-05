
import boto3
import random

class AWS:
    def __init__(self,
                 region = "us-west-1"
                 ):
        self.region = region
        self.ssh_key = None
        self.ec2_key_name = None
        self.ec2_resource_manager = None
        self.ec2_instance_type = "t2.medium"
        self.ec2_instance = None
        self.public_ip = None

        self.unqiue_id = str(random.randint(10, 1000))
        self.ec2_security_group = 'FOGROS_SECURITY_GROUP' + rand_int
        self.ec2_key_name = "FogROSKEY" + rand_int


    def create(self):
        create_security_group()
        generate_key_pair()
        create_ec2_instance()



    def create_security_group(self):
        response = ec2.describe_vpcs()
        vpc_id = response.get('Vpcs', [{}])[0].get('VpcId', '')
        try:
            response = ec2.create_security_group(GroupName=ec2_security_group,
                                                 Description='DESCRIPTION',
                                                 VpcId=vpc_id)
            security_group_id = response['GroupId']
            print('Security Group Created %s in vpc %s.' % (security_group_id, vpc_id))

            data = ec2.authorize_security_group_ingress(
                GroupId=security_group_id,
                IpPermissions=[
                    {'IpProtocol': '-1',
                     'FromPort': 0,
                     'ToPort': 65535,
                     'IpRanges': [{'CidrIp': '0.0.0.0/0'}]
                     }
                ])
            print('Ingress Successfully Set %s' % data)
            ec2_security_group_ids = [security_group_id]
        except ClientError as e:
            print(e)
        print("security group id is " + str(ec2_security_group_ids))
        return ec2_security_group_ids

    def generate_key_pair(self):
        ec2_keypair = ec2.create_key_pair(KeyName=self.ec2_key_name)
        ec2_priv_key = ec2_keypair['KeyMaterial']
        with open("/home/ubuntu/" + ec2_key_name + ".pem", "w") as f:
            f.write(ec2_priv_key)
        print(ec2_priv_key)
        self.ssh_key = ec2_priv_key
        return ec2_priv_key

    def create_ec2_instance(ec2_security_group_ids, ec2_instance_disk_size=30):
        #
        # start EC2 instance
        # note that we can start muliple instances at the same time
        #
        instances = ec2_resource.create_instances(
            ImageId='ami-0d255df33d23c5a9d',
            MinCount=1,
            MaxCount=1,
            InstanceType=self.ec2_instance_type,
            KeyName=self.ec2_key_name,
            SecurityGroupIds=ec2_security_group_ids,
            BlockDeviceMappings=[
                {
                    'DeviceName': '/dev/sda1',
                    'Ebs': {
                        'VolumeSize': ec2_instance_disk_size,
                        'VolumeType': 'standard'
                    }
                }
            ]
        )

        print("Have created the instance: ", instances)
        print("type: " + ec2_instance_type)
        instance = instances[0]
        # use the boto3 waiter
        print("wait for launching to finish")
        instance.wait_until_running()
        print("launch finished")
        # reload instance object
        instance.reload()
        # instance_dict = ec2.describe_instances().get('Reservations')[0]
        # print(instance_dict)
        self.ec2_instance = instance
        return instance