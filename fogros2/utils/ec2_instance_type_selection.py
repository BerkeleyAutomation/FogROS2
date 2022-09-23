import boto3
import json
from pkg_resources import resource_filename
from .region_ami_selection import haversine, aws_regions

FLT = '[{{"Field": "tenancy", "Value": "shared", "Type": "TERM_MATCH"}},'\
      '{{"Field": "operatingSystem", "Value": "{o}", "Type": "TERM_MATCH"}},'\
      '{{"Field": "preInstalledSw", "Value": "NA", "Type": "TERM_MATCH"}},'\
      '{{"Field": "instanceType", "Value": "{t}", "Type": "TERM_MATCH"}},'\
      '{{"Field": "location", "Value": "{r}", "Type": "TERM_MATCH"}},'\
      '{{"Field": "capacitystatus", "Value": "Used", "Type": "TERM_MATCH"}}]'


def get_price(region_name, instance, os):
    f = FLT.format(r=get_region(region_name), t=instance, o=os)
    pricing_client = boto3.client('pricing', 
        region_name=region_name, 
    )
    data = pricing_client.get_products(ServiceCode='AmazonEC2', Filters=json.loads(f))
    od = json.loads(data['PriceList'][0])['terms']['OnDemand']
    id1 = list(od)[0]
    id2 = list(od[id1]['priceDimensions'])[0]
    price = od[id1]['priceDimensions'][id2]['pricePerUnit']['USD']
    return price

def get_region(region_name):
    default_region = 'US East (N. Virginia)'
    endpoint_file = resource_filename('botocore', 'data/endpoints.json')
    try:
        with open(endpoint_file, 'r') as f:
            data = json.load(f)
        return data['partitions'][0]['regions'][region_name]['description'].replace('Europe', 'EU')
    except IOError:
        return default_region

def ec2_instance_types(region_name, cpu_architecture="x86_64", default_cores=2, default_threads_per_core=1, gpu=True):
    '''Yield all available EC2 instance types in region <region_name>'''
    ec2 = boto3.client('ec2', 
        region_name=region_name,
    )

    describe_args = {
        "Filters": [
            {
                'Name': 'processor-info.supported-architecture',
                'Values': [
                    cpu_architecture,
                ]
            },
            {
                'Name': 'vcpu-info.default-cores',
                'Values': [
                    str(default_cores),
                ]
            },
            {
                'Name': 'vcpu-info.default-threads-per-core',
                'Values': [
                    str(default_threads_per_core),
                ]
            },
        ],
    }

    while True:
        describe_result = ec2.describe_instance_types(**describe_args)
        yield from [i["InstanceType"] for i in describe_result['InstanceTypes'] if not gpu or "GpuInfo" in i]
        if 'NextToken' not in describe_result:
            break
        describe_args['NextToken'] = describe_result['NextToken']

def find_cheapest_ec2_instance_type(region_name, cpu_architecture="x86_64", default_cores=2, default_threads_per_core=1, gpu=False):
    (lat, lon) = aws_regions[region_name]
    region_name = min(['us-east-1', 'ap-south-1'], key= lambda region: haversine(region, lat, lon))
    return min(ec2_instance_types(region_name, cpu_architecture, default_cores, default_threads_per_core, gpu), key=lambda instance_type: get_price(region_name, instance_type, 'Linux'))
