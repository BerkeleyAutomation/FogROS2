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
    pricing_client = boto3.client('pricing', region_name=region_name)

    data = pricing_client.get_products(ServiceCode='AmazonEC2',
                                       Filters=json.loads(f))
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
        return data['partitions'][0]['regions'][region_name]['description']\
            .replace('Europe', 'EU')
    except IOError:
        return default_region


def ec2_instance_types(region_name, cpu_architecture="x86_64", default_cores=2,
                       default_threads_per_core=1, gpu=True):
    """Yield all available EC2 instance types in region <region_name>."""
    ec2 = boto3.client('ec2', region_name=region_name)

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
        yield from [i["InstanceType"] for i in describe_result['InstanceTypes']
                    if not gpu or "GpuInfo" in i]
        if 'NextToken' not in describe_result:
            break
        describe_args['NextToken'] = describe_result['NextToken']


def find_cheapest_ec2_instance_type(region_name, cpu_architecture="x86_64", default_cores=2,
                                    default_threads_per_core=1, gpu=False):
    (lat, lon) = aws_regions[region_name]
    region_name = min(['us-east-1', 'ap-south-1'], key=lambda region: haversine(region, lat, lon))
    return min(ec2_instance_types(region_name, cpu_architecture, default_cores,
               default_threads_per_core, gpu),
               key=lambda instance_type: get_price(region_name, instance_type, 'Linux'))
