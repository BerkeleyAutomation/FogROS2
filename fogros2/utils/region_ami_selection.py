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
import requests
from math import radians, cos, sin, asin, sqrt


aws_regions = {
    "us-east-2": (40.4167, -82.9167),
    "us-east-1": (38.0339, -78.4860),
    "us-west-1": (37.7749, -122.4194),
    "us-west-2": (45.5200, -122.6819),
    "af-south-1": (-33.9249, 18.4241),
    "ap-east-1": (22.2800, 114.1588),
    "ap-southeast-3": (-6.2315, 106.8275),
    "ap-south-1": (19.0760, 72.8777),
    "ap-northeast-3": (34.6723, 135.4848),
    "ap-northeast-2": (37.5665, 126.9780),
    "ap-southeast-1": (1.3521, 103.8198),
    "ap-southeast-2": (-33.8688, 151.2093),
    "ap-northeast-1": (35.6895, 139.6917),
    "ca-central-1": (43.6532, -79.3832),
    "eu-central-1": (50.1147, 8.6821),
    "eu-west-1": (53.4129, -8.2439),
    "eu-west-2": (51.5074, -0.1278),
    "eu-south-1": (45.4642, 9.1900),
    "eu-west-3": (48.8566, 2.3522),
    "eu-north-1": (59.3293, 18.0686),
    "me-south-1": (26.0667, 50.5577),
    "me-central-1": (23.4241, 53.8478),
    "sa-east-1": (-23.5505, -46.6333),
}


def find_nearest_region_and_ami(regions):
    ip = json.loads(requests.get("https://ip.seeip.org/jsonip?").text)["ip"]
    response = requests.get("http://ip-api.com/json/" + ip).json()
    lat = response["lat"]
    long = response["lon"]
    closest_region = min(regions, key=lambda region: haversine(region, lat, long))
    return closest_region, regions[closest_region]["ami_image"]


def haversine(region, lat, lon):
    lon1, lat1, lon2, lat2 = map(radians,
                                 [aws_regions[region][1], aws_regions[region][0], lon, lat])
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    km = 6371 * c
    return km
