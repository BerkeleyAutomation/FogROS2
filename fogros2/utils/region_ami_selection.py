import json
import requests
from math import radians, cos, sin, asin, sqrt

def find_nearest_region_and_ami(regions):
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
        ip = json.loads(requests.get("https://ip.seeip.org/jsonip?").text)["ip"]
        response = requests.get("http://ip-api.com/json/" + ip).json()
        lat = response["lat"]
        long = response["lon"]
        def haversine(lat1, lon1, lat2, lon2):
            lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
            dlon = lon2 - lon1 
            dlat = lat2 - lat1 
            a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
            c = 2 * asin(sqrt(a)) 
            km = 6371 * c
            return km
        closest_region = min(regions, key=lambda region: haversine(aws_regions[region][0], aws_regions[region][1], lat, long))
        return closest_region, regions[closest_region]["ami_image"]