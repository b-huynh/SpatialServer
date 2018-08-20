import argparse
import json
import os
import sys
import pprint
from urllib.parse import urljoin

import requests

from hsm import config

pp = pprint.PrettyPrinter(indent=4)
pprint = pp.pprint

parser = argparse.ArgumentParser(
            description='Python client for SpatialServer REST API.',
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('--api_key', type=str, default=config.CLIENT_API_KEY,
                    help='Your API Key. Default is from hsm.config')
parser.add_argument('--host', type=str, default='127.0.0.1',
                    help='Address for rest_api server')
parser.add_argument('--port', type=int, default=config.REST_API_PORT,
                    help='Port for rest_api server')

# TODO: Make this less specific.
parser.add_argument('--image_path', type=str,
                    help='Path to image file to upload')
parser.add_argument('--mesh_path', type=str,
                    help='Path to image file to upload')
parser.add_argument('--list_users', action='store_true',
                    help='List new users')
parser.add_argument('--list_data', action='store_true',
                    help='List new files')

def print_response(r):
    try:
        pprint(r.json())
    except ValueError:
        pprint(r.text)
    except Exception as e:
        pprint(e)
    print(r.status_code)

def list_users(base_url, api_key):
    api_url = urljoin(base_url, '/api/users')
    api_payload = {'api_key': api_key, 'device': 'python_client'}
    r = requests.get(api_url, json=api_payload)
    print_response(r)

def list_data(base_url, api_key):
    api_url = urljoin(base_url, '/api/data')
    api_payload = {'api_key': api_key, 'device': 'python_client'}
    r = requests.get(api_url, json=api_payload)
    print_response(r)

def upload_file(base_url, api_key, file_path, datatype):
    if not os.path.isfile(file_path):
        pprint("File '%s' not found" % file_path)
        return

    _, ext = os.path.splitext(file_path)
    ext = ext.lstrip('.')

    api_url = urljoin(base_url, '/api/data/upload')
    api_payload = {
        'client': {
            'api_key': api_key,
            'device': 'python_client'
        },
        'datatype': datatype,
        'ext': ext,
    }
    files = [
        ('json', ('json', json.dumps(api_payload), 'application/json')),
        ('file', ('file', open(file_path, 'rb'), 'application/octet-stream')),
    ]
    try:
        r = requests.post(api_url, files=files)
        print_response(r)
    except Exception as e:
        pprint(e)

if __name__ == '__main__':
    args = parser.parse_args()
    base_url = 'http://%s:%d' % (args.host, args.port)
    if args.list_users:
        list_users(base_url, args.api_key)
    elif args.list_data:
        list_data(base_url, args.api_key)
    elif args.image_path:
        upload_file(base_url, args.api_key, args.image_path, 'image')
    elif args.mesh_path:
        upload_file(base_url, args.api_key, args.mesh_path, 'mesh')
