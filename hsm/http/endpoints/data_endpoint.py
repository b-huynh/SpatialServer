import json

from bson import json_util
from flask import Blueprint, request

from hsm.db import file_management
from hsm.http import api_request as apireq

data_endpoint = Blueprint('data', __name__)

@data_endpoint.route('/api/data/upload', methods=['POST'])
def upload():
    """Upload new data"""
    api_request = apireq.APIRequest(request, 'upload_schema')
    if api_request.is_invalid():
        return api_request.error_text, 400
    api_data = api_request.api_data
    try:
        file_management.add_file(
            request.files['file'], api_data['datatype'], api_data['ext'])
    except ValueError as e:
        return 'Failed: {}'.format(e), 500
    return 'Success', 200

@data_endpoint.route('/api/data', methods=['GET'])
def list_data():
    """List recently added data"""
    api_request = apireq.APIRequest(request, 'client_schema')
    if api_request.is_invalid():
        return api_request.error_text, 400
    return json.dumps(file_management.list_recent(), indent=4,
                      default=json_util.default), 200
