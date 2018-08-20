import json

from bson import json_util
from flask import Blueprint, request

from hsm.db import user_management
from hsm.http import api_request as apireq

users_endpoint = Blueprint('users', __name__)

@users_endpoint.route('/api/users/create', methods=['POST'])
def create():
    """Create new user and return api key"""
    api_request = apireq.APIRequest(request, 'client_schema')
    if api_request.is_invalid():
        return api_request.error_text, 400
    return user_management.create_user(api_json['username'])

@users_endpoint.route('/api/users/info', methods=['GET'])
def info():
    """Get info about a user based on their api_key"""
    api_request = apireq.APIRequest(request, 'client_schema')
    if api_request.is_invalid():
        return api_request.error_text, 400
    return json.dumps(user_management.info(api_json['api_key']), indent=4,
                      default=json_util.default)

@users_endpoint.route('/api/users', methods=['GET'])
def list_users():
    """List recently created users"""
    api_request = apireq.APIRequest(request, 'client_schema')
    if api_request.is_invalid():
        return api_request.error_text, 400
    return json.dumps(user_management.list_new_users(), indent=4,
                      default=json_util.default)
