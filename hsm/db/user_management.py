import datetime
import uuid

import pymongo

from hsm.db import mongo_client

db = mongo_client.db

def create_user(username):
    user = {
        'user_id': uuid.uuid4().hex,
        'username': username,
        'date_created': datetime.datetime.utcnow(),
    }
    db.users.insert_one(user)
    return user['user_id']

def info(api_key):
    return db.users.find_one({'user_id': api_key})

def list_new_users(num=10):
    cursor = db.users.find(projection={'_id': False}).sort(
        'date_created',pymongo.DESCENDING)
    return [user for user in cursor]
