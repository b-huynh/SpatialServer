import datetime
import functools
import os
import shutil
import uuid

import pymongo

from hsm import config
from hsm.db import mongo_client

db = mongo_client.db

def add_file(file_obj, datatype, ext):
    """Add file to server file storage and register in db
    Args:
        file_obj: File object containing data to write
        datatype: The datatype of the file
        ext: The files extension
    Raises:
        ValueError: If `datatype` is not valid
    """
    if datatype == 'image':
        add_image_file(file_obj, ext)
    else:
        raise ValueError("Unknown datatype '{}'".format(datatype))

def add_image_file(file_obj, ext):
    file_id = str(uuid.uuid4())
    output_name = "{}.{}".format(file_id, ext)
    db.files.insert_one({
        'file_id': file_id,
        'path': write_file(file_obj, output_name),
        'datatype': 'image',
        'ext': ext,
        'date_created': datetime.datetime.utcnow(),
    })

def write_file(file_obj, output_name, bufsize=16384):
    """Writes file to server file storage
    Args:
        file_obj: File object to read data from
        output_name: Filename to save data to
        bufsize: Num bytes to use in memory while writing
    Returns:
        output_path: Path where the file was written to
    Raises:
        ValueError: `output_name` already exists in file storage
    """
    output_path = os.path.join(config.FS_ROOT, output_name)
    if os.path.isfile(output_path):
        raise ValueError("File exists on server {}".format(output_path))
    with open(output_path, 'xb') as dst:
        shutil.copyfileobj(file_obj.stream, dst, bufsize)
    return output_path

def list_recent(num=10):
    cursor = db.files.find(projection={'_id': False}).sort(
        'date_created', pymongo.DESCENDING).limit(num)
    return [file for file in cursor]
