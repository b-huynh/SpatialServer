import enum
import glob
import json
import os

from cerberus import Validator, schema_registry
import yaml

from hsm.http import schemas

class ValidationError(Exception):
    """Exception raised for errors in validation"""
    pass

def register_schemas():
    schema_dir = os.path.dirname(schemas.__file__)
    schema_fpaths = glob.glob(os.path.join(schema_dir, '*.yml'))
    def register_schema(fpath):
        schemas_dict = yaml.load(open(fpath, 'r'))
        for schema in schemas_dict:
            schema_registry.add(schema, schemas_dict[schema])
    [register_schema(fpath) for fpath in schema_fpaths]

def get_validator(schema_name):
    return Validator(schema_registry.get(schema_name))
