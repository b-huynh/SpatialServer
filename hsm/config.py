# Spatial server configuration settings

# Use this flag for adding debug settings!
DEBUG = False

# Filesystem root directory
FS_ROOT = '/data/spatial_fs'

# Used for exposing TCP based api server (for real time communications)
TCP_API_HOST = '0.0.0.0'
TCP_API_PORT = 8101

# This is used for both rest api and management webapps. Should separate.
REST_API_HOST = '0.0.0.0'
REST_API_PORT = 5000


# HTTP Related configurations

# TODO: Client probably should be separated from server config options...
CLIENT_API_KEY = 'fc1431e272194e33ae924a090749a49d'
