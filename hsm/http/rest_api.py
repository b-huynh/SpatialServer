from flask import Flask

from hsm import config
from hsm.http import validation
from hsm.http.endpoints import data_endpoint
from hsm.http.endpoints import users_endpoint

app = Flask(__name__, static_folder='static', static_url_path='/static')
app.register_blueprint(data_endpoint.data_endpoint)
app.register_blueprint(users_endpoint.users_endpoint)

if __name__ == '__main__':
    validation.register_schemas()
    app.run(host=config.REST_API_HOST, port=config.REST_API_PORT)
