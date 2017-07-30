import enum
import json

from hsm.http import validation

# TODO: Get rid of this and add a flag to schema definitions instead
CHECK_FILE_SCHEMAS = ['upload_schema']

class APIRequest(object):
    """Encapsulates processing and validation of API requests."""
    def __init__(self, request, schema):
        """Constructor for APIRequest.
        Args:
            request: Flask request object
            schema: The schema to validate against
        """
        self._flask_request = request
        self._schema = schema
        self._api_data = None
        self._error_text = None
        try:
            self._api_data = self._validate()
        except validation.ValidationError as e:
            self._error_text = str(e)

    @property
    def schema(self):
        return self._schema

    @property
    def api_data(self):
        return self._api_data

    @property
    def error_text(self):
        return self._error_text

    def is_valid(self):
        return self._error_text == None

    def is_invalid(self):
        return not self.is_valid()

    def _validate(self):
        """Validate request with `self._schema`.
        Returns:
            api_data: Validated and normalized api request data
        Raises:
            validation.ValidationError: If JSON is invalid
        """
        data = self._get_json()
        validator = validation.get_validator(self.schema)
        api_data = validator.validated(data)
        if api_data is None:
            raise validation.ValidationError("Invalid JSON.")
        return api_data

    def _get_json(self):
        """Get JSON from `self.flask_request` based on schema.
        Returns:
            data: Request JSON decoded to dictionary
        Raises:
            validation.ValidationError: If JSON is missing
        """
        if self._schema in CHECK_FILE_SCHEMAS:
            try:
                data = json.load(self._flask_request.files['json'])
            except Exception as e:
                raise validation.ValidationError("Missing Files or JSON.")
        else:
            data = self._flask_request.get_json()
            if data is None:
                raise validation.ValidationError("Missing JSON.")
        return data
