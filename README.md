# Hybrid Spatial Mapping Server
Four Eyes Lab Hybrid Spatial Mapping Server (HSM Server). Real name TBD.

This project requires [python3](https://www.python.org/downloads/).
Additionally, please install and use [virtualenv](https://virtualenv.pypa.io/en/stable/).

I've only tested this on Linux and OS X. It *should* work for Windows.



#####Setup Environment
Create a new virtual environment and use it
```bash
virtualenv -p python3 env
```

Activate the environment. You must do this every time you want to run the server or client. You can tell you're in the virtual environment if you see (env) by your command line prompt.
```bash
source env/bin/activate
```

Install dependencies
```bash
# From SpatialServer/
pip3 install -r requirements.txt
```

Install non-python dependencies. Either:
```bash
sudo apt-get install protobuf-compiler
```
OR on OS X assuming you use [homebrew](https://brew.sh/):
```bash
brew install protobuf
```

Build the protobuffers (only necessary if the pre-compiled ones are out of date)
```bash
# From SpatialServer/
protoc hsm/protos/*.proto --python_out=.
```

#####Running the server
```bash
# From SpatialServer/hsm/server
python3 server.py
```

#####Running the Python client
The currently the python client can only send images to the server. It is useful for testing purposes. There are some sample images in sample_image_data folder.
```bash
# From SpatialServer/hsm/client
python3 client.py <path to image file>
```

#####Adding new methods to the API
The API methods are defined in [hsm.proto](hsm/protos/hsm.proto). If you are not familiar, it would be useful to take a look at the [Protocol Buffers](https://developers.google.com/protocol-buffers/docs/pythontutorial) documentation.

New method names are defined in
```
enum Method {
    SEND_IMAGE = 0;
    ...
}
```

The API messages are defined in
```
message APIRequest {
    uint32 client_key = 1;
    ClientType client_type = 2;

    Method method = 3;
    uint64 blob_size = 4;

    // One of the following will be filled in.
    MeshInfo mesh_info = 100;
    ...
}

message APIResponse {
    uint32 code = 1;
    string text = 2;
}
```
Currently all values are not well defined (i.e. just use random numbers for a client key). ```blob_size``` is the byte size of any binary data that needs to be sent, such as an image file or a hololens mesh.

To use the API, first connect to the server wherever it is running (i.e. http://localhost:9999) from a TCP Socket on your client side.
```python
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.connect((HOST, PORT))
    ...
```

Construct your APIRequest object:
```python
msg = hsm_pb.APIRequest()
msg.client_key = key
msg.client_type = hsm_pb.UNKNOWN
msg.method = hsm_pb.SEND_IMAGE
...
```

You need to send to the server a single buffer* with the following in order:
1. The byte size of the APIRequest as an ```Int32```
2. The serialized APIRequest
3. The binary data (i.e. an image file)

See [client.py](hsm/client/client.py) for an example

\*Will fix this *soon*...

#####Related Repos
- [Hololens Client](https://github.ucsb.edu/ychang/HoloModeler) (Yun Suk Chang)
