import socketserver
import struct
import os
import json

import hsm.hsm_pb2 as hsm_pb

from utils import exif_utils

OUTDIR = 'image_data'
METADATA_FILE = 'metadata.json'

class MapserverTCPHandler(socketserver.BaseRequestHandler):

    def handle(self):
        # self.request is the TCP socket connected to the client
        self.data = self.request.recv(4)
        msg_size = struct.unpack('!i', self.data)[0]
        self.handle_message(msg_size)

        print("Received message from {}".format(self.client_address[0]))
        print(self.msg)

        self._send_response()

    def handle_message(self, msg_size):
        self.data = self.request.recv(msg_size)
        self.msg = hsm_pb.APIRequest()
        self.msg.ParseFromString(self.data)


        if self.msg.method == hsm_pb.SEND_IMAGE:
            self.write_image()
        elif self.msg.method == hsm_pb.SEND_MESH:
            self.write_mesh()

    def write_image(self):
        fname = self.msg.image_info.name + self.msg.image_info.ext
        fpath = os.path.join(OUTDIR, fname)
        self._write_file(fpath)

    def write_mesh(self):
        fpath = os.path.join(OUTDIR, str(self.msg.client_key) + "_mesh")
        self._write_file(fpath) 

    def _write_file(self, fpath):
        if os.path.isfile(fpath):
            os.remove(fpath)

        remaining = self.msg.blob_size
        with open(fpath, "ab") as f:
            while True:
                img_data = self.request.recv(4096)
                f.write(img_data)
                remaining = remaining - len(img_data)
                if remaining <= 0:
                    break

        self._update_image_metadata(fpath)

    def _update_image_metadata(self, image_path):
        exif = exif_utils.get_exif(image_path)
        if not exif_utils.has_gpsinfo(exif):
            return
        lat, lon = exif_utils.get_lat_lon(exif)

        with open(METADATA_FILE, 'r') as metadata_file:
            metadata = json.load(metadata_file)
            basename = os.path.basename(image_path)
            metadata[basename] = {
                'lat': lat,
                'lon': lon,
            }

        with open(METADATA_FILE, 'w') as metadata_file:
            json.dump(metadata, metadata_file)

    def _send_response(self):
        response = hsm_pb.APIResponse()
        response.code = 200
        response.text = "OK"

        data = bytearray(struct.pack('<i', response.ByteSize()))
        data.extend(response.SerializeToString())
        self.request.sendall(data)

if __name__ == "__main__":
    HOST, PORT = "0.0.0.0", 9999

    # Create the server, binding to localhost on port 9999
    server = socketserver.TCPServer((HOST, PORT), MapserverTCPHandler)

    # Activate the server; this will keep running until you
    # interrupt the program with Ctrl-C
    server.serve_forever()
