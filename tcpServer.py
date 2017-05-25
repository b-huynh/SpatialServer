import socketserver
import struct
import os

import proto.mapserver_pb2 as server_pb

IMAGE_DATA = 'image_data'
HOLOLENS_DATA = 'hololens_data'

class MapserverTCPHandler(socketserver.BaseRequestHandler):

    def handle(self):
        # self.request is the TCP socket connected to the client
        self.data = self.request.recv(4)
        msg_size = struct.unpack('!i', self.data)[0]
        self.handleMessage(msg_size)

        print("Received message from {}".format(self.client_address[0]))
        self.request.sendall("OK".encode('utf-8'))

    def handleMessage(self, msg_size):
        self.data = self.request.recv(msg_size)
        self.msg = server_pb.MapserverMessage()
        self.msg.ParseFromString(self.data)
        print(self.msg)

        if self.msg.client_type == server_pb.MapserverMessage.IMAGE:
            self.writeJPG()

    def writeJPG(self):
        fname = self.msg.image_info.name + self.msg.image_info.ext
        fpath = os.path.join(IMAGE_DATA, fname)

        if os.path.isfile(fpath):
            os.remove(fpath)

        remaining = self.msg.image_info.byte_size
        with open(fpath, "ab") as f:
            while True:
                img_data = self.request.recv(4096)
                f.write(img_data)
                remaining = remaining - len(img_data)
                if remaining <= 0:
                    break;

if __name__ == "__main__":
    HOST, PORT = "0.0.0.0", 9999

    # Create the server, binding to localhost on port 9999
    server = socketserver.TCPServer((HOST, PORT), MapserverTCPHandler)

    # Activate the server; this will keep running until you
    # interrupt the program with Ctrl-C
    server.serve_forever()