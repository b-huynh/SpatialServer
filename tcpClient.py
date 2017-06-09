import socket
import struct
import sys
import os

import hsm.hsm_pb2 as hsm_pb

HOST, PORT = "138.197.216.48", 9999
HOST, PORT = "localhost", 9999

class SendImageRequest:
	def __init__(self, key):
		# Create Message
		self.msg = hsm_pb.APIRequest()
		self.msg.client_key = key
		self.msg.client_type = hsm_pb.UNKNOWN
		self.msg.method = hsm_pb.SEND_IMAGE
		self.data = bytearray()

	def add_image(self, fpath):
		name, ext = os.path.splitext(os.path.basename(fpath))
		self.msg.image_info.name = name
		self.msg.image_info.ext = ext

		with open(fpath, "rb") as f:
			image_data = bytearray(f.read())
		self.msg.blob_size = len(image_data)

		# Construct bytearray to send
		self.data.extend(bytearray(struct.pack('!i', self.msg.ByteSize())))
		self.data.extend(bytearray(self.msg.SerializeToString()))
		self.data.extend(image_data)

if __name__ == '__main__':
	if len(sys.argv) != 2:
		print("No Image File Provided")
		sys.exit(-1)

	fpath = sys.argv[1]
	if not os.path.isfile(fpath):
		print("File not found")
		sys.exit(-1)

	request = SendImageRequest(12345)
	request.add_image(fpath)

	# Create a socket (SOCK_STREAM means a TCP socket)
	with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
	    # Connect to server and send data
	    sock.connect((HOST, PORT))
	    sock.sendall(request.data)

	    # Receive data from the server and shut down
	    received = sock.recv(4)
	    response_size = struct.unpack('<i', received)[0]
	    response_data = sock.recv(response_size)
	    response = hsm_pb.APIResponse()
	    response.ParseFromString(response_data)

	    print("Received: \n{}".format(response))
