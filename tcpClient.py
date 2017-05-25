import socket
import struct
import sys
import os

import proto.mapserver_pb2 as server_pb

HOST, PORT = "localhost", 9999

if __name__ == '__main__':
	if len(sys.argv) != 2:
		print("No Image File Provided")
		sys.exit(-1)

	fpath = sys.argv[1]

	if not os.path.isfile(fpath):
		print("File not found")
		sys.exit(-1)

	fname = os.path.basename(fpath)
	name, ext = os.path.splitext(fname)

	# Create Message
	msg = server_pb.MapserverMessage()
	msg.client_type = server_pb.MapserverMessage.IMAGE;
	msg.client_id = 1993
	msg.image_info.name = name
	msg.image_info.ext = ext

	with open(fpath, "rb") as f:
		image_data = bytearray(f.read())
	msg.image_info.byte_size = len(image_data)

	msg_size = msg.ByteSize()
	data = bytearray(struct.pack('!i', msg_size))
	data.extend(bytearray(msg.SerializeToString()))
	data.extend(image_data)

	# Create a socket (SOCK_STREAM means a TCP socket)
	with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
	    # Connect to server and send data
	    sock.connect((HOST, PORT))
	   # sock.sendall(bytes("hello"))
	    sock.sendall(data)

	    # Receive data from the server and shut down
	    received = str(sock.recv(1024), "utf-8")

	print("Sent:     {}".format(data))
	print("Received: {}".format(received))