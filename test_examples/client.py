'''
it is ana example to create a client socket to connect to server
on mentioned host and port address and send some data and close the socket
'''

import socket
from time import sleep
HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = 8090  # The port used by the server

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
s.sendall(b"Hello, world")
data = s.recv(1024)
print(f"Received {data!r}")
sleep(10)
s.sendall(b"Hello, world2")
print("sent...")
data = s.recv(1024)
print(f"Received {data!r}")
s.close()