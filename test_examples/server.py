'''
it is an example of server socket creation which can accept 10 concurrent clients (although should be multithreded)
connect to host and port address
connect to client adn wait for data to recv
and then sends the same data back then again wait for client to connect
'''
import socket

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 8090  # Port to listen on (non-privileged ports are > 1023)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(10)
try:
    while True:
        conn, addr = s.accept()
        print(f"Connected by {addr}")
        while True:
            data = conn.recv(1024)
            print("Received Data: ", data)
            if not data:
                break
            conn.sendall(data)
            print("Sent Data: ", data)
except Exception as e:
    print(e)
    s.close()