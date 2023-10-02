#!/usr/bin/env python

import socket

#HOST = '127.0.0.1'
HOST = '10.0.1.21'
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

print("Connected to server")

while True:
    message = input("Client > ")
    s.sendall(message.encode())
    data = s.recv(1024).decode()
    print("Server: " + data)

s.close()
