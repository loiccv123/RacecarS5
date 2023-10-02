#!/usr/bin/env python

import socket

#HOST = '127.0.0.1'
HOST = '10.0.1.21'
PORT = 65432

 

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(4)

 

print("Server is listening on {}:{}".format(HOST, PORT))

 

conn, addr = s.accept()
print("Connected by", addr)

 

while True:
    data = conn.recv(1024).decode()
    if not data:
        break
    print("Client: " + data)
    message = input("Server > ")
    conn.sendall(message.encode())

 

conn.close()

