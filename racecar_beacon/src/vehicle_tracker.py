#!/usr/bin/env python

import socket
import threading
import struct
import rospy

PORT = 65431

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.bind(('0.0.0.0', PORT))

print("Client listening on¸{}".format(PORT)) 
    
while True:
    data, addr = client_socket.recvfrom(1024)

    # Assuming the data format is "fffI", adjust it as needed
    data_format = "fffI"
    data_size = struct.calcsize(data_format)

    # Unpack the binary data into individual values
    unpacked_data = struct.unpack(data_format, data)

    # Now you can work with the unpacked data
    x, y, z, identifier = unpacked_data

    print("Received data from {}: x={}, y={}, z={}, id={}".format(addr, x, y, z, identifier))
    
    
    
    
    
    
    
    
    
