#!/usr/bin/python3
"""
Client code that connects to a server socket, receive stream of images in H264 format and display them.
FROM https://github.com/golubaca/python-streaming-server/blob/master/client/recv.py
"""
import cv2
import socket
import numpy as np

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = ('192.168.50.36', 8000)
sock.connect(server_address)

# Receive the data in small chunks and reassemble it
data = b""
while True:
    data += sock.recv(4096)
    start = 0
    while True:
        start = data.find(b'\xff\xd8', start)
        end = data.find(b'\xff\xd9', start)
        if start != -1 and end != -1:
            jpg = data[start:end+2]
            data = data[end+2:]
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            cv2.imshow('Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            start = 0
        else:
            break

cv2.destroyAllWindows()
