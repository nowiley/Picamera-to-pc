#!/usr/bin/python3
"""
Client code that connects to a server socket, receive stream of images in H264 format and display them.
FROM https://github.com/golubaca/python-streaming-server/blob/master/client/recv.py
"""
import cv2
import socket
import numpy as np

host_ip = "192.168.50.36"
host_port = 8000

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = (host_ip, host_port)
print('connecting to %s port %s' % server_address)
sock.connect(server_address)

# Create a VideoCapture object and specify the video codec and port
cap = cv2.VideoCapture('tcp://'+host_ip+':'+str(host_port), cv2.CAP_FFMPEG)

if not cap.isOpened():
    print('VideoCapture not opened')
    exit(-1)

while True:
    # Receive frame from the server
    readsuccess, frame = cap.read()

    if not readsuccess:
        print('frame not read, frame empty')
        break
    
    # Display the frame
    cv2.imshow('frame', frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture object and close the window
cap.release()
cv2.destroyAllWindows()

