#!/usr/bin/python3
"""
Client code that connects to a server socket, receive stream of images in H264 format and display them.
FROM https://github.com/golubaca/python-streaming-server/blob/master/client/recv.py
"""
import cv2
import socket
import numpy as np
import rospy
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


#####################
# SETUP ROS AND TCP #
#####################

# Initialize the ROS node
rospy.init_node('image_streamer')

# Create a publisher for the image topic
image_pub = rospy.Publisher("camera/image", Image, queue_size=10)

# Create a CvBridge object for converting between OpenCV images and ROS image messages
bridge = CvBridge()

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
ros_remote_ip = os.environ['ROS_IP']
server_address = ('192.168.50.36', 8000) # IP, PORT
sock.connect(server_address)

######################
# RECEIVE, SHOW, PUB #
######################

# Receive the data in small chunks and reassemble it
data = b""
while not rospy.is_shutdown():
    # Receive the data in small chunks and reassemble it
    data += sock.recv(4096)
    start = 0
    while True:
        # Find start and end of the image
        start = data.find(b'\xff\xd8', start)
        end = data.find(b'\xff\xd9', start)
        if start != -1 and end != -1:
            # Extract jpg and shift data
            jpg = data[start:end+2]
            data = data[end+2:]
            # Decode jpg and display
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

            # Convert the OpenCV image to a ROS image message and publish it
            image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            image_pub.publish(image_msg)

            cv2.imshow('Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            start = 0
        else:
            break

# Clean up the connection
sock.close()
cv2.destroyAllWindows()
