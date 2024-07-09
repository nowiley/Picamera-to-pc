# Description: Dockerfile for the controls container on the pi 
# Runs ROS noetic and has access to the pi's GPIO pins
# Author: Noah Wiley 

# Use the ROS noetic base image
FROM ros:noetic 

# Define env variables
ENV ROS_MASTER_URI=http://192.168.50.145:11311
ENV ROS_IP=192.168.50.36
ENV ROS_WS=/home/app/lean_ws-main 

# Install the necessary packages for the container
RUN apt-get update && apt-get install --no-install-recommends -y \
python3-pip \
python3-setuptools \
python3-dev \
libeigen3-dev \
python3-rpi.gpio \
ros-noetic-tf \
ros-noetic-joy \
openssh-server \
ros-noetic-rqt \
ros-noetic-rqt-common-plugins \
&& rm -rf /var/lib/apt/lists/*

# Install the necessary python packages for the container
RUN pip3 install adafruit-circuitpython-ina219 \
adafruit-circuitpython-typing==1.10.1 \
RPi.GPIO

# Set the working directory
WORKDIR /home/app

# Copy contents of lean_ws-main to the container
COPY lean_ws-main /home/app/lean_ws-main

# # Set up SSH server
# RUN mkdir /var/run/sshd
# RUN echo 'root:root' | chpasswd
# RUN sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
# RUN sed -i 's/#PermitRootLogin yes/PermitRootLogin yes/' /etc/ssh/sshd_config
# RUN mkdir /root/.ssh
# COPY vm_pub_key.pub /root/.ssh/authorized_keys

# Source ROS setup
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /app/lean_ws-main/devel/setup.bash" >> /root/.bashrc

# Build the ROS workspace
RUN cd /home/app/lean_ws-main && catkin_make

# Start SSH server
# CMD ["/usr/sbin/sshd", "-D"]