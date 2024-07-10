# Description: Dockerfile for the controls container on the pi 
# Runs ROS noetic and has access to the pi's GPIO pins
# Author: Noah Wiley 

# Use the ROS noetic base image
FROM ros:noetic 

# Define env variables
ENV ROS_MASTER_URI=http://192.168.50.145:11311
ENV ROS_IP=192.168.50.36
ENV ROS_WS=/home/app/test_remote_ws

# Install the necessary packages for the container
RUN apt-get update && apt-get install --no-install-recommends -y \
python3-pip \
python3-setuptools \
python3-dev \
&& rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /home/app/test_remote_ws

# Copy contents of lean_ws-main to the container
COPY  test_remote_ws /home/app/test_remote_ws

# # Set up SSH server
# RUN mkdir /var/run/sshd
# RUN echo 'root:root' | chpasswd
# RUN sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
# RUN sed -i 's/#PermitRootLogin yes/PermitRootLogin yes/' /etc/ssh/sshd_config
# RUN mkdir /root/.ssh
# COPY vm_pub_key.pub /root/.ssh/authorized_keys

# Source ROS setup
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /home/app/test_remote_ws/devel/setup.bash" >> ~/.bashrc
RUN rm -r build/

# Build the ROS workspace
RUN . /opt/ros/noetic/setup.sh && catkin_make

# Start SSH server
# CMD ["/usr/sbin/sshd", "-D"]