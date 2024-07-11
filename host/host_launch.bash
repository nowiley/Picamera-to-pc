#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/Picamera-to-pc/lean_ws-main/test_remote_ws/devel/setup.bash

echo "Launching local listener node in a new terminal..."

# Open a new GNOME Terminal and run the listener node
gnome-terminal -- bash -c "
source /opt/ros/noetic/setup.bash
source ~/Picamera-to-pc/lean_ws-main/devel/setup.bash
roslaunch controller host_car_joy.launch;
echo "Started Listener Node"
exec bash"

echo "Starting SSH session to bean@192.168.50.36 ..."

ssh bean@192.168.50.36 << 'EOF'

echo "Connected to remote"

pwd

echo "Changing directory to /home/bean/Picamera-to-pc/pi/controls_docker..."
cd /home/bean/Picamera-to-pc/pi/controls_docker
pwd
if [ $? -eq 0 ]; then
    echo "Successfully changed directory."
else
    echo "Failed to change directory."
    exit 1
fi

echo "Running docker compose up..."
docker compose up -d --build
if [ $? -eq 0 ]; then
    echo "Docker compose up executed successfully."
    exit 1
else
    echo "Docker compose up failed."
    exit 1
fi

echo "Exiting SSH session."
EOF

echo "SSH session completed."