#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/Picamera-to-pc/lean_ws-main/test_remote_ws/devel/setup.bash

echo "Launching local listener node in a new terminal..."

# Open a new GNOME Terminal launching local nodes
# Joy node, camera client and depth publisher node
gnome-terminal -- bash -c "
source /opt/ros/noetic/setup.bash
source ~/Picamera-to-pc/lean_ws-main/devel/setup.bash
echo "Started Listener Node"
roslaunch cam_joy host_cam_joy.launch;
exec bash"

echo "Starting SSH session to bean@192.168.50.36 ..."

# SSH into remote Raspberry Pi and run docker-compose in controls_docker/docker-compose.yml
# This will launch remote ros nodes
ssh bean@192.168.50.36 << 'EOF'

echo "Connected to remote"

echo "Changing directory to /home/bean/Picamera-to-pc/pi/controls_docker..."
cd /home/bean/Picamera-to-pc/pi/controls_docker

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