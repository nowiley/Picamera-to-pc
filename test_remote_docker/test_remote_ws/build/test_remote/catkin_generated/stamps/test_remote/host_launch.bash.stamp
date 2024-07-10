#!/bin/bash

echo "Starting SSH session to bean@192.168.50.36 ..."

ssh bean@192.168.50.36 << 'EOF'

echo "Connected to remote"

echo "Changing directory to /home/app/test_remote_docker..."
cd /home/app/test_remote_docker
if [ $? -eq 0 ]; then
    echo "Successfully changed directory."
else
    echo "Failed to change directory."
    exit 1
fi

echo "Running docker compose up..."
docker compose up
if [ $? -eq 0 ]; then
    echo "Docker compose up executed successfully."
else
    echo "Docker compose up failed."
    exit 1
fi

echo "Exiting SSH session."
EOF

echo "SSH session completed."