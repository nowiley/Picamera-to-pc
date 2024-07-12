# Picamera-to-pc
 Repo for a python, picamera2, and socket streaming platform between a raspberry pi to a pc over wifi network

## Goals:
* Integrate vision to LEAN robots
* Maintain existing functionality and minimal workflow changes
* Official support for ROS and essential libraries
* Simple and few "hacks"

---

## How this repo works: /controls_docker and /host
1. It is assumed by the system that the Picamera-to-pc repo is cloned in the home directory of both the host and remote Raspberry Pi such that ~/Picamera-to-pc is the same on the host and remote pi.
2. Please ensure that the lean_ws-main is loaded manually into ~/Picamera-to-pc/controls_docker/lean_ws-main, catkin_make is run in the workspace, and /opt/ros/noetic/setup.bash and .../lean_ws-main/devel/setup.bash are sourced. Note: If you are gettin an error from catkin_make, you may need to delete the .../lean_ws-main directory and its files; these likely were built on an external machine and do not jive well with the local machine.  
3. The host/host_launch.bash file is an automated script that launches the required nodes for the host in a new local terminal and the nodes required on the remote raspberry pi via an automated ssh session.

## How is this different than the original lean_ws-main?
### **System Changes:**
 The host runs the same nodes on the local version of ros just like before. The changes to the pi are listed below:
1. The raspberry pi now runs Debian Bookworm, the most recent version of Raspbian (July 2024).
2. Old nodes run inside a ros1 docker container.
3. The camera "node" either runs in a Debian Bookworm docker container or locally on the pi. Note: although it is referred to as a "node" it is not run directly with ros, rather it broadcasts its video stream to the raspberry pi's 8080 port. The host comptuer runs a python client with that listens to this TCP socket and finally publishes a video topic. Therefore, this change allows this system the highest levels of official support from ROS and Picamera2.

### Use, workflow, existing code:
 Care was taken to modify as little of the original code as possible. As of 7/11/2024, the only modifications necessary are to the launch files. 
 
 In more detail, they must be split into a launch file for the remote raspberry pi and a launch file for the host machine. This as simple as cloning the original launch file into two new files, comment out the nodes for the remote machine for the host launch file and visa-versa for the raspberry pi launch file. In summary, launch is now handled by a single bash script and launch files need to be made individually for the remote raspbery pi and host machine.

### The files and folders
**/controls_docker:** This folder contains all the files that are needed on the raspberry pi/robot system. Please note that the lean_ws-main folder is not included on the github repo and must be added manually to this folder.

**/host:** This folder contains all files required on the host pc. Please note that lean_ws-main must be added, catkin_make, and devel/setup.sh sourced manually. 

**/test_remote_docker:** This folder contains example files for a simple talker listener ros/docker system. Similar to the controls_docker and host pair, this folder contains a docker folder that is to be loaded onto the pi and the host_launch.bash script is run on the host. 

---

### Motivation and About this Repo
The Raspberry Pi Zero 2 W platform offers a low cost, energy efficient, and compact platform for robotics development, but its relationship with ROS is complicated. Plus, with the constant development of Linux, Raspberry Pi libraries, ROS1, ROS2, and throw in remote hosts, the development of new ROS/Raspberry Pi robots is intricate to say the least. After hours of searching the web, the task of using the raspberry pi camera, gpio pins, remote hosts, and ROS1 with official support and minimal changes to the standard ROS workflow is seamingly impossible. This repo adopts cloud-computing-like approaches to make the above possible in a minimally hacky and easily maintatable way. This repo offers valuable examples like using ROS in Docker containers, using GPIO and other devices in Docker, integrating non-ubuntu/linux compatible services into ROS networks, and remotley launching ROS nodes inside of Docker containers. 

### Why is it complicated? 
1. ROS1 Noetic is still officially supported and widely used, but ROS2 is now the official standard and not directly compatible with ROS1.
2. ROS is only officially supported (Tier 1) on Ubuntu LTS releases (Noetic uses 20.04). The official Raspberry Pi OS (with essential libraries like Picamera2) is Debian Linux. The only community supported (Tier 3) ROS Noetic Debian release is Debian Buster (2021 release, June 2024 EOL).
3. Rasbian Buster 64 bit is required to run ROS but does not support the Raspberry Pi camera at all (its device files do not contain any information). 
4. We want to use the Picamera2 library which can only be `sudo apt-get installed` on Debian Bullseye (1 Version above Buster). 
5. Building libraries in Ubuntu is involved and takes a long time on the Pi. For example, building libcam, an required dependency for Picam2, takes more than 2 hours to build on the Pi Zero 2 W. Picamera2 comes preinstalled in Rasbian Bookworm. Building ROS1 takes ~64 hours to build on the Pi Zero 2 W according to some sources.

We want to use Ubuntu 20.04 and ROS1 Noetic for ROS communication on the host and the Pi and Debian Bookworm for our Pi camera. How do you do this? VMs are impractical on a resource constrained Raspberry Pi, so we use containers. All of the old Raspberry Pi ROS stuff runs in a Docker container and the camera in a different one (with some extra steps for communication)

6. Roslaunch, which is used for remotley launching nodes, requires the remote machine to have ROS1 and the desired nodes avalible locally.

So we split the roslaunch files, use Docker-compose, and a simple bash script!

---