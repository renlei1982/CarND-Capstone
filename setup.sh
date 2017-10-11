#! /bin/bash

# Setup script to prepare environment on Ubuntu, e.g. AWS Deep Learning AMI
# Clone this repo in the VM first, then run this script
# Connect using:
#    ssh -L 4567:localhost:4567 user@host-name

# Install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -y ros-kinetic-ros-base

sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential



# Setup apt-get
echo 'Adding Dataspeed server to apt...'
sudo sh -c 'echo "deb [ arch=amd64 ] http://packages.dataspeedinc.com/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-dataspeed-public.list'
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 66F84AE1EB71A8AC108087DCAF677210FF6D3CDA
sudo apt-get update

# Setup rosdep
echo 'Setting up rosdep...'
if [ -z "$ROS_DISTRO" ]; then
  echo "Error! ROS not detected. Not updating rosdep!"
else
  sudo sh -c 'echo "yaml http://packages.dataspeedinc.com/ros/ros-public-'$ROS_DISTRO'.yaml '$ROS_DISTRO'" > /etc/ros/rosdep/sources.list.d/30-dataspeed-public-'$ROS_DISTRO'.list'
  rosdep update
  sudo apt-get install -y ros-$ROS_DISTRO-dbw-mkz
  sudo apt-get upgrade -y




# Install Python packages
sed -i 's/tensorflow==/tensorflow-gpu==/g' requirements.txt
sudo pip install -r requirements.txt

# Install required ros dependencies
sudo apt-get install -y ros-$ROS_DISTRO-cv-bridge
sudo apt-get install -y ros-$ROS_DISTRO-pcl-ros
sudo apt-get install -y ros-$ROS_DISTRO-image-proc

# Install socket io
sudo apt-get install -y netbase
