# barrett2-ros-pkg
This only workks for wam sim and it's services
### Install docker
sudo apt install docker.io
sudo usermod -aG docker <user>
exec sg docker newgrp

#### Pull the Ubuntu 22.04 (Jammy) image from hub.docker.com
docker pull ubuntu:jammy

#### Run a new container from that image, and log into it
docker run --network=host -it ubuntu:jammy

#### Install system base
apt update && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends sudo ca-certificates git make vim-tiny wget tzdata lsb-release curl gnupg g++ can-utils

#### Make user ‘robot’ with password ‘WAM’
useradd -m -G sudo -s /bin/bash -p `perl -e 'print crypt("WAM","saltydog")'` robot
su -l robot

#### Make a ‘code’ directory for our software packages
mkdir -p code && cd code

#### Download and Install patched Libconfig 1.4.5 (supporting C & C++ simultaneously)
wget http://web.barrett.com/support/WAM_Installer/libconfig-1.4.5-PATCHED-NOTHROW.tar.gz
tar -xf libconfig-1.4.5-PATCHED-NOTHROW.tar.gz

cd libconfig-1.4.5 && ./configure && make -j$(nproc)

sudo make install

sudo ldconfig

#### Download and install libbarrett 3 (and deps)
cd ~/code

sudo apt install -y --no-install-recommends libgsl-dev libeigen3-dev libncurses-dev cmakesudo apt install -y --no-install-recommends libboost-system-dev libboost-thread-dev libboost-python-dev

git clone https://git.barrett.com/software/libbarrett

cd libbarrett

git checkout feature/clearcan-and-nothrow

cmake . && make -j$(nproc)

sudo make install

#### Test libbarrett 3.x
cd examples

cmake .

make ex04_display_basic_info

./ex04_display_basic_info

We have a system that can run a WAM!
### Back in the docker container’s terminal
#### Install ROS Humble Desktop
sudo apt update

sudo apt install -y ros-humble-desktop

python3-colcon-common-extensions libudev-dev 
#### Activate ROS2
source /opt/ros/humble/setup.bash

### Build the barrett-ros2-pkg
cd ~/code

git clone https://git.barrett.com/software/barrett-ros2-pkg

cd barrett-ros2-pkg

git checkout origin/feature/ros2-humble-libbarrett3.0

colcon build

source install/setup.bash
### Instead of using ros2 launch, run it manually:
source /code/ros2_humble_ws/install/setup.bash

ros2 run wam_node wam_node

### To end
Press shift-idle to end command line


