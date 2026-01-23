# WAM ROS2 Docker

This container is set up around this branch of barrett's ROS2 package: https://git.barrett.com/software/barrett-ros2-pkg/-/tree/feature/ros2-humble-libbarrett3.0

**Note that the barrett-ros2-pkg in this container is modified from the one on barrett's gitlab, as there are some bugs in their code**

## WIP TODO's

- Further testing of cartesian orientation velocities, since they appear to behave poorly as of now
- Testing of BHand functions in barrett's ROS2 node. Additionally, it may be better to integrate Dylan's libgripper, as well as possibly other grippers.

# WAM Config Files

This repository contains the `barrett_cfgs` folder that has config files for the WAM. This is based on the defaults from libbarrett. 

There is a script `barrett_cfgs/link_configs` that links these config files to the `~/.barrett` directory which will cause the WAMs to use these config files. You can unlink with the `barrett_cfgs/unlink_configs`. In VSCode, you get prompted on startup whether to link the configs. Generally, you will want to link the config files, unless you want to be sure you are starting from the default config files. In that case, you can find the defaults in `/etc/barrett` and then once you have found the values you want, update the files in this repo. 

If you need to manually set a config file, you can use `export BARRETT_CONFIG_FILE={config file path}`.

# CAN
CAN connection needs to be established on the computer this container is running on. This should be setup outside of the container, but the container should get access as it is using privileged mode.

Most devices likely already have the pcan drivers needed installed, but if not you can look here to install them: https://www.peak-system.com/fileadmin/media/linux/index.php

Once you have the CAN cable connected from the wam (outside of docker):

`sudo modprobe peak_pci` OR `sudo modprobe peak_usb` depending on your CAN device

`sudo ip link set can0 up type can restart-ms 100 bitrate 1000000`

Test with `candump can0` (if no error shows up, you should be good.)

Also note, that if using the usb CAN adapter, I have noticed some stuttering of the WAM if your device is not able to send messages fast enough.

# Testing

You can start the WAM node with: `ros2 run wam_node wam_node`

The xbox controller can be started with `ros2 launch wam_xbox wam_xbox_launch.py`

As well as an example test script with many test movements can be started with `ros2 run wam_testing wam_test`