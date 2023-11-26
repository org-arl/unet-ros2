# UnetStack ROS2 Gateway

**Status: experimental**<br>
(API subject to change)

A ROS2 node that provides access to UnetStack-based communication networks. Provides access to `REMOTE` and `ADDRESS_RESOLUTION` services for data transmission and reception across a Unet.

## Prerequisites

- ROS2 (tested with ROS2 Iron)
- Python3 (typically installed with ROS2)
- `unetpy` python package
- `git`

## Installation

To download and build:
```
git clone https://github.com/org-arl/unet-ros2.git
cd unet-ros2
colcon build
source install/local_setup.bash
```

To start the node:
```
ros2 run unet_ros2 unet_gw --ros-args -p port:=1100 -p host:="192.168.100.207"
```
where you replace `192.168.100.207` with the IP address of your Unet node to connect to. The `port` number (defaults to `1100`) can be customized if necessary.

## Services

WIP

## Topics

WIP
