# UnetStack ROS2 Gateway

`v0.1.0`

**Status: experimental**<br>
(API subject to change, not extensively tested)

A ROS2 node that provides access to [UnetStack](www.unetstack.net)-based communication networks. Provides access to `REMOTE` and `ADDRESS_RESOLUTION` services for data transmission and reception across a Unet (in-water network).

**WORK IN PROGRESS**<br>
Partial functionality of `REMOTE` currently supported (only `DATAGRAM` service)

## Prerequisites

- ROS2 (tested against ROS2 Iron)
- Python3 (typically installed with ROS2)
- `pip` python package installer
- `git` (optional, for downloading `unet-ros2`)
- UnetStack 3 or UnetStack 4 based modems or simulator (optional, for testing)

## Installation

To download and build:
```
pip install unetpy
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

## API

Services exposed:
```
/unet/get/address [unet_msg/srv/AddressResolutionReq]
/unet/tx/datagram [unet_msg/srv/DatagramReq]
```

Topics exposed:
```
/unet/rx/datagram [unet_msg/msg/DatagramNtf]
```

## Command-line examples

Get own address:
```
$ ros2 service call /unet/get/address unet_msg/srv/AddressResolutionReq
waiting for service to become available...
requester: making request: unet_msg.srv.AddressResolutionReq_Request(name='')

response:
unet_msg.srv.AddressResolutionReq_Response(address=150)
```

Resolve address of peer node (`AUV`):
```
$ ros2 service call /unet/get/address unet_msg/srv/AddressResolutionReq '{name: "AUV"}'
waiting for service to become available...
requester: making request: unet_msg.srv.AddressResolutionReq_Request(name='AUV')

response:
unet_msg.srv.AddressResolutionReq_Response(address=173)
```

Transmit a datagram:
```
$ ros2 service call /unet/tx/datagram unet_msg/srv/DatagramReq '{to: 173, data: [1,2,3,4,5]}'
waiting for service to become available...
requester: making request: unet_msg.srv.DatagramReq_Request(data=[1, 2, 3, 4, 5], to=173, protocol=0, reliability=0, robustness=0, ttl=0.0, priority=2, shortcircuit=True, route='')

response:
unet_msg.srv.DatagramReq_Response(agree=True, id='b4b53432-c556-4e31-a7e9-431770e81cc7', reason='')
```

Listen for datagrams:
```
$ ros2 topic echo /unet/rx/datagram
data:
- 1
- 2
- 3
from_addr: 173
to: 150
protocol: 0
---
```

## Relevant references

- [UnetStack](https://www.unetstack.net)
- [Underwater networks handbook](https://unetstack.net/handbook/unet-handbook.html)
