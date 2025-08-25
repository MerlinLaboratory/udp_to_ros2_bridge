# udp_to_ros2_bridge

A generic ROS 2 Humble package to bridge UDP messages into ROS 2.
This package listens on a UDP port and republishes incoming messages on a configurable ROS 2 topic as strings.

---

## Features

- Configurable UDP IP and port via ROS 2 parameters or YAML.
- Configurable ROS 2 topic name.
- Pure Python implementation compatible with ROS 2 Humble.
- Clean separation of UDP listener (`UDPNode`) and ROS 2 node.


---

## Installation

1. Clone into your ROS 2 workspace:

cd ~/ros2_ws/src
git clone <repository_url> udp_to_ros2_bridge

2. Build the workspace:

cd ~/ros2_ws
colcon build --symlink-install --packages-select udp_to_ros2_bridge
source install/setup.bash

---

## Parameters

Parameters can be set via YAML or command line. Defaults are used if not specified.

| Parameter             | Type    | Default       | Description                       |
|-----------------------|---------|---------------|-----------------------------------|
| `communication.IP`    | string  | "0.0.0.0"     | UDP IP address to listen on       |
| `communication.port`  | int     | 61559         | UDP port to listen on             |
| `ros_topic`           | string  | "udp_raw"     | ROS 2 topic to publish UDP data   |

### Example YAML (config/comm_param.yaml)

udp_to_ros2_bridge:
  ros__parameters:
    communication:
      IP: "192.168.125.100"
      port: 61559
    ros_topic: "udp_raw"

---

## Launch

Launch the node with parameters from YAML:

ros2 launch udp_to_ros2_bridge udp_to_ros2_bridge_launch.py

---

## Example Usage

# Check that the node is publishing
ros2 topic echo /udp_raw

The node will publish the raw UDP messages as string messages (`std_msgs/msg/String`) on the specified topic.

---

## Notes

- Only supports **string messages** from UDP.
- The ROS 2 topic and UDP connection parameters are fully configurable.
- Make sure the `resource/udp_to_ros2_bridge` file exists, even if empty, to satisfy `ament_python` requirements.

---

## License

Apache 2.0
