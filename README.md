# udp_to_ros2_bridge

A generic ROS 2 Humble package to bridge UDP messages into ROS 2.
This package listens on a UDP port and republishes incoming messages on a configurable ROS 2 topic as strings.

---

## Example Usage

### Launch

Launch the node with parameters from YAML:

```
ros2 launch udp_to_ros2_bridge udp_to_ros2_bridge_launch.py
```

## Check that the node is publishing

```
ros2 topic echo /tactile_sensors_from_udp
```

The node will publish the raw UDP messages as string messages (`std_msgs/msg/String`) on the specified topic.

---

## Parameters

Parameters can be set via YAML or command line. Defaults are used if not specified.

| Parameter             | Type    | Default       | Description                       |
|-----------------------|---------|---------------|-----------------------------------|
| `communication.IP`    | string  | "0.0.0.0"     | UDP IP address to listen on       |
| `communication.port`  | int     | 61559         | UDP port to listen on             |
| `ros_topic`           | string  | "udp_raw"     | ROS 2 topic to publish UDP data   |

### Example YAML (config/comm_param.yaml)
```
udp_to_ros2_bridge:
  ros__parameters:
    communication:
      IP: "192.168.125.100"
      port: 61559
    ros_topic: "udp_raw"
```
