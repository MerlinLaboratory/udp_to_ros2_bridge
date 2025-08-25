#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from udp_to_ros2_bridge.udp_to_ros2 import UDPNode
import multiprocessing
import codecs
import queue


class UDPBridge(Node):
    """Generic UDP-to-ROS2 bridge that republishes UDP messages as ROS String."""

    def __init__(self):
        super().__init__('udp_bridge')

        # Declare parameters
        self.declare_parameter("communication.IP", "0.0.0.0")
        self.declare_parameter("communication.port", 61559)
        self.declare_parameter("ros_topic", "udp_raw")  # default topic

        # Retrieve parameters
        self._ip = self.get_parameter("communication.IP").get_parameter_value().string_value
        self._port = self.get_parameter("communication.port").get_parameter_value().integer_value
        self._topic_name = self.get_parameter("ros_topic").get_parameter_value().string_value

        if not self._ip or not self._port:
            raise Exception("Missing communication parameters (IP or port).")

        # Queue for incoming UDP messages
        self._message_queue = multiprocessing.Queue()

        # UDP listener
        self._udp_node = UDPNode(self._port, self._ip)
        self.get_logger().info(f"UDPNode created for {self._ip}:{self._port}")
        self._udp_node.set_callback(self.cbk_message)
        self._udp_node.start()

        # Publisher: configurable topic
        self._pub_udp_string = self.create_publisher(String, self._topic_name, 10)
        self.get_logger().info(f"Publishing UDP messages on topic '{self._topic_name}'")

        # Timer (100 Hz) to poll queue
        self.create_timer(0.01, self.update)

    def cbk_message(self, m):
        """UDP callback: put raw message into queue."""
        try:
            msg_raw = codecs.decode(m[0], 'utf-8')
            self._message_queue.put(msg_raw)
        except Exception as e:
            self.get_logger().error(f"Message decode error: {e}")

    def update(self):
        """Publish UDP messages as raw string."""
        try:
            msg = self._message_queue.get_nowait()
        except queue.Empty:
            return

        msg_to_publish = String()
        msg_to_publish.data = msg
        self._pub_udp_string.publish(msg_to_publish)
        self.get_logger().debug(f"Published UDP message: {msg}")

    def stop(self):
        """Stop the UDP listener process."""
        self._udp_node.stop()


def main(args=None):
    rclpy.init(args=args)
    node = UDPBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Stopping node...")
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
