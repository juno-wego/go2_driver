#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class PointCloud2Restamper(Node):
    def __init__(self):
        super().__init__("pointcloud2_restamper")
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(PointCloud2, "cloud_out", 10)
        self._subscription = self.create_subscription(
            PointCloud2, "cloud_in", self._on_cloud, reliable_qos
        )

    def _on_cloud(self, msg):
        restamped = copy.copy(msg)
        restamped.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(restamped)


def main():
    rclpy.init()
    node = PointCloud2Restamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
