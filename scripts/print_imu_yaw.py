#!/usr/bin/env python3
"""Quick script to print yaw in degrees from /imu/raw."""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ImuYawPrinter(Node):
    def __init__(self):
        super().__init__("imu_yaw_printer")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, "/imu/raw", self._callback, qos)

    def _callback(self, msg: Imu):
        yaw_deg = math.degrees(yaw_from_quaternion(msg.orientation))
        print(f"Yaw: {yaw_deg:.2f} deg")


def main():
    rclpy.init()
    node = ImuYawPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
