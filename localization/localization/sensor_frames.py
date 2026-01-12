import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler

from typing import TypeAlias

Xyzrpy: TypeAlias = tuple[float, float, float, float, float, float]

BASE_LINK = "base_link"
TRANSFORMS: dict[str, Xyzrpy] = {
    "imu_link": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "gps_link": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
}

class StaticSensorFrames(Node):
    def __init__(self) -> None:
        super().__init__("static_sensor_frames")

        self.broadcaster = StaticTransformBroadcaster(self)

        self.broadcaster.sendTransform([
            self.make_tf(child_link=child_link, xyzrpy=xyzrpy) for child_link, xyzrpy in TRANSFORMS.items()
        ])

        published = ", ".join(f"{BASE_LINK}->{child}" for child in TRANSFORMS)
        self.get_logger().info(f"Published static TFs: {published}")

    def make_tf(self, child_link: str, xyzrpy: Xyzrpy) -> TransformStamped:
        x, y, z, roll, pitch, yaw = xyzrpy

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = BASE_LINK
        t.child_frame_id = child_link
        
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        return t

def main() -> None:
    rclpy.init()
    node = StaticSensorFrames()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
