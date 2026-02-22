import math

import nav_utils.config
import rclpy
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
    Transform,
    TransformStamped,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from nav_msgs.msg import Odometry
from nav_utils.geometry import make_quaternion_from_yaw
from pyproj import Transformer
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from robot_localization.srv import FromLL
from std_msgs.msg import Empty, Header
from tf2_ros import TransformBroadcaster

from .localization_simulator_config import LocalizationSimulatorConfig


class LocalizationSimulator(Node):
    def __init__(self) -> None:
        super().__init__("localization_simulator")

        self.config: LocalizationSimulatorConfig = nav_utils.config.load(self, LocalizationSimulatorConfig)

        zone_number = min(60, math.floor((self.config.gps_origin_longitude + 180) / 6) + 1)
        epsg_code = f"EPSG:{zone_number + (32600 if self.config.gps_origin_latitude >= 0 else 32700)}"
        self.to_utm = Transformer.from_crs("EPSG:4326", epsg_code, always_xy=True)

        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        self.odom_local_publisher = self.create_publisher(Odometry, "odom/local", 10)
        self.odom_global_publisher = self.create_publisher(Odometry, "odom/global", 10)
        self.localization_init_publisher = self.create_publisher(
            Empty,
            "localization_initialized",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_service(FromLL, "fromLL", self.from_ll_callback)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.cmd_vel = Twist()
        self.last_cmd_time = self.get_clock().now()

        self.create_timer(self.config.update_period_s, self.update_position)
        self.localization_init_publisher.publish(Empty())

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.cmd_vel = msg
        self.last_cmd_time = self.get_clock().now()

    def from_ll_callback(self, request: FromLL.Request, response: FromLL.Response) -> None:
        origin_x, origin_y = self.to_utm.transform(self.config.gps_origin_longitude, self.config.gps_origin_latitude)
        utm_x, utm_y = self.to_utm.transform(request.ll_point.longitude, request.ll_point.latitude)
        response.map_point = Point(x=utm_x - origin_x, y=utm_y - origin_y, z=0.0)
        return response

    def update_position(self) -> None:
        now = self.get_clock().now()
        if (now - self.last_cmd_time) > Duration(seconds=self.config.cmd_vel_timeout_s):
            self.cmd_vel = Twist()

        self.x += self.cmd_vel.linear.x * self.config.update_period_s * math.cos(self.yaw)
        self.y += self.cmd_vel.linear.x * self.config.update_period_s * math.sin(self.yaw)
        self.yaw += self.cmd_vel.angular.z * self.config.update_period_s
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi  # wrap to [-pi, pi]

        self.tf_broadcaster.sendTransform(
            [
                TransformStamped(
                    header=Header(stamp=now.to_msg(), frame_id=self.config.odom_frame_id),
                    child_frame_id=self.config.base_frame_id,
                    transform=Transform(
                        translation=Vector3(x=self.x, y=self.y, z=0.0),
                        rotation=make_quaternion_from_yaw(self.yaw),
                    ),
                ),
                TransformStamped(
                    header=Header(stamp=now.to_msg(), frame_id=self.config.map_frame_id),
                    child_frame_id=self.config.odom_frame_id,
                    transform=Transform(
                        translation=Vector3(x=0.0, y=0.0, z=0.0),
                        rotation=Quaternion(w=1.0),
                    ),
                ),
            ]
        )

        self.odom_local_publisher.publish(
            Odometry(
                header=Header(stamp=now.to_msg(), frame_id=self.config.odom_frame_id),
                child_frame_id=self.config.base_frame_id,
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(x=self.x, y=self.y, z=0.0),
                        orientation=make_quaternion_from_yaw(self.yaw),
                    )
                ),
                twist=TwistWithCovariance(twist=self.cmd_vel),
            )
        )

        self.odom_global_publisher.publish(
            Odometry(
                header=Header(stamp=now.to_msg(), frame_id=self.config.map_frame_id),
                child_frame_id=self.config.base_frame_id,
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(x=self.x, y=self.y, z=0.0),
                        orientation=make_quaternion_from_yaw(self.yaw),
                    )
                ),
                twist=TwistWithCovariance(twist=self.cmd_vel),
            )
        )


def main() -> None:
    rclpy.init()
    node = LocalizationSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
