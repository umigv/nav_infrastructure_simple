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
from nav_utils.geometry import make_quarternion_from_yaw
from pyproj import Transformer
from rclpy.duration import Duration
from rclpy.node import Node
from robot_localization.srv import FromLL
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

from .localization_simulator_config import LocalizationSimulatorConfig


class LocalizationSimulator(Node):
    def __init__(self) -> None:
        super().__init__("localization_simulator")

        self.config: LocalizationSimulatorConfig = nav_utils.config.load(self, LocalizationSimulatorConfig)

        zone_number = min(60, math.floor((self.config.gps_origin_longitude + 180) / 6) + 1)
        epsg_code = f"EPSG:{zone_number + (32600 if self.config.gps_origin_latitude >= 0 else 32700)}"
        self._to_utm = Transformer.from_crs("EPSG:4326", epsg_code, always_xy=True)

        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_callback, 10)

        self._odom_local_publisher = self.create_publisher(Odometry, "/odom/local", 10)
        self._odom_global_publisher = self.create_publisher(Odometry, "/odom/global", 10)

        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_service(FromLL, "fromLL", self._from_ll_callback)

        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._cmd_vel = Twist()
        self._last_cmd_time = self.get_clock().now()

        self.create_timer(self.config.update_period_s, self._update_position)

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._cmd_vel = msg
        self._last_cmd_time = self.get_clock().now()

    def _from_ll_callback(self, request: FromLL.Request, response: FromLL.Response) -> None:
        _origin_x, _origin_y = self._to_utm.transform(self.config.gps_origin_longitude, self.config.gps_origin_latitude)
        utm_x, utm_y = self._to_utm.transform(request.ll_point.longitude, request.ll_point.latitude)
        response.map_point = Point(x=utm_x - _origin_x, y=utm_y - _origin_y, z=0.0)
        return response

    def _update_position(self) -> None:
        now = self.get_clock().now()
        if (now - self._last_cmd_time) > Duration(seconds=self.config.cmd_vel_timeout_s):
            self._cmd_vel = Twist()

        self._x += self._cmd_vel.linear.x * self.config.update_period_s * math.cos(self._theta)
        self._y += self._cmd_vel.linear.x * self.config.update_period_s * math.sin(self._theta)
        self._theta += self._cmd_vel.angular.z * self.config.update_period_s
        self._theta = (self._theta + math.pi) % (2 * math.pi) - math.pi  # wrap to [-pi, pi]

        self._tf_broadcaster.sendTransform(
            [
                TransformStamped(
                    header=Header(stamp=now.to_msg(), frame_id=self.config.odom_frame_id),
                    child_frame_id=self.config.base_frame_id,
                    transform=Transform(
                        translation=Vector3(x=self._x, y=self._y, z=0.0),
                        rotation=make_quarternion_from_yaw(self._theta),
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

        self._odom_local_publisher.publish(
            Odometry(
                header=Header(stamp=now.to_msg(), frame_id=self.config.odom_frame_id),
                child_frame_id=self.config.base_frame_id,
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(x=self._x, y=self._y, z=0.0),
                        orientation=make_quarternion_from_yaw(self._theta),
                    )
                ),
                twist=TwistWithCovariance(twist=self._cmd_vel),
            )
        )

        self._odom_global_publisher.publish(
            Odometry(
                header=Header(stamp=now.to_msg(), frame_id=self.config.map_frame_id),
                child_frame_id=self.config.base_frame_id,
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(x=self._x, y=self._y, z=0.0),
                        orientation=make_quarternion_from_yaw(self._theta),
                    )
                ),
                twist=TwistWithCovariance(twist=self._cmd_vel),
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
