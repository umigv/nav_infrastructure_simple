import random
from math import cos, sin

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

        zone_number = int((self.config.datum_longitude + 180) / 6) + 1
        epsg_code = f"EPSG:{32600 + zone_number}" if self.config.datum_latitude >= 0 else f"EPSG:{32700 + zone_number}"
        self._to_utm = Transformer.from_crs("EPSG:4326", epsg_code, always_xy=True)
        self._datum_x, self._datum_y = self._to_utm.transform(self.config.datum_longitude, self.config.datum_latitude)

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
        utm_x, utm_y = self._to_utm.transform(request.ll_point.longitude, request.ll_point.latitude)
        response.map_point = Point(x=utm_x - self._datum_x, y=utm_y - self._datum_y, z=0.0)

    def _update_position(self) -> None:
        now = self.get_clock().now()
        if (now - self._last_cmd_time) > Duration(seconds=self.config.cmd_vel_timeout_s):
            self._cmd_vel = Twist()

        self._x += self._cmd_vel.linear.x * self.config.update_period_s * cos(self._theta)
        self._y += self._cmd_vel.linear.x * self.config.update_period_s * sin(self._theta)
        self._theta += self._cmd_vel.angular.z * self.config.update_period_s

        map_odom_dx = (
            random.gauss(0.0, self.config.map_odom_noise_stddev_m) if self.config.map_odom_noise_stddev_m > 0 else 0.0
        )
        map_odom_dy = (
            random.gauss(0.0, self.config.map_odom_noise_stddev_m) if self.config.map_odom_noise_stddev_m > 0 else 0.0
        )
        orientation = Quaternion(z=sin(self._theta / 2.0), w=cos(self._theta / 2.0))

        self._tf_broadcaster.sendTransform(
            [
                TransformStamped(
                    header=Header(stamp=now.to_msg(), frame_id="odom"),
                    child_frame_id="base_link",
                    transform=Transform(
                        translation=Vector3(x=self._x, y=self._y, z=0.0),
                        rotation=orientation,
                    ),
                ),
                TransformStamped(
                    header=Header(stamp=now.to_msg(), frame_id="map"),
                    child_frame_id="odom",
                    transform=Transform(
                        translation=Vector3(x=map_odom_dx, y=map_odom_dy, z=0.0),
                        rotation=Quaternion(w=1.0),
                    ),
                ),
            ]
        )

        self._odom_local_publisher.publish(
            Odometry(
                header=Header(stamp=now.to_msg(), frame_id="odom"),
                child_frame_id="base_link",
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(x=self._x, y=self._y, z=0.0),
                        orientation=orientation,
                    )
                ),
                twist=TwistWithCovariance(twist=self._cmd_vel),
            )
        )

        self._odom_global_publisher.publish(
            Odometry(
                header=Header(stamp=now.to_msg(), frame_id="map"),
                child_frame_id="base_link",
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(x=self._x + map_odom_dx, y=self._y + map_odom_dy, z=0.0),
                        orientation=orientation,
                    )
                ),
                twist=TwistWithCovariance(twist=self._cmd_vel),
            )
        )


def main() -> None:
    rclpy.init()
    node = LocalizationSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
