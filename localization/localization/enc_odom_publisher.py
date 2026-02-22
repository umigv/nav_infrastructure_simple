import math

import nav_utils.config
import rclpy
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    Transform,
    TransformStamped,
    Twist,
    TwistWithCovariance,
    TwistWithCovarianceStamped,
    Vector3,
)
from nav_msgs.msg import Odometry
from nav_utils.geometry import make_quaternion_from_yaw
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

from .enc_odom_publisher_config import EncOdomPublisherConfig


class EncOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("enc_odom_publisher")

        self.config: EncOdomPublisherConfig = nav_utils.config.load(self, EncOdomPublisherConfig)

        self.create_subscription(TwistWithCovarianceStamped, "enc_vel", self.enc_vel_callback, 10)

        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_time: Time | None = None

        self.pose_covariance = [0.0] * 36
        self.pose_covariance[0] = self.config.pose_x_variance_m2
        self.pose_covariance[7] = self.config.pose_y_variance_m2
        self.pose_covariance[35] = self.config.pose_yaw_variance_rad2

    def enc_vel_callback(self, msg: TwistWithCovarianceStamped) -> None:
        if msg.header.frame_id != self.config.base_frame_id:
            self.get_logger().warn(f"Dropping enc_vel: frame '{msg.header.frame_id}' != '{self.config.base_frame_id}'")
            return

        current_time = Time.from_msg(msg.header.stamp)
        try:
            if self.prev_time is None:
                return

            dt = (current_time - self.prev_time).nanoseconds * 1e-9
            if dt <= 0.0 or dt > 1.0:
                return

            linear_vel = msg.twist.twist.linear.x
            angular_vel = msg.twist.twist.angular.z

            self.x += linear_vel * math.cos(self.theta) * dt
            self.y += linear_vel * math.sin(self.theta) * dt
            self.theta += angular_vel * dt
            self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

            now = current_time.to_msg()
            quaternion = make_quaternion_from_yaw(self.theta)

            self.odom_publisher.publish(
                Odometry(
                    header=Header(stamp=now, frame_id=self.config.odom_frame_id),
                    child_frame_id=self.config.base_frame_id,
                    pose=PoseWithCovariance(
                        pose=Pose(
                            position=Point(x=self.x, y=self.y, z=0.0),
                            orientation=quaternion,
                        ),
                        covariance=self.pose_covariance,
                    ),
                    twist=TwistWithCovariance(
                        twist=Twist(
                            linear=Vector3(x=linear_vel),
                            angular=Vector3(z=angular_vel),
                        ),
                        covariance=msg.twist.covariance,
                    ),
                )
            )

            self.tf_broadcaster.sendTransform(
                TransformStamped(
                    header=Header(stamp=now, frame_id=self.config.odom_frame_id),
                    child_frame_id=self.config.base_frame_id,
                    transform=Transform(
                        translation=Vector3(x=self.x, y=self.y, z=0.0),
                        rotation=quaternion,
                    ),
                )
            )
        finally:
            self.prev_time = current_time


def main() -> None:
    rclpy.init()
    node = EncOdomPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
