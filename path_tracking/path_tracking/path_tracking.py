from __future__ import annotations

import math
from dataclasses import dataclass

import nav_utils.config
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, TwistStamped, Vector3
from nav_msgs.msg import Odometry, Path
from nav_utils.geometry import get_yaw_radians_from_quaternion
from rclpy.node import Node
from scipy.interpolate import splev, splprep
from std_msgs.msg import Header

from .path_tracking_config import PathTrackingConfig


@dataclass
class Point2d:
    x: float
    y: float

    def __add__(self, other: Point2d) -> Point2d:
        return Point2d(x=self.x + other.x, y=self.y + other.y)

    def __sub__(self, other: Point2d) -> Point2d:
        return Point2d(x=self.x - other.x, y=self.y - other.y)

    def __mul__(self, scalar: float) -> Point2d:
        return Point2d(x=self.x * scalar, y=self.y * scalar)

    def rotate_by(self, angle: float) -> Point2d:
        c, s = math.cos(angle), math.sin(angle)
        return Point2d(x=c * self.x - s * self.y, y=s * self.x + c * self.y)

    def mag(self) -> float:
        return math.hypot(self.x, self.y)

    def to_ros(self) -> Point:
        return Point(x=self.x, y=self.y, z=0.0)


@dataclass
class Pose2d:
    point: Point2d
    yaw: float

    def to_local(self, point: Point2d) -> Point2d:
        return (point - self.point).rotate_by(-self.yaw)


class PathTracking(Node):
    def __init__(self) -> None:
        super().__init__("path_tracking")

        self.config: PathTrackingConfig = nav_utils.config.load(self, PathTrackingConfig)

        self.smoothed_path_points: list[Point2d] = []
        self.path_cursor: int = 0
        self.pose: Pose2d | None = None
        self.current_speed: float = 0.0

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.create_subscription(Path, "path", self.path_callback, 10)

        self.cmd_vel_publisher = self.create_publisher(TwistStamped, "nav_cmd_vel", 10)
        self.path_publisher = self.create_publisher(Path, "smoothed_path", 10)

        self.create_timer(self.config.control_period_s, self.control_loop)

    def odom_callback(self, msg: Odometry) -> None:
        if msg.header.frame_id != self.config.odom_frame_id:
            self.get_logger().warn(f"Dropping odometry: frame '{msg.header.frame_id}' != '{self.config.odom_frame_id}'")
            return
        if msg.child_frame_id != self.config.base_frame_id:
            self.get_logger().warn(
                f"Dropping odometry: child_frame_id '{msg.child_frame_id}' != '{self.config.base_frame_id}'"
            )
            return

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.pose = Pose2d(point=Point2d(x=position.x, y=position.y), yaw=get_yaw_radians_from_quaternion(orientation))
        self.current_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def path_callback(self, path_msg: Path) -> None:
        if path_msg.header.frame_id != self.config.odom_frame_id:
            self.get_logger().warn(
                f"Dropping path: frame '{path_msg.header.frame_id}' != '{self.config.odom_frame_id}'"
            )
            return

        self.get_logger().info("Received a new path from subscription.")
        self.smoothed_path_points = self.fit_b_spline(path_msg)
        self.publish_smoothed_path(self.smoothed_path_points)
        self.path_cursor = 0

    def publish_smoothed_path(self, path: list[Point2d]) -> None:
        if not path:
            return

        now = self.get_clock().now().to_msg()
        self.path_publisher.publish(
            Path(
                header=Header(stamp=now, frame_id=self.config.odom_frame_id),
                poses=[
                    PoseStamped(
                        header=Header(stamp=now, frame_id=self.config.odom_frame_id),
                        pose=Pose(position=point.to_ros(), orientation=Quaternion(w=1.0)),
                    )
                    for point in path
                ],
            )
        )

    def fit_b_spline(self, path: Path) -> list[Point2d]:
        if len(path.poses) <= 3:
            return [Point2d(x=p.pose.position.x, y=p.pose.position.y) for p in path.poses]

        xs = [pose_stamped.pose.position.x for pose_stamped in path.poses]
        ys = [pose_stamped.pose.position.y for pose_stamped in path.poses]

        # Fit spline with no periodicity, and smoothing factor
        tck, _ = splprep([xs, ys], s=self.config.spline_smoothing, per=0)

        # Interpolate with 3 times the number of points
        u_fine = np.linspace(0, 1, 3 * len(path.poses))
        x_smooth, y_smooth = splev(u_fine, tck)

        return [Point2d(x=float(x), y=float(y)) for x, y in zip(x_smooth, y_smooth, strict=True)]

    def compute_lookahead_distance(self) -> float:
        lookahead = self.config.base_lookahead_distance_m + self.config.lookahead_speed_gain * self.current_speed
        return max(self.config.min_lookahead_distance_m, min(lookahead, self.config.max_lookahead_distance_m))

    def find_lookahead_point(self, lookahead_distance: float) -> Point2d | None:
        # Allows one index backwards due to adaptive lookahead
        start = self.path_cursor - 1 if self.path_cursor != 0 else 0
        end = len(self.smoothed_path_points) - 1

        assert self.pose is not None
        for i in range(start, end):
            local1 = self.pose.to_local(self.smoothed_path_points[i])
            local2 = self.pose.to_local(self.smoothed_path_points[i + 1])

            if local1.x < -0.05 and local2.x < -0.05:
                self.path_cursor = i + 1
                continue

            d1 = local1.mag()
            d2 = local2.mag()
            if d1 < lookahead_distance <= d2 and local2.x > 0.0:
                t = max(0.0, min(1.0, (lookahead_distance - d1) / (d2 - d1)))
                self.path_cursor = i
                return local1 + (local2 - local1) * t

        return None

    def find_forward_point(self, lookahead_distance: float) -> Point2d | None:
        assert self.pose is not None
        for j in range(self.path_cursor, len(self.smoothed_path_points)):
            local = self.pose.to_local(self.smoothed_path_points[j])
            if local.x > -0.1 and local.mag() >= lookahead_distance:
                self.path_cursor = j
                self.get_logger().warn(f"Lookahead intersection not found, chasing forward point at index {j}")
                return local

        return None

    def control_loop(self) -> None:
        if self.pose is None or not self.smoothed_path_points:
            return

        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.config.base_frame_id)
        lookahead_distance = self.compute_lookahead_distance()
        if (self.smoothed_path_points[-1] - self.pose.point).mag() < lookahead_distance:
            self.get_logger().info("Reached goal - stopping")
            self.smoothed_path_points = []
            self.cmd_vel_publisher.publish(TwistStamped(header=header))
            return

        lookahead_point = self.find_lookahead_point(lookahead_distance) or self.find_forward_point(lookahead_distance)
        if lookahead_point is None:
            self.get_logger().warn("No valid lookahead point found - stopping")
            self.cmd_vel_publisher.publish(TwistStamped(header=header))
            return

        curvature = 2 * lookahead_point.y / (lookahead_point.x**2 + lookahead_point.y**2)
        linear = lookahead_distance * self.config.linear_speed_gain
        angular = linear * curvature
        scale = min(1.0, self.config.max_angular_speed_radps / abs(angular)) if angular != 0.0 else 1.0
        self.cmd_vel_publisher.publish(
            TwistStamped(
                header=header,
                twist=Twist(linear=Vector3(x=linear * scale), angular=Vector3(z=angular * scale)),
            )
        )


def main() -> None:
    rclpy.init()
    node = PathTracking()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
