import math

import nav_utils.config
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from nav_utils.geometry import get_yaw_radians_from_quaternion
from rclpy.node import Node
from scipy.interpolate import splev, splprep

from .pure_pursuit_config import PurePursuitConfig


class PurePursuitNode(Node):
    def __init__(self) -> None:
        super().__init__("pure_pursuit_lookahead")
        self.get_logger().info("Pure Pursuit Node started.")

        # Parameters
        self.config: PurePursuitConfig = nav_utils.config.load(self, PurePursuitConfig)

        # State
        self.smoothed_path_points: list[tuple[float, float]] = []  # x, y
        self.pose: tuple[float, float, float] | None = None  # x, y, yaw
        self.visited: int = 0  # index of last visited/processed point in path when finding lookahead point
        self.reached_goal: bool = False
        self.current_speed: float = 0.0

        self.odom_sub: rclpy.Subscription = self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.path_sub: rclpy.Subscription = self.create_subscription(Path, "path", self.path_callback, 10)
        self.cmd_pub: rclpy.Publisher = self.create_publisher(Twist, "nav_cmd_vel", 10)
        self.path_pub: rclpy.Publisher = self.create_publisher(Path, "smoothed_path", 10)

        self.create_timer(self.config.control_loop_sample_time, self.control_loop)

    def path_callback(self, path_msg: Path) -> None:
        self.get_logger().info("Received a new path from subscription.")
        self.smoothed_path_points = self.smooth_path_spline(path_msg)
        self.publish_smoothed_path(self.smoothed_path_points)
        self.reached_goal = False
        self.visited = 0

    def publish_smoothed_path(self, path: list[tuple[float, float]]) -> None:
        # Create and publish a nav_msgs/msg/Path.msg using the smoothed path points
        if not path:
            return
        path_msg = Path()
        now = self.get_clock().now().to_msg()
        path_msg.header.stamp = now
        path_msg.header.frame_id = "odom"
        for x, y in path:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = "odom"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def smooth_path_spline(self, path: Path) -> list[tuple[float, float]]:
        # Fit a B-spline to the waypoints in the path
        # Need at least 4 points to fit a spline, return the original path if < 4 points
        if len(path.poses) <= 3:
            return [(pose_stamped.pose.position.x, pose_stamped.pose.position.y) for pose_stamped in path.poses]

        x = [pose_stamped.pose.position.x for pose_stamped in path.poses]
        y = [pose_stamped.pose.position.y for pose_stamped in path.poses]

        # Fit spline with no periodicity, and smoothing factor `s`
        tck, _ = splprep([x, y], s=self.config.smoothing, per=0)

        # Interpolate with 3 times the number of points
        num_points = 3 * len(path.poses)
        u_fine = np.linspace(0, 1, num_points)
        x_smooth, y_smooth = splev(u_fine, tck)

        return list(zip(x_smooth, y_smooth, strict=True))

    def odom_callback(self, msg: Odometry) -> None:
        # Calculate and update the pose and current speed
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.pose = (pos.x, pos.y, get_yaw_radians_from_quaternion(ori))
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.hypot(vx, vy)

    def find_lookahead_point(self) -> tuple[float, float] | None:
        # Find point on path to move toward based on current pose, smoothed path, and current speed
        if self.pose is None or not self.smoothed_path_points:
            return None

        x, y, yaw = self.pose
        goal_x, goal_y = self.smoothed_path_points[-1]
        goal_dist = math.hypot(goal_x - x, goal_y - y)

        self.lookahead_distance = max(
            self.config.min_lookahead,
            min(self.config.max_lookahead, self.config.base_lookahead + self.config.k_speed * self.current_speed),
        )
        r = self.lookahead_distance

        if goal_dist < r:
            self.get_logger().info("REACHED GOAL")
            self.reached_goal = True
            self.smoothed_path_points = []
            return None

        # Try to find interpolated segment intersection
        # Allows one index backwards due to adaptive lookahead
        for i in range(max(0, self.visited - 1), len(self.smoothed_path_points) - 1):
            gx1, gy1 = self.smoothed_path_points[i]
            gx2, gy2 = self.smoothed_path_points[i + 1]

            dx1, dy1 = gx1 - x, gy1 - y
            dx2, dy2 = gx2 - x, gy2 - y

            lx1 = math.cos(-yaw) * dx1 - math.sin(-yaw) * dy1
            ly1 = math.sin(-yaw) * dx1 + math.cos(-yaw) * dy1
            lx2 = math.cos(-yaw) * dx2 - math.sin(-yaw) * dy2
            ly2 = math.sin(-yaw) * dx2 + math.cos(-yaw) * dy2

            d1 = math.hypot(lx1, ly1)
            d2 = math.hypot(lx2, ly2)

            # Skip segment if it's completely behind
            if lx1 < -0.05 and lx2 < -0.05:
                self.visited = i + 1
                continue

            # If segment crosses the lookahead radius, do linear interpolation
            if d1 < r <= d2 and lx2 > 0.0:
                ratio = (r - d1) / (d2 - d1)
                ratio = max(0.0, min(1.0, ratio))  # clamp for safety
                lx = lx1 + ratio * (lx2 - lx1)
                ly = ly1 + ratio * (ly2 - ly1)
                self.visited = i
                return lx, ly

        # Fallback: no intersection with lookahead circle, pick the first forward point outside lookahead distance
        for j in range(self.visited, len(self.smoothed_path_points)):
            gx, gy = self.smoothed_path_points[j]
            dx, dy = gx - x, gy - y
            lx = math.cos(-yaw) * dx - math.sin(-yaw) * dy
            ly = math.sin(-yaw) * dx + math.cos(-yaw) * dy
            d = math.hypot(lx, ly)
            if lx > -0.1 and d >= r:
                self.visited = j
                self.get_logger().warn(f"Fallback: chasing ahead point at index {j}")
                return lx, ly

        # Nothing usable found
        self.get_logger().warn("No valid lookahead point found - stopping")
        return None

    def control_loop(self) -> None:
        # Calculate and publish the linear and angular velocity to drive toward the lookahead point
        local_point = self.find_lookahead_point()

        # If no valid lookahead point found, stop (i.e., publish message with zero velocity)
        if local_point is None:
            self.cmd_pub.publish(Twist())
            return

        local_x, local_y = local_point
        curvature = 2 * local_y / (local_x**2 + local_y**2)

        # Scale up linear
        raw_linear = self.lookahead_distance * self.config.speed_percent

        # More linear velocity on straights
        raw_angular = raw_linear * curvature

        # Scale both linear and angular if angular exceeds limit
        # This keeps the proportions of pure pursuit
        if abs(raw_angular) > self.config.max_angular_speed:
            scale = self.config.max_angular_speed / abs(raw_angular)
            linear = raw_linear * scale
            angular = raw_angular * scale
        else:
            linear = raw_linear
            angular = raw_angular

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
