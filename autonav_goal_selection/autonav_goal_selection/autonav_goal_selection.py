import json

import nav_utils.config
import rclpy
import tf2_geometry_msgs  # noqa: F401 — registers PointStamped transform
import tf2_ros
from builtin_interfaces.msg import Time
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_utils.geometry import distance
from nav_utils.world_occupancy_grid import WorldOccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from robot_localization.srv import FromLL
from std_msgs.msg import Empty, Header

from .autonav_goal_selection_config import AutonavGoalSelectionConfig
from .autonav_goal_selection_impl import select_goal


class AutonavGoalSelection(Node):
    def __init__(self) -> None:
        super().__init__("autonav_goal_selection")

        self.config: AutonavGoalSelectionConfig = nav_utils.config.load(self, AutonavGoalSelectionConfig)

        self.odometry: Odometry | None = None
        self.grid: WorldOccupancyGrid | None = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("Waiting for localization_initialized...")
        future = rclpy.Future()
        sub = self.create_subscription(
            Empty,
            "localization_initialized",
            lambda _: future.set_result(True),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        rclpy.spin_until_future_complete(self, future)
        self.destroy_subscription(sub)
        self.get_logger().info("Localization initialized.")

        self.from_ll_client = self.create_client(FromLL, "fromLL")
        self.get_logger().info("Waiting for fromLL service...")
        self.from_ll_client.wait_for_service()
        self.get_logger().info("fromLL service available.")

        self.waypoints: list[Point] = [self.convert_to_map_point(waypoint) for waypoint in self.load_waypoints()]
        self.current_waypoint_index = 0

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, "occupancy_grid", self.occupancy_grid_callback, 10)

        self.goal_publisher = self.create_publisher(PointStamped, "goal", 10)

        self.gps_waypoint_publisher = self.create_publisher(
            PointStamped, "gps_waypoint", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.create_timer(self.config.goal_publish_period_s, self.publish_goal)
        self.publish_gps_waypoint()

    def odom_callback(self, msg: Odometry) -> None:
        if msg.header.frame_id != self.config.world_frame_id:
            self.get_logger().error(
                f"Frame ID of odometry ({msg.header.frame_id}) does not match config world frame ID ({self.config.world_frame_id})"
            )
            return

        self.odometry = msg
        self.advance_waypoint_if_reached()

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        if msg.header.frame_id != self.config.world_frame_id:
            self.get_logger().error(
                f"Frame ID of occupancy grid ({msg.header.frame_id}) does not match config world frame ID ({self.config.world_frame_id})"
            )
            return

        self.grid = WorldOccupancyGrid(msg)

    def load_waypoints(self) -> list[GeoPoint]:
        with open(self.config.waypoints_file_path) as f:
            data = json.load(f)

        return [
            GeoPoint(latitude=waypoint["latitude"], longitude=waypoint["longitude"], altitude=0.0)
            for waypoint in data["waypoints"]
        ]

    def convert_to_map_point(self, waypoint: GeoPoint) -> Point:
        request = FromLL.Request(ll_point=waypoint)
        future = self.from_ll_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map_point

    def transform_map_to_world(self, point: Point) -> Point | None:
        try:
            stamped = PointStamped(header=Header(frame_id=self.config.map_frame_id), point=point)
            return self.tf_buffer.transform(stamped, self.config.world_frame_id).point
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"TF transform failed: {e}")
            return None

    def publish_gps_waypoint(self) -> None:
        map_point = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(
            f"Publishing gps waypoint ({map_point.x:.2f}, {map_point.y:.2f}) in {self.config.map_frame_id} frame"
        )
        self.gps_waypoint_publisher.publish(
            PointStamped(
                header=Header(frame_id=self.config.map_frame_id, stamp=Time(sec=0, nanosec=0)),
                point=map_point,
            )
        )

    def advance_waypoint_if_reached(self) -> None:
        if self.odometry is None or self.current_waypoint_index >= len(self.waypoints):
            return

        robot_position = self.odometry.pose.pose.position
        waypoint = self.transform_map_to_world(self.waypoints[self.current_waypoint_index])
        if waypoint is None:
            return

        if distance(robot_position, waypoint) < self.config.waypoint_reached_threshold:
            self.current_waypoint_index += 1

            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info("Final waypoint reached, stopping navigation")
                return

            self.get_logger().info(f"Waypoint reached, advancing to index {self.current_waypoint_index}")
            self.publish_gps_waypoint()

    def publish_goal(self) -> None:
        if self.odometry is None or self.grid is None or self.current_waypoint_index >= len(self.waypoints):
            return

        waypoint = self.transform_map_to_world(self.waypoints[self.current_waypoint_index])
        if waypoint is None:
            return

        goal = select_goal(self.grid, self.odometry.pose.pose, waypoint, self.config.goal_selection_params)
        if goal is None:
            self.get_logger().warn("No drivable goal found in occupancy grid")
            return

        self.get_logger().info(
            f"Publishing local goal ({goal.x:.2f}, {goal.y:.2f}) in {self.config.world_frame_id} frame"
        )
        self.goal_publisher.publish(
            PointStamped(
                header=Header(frame_id=self.config.world_frame_id, stamp=self.get_clock().now().to_msg()), point=goal
            )
        )


def main() -> None:
    rclpy.init()
    node = AutonavGoalSelection()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
