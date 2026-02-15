import json

import nav_utils.config
import rclpy
import tf2_geometry_msgs  # noqa: F401 — registers PointStamped transform
import tf2_ros
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_utils.geometry import distance
from nav_utils.world_occupancy_grid import WorldOccupancyGrid
from rclpy.node import Node
from robot_localization.srv import FromLL
from std_msgs.msg import Header

from .autonav_goal_selection_config import AutonavGoalSelectionConfig
from .autonav_goal_selection_impl import select_goal


class GoalSelection(Node):
    def __init__(self):
        super().__init__("goal_selection")

        self._config: AutonavGoalSelectionConfig = nav_utils.config.load(self, AutonavGoalSelectionConfig)

        self._odometry: Odometry | None = None
        self._grid: WorldOccupancyGrid | None = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._from_ll_client = self.create_client(FromLL, "fromLL")
        self._from_ll_client.wait_for_service()

        self._waypoints: list[Point] = [self._convert_to_map_point(waypoint) for waypoint in self._load_waypoints()]
        self._current_waypoint_index = 0

        self.create_subscription(Odometry, "odom", self._odom_callback, 10)
        self.create_subscription(OccupancyGrid, "occupancy_grid", self._occupancy_grid_callback, 10)

        self._goal_publisher = self.create_publisher(PointStamped, "goal", 10)

        self.create_timer(self._config.goal_publish_period_s, self._publish_goal)

    def _odom_callback(self, msg: Odometry) -> None:
        if msg.header.frame_id != self._config.world_frame_id:
            self.get_logger().error(
                f"Frame ID of odometry ({msg.header.frame_id}) does not match config world frame ID ({self._config.world_frame_id})"
            )
            return

        self._odometry = msg
        self._advance_waypoint_if_reached()

    def _occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        if msg.header.frame_id != self._config.world_frame_id:
            self.get_logger().error(
                f"Frame ID of occupancy grid ({msg.header.frame_id}) does not match config world frame ID ({self._config.world_frame_id})"
            )
            return

        self._grid = WorldOccupancyGrid(msg)

    def _load_waypoints(self) -> list[GeoPoint]:
        with open(self._config.waypoints_file_path) as f:
            data = json.load(f)

        return [
            GeoPoint(latitude=waypoint["latitude"], longitude=waypoint["longitude"], altitude=0.0)
            for waypoint in data["waypoints"]
        ]

    def _convert_to_map_point(self, waypoint: GeoPoint) -> Point:
        request = FromLL.Request(ll_point=waypoint)
        future = self._from_ll_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map_point

    def _transform_map_to_world(self, point: Point) -> Point | None:
        try:
            stamped = PointStamped(header=Header(frame_id=self._config.map_frame_id), point=point)
            return self._tf_buffer.transform(stamped, self._config.world_frame_id).point
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"TF transform failed: {e}")
            return None

    def _advance_waypoint_if_reached(self) -> None:
        if self._odometry is None:
            return

        robot_position = self._odometry.pose.pose.position
        waypoint = self._transform_map_to_world(self._waypoints[self._current_waypoint_index])
        if waypoint is None:
            return

        if distance(robot_position, waypoint) < self._config.waypoint_reached_threshold:
            self._current_waypoint_index = (self._current_waypoint_index + 1) % len(self._waypoints)
            self.get_logger().info(f"Waypoint reached, advancing to index {self._current_waypoint_index}")

    def _publish_goal(self) -> None:
        if self._odometry is None or self._grid is None:
            return

        waypoint = self._transform_map_to_world(self._waypoints[self._current_waypoint_index])
        if waypoint is None:
            return

        goal = select_goal(self._grid, self._odometry.pose.pose, waypoint, self._config.goal_selection_params)
        if goal is None:
            self.get_logger().warn("No drivable goal found in occupancy grid")
            return

        self._goal_publisher.publish(
            PointStamped(
                header=Header(frame_id=self._config.world_frame_id, stamp=self.get_clock().now().to_msg()), point=goal
            )
        )


def main() -> None:
    rclpy.init()
    node = GoalSelection()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
