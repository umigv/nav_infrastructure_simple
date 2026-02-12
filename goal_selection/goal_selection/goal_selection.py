import json

import rclpy
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from robot_localization.srv import FromLL

import nav_utils.config
from nav_utils.geometry import distance
from nav_utils.world_occupancy_grid import WorldOccupancyGrid
from goal_selection.goal_selection_config import GoalSelectionConfig
from goal_selection.goal_selection_impl import select_goal


class GoalSelection(Node):
    def __init__(self):
        super().__init__("goal_selection")

        self._config: GoalSelectionConfig = nav_utils.config.load(self, GoalSelectionConfig)

        self._odometry: Odometry | None = None
        self._occupancy_grid: OccupancyGrid | None = None

        self._from_ll_client = self.create_client(FromLL, "fromLL")
        self._from_ll_client.wait_for_service()

        self._waypoints: list[Point] = [
            self._convert_to_map_point(wp) for wp in self._load_waypoints()
        ]
        self._current_waypoint_index = 0

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, "occupancy_grid", self.occupancy_grid_callback, 10)

        self._goal_publisher = self.create_publisher(Point, "goal", 10)

        self.create_timer(self._config.goal_publish_period_seconds, self.publish_goal)

    def _load_waypoints(self) -> list[GeoPoint]:
        """Load GPS waypoints from the configured JSON file."""
        with open(self._config.waypoints_file_path) as f:
            data = json.load(f)

        return [
            GeoPoint(latitude=wp["latitude"], longitude=wp["longitude"], altitude=0.0)
            for wp in data["waypoints"]
        ]

    def _convert_to_map_point(self, waypoint: GeoPoint) -> Point:
        """Call the fromLL service to convert a GPS waypoint to a map-frame point."""
        request = FromLL.Request(ll_point=waypoint)
        future = self._from_ll_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map_point

    def odom_callback(self, msg: Odometry) -> None:
        self._odometry = msg
        self._advance_waypoint_if_reached()

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        self._occupancy_grid = msg

    def _advance_waypoint_if_reached(self) -> None:
        """Advance to the next waypoint if the robot is close enough."""
        robot_position = self._odometry.pose.pose.position
        current_waypoint = self._waypoints[self._current_waypoint_index]

        if distance(robot_position, current_waypoint) < self._config.waypoint_reached_threshold:
            self._current_waypoint_index = (self._current_waypoint_index + 1) % len(self._waypoints)
            self.get_logger().info(f"Waypoint reached, advancing to index {self._current_waypoint_index}")

    def publish_goal(self) -> None:
        """Select a goal from the occupancy grid and publish it."""
        if self._odometry is None or self._occupancy_grid is None:
            return

        grid = WorldOccupancyGrid(self._occupancy_grid)
        robot_pose = self._odometry.pose.pose
        waypoint = self._waypoints[self._current_waypoint_index]

        goal = select_goal(grid, robot_pose, waypoint, self._config.waypoint_alignment_weight)
        if goal is None:
            self.get_logger().warn("No drivable goal found in occupancy grid")
            return

        self._goal_publisher.publish(goal)


def main() -> None:
    rclpy.init()
    node = GoalSelection()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
