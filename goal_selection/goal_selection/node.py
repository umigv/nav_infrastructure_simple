import json
import math
import pathlib
import sys
from typing import Any
from geometry_msgs.msg import Point
from goal_selection import path_generator
from nav_msgs.msg import Odometry, Path, OccupancyGrid
import rclpy
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node, Publisher
from pyproj import Transformer

def degrees_to_radians(degrees: float) -> float:
    """Convert degrees to radians."""
    return math.pi/180 * degrees


def lat_long_to_meters(latitude: float, longitude: float) -> tuple[float, float]:
    """
    Convert latitude and longitude from degrees to meters.
    
    This conversion process depends on which "meters" coordinate system you use.
    
    TODO: It's correctness should be validated.
    """
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:32618", always_xy=True)
    return transformer.transform(longitude, latitude)


# Waypoints are constant goals, supplied by the competition organizer
WAYPOINTS_FILE_PATH = pathlib.Path("/home/arv/arv-ws/src/nav_infrastructure_simple/goal_selection/waypoints.json")
# How often a path is generated and published, in seconds.
PATH_PUBLISH_PERIOD_SECONDS = 2

class GoalSelectionNode(Node):
    # The most recent odometry for the robot
    odometry: Odometry | None = None
    # The closest waypoint to the robot in robot relative coordinates
    waypoint_robot_relative: Point | None = None
    # The most recent occupancy grid, inflated by the inflation layer node
    inflated_occupancy_grid: OccupancyGrid | None = None
    # The publisher for generated paths
    path_publisher: Publisher

    def __init__(self):
        """Initialize goal selection node."""
        super().__init__('goal_selection')

        self.create_subscription(
            Odometry,
            "/odom",
            self.odometry_callback,
            10
        )

        self.create_subscription(
            NavSatFix,
            "/gps_coords",
            self.gps_callback,
            10
        )

        self.create_subscription(
            OccupancyGrid,
            "/inflated_occupancy_grid",
            self.inflated_occupancy_grid_callback,
            10
        )

        self.create_subscription(
            Point,
            "/waypoint_override",
            self.waypoint_override_callback,
            10
        )

        self.path_publisher = self.create_publisher(
            Path,
            "/path",
            10
        )

        self.create_timer(PATH_PUBLISH_PERIOD_SECONDS, self.generate_and_publish_path)

    def odometry_callback(self, new_odometry: Odometry):
        """Store odometry data into member variable."""
        self.odometry = new_odometry

    def gps_callback(self, new_gps_data: NavSatFix):
        """
        Consumes the latest GPS data to find the robot relative coordinates of the
        competition waypoints.

        TODO: Currently always choses the first waypoint in the list, but should choose
        the logically next waypoint.
        """
        if self.odometry is None:
            return

        waypoints: Any
        with open(WAYPOINTS_FILE_PATH, "r") as waypoints_file:
            waypoints = json.load(waypoints_file)["waypoints"]

        current_waypoint = waypoints[0]

        current_long_meters, current_lat_meters = lat_long_to_meters(current_waypoint['latitude'], current_waypoint['longitude'])
        new_long_meters, new_lat_meters = lat_long_to_meters(new_gps_data.latitude, new_gps_data.longitude)

        self.waypoint_robot_relative = Point(
            x=current_lat_meters-new_lat_meters,
            y=-(current_long_meters-new_long_meters)
        )

        self.get_logger().info(f"Robot relative waypoint: {self.waypoint_robot_relative}")

    def waypoint_override_callback(self, waypoint: Point):
        """Override waypoint to specified value"""
        if self.odometry is None:
            return
        
        self.waypoint_robot_relative = waypoint

    def inflated_occupancy_grid_callback(self, new_occupancy_grid: OccupancyGrid):
        """Store latest inflated occupancy grid."""
        self.inflated_occupancy_grid = new_occupancy_grid

    def generate_and_publish_path(self):
        """Generate and publish path for robot to follow."""
        if self.inflated_occupancy_grid is None or self.odometry is None or self.waypoint_robot_relative is None:
            return

        path = path_generator.generate_path(
            goal_selection_node=self,
            occupancy_grid=self.inflated_occupancy_grid,
            robot_pose=self.odometry.pose.pose,
            waypoint_robot_relative=self.waypoint_robot_relative
        )

        self.path_publisher.publish(path)

        


def main(args=None):
    """Node entrypoint."""
    rclpy.init(args=args)

    goal_selection = GoalSelectionNode()

    rclpy.spin(goal_selection)

    goal_selection.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()