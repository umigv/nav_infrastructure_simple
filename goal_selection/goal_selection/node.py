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
    return math.pi/180 * degrees


def lat_long_to_meters(latitude: float, longitude: float) -> tuple[float, float]:
    # WGS84 to UTM (automatically selects zone)
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:32618", always_xy=True)
    return transformer.transform(longitude, latitude)


WAYPOINTS_FILE_PATH = pathlib.Path("/home/umarv/ros2_ws/src/nav_infrastructure_simple/goal_selection/waypoints.json")
PATH_PUBLISH_PERIOD_SECONDS = 2

class GoalSelectionNode(Node):
    odometry: Odometry | None = None
    waypoint_robot_relative: Point | None = None
    inflated_occupancy_grid: OccupancyGrid | None = None
    path_publisher: Publisher

    def __init__(self):
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

        self.path_publisher = self.create_publisher(
            Path,
            "/path",
            10
        )

        self.create_timer(PATH_PUBLISH_PERIOD_SECONDS, self.generate_and_publish_path)

    def odometry_callback(self, new_odometry: Odometry):
        self.odometry = new_odometry

    def gps_callback(self, new_gps_data: NavSatFix):
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

    def inflated_occupancy_grid_callback(self, new_occupancy_grid: OccupancyGrid):
        self.inflated_occupancy_grid = new_occupancy_grid

    def generate_and_publish_path(self):
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
    rclpy.init(args=args)

    goal_selection = GoalSelectionNode()

    rclpy.spin(goal_selection)

    goal_selection.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()