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

def degrees_to_radians(degrees: float) -> float:
    return math.pi/180 * degrees

# https://en.wikipedia.org/wiki/Geographic_coordinate_system#Length_of_a_degree
def calculate_lat_long_difference_in_meters(lat1, long1, lat2, long2) -> tuple[float, float]:
    latitude_midpoint = (lat1+long1)/2.0

    meters_per_degree_latitude = 111132.954 - 559.822 * math.cos(2.0 * latitude_midpoint) + 1.175 * math.cos(4.0 * latitude_midpoint)
    meters_per_degree_longitude = (math.pi/180) * 6367449 * math.cos (latitude_midpoint)

    return (lat2 - lat1) * meters_per_degree_latitude, (long2 - long1) * meters_per_degree_longitude


WAYPOINTS_FILE_PATH = pathlib.Path("/home/arv/arv-ws/src/nav_infrastructure_simple/goal_selection/waypoints.json")
PATH_PUBLISH_PERIOD_SECONDS = 0.1

class GoalSelectionNode(Node):
    odometry: Odometry | None = None
    waypoint_robot_relative: Point | None = None
    occupancy_grid: OccupancyGrid | None = None
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
            "/occupancy_grid",
            self.occupancy_grid_callback,
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

        latitude_difference, longitude_difference = calculate_lat_long_difference_in_meters(
            current_waypoint["latitude"], current_waypoint["longitude"],
            new_gps_data.latitude, new_gps_data.longitude
        )

        self.waypoint_robot_relative = Point(
            x=self.odometry.pose.pose.position.x + longitude_difference, 
            y=self.odometry.pose.pose.position.y + latitude_difference, 
        )

    def occupancy_grid_callback(self, new_occupancy_grid: OccupancyGrid):
        self.occupancy_grid = new_occupancy_grid

    def generate_and_publish_path(self):
        if self.occupancy_grid is None or self.odometry is None or self.waypoint_robot_relative is None:
            return

        path = path_generator.generate_path(
            occupancy_grid=self.occupancy_grid,
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