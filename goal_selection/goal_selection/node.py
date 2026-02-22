import json
import math
import pathlib
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from pyproj import Transformer
from rclpy.node import Node, Publisher
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from goal_selection import path_generator


def degrees_to_radians(degrees: float) -> float:
    """Convert degrees to radians."""
    return math.pi / 180 * degrees


GPS_ZONE = "EPSG:4326"
FMCRB_METERS_ZONE = "EPSG:32617"


def lat_long_degrees_to_meters(latitude: float, longitude: float) -> tuple[float, float]:
    """
    Convert latitude and longitude from degrees to meters.

    The meters zone should be updated based on the (extremely rough) location of the robot, as it
    is only valid for a specific longitude range and hemisphere.

    TODO: It's correctness should be validated.
    """
    transformer = Transformer.from_crs(GPS_ZONE, FMCRB_METERS_ZONE, always_xy=True)
    return transformer.transform(longitude, latitude)[::-1]


@dataclass
class GPSWaypoint:
    latitude: float
    longitude: float


# Waypoints are constant goals, supplied by the competition organizer
WAYPOINTS_FILE_PATH = pathlib.Path("/home/arv/arv-ws/src/nav_infrastructure_simple/goal_selection/waypoints.json")
# How often a path is generated and published, in seconds.
PATH_PUBLISH_PERIOD_SECONDS = 2


class GoalSelectionNode(Node):
    # The most recent odometry for the robot
    odometry: Odometry | None = None
    # The closest waypoint to the robot with origin = robot start point
    waypoint_meters: Point | None = None
    # The most recent occupancy grid, inflated by the inflation layer node
    inflated_occupancy_grid: OccupancyGrid | None = None
    # The publisher for generated paths
    path_publisher: Publisher
    # The hardcoded waypoints for the course
    waypoints: list[GPSWaypoint]
    # Index of waypoint that we are currently trying to travel to
    current_waypoint_index: int = 0
    # GPS Coordinates for start of course
    origin_waypoint: GPSWaypoint | None = None

    def __init__(self):
        """Initialize goal selection node."""
        super().__init__("goal_selection")

        self.create_subscription(Odometry, "/odom", self.odometry_callback, 10)

        self.create_subscription(NavSatFix, "/gps_coords", self.gps_callback, 10)

        self.create_subscription(OccupancyGrid, "/inflated_occupancy_grid", self.inflated_occupancy_grid_callback, 10)

        self.path_publisher = self.create_publisher(Path, "/path", 10)

        self.waypoint_marker_publisher = self.create_publisher(Marker, "/waypoint_marker", 10)

        self.gps_marker_publisher = self.create_publisher(Marker, "/gps_marker", 10)

        self.create_timer(PATH_PUBLISH_PERIOD_SECONDS, self.generate_and_publish_path)

        self.waypoints = []

        with open(WAYPOINTS_FILE_PATH) as waypoints_file:
            for waypoint_json_object in json.load(waypoints_file)["waypoints"]:
                self.waypoints.append(
                    GPSWaypoint(latitude=waypoint_json_object["latitude"], longitude=waypoint_json_object["longitude"])
                )

    def odometry_callback(self, new_odometry: Odometry):
        """Store odometry data into member variable."""
        self.odometry = new_odometry

    def publish_waypoint_marker(self):
        if self.waypoint_meters is None:
            return

        waypoint_marker = Marker()

        waypoint_marker.header.frame_id = "odom"
        waypoint_marker.header.stamp = self.get_clock().now().to_msg()
        waypoint_marker.ns = "goal_selection"
        waypoint_marker.id = 0
        waypoint_marker.type = Marker.SPHERE
        waypoint_marker.action = Marker.ADD
        waypoint_marker.pose.position.x = self.waypoint_meters.x
        waypoint_marker.pose.position.y = self.waypoint_meters.y
        waypoint_marker.pose.position.z = 0.0
        waypoint_marker.scale.x = 0.4
        waypoint_marker.scale.y = 0.4
        waypoint_marker.scale.z = 0.4
        waypoint_marker.color.a = 1.0
        waypoint_marker.color.r = 0.0
        waypoint_marker.color.g = 0.0
        waypoint_marker.color.b = 1.0

        self.waypoint_marker_publisher.publish(waypoint_marker)

    def publish_gps_marker(self, x: float, y: float):
        gps_marker = Marker()

        gps_marker.header.frame_id = "odom"
        gps_marker.header.stamp = self.get_clock().now().to_msg()
        gps_marker.ns = "goal_selection"
        gps_marker.id = 0
        gps_marker.type = Marker.SPHERE
        gps_marker.action = Marker.ADD
        gps_marker.pose.position.x = x
        gps_marker.pose.position.y = y
        gps_marker.pose.position.z = 0.0
        gps_marker.scale.x = 0.4
        gps_marker.scale.y = 0.4
        gps_marker.scale.z = 0.4
        gps_marker.color.a = 1.0
        gps_marker.color.r = 0.0
        gps_marker.color.g = 0.5
        gps_marker.color.b = 0.5

        self.gps_marker_publisher.publish(gps_marker)

    def gps_callback(self, new_gps_data: NavSatFix):
        """
        Consumes the latest GPS data to find the robot relative coordinates of the
        competition waypoints.

        TODO: Currently always choses the first waypoint in the list, but should choose
        the logically next waypoint.
        """
        if self.origin_waypoint is None:
            self.origin_waypoint = GPSWaypoint(latitude=new_gps_data.latitude, longitude=new_gps_data.longitude)

        if self.odometry is None or self.current_waypoint_index >= len(self.waypoints):
            self.waypoint_meters = None
            return

        current_waypoint = self.waypoints[self.current_waypoint_index]

        origin_gps_lat_meters, origin_gps_long_meters = lat_long_degrees_to_meters(
            self.origin_waypoint.latitude, self.origin_waypoint.longitude
        )
        current_gps_lat_meters, current_gps_long_meters = lat_long_degrees_to_meters(
            new_gps_data.latitude, new_gps_data.longitude
        )

        current_waypoint_lat_meters, current_waypoint_long_meters = lat_long_degrees_to_meters(
            current_waypoint.latitude, current_waypoint.longitude
        )

        x_from_gps = current_gps_lat_meters - origin_gps_lat_meters
        y_from_gps = -(current_gps_long_meters - origin_gps_long_meters)

        self.publish_gps_marker(x_from_gps, y_from_gps)

        self.waypoint_meters = Point(
            x=current_waypoint_lat_meters - origin_gps_lat_meters + (self.odometry.pose.pose.position.x - x_from_gps),
            y=-(current_waypoint_long_meters - origin_gps_long_meters)
            + (self.odometry.pose.pose.position.y - y_from_gps),
        )

        self.publish_waypoint_marker()

        dist_to_waypoint = math.hypot(
            self.waypoint_meters.x - self.odometry.pose.pose.position.x,
            self.waypoint_meters.y - self.odometry.pose.pose.position.y,
        )

        self.get_logger().info(
            f"Current waypoint: {current_waypoint}\nNew GPS Data: {new_gps_data}\nDist to waypoint: {dist_to_waypoint}"
        )

        if dist_to_waypoint < 1:
            self.current_waypoint_index += 1
            self.gps_callback(new_gps_data)

        self.get_logger().info(f"Waypoint meters: {self.waypoint_meters}")

    def inflated_occupancy_grid_callback(self, new_occupancy_grid: OccupancyGrid):
        """Store latest inflated occupancy grid."""
        self.inflated_occupancy_grid = new_occupancy_grid

    def generate_and_publish_path(self):
        """Generate and publish path for robot to follow."""
        if self.inflated_occupancy_grid is None or self.odometry is None or self.waypoint_meters is None:
            self.path_publisher.publish(Path(header=Header(frame_id="odom"), poses=[]))
            return

        path = path_generator.generate_path(
            goal_selection_node=self,
            occupancy_grid=self.inflated_occupancy_grid,
            robot_pose=self.odometry.pose.pose,
            waypoint_meters=self.waypoint_meters,
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
