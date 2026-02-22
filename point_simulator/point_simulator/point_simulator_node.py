import random
import time
from math import cos, pi, sin

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from pyproj import Transformer
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

GPS_ZONE = "EPSG:4326"
FMCRB_METERS_ZONE = "EPSG:32617"


def lat_long_degrees_to_meters(latitude: float, longitude: float) -> tuple[float, float]:
    """
    Convert latitude and longitude from degrees to meters.
    """
    transformer = Transformer.from_crs(GPS_ZONE, FMCRB_METERS_ZONE, always_xy=True)
    return transformer.transform(longitude, latitude)[::-1]


def lat_long_meters_to_degrees(x: float, y: float) -> tuple[float, float]:
    """
    Convert latitude and longitude from degrees to meters.
    """
    transformer = Transformer.from_crs(FMCRB_METERS_ZONE, GPS_ZONE, always_xy=True)
    return transformer.transform(y, x)[::-1]


class PointSimulator(Node):
    def __init__(self):
        super().__init__("point_simulator")
        self.sub = self.create_subscription(TwistStamped, "/cmd_vel", self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.marker_pub = self.create_publisher(Marker, "/visualization_marker", 10)
        self.gps_pub = self.create_publisher(NavSatFix, "/gps_coords", 10)

        # Subscribe to gps_coords to get the origin
        self.origin_lat = None
        self.origin_lon = None
        self.origin_sub = self.create_subscription(NavSatFix, "/gps_coords", self.origin_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.update_position)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vtheta = 0.0
        self.last_time_nanoseconds = time.time() * 1e9
        self.last_cmd_time_nanoseconds = time.time() * 1e9
        self.last_gps_update_time_seconds = 0

    def origin_callback(self, msg: NavSatFix):
        self.origin_lat = msg.latitude
        self.origin_lon = msg.longitude
        self.get_logger().info(f"GPS Origin set to: {self.origin_lat}, {self.origin_lon}")

        self.destroy_subscription(self.origin_sub)

    def cmd_vel_callback(self, msg: TwistStamped):
        """Sets the robot velocity and updates when robot last recieved command. Called from the joystick subcription."""
        self.vx = msg.twist.linear.x
        self.vtheta = msg.twist.angular.z
        self.last_cmd_time_nanoseconds = self.get_clock().now().nanoseconds

    def update_position(self):
        """Sets the robot's current position based on velocity; publishes odometry and position data."""
        current_time_nanos = self.get_clock().now().nanoseconds
        dt = (current_time_nanos - self.last_time_nanoseconds) / 1e9  # Convert nanoseconds to seconds
        self.last_time_nanoseconds = current_time_nanos

        # Check if no cmd_vel was received recently (e.g., within 0.5 sec)
        if current_time_nanos - self.last_cmd_time_nanoseconds > 5e8:  # 0.5 seconds in nanoseconds
            self.vx = 0.0
            self.vtheta = 0.0  # Stop rotation to prevent drift

        # Update position
        self.x += self.vx * dt * cos(self.theta)
        self.y += self.vx * dt * sin(self.theta)
        self.theta += self.vtheta * dt

        # Publish transform
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = sin(self.theta / 2.0)
        transform.transform.rotation.w = cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(transform)

        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = cos(self.theta / 2.0)
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.angular.z = self.vtheta

        self.odom_pub.publish(odom_msg)

        # GPS Simulation
        if self.origin_lat is not None and self.origin_lon is not None:
            current_time_seconds = current_time_nanos / 1e9
            if current_time_seconds - self.last_gps_update_time_seconds >= 1:
                self.last_gps_update_time_seconds = current_time_seconds

                error_magnitude = random.uniform(0, 1)
                error_angle_radians = random.uniform(0, pi * 2)

                origin_gps_lat_meters, origin_gps_long_meters = lat_long_degrees_to_meters(
                    self.origin_lat, self.origin_lon
                )

                lat, long = lat_long_meters_to_degrees(
                    origin_gps_lat_meters + self.x + error_magnitude * cos(error_angle_radians),
                    origin_gps_long_meters - self.y + error_magnitude * sin(error_angle_radians),
                )

                gps_msg = NavSatFix()
                gps_msg.header.stamp = self.get_clock().now().to_msg()
                gps_msg.header.frame_id = "gps"
                gps_msg.latitude = lat
                gps_msg.longitude = long

                self.gps_pub.publish(gps_msg)

        # Publish Marker
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "point_simulation"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = PointSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
