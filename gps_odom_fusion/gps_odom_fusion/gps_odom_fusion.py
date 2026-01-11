from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Quaternion, Twist

from collections import deque

from typing import Deque, Tuple, Optional

from dataclasses import dataclass

@dataclass
class Coordinate:
    x: float
    y: float

@dataclass
class GpsCoordinate:
    longitude: float
    latitude: float

def toCoordinate(gps_coord: GpsCoordinate, origin: GpsCoordinate) -> Coordinate:
    pass

class GpsOdomFusion(Node):
    def __init__(self):
        super().__init__('gps_odom_fusion')

        # Subscription
        self.odom_subscription = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.gps_subscription = self.create_subscription(NavSatFix, "/gps_coords", self.gps_callback, 10)

        # Publisher
        self.global_odom_publisher = self.create_publisher(Odometry, "/odom_global", 10)
        self.create_timer(0.01, self.publish_global_odom)

        # State
        self.gps_origin: Optional[GpsCoordinate] = None
        self.global_odom: Optional[Odometry] = None
        self.prev_odom: Optional[Odometry] = None

        self.initial_gps_readings: Deque[GpsCoordinate] = deque()

    def odom_callback(self, msg: Odometry) -> None:
        if not self.gps_origin:
            self.prev_odom = msg
            return

        # TODO: integrate global odom with odom differential

    def gps_callback(self, msg: NavSatFix) -> None:
        coord = GpsCoordinate(longitude=msg.longitude, latitude=msg.latitude)

        if self.gps_origin is None:
            self.initialize_gps(coord)
            return
        
        # TODO: update global odom x/y with gps data

    def publish_global_odom(self) -> None:
        if self.global_odom:
            self.global_odom_publisher.publish(self.global_odom)

    def initialize_gps(self, coord: GpsCoordinate) -> None:
        self.initial_gps_readings.append(coord)

        if len(self.initial_gps_readings) < 10:
            return
        
        avg_lon = sum(coord.longitude for coord in self.initial_gps_readings) / len(self.initial_gps_readings)
        avg_lat = sum(coord.latitude for coord in self.initial_gps_readings) / len(self.initial_gps_readings)
        self.gps_origin = GpsCoordinate(longitude=avg_lon, latitude=avg_lat)

        self.global_odom = Odometry()
        self.global_odom.header.stamp = self.get_clock().now().to_msg()
        self.global_odom.header.frame_id = "map"
        self.global_odom.child_frame_id = "base_link"

        if self.prev_odom is not None:
            self.global_odom.pose.pose.orientation = self.prev_odom.pose.pose.orientation
            self.global_odom.twist.twist = self.prev_odom.twist.twist
        else:
            self.global_odom.pose.pose.orientation = Quaternion()
            self.global_odom.twist.twist = Twist()
