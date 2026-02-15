from dataclasses import dataclass
import json
import math
import pathlib
import sys
from typing import Any
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion
from goal_selection import path_generator
from nav_msgs.msg import Odometry, Path, OccupancyGrid
import rclpy
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from rclpy.node import Node, Publisher
import tf2_geometry_msgs  # noqa: F401 – registers PoseStamped transform handlers
from tf2_ros import Buffer, TransformListener

# How often a path is generated and published, in seconds.
PATH_PUBLISH_PERIOD_SECONDS = 2

class GoalSelectionNode(Node):
    # The most recent odometry for the robot
    odometry: Odometry | None = None
    # The current goal position in meters in the odometry frame
    waypoint_meters: Point | None = None
    # The most recent occupancy grid, inflated by the inflation layer node
    inflated_occupancy_grid: OccupancyGrid | None = None
    # The publisher for generated paths
    path_publisher: Publisher
    # TF buffer and listener for coordinate transforms
    tf_buffer: Buffer
    tf_listener: TransformListener

    def __init__(self):
        """Initialize goal selection node."""
        super().__init__('goal_selection')

        # Initialize TF buffer and listener for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Odometry,
            "/odom",
            self.odometry_callback,
            10
        )

        self.create_subscription(
            PointStamped,
            "/waypoint/self_drive",
            self.goal_point_callback,
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

        self.waypoint_marker_publisher = self.create_publisher(
            Marker,
            "/waypoint_marker",
            10
        )

        self.create_timer(PATH_PUBLISH_PERIOD_SECONDS, self.generate_and_publish_path)

    def odometry_callback(self, new_odometry: Odometry):
        """Store odometry data into member variable."""
        self.odometry = new_odometry

    def publish_waypoint_marker(self):
        if self.odometry is None:
            return
            
        waypoint_marker = Marker()

        waypoint_marker.header.frame_id = self.odometry.header.frame_id
        waypoint_marker.header.stamp = self.get_clock().now().to_msg()
        waypoint_marker.ns = 'goal_selection'
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

    def goal_point_callback(self, new_goal_point: PointStamped):
        """
        Consumes the latest goal point data and transforms it to the odometry frame.
        """
        if self.odometry is None:
            self.waypoint_meters = None
            return

        target_frame = self.odometry.header.frame_id
        source_frame = new_goal_point.header.frame_id

        # Transform the goal point to the odometry frame
        try:
            goal_pose_stamped = PoseStamped(
                header=new_goal_point.header,
                pose=Pose(
                    position=new_goal_point.point,
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                )
            )
            
            goal_transformed = self.tf_buffer.transform(goal_pose_stamped, target_frame)
            
            self.waypoint_meters = Point(
                x=goal_transformed.pose.position.x,
                y=goal_transformed.pose.position.y,
                z=goal_transformed.pose.position.z
            )
            
        except Exception as e:
            self.get_logger().error(f"TF {source_frame}->{target_frame} unavailable, skipping goal point: {e}")
            self.waypoint_meters = None
            return

        self.publish_waypoint_marker()

        dist_to_waypoint = math.hypot(
            self.waypoint_meters.x - self.odometry.pose.pose.position.x,
            self.waypoint_meters.y - self.odometry.pose.pose.position.y
        )

        self.get_logger().info(
            f"New goal point ({source_frame} frame): {new_goal_point.point}\n" + 
            f"Transformed waypoint ({target_frame} frame): {self.waypoint_meters}\n" +
            f"Dist to waypoint: {dist_to_waypoint}"
        )

        self.get_logger().info(f"Waypoint meters: {self.waypoint_meters}")

    def inflated_occupancy_grid_callback(self, new_occupancy_grid: OccupancyGrid):
        """Store latest inflated occupancy grid."""
        self.inflated_occupancy_grid = new_occupancy_grid

    def generate_and_publish_path(self):
        """Generate and publish path for robot to follow."""
        if self.inflated_occupancy_grid is None or self.odometry is None or self.waypoint_meters is None:
            self.path_publisher.publish(
                Path(
                    header=Header(
                        frame_id="odom"
                    ),
                    poses=[]
                )
            )
            return

        path = path_generator.generate_path(
            goal_selection_node=self,
            occupancy_grid=self.inflated_occupancy_grid,
            robot_pose=self.odometry.pose.pose,
            waypoint_meters=self.waypoint_meters
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