import nav_utils.config
import rclpy
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav_utils.world_occupancy_grid import WorldOccupancyGrid
from rclpy.node import Node
from std_msgs.msg import Header

from .path_planning_config import PathPlanningConfig
from .path_planning_impl import find_closest_drivable_point, generate_path, interpolate_points, smooth_path


class PathPlanner(Node):
    def __init__(self) -> None:
        super().__init__("path_planning")

        self.config: PathPlanningConfig = nav_utils.config.load(self, PathPlanningConfig)

        self.odometry: Odometry | None = None
        self.occupancy_grid: OccupancyGrid | None = None

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, "occupancy_grid", self.occupancy_grid_callback, 10)
        self.create_subscription(PointStamped, "goal", self.goal_callback, 10)

        self.path_publisher = self.create_publisher(Path, "path", 10)

    def odom_callback(self, msg: Odometry) -> None:
        if msg.header.frame_id != self.config.frame_id:
            self.get_logger().error(
                f"Frame ID of odometry ({msg.header.frame_id}) does not match config frame ID ({self.config.frame_id})"
            )
            return

        self.odometry = msg

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        if msg.header.frame_id != self.config.frame_id:
            self.get_logger().error(
                f"Frame ID of occupancy grid ({msg.header.frame_id}) does not match config frame ID ({self.config.frame_id})"
            )
            return

        self.occupancy_grid = msg

    def goal_callback(self, msg: PointStamped) -> None:
        if msg.header.frame_id != self.config.frame_id:
            self.get_logger().error(
                f"Frame ID of goal ({msg.header.frame_id}) does not match config frame ID ({self.config.frame_id})"
            )
            return

        if self.odometry is None or self.occupancy_grid is None:
            return

        grid = WorldOccupancyGrid(self.occupancy_grid)
        robot_position = self.odometry.pose.pose.position

        start = find_closest_drivable_point(grid, robot_position, self.config.max_search_radius_m)
        if start is None:
            self.get_logger().warn("No drivable area found near robot")
            return

        self.get_logger().info(f"Starting from ({start.x:.2f}, {start.y:.2f})")
        path_points = generate_path(grid, start, msg.point)
        if path_points is None:
            self.get_logger().warn("No path found to goal")
            return

        bridge = interpolate_points(robot_position, start, self.config.interpolation_resolution_m)
        self.get_logger().info(f"Bridge size: {len(bridge)}; Path size: {len(path_points)}")
        full_path = smooth_path(bridge[:-1] + path_points, self.config.spline_smoothing)

        header = Header(frame_id=self.config.frame_id, stamp=self.get_clock().now().to_msg())
        self.path_publisher.publish(
            Path(
                header=header,
                poses=[
                    PoseStamped(
                        header=header,
                        pose=Pose(position=point, orientation=Quaternion(w=1.0)),
                    )
                    for point in full_path
                ],
            )
        )


def main() -> None:
    rclpy.init()
    node = PathPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
