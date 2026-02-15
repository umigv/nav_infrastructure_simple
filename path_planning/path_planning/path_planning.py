import nav_utils.config
import rclpy
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav_utils.world_occupancy_grid import WorldOccupancyGrid
from rclpy.node import Node
from std_msgs.msg import Header

from .path_planning_config import PathPlanningConfig
from .path_planning_impl import find_closest_drivable_point, generate_path, interpolate_points


class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner")

        self._config: PathPlanningConfig = nav_utils.config.load(self, PathPlanningConfig)

        self._odometry: Odometry | None = None
        self._occupancy_grid: OccupancyGrid | None = None

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, "occupancy_grid", self.occupancy_grid_callback, 10)
        self.create_subscription(PointStamped, "goal", self.goal_callback, 10)

        self._path_publisher = self.create_publisher(Path, "path", 10)

    def odom_callback(self, msg: Odometry) -> None:
        if msg.header.frame_id != self._config.frame_id:
            self.get_logger().error(
                f"Frame ID of odometry ({msg.header.frame_id}) does not match config frame ID ({self._config.frame_id})"
            )
            return

        self._odometry = msg

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        if msg.header.frame_id != self._config.frame_id:
            self.get_logger().error(
                f"Frame ID of occupancy grid ({msg.header.frame_id}) does not match config frame ID ({self._config.frame_id})"
            )
            return

        self._occupancy_grid = msg

    def goal_callback(self, msg: PointStamped) -> None:
        if msg.header.frame_id != self._config.frame_id:
            self.get_logger().error(
                f"Frame ID of goal ({msg.header.frame_id}) does not match config frame ID ({self._config.frame_id})"
            )
            return

        if self._odometry is None or self._occupancy_grid is None:
            return

        grid = WorldOccupancyGrid(self._occupancy_grid)
        robot_position = self._odometry.pose.pose.position

        start = find_closest_drivable_point(grid, robot_position, self._config.max_search_radius)
        if start is None:
            self.get_logger().warn("No drivable area found near robot")
            return

        path_points = generate_path(grid, start, msg.point)
        if path_points is None:
            self.get_logger().warn("No path found to goal")
            return

        bridge = interpolate_points(robot_position, start, self._config.interpolation_resolution)
        full_path = bridge[:-1] + path_points

        header = Header(frame_id=self._config.frame_id, stamp=self.get_clock().now().to_msg())
        self._path_publisher.publish(
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
