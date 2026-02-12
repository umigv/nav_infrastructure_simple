import nav_utils.config
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Header

from nav_utils.world_occupancy_grid import WorldOccupancyGrid
from path_planning.path_planner_config import PathPlannerConfig
from path_planning.path_planner_impl import find_drivable_start, interpolate_points, plan_path


class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner")

        self._config: PathPlannerConfig = nav_utils.config.load(self, PathPlannerConfig)

        self._odometry: Odometry | None = None
        self._occupancy_grid: OccupancyGrid | None = None

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, "occupancy_grid", self.occupancy_grid_callback, 10)
        self.create_subscription(Point, "goal", self.goal_callback, 10)

        self._path_publisher = self.create_publisher(Path, "path", 10)

    def odom_callback(self, msg: Odometry) -> None:
        self._odometry = msg

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        self._occupancy_grid = msg

    def goal_callback(self, goal: Point) -> None:
        if self._odometry is None or self._occupancy_grid is None:
            return

        grid = WorldOccupancyGrid(self._occupancy_grid)
        robot_position = self._odometry.pose.pose.position

        start = find_drivable_start(grid, robot_position, self._config.max_search_radius)
        if start is None:
            self.get_logger().warn("No drivable area found near robot")
            return

        path_points = plan_path(grid, start, goal)
        if path_points is None:
            self.get_logger().warn("No path found to goal")
            return

        bridge = interpolate_points(robot_position, start, self._config.interpolation_resolution)
        full_path = bridge[:-1] + path_points

        self._path_publisher.publish(
            Path(
                header=Header(frame_id="odom"),
                poses=[
                    PoseStamped(
                        header=Header(frame_id="odom"),
                        pose=Pose(position=point),
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
