from __future__ import annotations

import nav_utils.config
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from nav_utils.geometry import point_is_close
from nav_utils.world_occupancy_grid import WorldOccupancyGrid
from rclpy.node import Node

from .self_drive_goal_selection_config import SelfDriveGoalSelectionConfig


class SelfDriveGoalSelection(Node):
    def __init__(self) -> None:
        super().__init__("self_drive_goal_selection")

        self.config: SelfDriveGoalSelectionConfig = nav_utils.config.load(self, SelfDriveGoalSelectionConfig)

        self.grid: WorldOccupancyGrid | None = None
        self.last_goal: Point | None = None

        self.create_subscription(OccupancyGrid, "occupancy_grid", self.occupancy_grid_callback, 10)

        self.publisher = self.create_publisher(Point, "goal", 10)
        self.create_timer(1.0 / self.config.goal_publish_rate_hz, self.find_and_publish_goal_if_changed)

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        self.grid = WorldOccupancyGrid(msg)

    def find_and_publish_goal_if_changed(self) -> None:
        goal = self.find_goal()
        if goal is None:
            return

        if self.last_goal is None or not point_is_close(self.last_goal, goal):
            self.last_goal = goal
            self.publisher.publish(goal)

    def find_goal(self) -> Point | None:
        if self.grid is None:
            return None

        for point in self.grid.inBoundPoints():
            if self.grid.state(point).isSelfDriveGoal:
                return point

        return None


def main() -> None:
    rclpy.init()
    node = SelfDriveGoalSelection()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
