import math

from geometry_msgs.msg import Point, Pose
from nav_utils.world_occupancy_grid import WorldOccupancyGrid

from .autonav_goal_selection_config import GoalSelectionParams


def select_goal(
    grid: WorldOccupancyGrid, robot_pose: Pose, waypoint: Point, params: GoalSelectionParams
) -> Point | None:
    """Select the best drivable goal in the occupancy grid.

    Scores every in-bounds drivable cell using a heuristic and returns
    the highest-scoring point. Non-drivable cells are excluded.

    Args:
        grid: World-coordinate occupancy grid.
        robot_pose: Robot pose in world coordinates.
        waypoint: Target waypoint in world coordinates.
        params: Goal selection algorithm parameters.
    Returns:
        The best drivable goal point, or None if no drivable cells exist.
    """

    def heuristic(point: Point) -> float:
        if not grid.state(point).isDrivable:
            return -math.inf

        # TODO: implement a heuristic
        return 0.0

    best_point = max(grid.inBoundPoints(), key=heuristic)
    return best_point if heuristic(best_point) > -math.inf else None
