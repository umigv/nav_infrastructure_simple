import math

from geometry_msgs.msg import Point, Pose
from nav_utils.geometry import distance, get_yaw_radians_from_quaternion, rotate_by_yaw
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
        if not grid.state(point).is_drivable:
            return math.inf

        # this is rotated to the local
        delta: Point = rotate_by_yaw(
            Point(x=point.x - robot_pose.position.x, y=point.y - robot_pose.position.y),
            -get_yaw_radians_from_quaternion(robot_pose.orientation),
        )

        # add the zone-based
        x_weight = 0
        if delta.x < params.behind_robot_penalty_distance_m:
            x_weight += (params.behind_robot_penalty_distance_m - delta.x) * params.behind_robot_linear_factor
        y_weight = params.lateral_quadratic_factor * (delta.y**2)

        # because we aren't doing this as a full A*, I don't think we need the prior distance as a factor--it should already be accounted for through zone weight.
        # the last term here checks if the point is within 1m of the waypoint, and if so, lowers the priority accordingly.
        return (
            x_weight
            + y_weight
            + grid.state(point).value
            - (
                params.waypoint_proximity_weight
                * (params.waypoint_proximity_radius_m * math.sqrt(2) >= distance(point, waypoint))
            )
        )

    best_point = min(grid.in_bound_points(), key=heuristic)
    return best_point if heuristic(best_point) < math.inf else None
