import math

from geometry_msgs.msg import Point, Pose

from nav_utils.geometry import distance, get_yaw_radians_from_quaternion
from nav_utils.world_occupancy_grid import WorldOccupancyGrid

def select_goal(
    grid: WorldOccupancyGrid,
    robot_pose: Pose,
    waypoint: Point,
    waypoint_alignment_weight: float,
) -> Point | None:
    """Select the best drivable goal in the occupancy grid.

    Scores every drivable cell by combining forward distance from the robot
    with alignment toward the waypoint. Forward distance ensures the robot
    always drives forward, while alignment biases the direction toward the
    waypoint.

    score = forward_distance + weight * alignment_score

    Args:
        grid: World-coordinate occupancy grid.
        robot_pose: Robot pose in world coordinates.
        waypoint: Target waypoint in map coordinates.

    Returns:
        The best drivable goal point, or None if no drivable cells exist.
    """
    robot_position = robot_pose.position
    yaw = get_yaw_radians_from_quaternion(robot_pose.orientation)

    # Robot's forward direction
    forward_x = math.cos(yaw)
    forward_y = math.sin(yaw)

    # Unit vector from robot toward waypoint
    waypoint_distance = distance(robot_position, waypoint)
    if waypoint_distance == 0.0:
        return None

    waypoint_dir_x = (waypoint.x - robot_position.x) / waypoint_distance
    waypoint_dir_y = (waypoint.y - robot_position.y) / waypoint_distance

    best_goal: Point | None = None
    best_score = float("-inf")

    for cell in grid.inBoundPoints():
        if not grid.state(cell).isDrivable:
            continue

        cell_dx = cell.x - robot_position.x
        cell_dy = cell.y - robot_position.y

        forward_distance = cell_dx * forward_x + cell_dy * forward_y
        alignment_score = cell_dx * waypoint_dir_x + cell_dy * waypoint_dir_y

        score = forward_distance + waypoint_alignment_weight * alignment_score

        if score > best_score:
            best_score = score
            best_goal = cell

    return best_goal
