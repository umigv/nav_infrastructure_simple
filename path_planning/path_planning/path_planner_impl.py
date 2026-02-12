import heapq
import math
from collections import deque
from dataclasses import dataclass, field

from geometry_msgs.msg import Point

from nav_utils.geometry import distance
from nav_utils.world_occupancy_grid import WorldOccupancyGrid


@dataclass(order=True)
class _PriorityEntry:
    cost: float
    key: int = field(compare=False)


def interpolate_points(start: Point, end: Point, resolution: float) -> list[Point]:
    """Linearly interpolate between two points at a given resolution.

    Generates evenly spaced points from start to end (inclusive of both).

    Args:
        start: Starting point.
        end: Ending point.
        resolution: Distance between consecutive interpolated points.

    Returns:
        List of interpolated points from start to end.
    """
    total_distance = distance(start, end)
    num_steps = math.ceil(total_distance / resolution)

    return [
        Point(
            x=start.x + (end.x - start.x) * i / num_steps,
            y=start.y + (end.y - start.y) * i / num_steps,
            z=0.0,
        )
        for i in range(num_steps + 1)
    ]


def find_drivable_start(
    grid: WorldOccupancyGrid,
    robot_position: Point,
    max_search_radius: float,
) -> Point | None:
    """BFS from the robot position to find the nearest drivable cell.

    The robot sits in unknown space on the occupancy grid (behind the camera
    view), so we search forward until we hit a drivable cell.

    Args:
        grid: World-coordinate occupancy grid.
        robot_position: Robot position in world coordinates.
        max_search_radius: Maximum distance (meters) from robot_position to
            search. Prevents unbounded expansion since the grid returns
            UNKNOWN for out-of-bounds points.

    Returns:
        The nearest drivable point, or None if none exists within the radius.
    """
    if grid.state(robot_position).isDrivable:
        return robot_position

    visited: set[int] = {grid.hash_key(robot_position)}
    candidates: deque[Point] = deque([robot_position])

    while candidates:
        current = candidates.popleft()

        for neighbor in grid.neighbors_forward(current):
            key = grid.hash_key(neighbor)
            if key in visited:
                continue
            visited.add(key)

            if distance(neighbor, robot_position) > max_search_radius:
                continue

            if grid.state(neighbor).isDrivable:
                return neighbor

            if grid.state(neighbor).isUnknown:
                candidates.append(neighbor)

    return None


def plan_path(grid: WorldOccupancyGrid, start: Point, goal: Point) -> list[Point] | None:
    """A* from start toward goal over a WorldOccupancyGrid.

    Both start and goal should be drivable cells within the grid. If the goal
    is unreachable (e.g. blocked by obstacles), the path leads to the closest
    reachable cell instead.

    Args:
        grid: World-coordinate occupancy grid.
        start: Drivable start point in world coordinates.
        goal: Drivable goal point in world coordinates.

    Returns:
        List of world-coordinate points from start to goal (or closest
        reachable point), or None if no drivable cells are reachable.
    """
    start_key = grid.hash_key(start)
    goal_key = grid.hash_key(goal)

    if start_key == goal_key:
        return [start]

    came_from: dict[int, int | None] = {start_key: None}
    point_of: dict[int, Point] = {start_key: start}
    cost_so_far: dict[int, float] = {start_key: 0.0}
    priority_queue: list[_PriorityEntry] = [_PriorityEntry(distance(start, goal), start_key)]

    best_goal_key = start_key
    best_goal_distance = distance(start, goal)

    while priority_queue:
        current_key = heapq.heappop(priority_queue).key
        current_point = point_of[current_key]

        if current_key == goal_key:
            best_goal_key = goal_key
            break

        current_distance = distance(current_point, goal)
        if current_distance < best_goal_distance:
            best_goal_key = current_key
            best_goal_distance = current_distance

        for neighbor in grid.neighbors8(current_point):
            if not grid.state(neighbor).isDrivable:
                continue

            neighbor_key = grid.hash_key(neighbor)
            neighbor_cost = cost_so_far[current_key] + distance(current_point, neighbor)

            if neighbor_key not in cost_so_far or neighbor_cost < cost_so_far[neighbor_key]:
                cost_so_far[neighbor_key] = neighbor_cost
                came_from[neighbor_key] = current_key
                point_of[neighbor_key] = neighbor
                priority = neighbor_cost + distance(neighbor, goal)
                heapq.heappush(priority_queue, _PriorityEntry(priority, neighbor_key))

    if best_goal_key == start_key:
        return None

    path: list[Point] = []
    key: int | None = best_goal_key
    while key is not None:
        path.append(point_of[key])
        key = came_from[key]
    path.reverse()

    return path
