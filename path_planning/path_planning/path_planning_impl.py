import heapq
import math
from collections import deque
from dataclasses import dataclass, field

from geometry_msgs.msg import Point
from nav_utils.geometry import distance
from nav_utils.world_occupancy_grid import WorldOccupancyGrid


@dataclass(order=True)
class KeyAndCost:
    """Store the coordinates to a node and the cost of that node."""

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


def find_closest_drivable_point(
    grid: WorldOccupancyGrid, robot_position: Point, max_search_radius: float
) -> Point | None:
    """BFS from the robot position to find the nearest drivable cell.

    The robot is in unknown space in the occupancy grid, meaning we don't know if its current location and the location
    around it is drivable. To account for this, we search for the closest drivable point, starting from the robot.

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
    search_container: deque[Point] = deque([robot_position])

    while len(search_container) > 0:
        current = search_container.popleft()

        for neighbor in grid.neighbors_forward(current):
            neighbor_key = grid.hash_key(neighbor)
            if neighbor_key in visited:
                continue
            visited.add(neighbor_key)

            if distance(neighbor, robot_position) > max_search_radius:
                continue

            if grid.state(neighbor).isDrivable:
                return neighbor

            if grid.state(neighbor).isUnknown:
                search_container.append(neighbor)

    return None


def generate_path(grid: WorldOccupancyGrid, start: Point, goal: Point) -> list[Point] | None:
    """Generate a good path for the robot to follow towards the goal using the A* search algorithm.
    https://en.wikipedia.org/wiki/A*_search_algorithm.

    Both start and goal should be drivable cells within the grid. If the goal is unreachable (e.g. blocked by
    obstacles), the path leads to the closest reachable cell instead.

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

    point_of: dict[int, Point] = {start_key: start}
    came_from: dict[int, int | None] = {start_key: None}
    cost_so_far: dict[int, float] = {start_key: 0.0}
    priority_queue: list[KeyAndCost] = [KeyAndCost(distance(start, goal), start_key)]

    best_goal_key = start_key
    best_goal_distance = distance(start, goal)

    while len(priority_queue) > 0:
        current_key = heapq.heappop(priority_queue).key
        current_point = point_of[current_key]

        if distance(current_point, goal) < best_goal_distance:
            best_goal_key = current_key
            best_goal_distance = distance(current_point, goal)

        if current_key == goal_key:
            best_goal_key = current_key
            break

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
                heapq.heappush(priority_queue, KeyAndCost(priority, neighbor_key))

    if best_goal_key == start_key:
        return None

    backtrace: list[Point] = []
    backtrace_key: int | None = best_goal_key
    while backtrace_key is not None:
        backtrace.append(point_of[backtrace_key])
        backtrace_key = came_from[backtrace_key]

    return list(reversed(backtrace))
