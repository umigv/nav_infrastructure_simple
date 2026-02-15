from dataclasses import dataclass, field
from collections import deque
import heapq
from geometry_msgs.msg import Point
from nav_utils.nav_utils.geometry import distance
from nav_utils.nav_utils.world_occupancy_grid import WorldOccupancyGrid 

@dataclass(order=True)
class KeyAndCost:
    """Store the coordinates to a node and the cost of that node."""
    cost: float
    key: int = field(compare=False)


def find_closest_drivable_point(
    grid: WorldOccupancyGrid, 
    robot_position: Point, 
    max_search_radius: float
) -> Point | None:
    """
    The robot is in unknown space in the occupancy grid, meaning we don't know if its
    current location and the location around it is drivable. 
    
    To account for this, we search for the closest drivable node, starting from the node
    containing the robot.

    Used to traverse the area between the robot's real position and where its FoV begins.
    """
    if grid.state(robot_position).is_drivable:
        return robot_position

    visited: set[int] = {grid.hash_key(robot_position)}
    search_container: deque[Point] = deque([robot_position])
    
    while len(search_container) > 0:
        current = search_container.popleft()

        for neighbor in grid.neighbors_forward(current):
            neighor_key = grid.hash_key(neighbor)
            if neighor_key in visited:
                continue
            visited.add(neighor_key)

            if distance(neighbor, robot_position) > max_search_radius:
                continue

            if grid.state(neighbor).is_driveable:
                return neighbor
            
            if grid.state(neighbor).is_unknown:
                search_container.append(neighbor)
    
    return None

def generate_path(grid: WorldOccupancyGrid, start: Point, goal: Point) -> list[Point] | None:
    """
    Generate a good path for the robot to follow towards the goal using the A* search
    algorithm https://en.wikipedia.org/wiki/A*_search_algorithm.
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
            if not grid.state(neighbor).is_drivable:
                continue

            neighbor_key = grid.hash_key(neighbor)
            neighbor_cost = cost_so_far[current_key] + distance(current_point, neighbor)

            if neighbor not in cost_so_far or neighbor_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor_key] = neighbor_cost
                came_from[neighbor_key] = current_key
                point_of[neighbor_key] = neighbor
                priority = neighbor_cost + distance(neighbor, goal)
                heapq.heappush(priority_queue, KeyAndCost(priority, neighbor_key))

    if best_goal_key == start_key:
        return None

    backtrace: list[Point] = []
    current_key: int | None = best_goal_key
    while current_key != None:
        backtrace.append(point_of[current_key])
        current_key = came_from[current_key]

    return reversed(backtrace)
