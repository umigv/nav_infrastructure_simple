from dataclasses import dataclass, field
import dataclasses
import itertools
import json
import math
import sys
from collections import deque
import heapq
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.node import Node
from std_msgs.msg import Header
import nav_utils.geometry import distance


@dataclass(order=True)
class KeyAndCost:
    """Store the coordinates to a node and the cost of that node."""
    cost: float
    key: int = field(compare=False)


def find_closest_drivable_point(grid: WorldOccupancyGrid, robot_position: Point) -> Point | None:
    """
    The robot is in unknown space in the occupancy grid, meaning we don't know if its
    current location and the location around it is drivable. 
    
    To account for this, we search for the closest drivable node, starting from the node
    containing the robot.

    Used to traverse the area between the robot's real position and where its FoV begins.
    """
    visited: set[int] = {grid.hash_key(robot_position)}
    search_container: deque[Point] = deque([robot_position])
    
    while len(search_container) > 0:
        current = search_container.popleft()

        # Start by searching forwards, then forwards-left, forwards-right, and finally
        # just left and right. Searching backwards is not necessary because all nodes
        # behind the robot are of unknown value.
        for neighbor in grid.neighbors_forward(current):
            neighor_key = grid.hash_key(neighbor)
            if neighor_key in visited:
                continue
            visited.add(neighor_key)

            if grid.state(neighbor).is_driveable:
                return neighbor
            
            if grid.state(neighbor).is_unknown:
                search_container.append(neighbor)
    
    return None



def generate_path(grid: WorldOccupancyGrid, start: Point, goal: Point):
    """
    Generate a good path for the robot to follow towards the goal using the A* search
    algorithm https://en.wikipedia.org/wiki/A*_search_algorithm.
    
    Uses occupancy grid indices as the coordinate system.
    """
    start_key = grid.hash_key(start)
    goal_key = grid.hash_key(goal)

    point_of: dict[int, Point] = {start_key: start}
    came_from: dict[int, int | None] = {start_key: None}
    cost_so_far: dict[int, float] = {start_key: 0.0}
    priority_queue: list[KeyAndCost] = [KeyAndCost(distance(start, goal), start_key)]

    best_goal_key = start_key
    best_goal_distance = distance(start, goal)

    while len(priority_queue) > 0:
        current_key = heapq.heappop(priority_queue).key
        current_point = point_of[current_key]
        current_cost = cost_so_far[current_key]

        if distance(current_point, goal) < best_goal_distance:
            best_goal_key = current_key
            best_goal_distance = distance(current_point, goal)

        for neighbor in grid.neighbors8(current_point):
            if grid.state(neighbor).is_drivable:
                continue

            # Edge cost distance between indices (1 for straight, sqrt(2) for diagonal)
            edge_cost = math.sqrt(dy * dy + dx * dx) * occupancy_grid.info.resolution
            
            new_cost = current_cost + edge_cost

            # Unexplored node or we found a better way to get to the node
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current_index
                
                # A* priority: cost_so_far + heuristic (distance to waypoint)
                heuristic = index_cost(
                    occupancy_grid_resolution=occupancy_grid.info.resolution,
                    index=neighbor,
                    robot_pose=robot_pose,
                    waypoint_meters=waypoint_meters
                )
                priority = new_cost + heuristic
                heapq.heappush(priority_queue, IndexAndCost(cost=priority, index=neighbor))

    goal_selection_node.get_logger().info(f"Generated path from {start_index} to {best_goal_index}!")

    # In order to construct a path from the search process, start from the best node
    # within the occupancy grid, and work backwards until you reach the start node. 
    backtrace: list[OccupancyGridIndex] = []
    current_backtrace_index = dataclasses.replace(best_goal_index)
    while came_from[current_backtrace_index] != null_occ_grid_index:
        backtrace.append(current_backtrace_index)
        current_backtrace_index = came_from[current_backtrace_index]

    return reversed(backtrace)



    