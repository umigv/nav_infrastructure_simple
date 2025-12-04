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

@dataclass(unsafe_hash=True)
class OccupancyGridIndex:
    """Store the coordinates to a spot in the occupancy grid."""
    y: int # +y = left
    x: int # +x = forward

@dataclass(order=True)
class IndexAndCost:
    """Store the coordinates to a node and the cost of that node."""
    cost: float
    index: OccupancyGridIndex = field(compare=False)

# Used as an placeholder for algorithms that use occupancy grid indices, 
# since it is not a possible value to reach.
NULL_OCC_GRID_INDEX = OccupancyGridIndex(y=-1, x=-1)
# The robot's position within the occupancy grid is constant, since the camera
# is fixed to the robot.
ROBOT_POSITION_IN_OCC_GRID = OccupancyGridIndex(y=77, x=12)
# What value on the occupancy grid represents drivable area
DRIVABLE_CELL_VALUE = 0

def index_occupancy_grid(occupancy_grid: OccupancyGrid, index: OccupancyGridIndex):
    """Index occupancy grid 1D data array using 2D coordinates."""
    return occupancy_grid.data[(index.x + ROBOT_POSITION_IN_OCC_GRID.x) * occupancy_grid.info.width + -index.y + ROBOT_POSITION_IN_OCC_GRID.y]

def get_yaw_radians_from_quaternion(q: Quaternion):
    """Extract radians of yaw rotation from Quaternion https://en.wikipedia.org/wiki/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def convert_occupancy_grid_index_to_robot_relative_position(
    occupancy_grid_resolution: float,
    occupancy_grid_coordinates: OccupancyGridIndex,
    robot_pose: Pose
) -> Point:
    """Convert from occupancy grid coordinates to 'meters from the robot'."""
    yaw_radians = get_yaw_radians_from_quaternion(robot_pose.orientation)

    rotated_occ_point_with_robot_occ_origin = Point(
        x=occupancy_grid_coordinates.x * math.cos(yaw_radians) - occupancy_grid_coordinates.y * math.sin(yaw_radians),
        y=occupancy_grid_coordinates.x * math.sin(yaw_radians) + occupancy_grid_coordinates.y * math.cos(yaw_radians)
    )

    return Point(
        x=rotated_occ_point_with_robot_occ_origin.x * occupancy_grid_resolution + robot_pose.position.x,
        y=rotated_occ_point_with_robot_occ_origin.y * occupancy_grid_resolution + robot_pose.position.y
    )

def find_closest_drivable_point(occupancy_grid: OccupancyGrid) -> OccupancyGridIndex | None:
    """
    The robot is in unknown space in the occupancy grid, meaning we don't know if its
    current location and the location around it is drivable. 
    
    To account for this, we search for the closest drivable node, starting from the node
    containing the robot.
    """
    visited: set[OccupancyGridIndex] = set()
    search_container: deque[OccupancyGridIndex] = deque()

    current_position = OccupancyGridIndex(y=0, x=0)
    visited.add(current_position)
    search_container.append(current_position)
    
    while len(search_container) > 0:
        index = search_container.popleft()

        # Start by searching forwards, then forwards-left, forwards-right, and finally
        # just left and right. Searching backwards is not necessary because all nodes
        # behind the robot are of unknown value.
        for dy, dx in [
            (0, 1),
            (1, 1),
            (-1, 1),
            (1, 0),
            (-1, 0)
        ]:
            potential_position = OccupancyGridIndex(
                y=index.y + dy,
                x=index.x + dx
            )

            if -potential_position.y + ROBOT_POSITION_IN_OCC_GRID.y < 0 or -potential_position.y + ROBOT_POSITION_IN_OCC_GRID.y >= occupancy_grid.info.width\
                or potential_position.x + ROBOT_POSITION_IN_OCC_GRID.x < 0 or potential_position.x + ROBOT_POSITION_IN_OCC_GRID.x >= occupancy_grid.info.height:
                continue

            if potential_position in visited:
                continue

            if index_occupancy_grid(occupancy_grid, potential_position) == DRIVABLE_CELL_VALUE:
                return potential_position
            
            search_container.append(potential_position)
            visited.add(potential_position)
    
    return None

def index_cost(
    occupancy_grid_resolution: float,
    index: OccupancyGridIndex,
    robot_pose: Pose,
    waypoint_robot_relative: Point
) -> float:
    """
    Calculate cost of a node on the occupancy grid as its Euclidean distance from
    the goal node.
    """
    index_robot_relative_coords = convert_occupancy_grid_index_to_robot_relative_position(
        occupancy_grid_resolution,
        index,
        robot_pose
    )

    return math.sqrt(
        (index_robot_relative_coords.x - waypoint_robot_relative.x) ** 2
        + (index_robot_relative_coords.y - waypoint_robot_relative.y) ** 2
    )

def generate_path_occupancy_grid_indices(
    goal_selection_node: Node,
    occupancy_grid: OccupancyGrid,
    start_index: OccupancyGridIndex,
    robot_pose: Pose,
    waypoint_robot_relative: Point,
):
    """
    Generate a good path for the robot to follow towards the goal using the A* search
    algorithm https://en.wikipedia.org/wiki/A*_search_algorithm.
    
    Uses occupancy grid indices as the coordinate system.
    """
    came_from: dict[OccupancyGridIndex, OccupancyGridIndex] = {}
    cost_so_far: dict[OccupancyGridIndex, float] = {}
    priority_queue: list[IndexAndCost] = []

    cost_so_far[start_index] = 0.0
    start_heuristic = index_cost(
        occupancy_grid_resolution=occupancy_grid.info.resolution,
        index=start_index,
        robot_pose=robot_pose,
        waypoint_robot_relative=waypoint_robot_relative
    )
    heapq.heappush(priority_queue, IndexAndCost(cost=start_heuristic, index=start_index))
    came_from[start_index] = NULL_OCC_GRID_INDEX

    best_goal_index = start_index
    best_goal_distance = start_heuristic

    while len(priority_queue) > 0:
        current_item = heapq.heappop(priority_queue)
        current_index = current_item.index

        # Get the actual cost_so_far for this index
        current_cost = cost_so_far.get(current_index, float('inf'))

        # Check if this index is closer to waypoint than current best
        distance_to_waypoint = index_cost(
            occupancy_grid_resolution=occupancy_grid.info.resolution,
            index=current_index,
            robot_pose=robot_pose,
            waypoint_robot_relative=waypoint_robot_relative
        )

        if distance_to_waypoint < best_goal_distance:
            best_goal_index = current_index
            best_goal_distance = distance_to_waypoint

        for dy, dx in itertools.product([-1, 0, 1], repeat=2):
            if dy == 0 and dx == 0:
                continue

            neighbor = OccupancyGridIndex(
                y=current_index.y + dy,
                x=current_index.x + dx
            )

            # Invalid indices
            if -neighbor.y + ROBOT_POSITION_IN_OCC_GRID.y < 0 or -neighbor.y + ROBOT_POSITION_IN_OCC_GRID.y >= occupancy_grid.info.width\
                or neighbor.x + ROBOT_POSITION_IN_OCC_GRID.x < 0 or neighbor.x + ROBOT_POSITION_IN_OCC_GRID.x >= occupancy_grid.info.height:
                continue

            # Cell isn't drivable
            if index_occupancy_grid(occupancy_grid, neighbor) != DRIVABLE_CELL_VALUE:
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
                    waypoint_robot_relative=waypoint_robot_relative
                )
                priority = new_cost + heuristic
                heapq.heappush(priority_queue, IndexAndCost(cost=priority, index=neighbor))

    goal_selection_node.get_logger().info(f"Generated path from {start_index} to {best_goal_index}!")

    # In order to construct a path from the search process, start from the best node
    # within the occupancy grid, and work backwards until you reach the start node. 
    backtrace: list[OccupancyGridIndex] = []
    current_backtrace_index = dataclasses.replace(best_goal_index)
    while came_from[current_backtrace_index] != NULL_OCC_GRID_INDEX:
        backtrace.append(current_backtrace_index)
        current_backtrace_index = came_from[current_backtrace_index]

    return reversed(backtrace)

def generate_path(
    goal_selection_node: Node,
    occupancy_grid: OccupancyGrid,
    robot_pose: Pose,
    waypoint_robot_relative: Point,
) -> Path:
    """
    Generate path in occupancy grid indices coordinate system, convert it to robot
    relative coodinates, and package it in a nav_msgs.msg.Path message.
    """
    start_point = find_closest_drivable_point(occupancy_grid)
    assert start_point is not None, "Could not find drivable area in front of robot!"

    return Path(
        header=Header(
            frame_id="odom"
        ),
        poses=[
            PoseStamped(
                header=Header(
                    frame_id="odom"
                ),
                pose=Pose(
                    position=convert_occupancy_grid_index_to_robot_relative_position(
                        occupancy_grid.info.resolution,
                        index,
                        robot_pose
                    )
                )
            )
            for index in generate_path_occupancy_grid_indices(
                goal_selection_node=goal_selection_node,
                occupancy_grid=occupancy_grid,
                start_index=start_point,
                robot_pose=robot_pose,
                waypoint_robot_relative=waypoint_robot_relative
            )
        ]
    )