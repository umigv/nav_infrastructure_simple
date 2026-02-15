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
    # (y=0, x=0) is the robot position (i.e. robot is origin)

@dataclass(order=True)
class IndexAndCost:
    """Store the coordinates to a node and the cost of that node."""
    cost: float
    index: OccupancyGridIndex = field(compare=False)

# The robot's position within the occupancy grid is constant, since the camera
# is fixed to the robot.
ROBOT_FORWARDS_BACKWARDS_POSITION_RELATIVE_TO_BOTTOM_OF_CAMERA_VIEW = -0.60
# What value on the occupancy grid represents drivable area
DRIVABLE_CELL_VALUE = 0

def index_occupancy_grid(occupancy_grid: OccupancyGrid, index: OccupancyGridIndex):
    """Index occupancy grid 1D data array using 2D coordinates."""
    x_component = index.x + int(ROBOT_FORWARDS_BACKWARDS_POSITION_RELATIVE_TO_BOTTOM_OF_CAMERA_VIEW / occupancy_grid.info.resolution)
    y_component = index.y + occupancy_grid.info.height//2
    return occupancy_grid.data[y_component * occupancy_grid.info.width + x_component]

def is_index_out_of_bounds(occupancy_grid: OccupancyGrid, index: OccupancyGridIndex) -> bool:
    """Calculate whether or not index is out of bounds"""
    # Convert to bottom of left of occupancy origin
    adjusted_y = index.y + occupancy_grid.info.height//2
    adjusted_x = index.x + int(ROBOT_FORWARDS_BACKWARDS_POSITION_RELATIVE_TO_BOTTOM_OF_CAMERA_VIEW / occupancy_grid.info.resolution)

    return adjusted_y < 0 \
            or adjusted_y >= occupancy_grid.info.height \
            or adjusted_x < 0 \
            or adjusted_x >= occupancy_grid.info.width

def get_yaw_radians_from_quaternion(q: Quaternion):
    """Extract radians of yaw rotation from Quaternion https://en.wikipedia.org/wiki/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def convert_occupancy_grid_index_to_meters(
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

    current_position = OccupancyGridIndex(y=0, x=-int(ROBOT_FORWARDS_BACKWARDS_POSITION_RELATIVE_TO_BOTTOM_OF_CAMERA_VIEW / occupancy_grid.info.resolution))
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

            if is_index_out_of_bounds(occupancy_grid, potential_position):
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
    waypoint_meters: Point
) -> float:
    """
    Calculate cost of a node on the occupancy grid as its Euclidean distance from
    the goal node.
    """
    index_meters = convert_occupancy_grid_index_to_meters(
        occupancy_grid_resolution,
        index,
        robot_pose
    )

    return math.sqrt(
        (index_meters.x - waypoint_meters.x) ** 2
        + (index_meters.y - waypoint_meters.y) ** 2
    )

def generate_path_occupancy_grid_indices(
    goal_selection_node: Node,
    occupancy_grid: OccupancyGrid,
    start_index: OccupancyGridIndex,
    robot_pose: Pose,
    waypoint_meters: Point,
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
        waypoint_meters=waypoint_meters
    )
    heapq.heappush(priority_queue, IndexAndCost(cost=start_heuristic, index=start_index))

    # Used as an placeholder for algorithms that use occupancy grid indices since it 
    # is not a possible value to reach.

    null_occ_grid_index = OccupancyGridIndex(y=0, x=int(ROBOT_FORWARDS_BACKWARDS_POSITION_RELATIVE_TO_BOTTOM_OF_CAMERA_VIEW / occupancy_grid.info.resolution) - 1)

    came_from[start_index] = null_occ_grid_index

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
            waypoint_meters=waypoint_meters
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

            if is_index_out_of_bounds(occupancy_grid, neighbor):
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

def generate_path(
    goal_selection_node: Node,
    occupancy_grid: OccupancyGrid,
    robot_pose: Pose,
    waypoint_meters: Point,
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
                    position=convert_occupancy_grid_index_to_meters(
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
                waypoint_meters=waypoint_meters
            )
        ]
    )

if __name__ == "__main__":
    @dataclass
    class FakeInfo:
        resolution = 0.05
        height = 70
        width = 30

    @dataclass
    class FakeGrid:
        info: FakeInfo
        data: list[int]

    fakeGrid = FakeGrid(info=FakeInfo(), data=[])

    if not is_index_out_of_bounds(fakeGrid, OccupancyGridIndex(0, 0)):
        index_occupancy_grid(fakeGrid, OccupancyGridIndex(0, 0))