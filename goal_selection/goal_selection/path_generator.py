from dataclasses import dataclass, field
import dataclasses
import itertools
import json
import math
import sys
from numpy import ndarray, zeros
from collections import deque
import heapq
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.node import Node
from std_msgs.msg import Header
from nav_utils.world_occupancy_grid import WorldOccupancyGrid, CellState



# @dataclass(unsafe_hash=True)
# class WorldOccupancyGridIndex:
#     """Store the coordinates to a spot in the occupancy grid."""
#     y: int # +y = left
#     x: int # +x = forward
#     # (y=0, x=0) is the robot position (i.e. robot is origin)

@dataclass(order=True)
class IndexAndCost:
    """Store the coordinates to a node and the cost of that node."""
    cost: float
    index: int = field(compare=False)

# The robot's position within the occupancy grid is constant, since the camera
# is fixed to the robot.
ROBOT_FORWARDS_BACKWARDS_POSITION_RELATIVE_TO_BOTTOM_OF_CAMERA_VIEW = -0.60
# What value on the occupancy grid represents drivable area
DRIVABLE_CELL_VALUE = 0



# def index_occupancy_grid(occupancy_grid: WorldOccupancyGrid, index: int):
#     """Index occupancy grid 1D data array using 2D coordinates."""
#     return occupancy_grid.state(index)

# def is_index_out_of_bounds(occupancy_grid: WorldOccupancyGrid, index: int) -> bool:
#     """Calculate whether or not index is out of bounds"""
#     return occupancy_grid.state(index) == CellState.UNKNOWN

# def get_yaw_radians_from_quaternion(q: Quaternion):
#     """Extract radians of yaw rotation from Quaternion https://en.wikipedia.org/wiki/Quaternion."""
#     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#     return math.atan2(siny_cosp, cosy_cosp)

# def convert_occupancy_grid_index_to_meters(
#     occupancy_grid: WorldOccupancyGrid,
#     occupancy_grid_resolution: float,
#     occupancy_grid_coordinates: WorldOccupancyGridIndex
#     #robot_pose: Pose
# ) -> Point:
#     """Convert from occupancy grid coordinates to 'meters from the robot'."""
#     #yaw_radians = get_yaw_radians_from_quaternion(robot_pose.orientation)

#     return (occupancy_grid._world_to_grid_index(occupancy_grid_coordinates.x, occupancy_grid_coordinates.y) * occupancy_grid_resolution)

    # rotated_occ_point_with_robot_occ_origin = Point(
    #     x=occupancy_grid_coordinates.x * math.cos(yaw_radians) - occupancy_grid_coordinates.y * math.sin(yaw_radians),
    #     y=occupancy_grid_coordinates.x * math.sin(yaw_radians) + occupancy_grid_coordinates.y * math.cos(yaw_radians)
    # )

    # return Point(
    #     x=rotated_occ_point_with_robot_occ_origin.x * occupancy_grid_resolution + robot_pose.position.x,
    #     y=rotated_occ_point_with_robot_occ_origin.y * occupancy_grid_resolution + robot_pose.position.y
    # )

def find_closest_drivable_point(occupancy_grid: WorldOccupancyGrid, robot_position: Point) -> Point | None:
    """
    The robot is in unknown space in the occupancy grid, meaning we don't know if its
    current location and the location around it is drivable. 
    
    To account for this, we search for the closest drivable node, starting from the node
    containing the robot.
    """
    visited: set[int] = set()
    search_container: deque[Point] = deque()

    visited.add(occupancy_grid.hash_key(robot_position))
    search_container.append(robot_position)
    
    while len(search_container) > 0:
        current_position = search_container.popleft()

        # Start by searching forwards, then forwards-left, forwards-right, and finally
        # just left and right. Searching backwards is not necessary because all nodes
        # behind the robot are of unknown value.
        for neighbor in occupancy_grid.neighbors_forward(current_position):   
            if occupancy_grid.state(neighbor) == CellState.OCCUPIED:
                continue

            if occupancy_grid.hash_key(neighbor) in visited:
                continue

            if occupancy_grid.state(neighbor) == CellState.FREE:
                return neighbor
            
            search_container.append(neighbor)
            visited.add(occupancy_grid.hash_key(neighbor))    

    return None

def index_cost(
    occupancy_grid: WorldOccupancyGrid,
    occupancy_grid_resolution: float,
    #index: WorldOccupancyGridIndex,
    robot_pose: Pose,
    waypoint_meters: Point
) -> float:
    """
    Calculate cost of a node on the occupancy grid as its Euclidean distance from
    the goal node.
    """
    index_meters = convert_occupancy_grid_index_to_meters(
        occupancy_grid_resolution,
        occupancy_grid.hash_key(robot_pose.point),
        robot_pose
    )
    return math.sqrt(
        (index_meters.x - waypoint_meters.x) ** 2
        + (index_meters.y - waypoint_meters.y) ** 2
    )

#once we get to testing: because we're adding, may need to make sure the values don't exceed 100. Will probably need to be toned down a lot to align with the rest of priority.
def generate_zone_weighting(
        grid: ndarray,
        #keep all of the weighting values integers--if need to adjust for granularity, round up/down
        quadratic_factor: float = .25,
        linear_factor: float = 1,
        linear_ratio:  float = .75,
        top_bar_size: int = 30,
        top_bar_weight: int = 15
    ) -> ndarray:
    """Generates the weighting grid of an occupancy grid of a given size as a 2D Numpy Array. Will need to play with default weightings"""


    #hopefully this doesn't cause pointer weirdness


    zone_weight_grid = zeros((grid.info.height, grid.info.width))
    #see if these need to be changed
    width, height = grid.shape 

    #see if these need to be changed
    x = 0
    while (x <= width/2): 
        y = 0
        while (y < height): 
               #weight the bottom. this is weighted assuming the top is 0.
                if (y < (height - (height* linear_ratio))):
                    grid[x,y] += (y+1) *  linear_factor
                #weight the top bar a little. this is weighted assuming the top is 0.
                if (y >= height - top_bar_size):
                    grid[x,y] += top_bar_weight
                #quadratic rating on the center
                #change the weighting as needed
                grid[x,y] += quadratic_factor * pow(float(width/2) - float(x), 2)

                #set this to max if it's greater
                grid[x,y] = min(grid[x,y], 100)
                grid[width-1-x,y] = grid[x,y]

                #I've just thought of something. Last year we used a matrix to store the costs. This year we're just using inflation grids.
                y+=1
        x+=1

    return zone_weight_grid


def generate_path_occupancy_grid_indices(
    goal_selection_node: Node,
    #this is the part to convert to world
    occupancy_grid: WorldOccupancyGrid,
    start_index: int,
    robot_pose: Pose,
    waypoint_meters: Point,
    zone_weighting: ndarray
):
    """
    Generate a good path for the robot to follow towards the goal using the A* search
    algorithm https://en.wikipedia.org/wiki/A*_search_algorithm.
    
    Uses occupancy grid indices as the coordinate system.
    """
    came_from: dict[int, int] = {}
    cost_so_far: dict[int, float] = {}
    key_to_point: dict[int, Point]
    priority_queue: list[int] = []

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

    null_occ_grid_index = WorldOccupancyGridIndex(y=0, x=int(ROBOT_FORWARDS_BACKWARDS_POSITION_RELATIVE_TO_BOTTOM_OF_CAMERA_VIEW / occupancy_grid.info.resolution) - 1)

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

        neighbors = occupancy_grid.neighbors_forward(current_item.index)

        for neighbor in neighbors:
            if occupancy_grid.state(neighbor) != CellState.Free:
                continue

            new_cost = current_cost + math.hypot(neighbor.x - current_item.x, neighbor.y - current_item.y)

            neighbor_key = occupancy_grid.hash_key(neighbor)
            if neighbor_key not in cost_so_far or new_cost < cost_so_far[neighbor_key]:
                cost_so_far[neighbor_key] = new_cost
                came_from[neighbor_key] = current_index
                
                # A* priority: cost_so_far + heuristic (distance to waypoint)
                heuristic = index_cost(
                    occupancy_grid_resolution=occupancy_grid.info.resolution,
                    index=neighbor,
                    robot_pose=robot_pose,
                    waypoint_meters=waypoint_meters
                )
                #may need to change this priority.
                priority = new_cost + heuristic+ zone_weighting[neighbor.x + int(ROBOT_FORWARDS_BACKWARDS_POSITION_RELATIVE_TO_BOTTOM_OF_CAMERA_VIEW / occupancy_grid.info.resolution), - neighbor.y + occupancy_grid.info.width//2]
                heapq.heappush(priority_queue, IndexAndCost(cost=priority, index=neighbor_key))

    goal_selection_node.get_logger().info(f"Generated path from {start_index} to {best_goal_index}!")

    backtrace: list[WorldOccupancyGridIndex] = []
    current_backtrace_index = dataclasses.replace(best_goal_index)
    while came_from[current_backtrace_index] != null_occ_grid_index:
        backtrace.append(current_backtrace_index)
        current_backtrace_index = came_from[current_backtrace_index]

    return reversed(backtrace)


def generate_path(
    goal_selection_node: Node,
    occupancy_grid: WorldOccupancyGrid,
    robot_pose: Pose,
    waypoint_meters: Point,
) -> Path:
    """
    Generate path in occupancy grid indices coordinate system, convert it to robot
    relative coodinates, and package it in a nav_msgs.msg.Path message.
    """
    start_point = find_closest_drivable_point(occupancy_grid)
    assert start_point is not None, "Could not find drivable area in front of robot!"



    zone_weights = generate_zone_weighting(occupancy_grid)

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
                robot_pose=occupancy_grid.pose,
                waypoint_meters=waypoint_meters,
                zone_weighting=zone_weights
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

    if not is_index_out_of_bounds(fakeGrid, WorldOccupancyGridIndex(0, 0)):
        index_occupancy_grid(fakeGrid, WorldOccupancyGridIndex(0, 0))