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
    y: int # +y = left
    x: int # +x = forward

@dataclass(order=True)
class IndexAndCost:
    cost: float
    index: OccupancyGridIndex = field(compare=False)


def generate_path_occupancy_grid_indices(
    goal_selection_node: Node,
    occupancy_grid: OccupancyGrid,
    start_index: OccupancyGridIndex,
    robot_pose: Pose,
    waypoint_robot_relative: Point,
):
    came_from: dict[OccupancyGridIndex, OccupancyGridIndex] = {}
    cost_so_far: dict[OccupancyGridIndex, float] = {}
    priority_queue: list[IndexAndCost] = []

    #add this wherever appropriate.
    zone_weight_grid = generate_zone_weighting(occupancy_grid)

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

            if neighbor.y < 0 or neighbor.y >= occupancy_grid.info.width\
                or neighbor.x < 0 or neighbor.x >= occupancy_grid.info.height:
                continue

            if index_occupancy_grid(occupancy_grid, neighbor) != DRIVABLE_CELL_VALUE:
                continue

            # Edge cost distance between indices (1 for straight, sqrt(2) for diagonal)
            edge_cost = math.sqrt(dy * dy + dx * dx) * occupancy_grid.info.resolution
            
            new_cost = current_cost + edge_cost

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

    backtrace: list[OccupancyGridIndex] = []
    current_backtrace_index = dataclasses.replace(best_goal_index)
    while came_from[current_backtrace_index] != NULL_OCC_GRID_INDEX:
        backtrace.append(current_backtrace_index)
        current_backtrace_index = came_from[current_backtrace_index]

    return reversed(backtrace)



NULL_OCC_GRID_INDEX = OccupancyGridIndex(y=-1, x=-1)
ROBOT_POSITION_IN_OCC_GRID = OccupancyGridIndex(y=77, x=12)
DRIVABLE_CELL_VALUE = 0




def index_occupancy_grid(occupancy_grid: OccupancyGrid, index: OccupancyGridIndex):
    return occupancy_grid.data[index.x * occupancy_grid.info.width + index.y]


def get_yaw_radians_from_quaternion(q: Quaternion):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def convert_occupancy_grid_index_to_robot_relative_position(
    occupancy_grid_resolution: float,
    occupancy_grid_coordinates: OccupancyGridIndex,
    robot_pose: Pose
) -> Point:
    yaw_radians = get_yaw_radians_from_quaternion(robot_pose.orientation)

    occ_point_with_robot_occ_origin = OccupancyGridIndex(
        y=occupancy_grid_coordinates.y - ROBOT_POSITION_IN_OCC_GRID.y,
        x=occupancy_grid_coordinates.x - ROBOT_POSITION_IN_OCC_GRID.x
    )

    rotated_occ_point_with_robot_occ_origin = Point(
        x=occ_point_with_robot_occ_origin.x * math.cos(yaw_radians) - occ_point_with_robot_occ_origin.y * math.sin(yaw_radians),
        y=occ_point_with_robot_occ_origin.x * math.sin(yaw_radians) + occ_point_with_robot_occ_origin.y * math.cos(yaw_radians)
    )

    return Point(
        x=rotated_occ_point_with_robot_occ_origin.x * occupancy_grid_resolution + robot_pose.position.x,
        y=rotated_occ_point_with_robot_occ_origin.y * occupancy_grid_resolution + robot_pose.position.y
    )

# The robot is in unknown space in the occupancy grid, 
# so we traverse forwards until we find the first drivable index.
#
# If going forward is not enough, we then traverse left and right as well
def find_closest_drivable_point(occupancy_grid: OccupancyGrid) -> OccupancyGridIndex | None:
    visited: set[OccupancyGridIndex] = set()
    search_container: deque[OccupancyGridIndex] = deque()

    current_position = dataclasses.replace(ROBOT_POSITION_IN_OCC_GRID)
    visited.add(current_position)
    search_container.append(current_position)
    
    while len(search_container) > 0:
        index = search_container.popleft()

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

            if potential_position.y < 0 or potential_position.y >= occupancy_grid.info.width\
                or potential_position.x < 0 or potential_position.x >= occupancy_grid.info.height:
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
    index_robot_relative_coords = convert_occupancy_grid_index_to_robot_relative_position(
        occupancy_grid_resolution,
        index,
        robot_pose
    )
    #this is where we add the zone_weighting. running into an issue that it doesn't take in an occupancy grid object.
    #ask if it is ok to pass the grid in or if we need to work backwards.
    #index_zone_weighting()


    return math.sqrt(
        (index_robot_relative_coords.x - waypoint_robot_relative.x) ** 2
        + (index_robot_relative_coords.y - waypoint_robot_relative.y) ** 2
    )
#once we get to testing: because we're adding, may need to make sure the values don't exceed 100.
def generate_zone_weighting(
        grid: OccupancyGrid,
        quadratic_factor: float = 2,
        linear_factor: float = 1,
        linear_ratio:  float = 1.75,
        top_bar_size: int = 5,
        top_bar_weight: int = 5
):

    zone_weight_grid = grid
    zone_weight_grid.data = []
    width = grid.info.width
    height = grid.info.height

    x=0
    while (x < height): 
        y=0
        while (y < width): 
                #weight the bottom in a linear gradient
                
                if (x <= (height * linear_ratio)):
                    zone_weight_grid.data[x * width + y] += ((height * linear_ratio) - x) *  linear_factor
                #weight the top bar a little
                if (x > (height - top_bar_size)):
                    zone_weight_grid.data[x * width + y] += top_bar_weight
                #quadratic rating on the center
                #change the weighting as needed
                zone_weight_grid.data[x * width + y] += quadratic_factor * pow((width/2 - y), 2)
                y+=1
        x+=1

        

    return zone_weight_grid


def generate_path(
    goal_selection_node: Node,
    occupancy_grid: OccupancyGrid,
    robot_pose: Pose,
    waypoint_robot_relative: Point,
) -> Path:
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