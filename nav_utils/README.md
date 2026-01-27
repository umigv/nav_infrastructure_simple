## nav_utils
Small utilities for ROS 2 Python nodes:

### nav_utils.config
Adds a simple config-loading utility for ROS 2.

Example:
```py
from dataclasses import dataclass, field
from rclpy.node import Node
import nav_utils.config

@dataclass
class Weights:
    heading: float = 1.0
    clearance: float # Required

@dataclass
class PlannerConfig:
    max_iters: int = 10_000
    timeout_s: float = 0.2
    weights: Weights

class Planner(Node):
    __init__(self):
        self.config = nav_utils.config.load(self, PlannerConfig)
```

You can then load this using a ROS2 config yaml file like this:
```yaml
planner:
  ros__parameters:
    max_iters: 10000
    timeout_s: 0.2
    weights:
      heading: 1.0
      clearance: 0.75
```

### nav_utils.geometry
2D geometry helpers for ROS2 message types

### nav_utils.world_occupancy_grid
World-facing wrapper around a robot-centric `nav_msgs/msg/OccupancyGrid`.

This class provides a world-coordinate view of a discrete, robot-centric occupancy grid. 

It allows planners to operate entirely on world `Point`s—querying occupancy, expanding neighbors, and hashing locations—without directly interacting with grid indices. Conceptually, the occupancy grid is treated as an infinite world representation:
world points are projected into grid cells on demand, and any point outside the underlying grid bounds is treated as `UNKNOWN`.

#### Conventions / Transformations
This assumes that the occupancy grid is centered at the robot and begins `robot_forward_offset_m` meters in front of the robot. The input `OccupancyGrid.data` is interpreted as column-major, with index 0 corresponding to the top-left cell.

Internally, the occupancy grid is transformed into a robot-aligned coordinate system with the following conventions (matching the odometry frame):
- +x points forward from the robot
- +y points to the left of the robot
- the grid origin is the bottom-left corner of the grid

#### State
State of the occupancy grid at some point can be queried using `state(point)`. The state is treated as `CellState.UNKNOWN` if `(grid_x, grid_y)` lies outside `[0..width) × [0..height)`.

#### Discrete “search” in continuous space via neighbors
Although planner code operates on continuous world `Point`s, discrete graph search can still be performed using `neighbors4(point)`, `neighbors8(point)`, or `neighbors_forward(point)`.

Each neighbor expansion:
1. Projects the input world point into a grid cell
2. Expands neighboring cells in grid index space
3. Converts those neighboring cells back into world points by returning the center of each cell

As a result, the search is discrete in the occupancy grid, while planner logic remains entirely in world coordinates.

Example pattern:
```py
for candidate in grid.neighbors8(current):
    if not grid.state(candidate).drivable:
        continue
```
#### Hashing
To support discrete search bookkeeping (e.g. visited sets), `WorldOccupancyGrid` provides `hash_key(point)`, which returns a stable integer identifier corresponding to the grid cell that the point belongs to.

The hash is derived from the projected grid indices, ensuring that all world points falling within the same grid cell map to the same key. This allows planners to treat grid cells as discrete states without storing raw grid indices or floating-point coordinates.
