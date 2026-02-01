## occupancy_grid_transform
This package consumes the occupancy grid provided by CV, converts it into ROS row-major convention, applies obstacle 
inflation, and republishes a world-aligned occupancy grid suitable for planning.

### Grid conventions
The occupancy grid from CV is expected to have the following conventions
- Column major
- Height = number of cells in +x direction, width = number of cells in +y direction
- Top left is origin
- Is centered around the robot in the y axis, and starts `robot_forward_offset_m` meters in front of the robot

The occupancy grid transformed by the node has the standard ROS2 convention, where
- Row major
- Height = number of cells in +y direction, width = number of cells in +x direction
- Bottom left is origin
- +x is forward, +y is left of the robot
- info.origin is the bottom left corner of the occupancy grid

### Subscribed Topics
- occupancy_grid (`nav_msgs/msg/OccupancyGrid`) - Input occupancy grid
- odom (`nav_msgs/msg/Odometry`) - Robot odometry, used to orient and place the grid in the world frame

### Published Topics
- occupancy_grid_transform (`nav_msgs/msg/OccupancyGrid`) - Transformed and inflated occupancy grid
