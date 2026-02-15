## occupancy_grid_transform
This package consumes the occupancy grid provided by CV, converts it into ROS row-major convention, adds a border on
the far edges, applies obstacle inflation, and republishes the occupancy grid suitable for planning. The
grid origin is transformed from the incoming frame to the configured output frame using TF2.

### Grid conventions
The occupancy grid from CV is expected to have the following conventions
- Column major
- Height = number of cells in +x direction, width = number of cells in +y direction
- Top left is origin
- Origin is set by the publisher (e.g. CV pipeline) in its frame

The occupancy grid transformed by the node has the standard ROS2 convention, where
- Row major
- Height = number of cells in +y direction, width = number of cells in +x direction
- Bottom left is origin
- +x is forward, +y is left of the robot
- info.origin is the bottom left corner of the occupancy grid

For example, after transform, with a 3 wide 2 tall grid:

```
        1d indexing by (y * width + x):

               4 5 
        R -->  2 3
               0 1


        2d indexing by (y, x):

              (2, 0) (2, 1)
        R --> (1, 0) (1, 1)
              (0, 0) (0, 1)

^
|
y 
  x-->
```

R is the robot position, and the numbers represent the index of each cell in the incoming grid data array.
The grid is indexed with (y, x) in 2d, and (y * width + x) in 1d.

### Subscribed Topics
- occupancy_grid (`nav_msgs/msg/OccupancyGrid`) - Input occupancy grid

### Published Topics
- transformed_occupancy_grid (`nav_msgs/msg/OccupancyGrid`) - Bordered and inflated occupancy grid
