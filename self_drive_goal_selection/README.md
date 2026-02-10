## Self Drive Goal Selection
This node implements goal selection logic for the self drive contest. A special cell value that represents a 
"self drive goal" is produced by CV. This node scans the occupancy grid for the special value, converts the grid into
a world coordinate point, then publishes it as the selected goal

### Conventions
- To avoid spamming the topic, a goal is only published when it changes from the last published goal
- The special value as agreed with CV is 127

### Subscribed Topics
- occupancy_grid (`nav_msgs/msg/OccupancyGrid`) - Input occupancy grid

### Published Topics
- goal (`geometry_msgs/msg/Point`) - The selected goal
