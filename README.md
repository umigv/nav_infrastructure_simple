# UMARV Navigation Stack 2025-2026

## Documentation

## Warning: Intellisense
When building, make sure you include `--symlink-install`. Otherwise, when control clicking a dependency in VSCode, it 
will take you to a copy of the dependency, instead of the actual source file. For example:

```bash
colcon build --symlink-install
```

### Dependencies
You can install all dependencies of nav by running
```bash
./scripts/setup.sh
```

### Simulation
You can run simulation by:
1. Publishing occupancy grid
2. Running this stack with simulation enabled
3. Publishing initial gps coordinates

The following describes how to run the point simulator (basic non-physics based simulator with an empty occupancy grid). 
Run each of these commands in separate terminals.

1. Publish an empty occupancy grid repeatedly:
    ```bash
    ros2 topic pub -r 1 /occ_grid nav_msgs/msg/OccupancyGrid "header:
      frame_id: 'base_link'
    info:
      resolution: 0.05
      width: 100
      height: 100
      origin:
        position: {x: 0.6, y: -2.5, z: 0.0}
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    data: [$(python3 -c 'print(", ".join(["0"]*100*100))')]"
    ```
2. Run navigation stack with simulation enabled:
    ```bash
    ros2 launch nav_brinngup core.launch.py
    ros2 launch nav_bringup navigation.launch.py simulation:=true
    ```
3. Publish initial gps coords
    ```bash
    ros2 topic pub /gps_coords sensor_msgs/msg/NavSatFix "{header: {frame_id: 'gps'}, status: {status: 0, service: 1}, latitude: 42.294621, longitude: -83.708112, altitude: 10.0, position_covariance: [0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0], position_covariance_type: 0}" --once
    ```

Feel free to modify the initial gps coords. 

Keep in mind that full occupancy grid simulation is likely infeasible, since the occupancy grid needs to "turn" with the robot.
