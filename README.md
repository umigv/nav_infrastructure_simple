# UMARV Navigation Stack 2025-2026

## Documentation

## Warning: Intellisense
When building, make sure you include `--symlink-install`. Otherwise, when control clicking a dependency in VSCode, it will take you to a copy of the dependency, instead of the actual source file. For example:

```bash
colcon build --symlink-install
```

### Simulation
You can run simulation by:
1. Publishing occupancy grid
2. Publishing odometry
3. Publishing current gps coordinates
4. Running this stack
5. Running your simulation node

The following describes how to run the point simulator (basic non-physics based simulator with an empty occupancy grid). Run each of these commands in separate terminals.

1. Publish an empty occupancy grid repeatedly:
    ```bash
    ros2 topic pub -r 1 /occupancy_grid nav_msgs/msg/OccupancyGrid "header:
      frame_id: 'odom'
    info:
      resolution: 0.05
      width: 155
      height: 76
      origin:
        position: {x: 0.0, y: 0.0, z: 0.0}
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    data: [$(python3 -c 'print(", ".join(["0"]*155*76))')]"
    ```
2. Run navigation stack
    ```bash
    ros2 launch nav_infrastructure_launch infra.launch.py
    ```
3. Run point simulator
    ```bash
    ros2 run point_simulator point_simulator
    ```
4. Publish current gps coords
    ```bash
    ros2 topic pub /gps_coords sensor_msgs/msg/NavSatFix "{header: {frame_id: 'gps'}, status: {status: 0, service: 1}, latitude: 42.667918, longitude: -83.218317, altitude: 10.0, position_covariance: [0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0], position_covariance_type: 0}" --once
    ```

Feel free to modify the current gps coords. 

Keep in mind that full occupancy grid simulation is likely infeasible, since the occupancy grid needs to "turn" with the robot.