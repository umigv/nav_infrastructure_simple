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
    ros2 topic pub -r 1 /occ_grid nav_msgs/msg/OccupancyGrid "header:
      frame_id: 'odom'
    info:
      resolution: 0.05
      width: 100
      height: 100
      origin:
        position: {x: 0.0, y: 0.0, z: 0.0}
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    data: [$(python3 -c 'print(", ".join(["0"]*100*100))')]"
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
    ros2 topic pub /gps_coords sensor_msgs/msg/NavSatFix "{header: {frame_id: 'gps'}, status: {status: 0, service: 1}, latitude: 42.294377, longitude: -83.708555, altitude: 10.0, position_covariance: [0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0], position_covariance_type: 0}" --once
    ```

Feel free to modify the current gps coords. 

Keep in mind that full occupancy grid simulation is likely infeasible, since the occupancy grid needs to "turn" with the robot.


### Ramp Mode Planning/ideas
- Will depend on CV Output to trigger mode
- Should reduce speed over ramp?
- Max ramp grade is 15 degrees
- In general, I think a good strategy is to have the robot drive square to the ramp so it doesn't tip too much or drive off.
    - Increase the lookahead distance on Pure Pursuit? (selecting a further lookahead point will return a straighter path)
    - Create a state machine that detects when the robot's base is at a >12.5 degree angle with the ground, then orient the robot to align with that angle. Could probably do some math with pitch and roll to calculate this
    - Turn off path planning and pure pursuit once we get a signal from CV until the robot's pitch is within a certain threshhold of 0 for more than the time it is at the top. During that time, use some kind of motion controller to minimize the robot's roll by adjusting the yaw of its path, because if the robot is driving straight onto the ramp, only its pitch should change. May be more an embedded-oriented idea, idk.

    
