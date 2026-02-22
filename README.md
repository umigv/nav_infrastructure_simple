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
2. Run core:
    ```bash
    ros2 launch nav_bringup core.launch.py
    ```
3. Run simulated localization:
    ```bash
    ros2 launch nav_bringup localization.launch.py simulation:=true
    ```
4. Run nav stack:
    ```bash
    ros2 launch nav_bringup navigation.launch.py
    ```

Keep in mind that full occupancy grid simulation is likely infeasible, since the occupancy grid needs to "turn" with the robot.
