## Teleop
Joystick-based teleoperation for a ROS2 robot, with controller-specific mappings (Xbox / PS4) and a twist_mux layer 
to arbitrate between teleop and autonomy

### Controller Mappings
For both Xbox and PS4 controllers
- Left joystick controls linear motion 
- Right joystick controls turning. 
- Enable is mapped to the right shoulder button (RB / R1)
- Turbo is mapped to the left shoulder button (LB / L1)

The configs live in
- config/teleop_ps4.yaml
- config/teleop_xbox.yaml

### Command Velocity Multiplexing
the twist multiplexing layer prevents the teleop command velocity from fighting with the autonomy command velocity.

If both teleop and autonomy publish velocity commands:
- Teleop is prioritized
- If teleop stops publishing, control automatically falls back to autonomy after a short timeout

### Launch 
```
ros2 launch teleop teleop.launch.py controller:=<controller>
```

Parameters:
- controller: The controller config to use ('xbox' or 'ps4'), required.
- joystick_dev: The joystick device, defaulted to `/dev/input/js0`

### Subscribed Topics
`nav_cmd_vel` (`geometry_msgs/msg/Twist`) - The command velocity produced by nav autonomy

### Published Topics
`joy` (`sensor_msgs/msg/Joy`) - Joystick output   
`teleop_cmd_vel` (`geometry_msgs/msg/Twist`) - The command velocity produced by joystick   
`cmd_vel` (`geometry_msgs/msg/Twist`) - The combined velocity between teleop and autonomy
