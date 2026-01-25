# State Machine

This package provides a State Machine that tracks and publishes the robot’s high-level operating state. The state machine exposes services for state transitions and publishes the robot state to a topic for other nodes to subscribe to and react accordingly.

## States

There are three mutually exclusive states:
- **`normal`** – default operating mode  
- **`ramp`** – ramp-specific behavior enabled  
- **`recovery`** – recovery / fault handling mode  

1. State priority is fixed and deterministic: recovery > ramp > normal
2. States are published only on state changes
3. On startup, the node immediately publishes the initial state (`normal`)
4. Late-joining subscribers immediately receive the most recent state

## Published Topics
`/state` (`std_msgs/String`) - The current robot state as one of `"normal"`,  `"ramp"`, or `"recovery"`

## Services
`/state/set_ramp` (`std_srvs/SetBool`) - Enable or disable ramp mode.
`/state/set_recovery` (`std_srvs/SetBool`) - Enable or disable recovery mode.

## Example Usage

```bash
ros2 topic echo /state # Observe current state
ros2 service call /state/set_ramp std_srvs/srv/SetBool "{data: true}" # Enable ramp mode
ros2 service call /state/set_recovery std_srvs/srv/SetBool "{data: true}" # Enter recovery mode (overrides ramp)
ros2 service call /state/set_recovery std_srvs/srv/SetBool "{data: false}" # Exit recovery mode
ros2 service call /state/set_ramp std_srvs/srv/SetBool "{data: false}" # Disable ramp mode
```
