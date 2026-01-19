# GPS Publiher 
This node runs the ublox ~~origin~~~ GPS and publishes the data every second

## Dependencies
### Packages
- ```pyubx2```
- ```python3-serial```

## Setup
1. Install the two python packages above (TODO: write script)

## Running the node
1. Find port
2. Start the GPS Transform node by calling ```ros2 launch gps_publisher gps.launch.py [args]``` (todo: write launch file)

## Launch File Arguments
### waypoints.launch.py
- ```port``` (default: ```/dev/ttyACM0```) - Serial port of the GPS

## Testing
- Run the node and check output
- TODO: rosbag stuff
