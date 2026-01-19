import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from serial import Serial # pip install pyserial to get this package, not pip install serial
from pyubx2 import  

class GPSCoordPublisher(Node):
    def __init__(self):
        super().__init__('gps')

        self.coords_publisher_ = self.create_publisher(NavSatFix, 'gps/raw', 10)

        self.stream = Serial('/dev/ttyACM0', 460800, timeout=3)
        self.ubx_reader = UBXReader(self.stream)

        self.create_timer(1, self.publish)
        self.frame_id = "gps_link"
        
    def publish(self):
        try:
            (_, data) = self.ubx_reader.read()
        except Exception as e:
            self.get_logger().error(f"Error reading GPS data: {e}")
            return

        self.get_logger().debug(f"GPS Data: {data}")

        if data.status == 'V':
            self.get_logger().error("GPS data is invalid, skipping publish.")
            return

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self.frame_id
        fix.status.service = 1
        fix.latitude = data.lat
        fix.longitude = data.lon
        fix.altitude = float(0)
        fix.position_covariance = [0.0] * 9
        fix.position_covariance_type = 0 

        self.coords_publisher_.publish(fix)

def main(args=None):
    rclpy.init(args=args)
    publisher = GPSCoordPublisher()
    rclpy.spin(publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
