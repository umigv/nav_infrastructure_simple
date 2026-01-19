import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from serial import Serial # pip install pyserial to get this package, not pip install serial
from pyubx2 import UBXReader, UBXMessage, UBX_PROTOCOL

UBX_FIX_TYPE_NO_FIX = 0
UBX_FIX_TYPE_TIME_ONLY = 5
UBX_FLAG_GNSSFIXOK = 0x01

class GPSCoordPublisher(Node):
    def __init__(self):
        super().__init__('gps')

        self.publisher = self.create_publisher(NavSatFix, 'gps/raw', 10)

        self.stream = Serial('/dev/ttyACM0', 460800, timeout=3)
        self.ubx_reader = UBXReader(self.stream, protfilter=UBX_PROTOCOL)

        self.create_timer(0.01, self.poll)
        self.frame_id = "gps_link"
        
    def poll(self):
        try:
            _, msg = self.ubx_reader.read()
        except Exception as e:
            self.get_logger().error(f"Error reading GPS data: {e}")
            return

        if msg is None:
            return

        if msg.identity not in ("NAV-PVT", "NAV2-PVT"):
            return

        if msg.fixType == UBX_FIX_TYPE_NO_FIX or msg.fixType == UBX_FIX_TYPE_TIME_ONLY:
            return

        if (msg.flags & UBX_FLAG_GNSSFIXOK) == 0:
            return

        self.get_logger().debug(f"GPS Msg: {msg}")
        self.publish_from_pvt(msg)

    def publish_from_pvt(self, data: UBXMessage):
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self.frame_id

        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS

        fix.latitude = data.lat * 1e-7
        fix.longitude = data.lon * 1e-7
        fix.altitude = data.hMSL * 1e-3
        
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        hcov = max(data.hAcc * 1e-3, 0.05) ** 2
        vcov = max(data.vAcc * 1e-3, 0.08) ** 2
        fix.position_covariance = [
            hcov, 0.0, 0.0,
            0.0, hcov, 0.0,
            0.0, 0.0, vcov
        ]

        self.publisher.publish(fix)

def main(args=None):
    rclpy.init(args=args)
    publisher = GPSCoordPublisher()
    rclpy.spin(publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
