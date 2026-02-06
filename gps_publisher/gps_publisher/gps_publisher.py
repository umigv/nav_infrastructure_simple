from datetime import datetime, timezone
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from builtin_interfaces.msg import Time
from serial import Serial
from pyubx2 import UBXReader, UBX_PROTOCOL

UBX_FIX_TYPE_NO_FIX = 0
UBX_FIX_TYPE_TIME_ONLY = 5

class GPSCoordPublisher(Node):
    def __init__(self):
        super().__init__('gps')

        self.publisher = self.create_publisher(NavSatFix, 'gps/raw', 10)

        self.stream = Serial('/dev/ttyACM0', 460800, timeout=0.1)
        self.ubx_reader = UBXReader(self.stream, protfilter=UBX_PROTOCOL)

        self.create_timer(0.01, self.poll)
        self.frame_id = "gps_link"
        
    def poll(self):
        try:
            _, msg = self.ubx_reader.read()
        except Exception as e:
            self.get_logger().error(f"Error reading GPS msg: {e}")
            return

        if msg is None:
            return

        if msg.identity not in ("NAV-PVT", "NAV2-PVT"):
            return

        if msg.fixType in (UBX_FIX_TYPE_NO_FIX, UBX_FIX_TYPE_TIME_ONLY):
            self.get_logger().debug(f"Dropping GPS msg with no fix: {msg}")
            return

        if not msg.gnssFixOk:
            self.get_logger().debug(f"Dropping GPS msg with fix not ok: {msg}")
            return
        
        if msg.invalidLlh:
            self.get_logger().debug(f"Dropping GPS msg with invalid LLH: {msg}")
            return

        self.get_logger().debug(f"Publishing GPS Msg: {msg}")
        self.publish_from_pvt(msg)

    def publish_from_pvt(self, data):
        fix = NavSatFix()
        fix.header.stamp = self.resolve_timestamp(data)
        fix.header.frame_id = self.frame_id

        fix.status.status = NavSatStatus.STATUS_GBAS_FIX if data.diffSoln else NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS

        fix.latitude = data.lat
        fix.longitude = data.lon
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

    def resolve_timestamp(self, data) -> Time:
        if not (data.validDate and data.validTime and data.fullyResolved):
            return self.get_clock().now().to_msg()

        seconds = datetime(
            year=data.year,
            month=data.month,
            day=data.day,
            hour=data.hour,
            minute=data.min,
            second=data.second,
            tzinfo=timezone.utc
        ).timestamp()
        
        if data.nano < 0:
            return Time(sec=int(seconds) - 1, nanosec=int(data.nano) + 1_000_000_000)

        return Time(sec=int(seconds), nanosec=int(data.nano))

def main(args=None):
    rclpy.init(args=args)
    publisher = GPSCoordPublisher()
    rclpy.spin(publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
