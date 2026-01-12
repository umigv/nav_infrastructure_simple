import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from robot_localization.srv import SetDatum
from statistics import mean

class GpsInitializer(Node):
    def __init__(self):
        super().__init__("gps_initializer")

        self.get_logger().info("GPS Initializer Node Started")

        self.client = self.create_client(SetDatum, "/navsat_transform/set_datum")
        self.gps_subscriber = self.create_subscription(NavSatFix, "/gps_coords", self.gps_callback, 10)
        self.gps_samples: list[NavSatFix] = []
        self.sent = False

    def gps_callback(self, msg: NavSatFix):
        if self.sent:
            return
        
        self.get_logger().info(f"Received GPS fix: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}")
        self.gps_samples.append(msg)

        if len(self.gps_samples) < 10:
            return
        
        if not self.client.service_is_ready():
            self.get_logger().warn(f"Service /navsat_transform/set_datum not ready yet; waiting for next fix...")
            return

        self.get_logger().info(f"Collected {len(self.gps_samples)} GPS samples, calculating average...")
        latitude = mean(sample.latitude for sample in self.gps_samples)
        longitude = mean(sample.longitude for sample in self.gps_samples)
        altitude = mean(sample.altitude for sample in self.gps_samples)

        self.get_logger().info(f"Setting datum to average latitude={latitude:.8f}, longitude={longitude:.8f}, altitude={altitude:.2f}")
        request = SetDatum.Request()
        request.geo_pose.position.latitude = latitude
        request.geo_pose.position.longitude = longitude
        request.geo_pose.position.altitude = altitude
        request.geo_pose.orientation.w = 1.0

        fut = self.client.call_async(request)
        fut.add_done_callback(self.on_response)

        self.sent = True
        self.destroy_subscription(self.gps_subscriber)

    def on_response(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info("Datum set successfully. map frame is now aligned to startup GPS origin.")
            else:
                self.get_logger().error(f"Datum set failed: {resp.message}")
        except Exception as e:
            self.get_logger().error(f"SetDatum call error: {e}")

def main():
    rclpy.init()
    node = GpsInitializer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
