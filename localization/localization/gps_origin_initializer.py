import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from robot_localization.srv import SetDatum
from statistics import mean

class GpsOriginInitializer(Node):
    def __init__(self):
        super().__init__("gps_origin_initializer")

        self.declare_parameter("gps_data_topic", "/gps_coords")
        self.declare_parameter("min_samples_required", 10)
        self.declare_parameter("set_origin_service", "/navsat_transform/set_datum")

        gps_data_topic = self.get_parameter("gps_data_topic").value
        self.get_logger().info(f"Subscribing to GPS data on topic: {gps_data_topic}")
        self.gps_subscriber = self.create_subscription(NavSatFix, gps_data_topic, self.gps_callback, 10)

        self.set_origin_service = self.get_parameter("set_origin_service").value
        self.get_logger().info(f"Waiting for service {self.set_origin_service} to be available...")
        self.client = self.create_client(SetDatum, self.set_origin_service)
        self.client.wait_for_service()

        self.min_samples_required = self.get_parameter("min_samples_required").value
        self.gps_samples: list[NavSatFix] = []
        self.sent = False
        self.get_logger().info(f"Waiting for {self.min_samples_required} samples to set GPS origin, don't move the robot")

    def gps_callback(self, msg: NavSatFix):
        if self.sent:
            return
        
        self.get_logger().info(f"Received data: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}")
        self.gps_samples.append(msg)

        if len(self.gps_samples) < self.min_samples_required:
            return

        self.get_logger().info(f"Collected {len(self.gps_samples)} GPS samples")
        latitude = mean(sample.latitude for sample in self.gps_samples)
        longitude = mean(sample.longitude for sample in self.gps_samples)
        altitude = mean(sample.altitude for sample in self.gps_samples)

        self.get_logger().info(f"Setting origin to average lat={latitude:.8f}, lon={longitude:.8f}, alt={altitude:.2f}")
        request = SetDatum.Request()
        request.geo_pose.position.latitude = latitude
        request.geo_pose.position.longitude = longitude
        request.geo_pose.position.altitude = altitude
        request.geo_pose.orientation.w = 1.0

        self.sent = True
        self.destroy_subscription(self.gps_subscriber)

        fut = self.client.call_async(request)
        fut.add_done_callback(self.on_response)

    def on_response(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info("Origin set successfully. map frame is now aligned to startup GPS origin.")
            else:
                self.get_logger().error(f"{self.set_origin_service} failed: {resp.message}")
        except Exception as e:
            self.get_logger().error(f"{self.set_origin_service} call error: {e}")
        
        self.destroy_node()

def main():
    rclpy.init()
    node = GpsOriginInitializer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
