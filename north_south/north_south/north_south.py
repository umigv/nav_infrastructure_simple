import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import MagneticField


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            MagneticField,
            '/zed/zed_node/imu/mag',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg: MagneticField):
        # 0 radians is due north, need to verify this
        curr_compass_heading = math.atan2(msg.magnetic_field.y, msg.magnetic_field.x)
        curr_compass_heading = (curr_compass_heading + (2 * math.pi)) % (2 * math.pi)

        self.get_logger().info(f"Compass Heading: {math.degrees(curr_compass_heading)} degrees")



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()