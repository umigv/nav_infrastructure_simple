import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TestMovement(Node):
    def __init__(self):
        super().__init__("test_movement")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(1.0, self.move_sequence)
        self.step = 0
        self.get_logger().info("Test movement node started.")

    def move_sequence(self):
        """Runs the robot through a series of sample movements to test."""
        cmd = Twist()

        # Move forward
        if self.step < 5:
            cmd.linear.x = 0.5  # Move forward
            cmd.angular.z = 0.0

        # Turn left
        elif self.step < 10:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Rotate

        # Move backward
        elif self.step < 15:
            cmd.linear.x = 0.5  # Move backward
            cmd.angular.z = 0.0

        # Turn right
        elif self.step < 20:
            cmd.linear.x = 0.0
            cmd.angular.z = -0.5  # Rotate

        else:
            self.get_logger().info("Test sequence complete.")
            self.destroy_node()
            return

        self.pub.publish(cmd)
        self.step += 1


def main():
    rclpy.init()
    node = TestMovement()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
