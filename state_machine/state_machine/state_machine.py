from __future__ import annotations

from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from std_msgs.msg import String
from std_srvs.srv import SetBool

class RobotMode(str, Enum):
    NORMAL = "normal"
    RAMP = "ramp"
    RECOVERY = "recovery"

class RobotModePublisher(Node):
    def __init__(self) -> None:
        super().__init__("state_machine")

        self.publisher = self.create_publisher(String, "/state", QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        ))
        self.timer = self.create_timer(10, self.publish_state)

        self.set_ramp_service = self.create_service(SetBool, "/state/set_ramp", self.set_ramp_callback)
        self.ramp_enabled = False

        self.set_recovery_service = self.create_service(SetBool, "/state/set_recovery", self.set_recovery_callback)
        self.recovery_enabled = False

    def compute_state(self) -> RobotMode:
        if self._recovery_enabled:
            return RobotMode.RECOVERY
        
        if self._ramp_enabled:
            return RobotMode.RAMP
        
        return RobotMode.NORMAL

    def publish_state(self) -> None:
        msg = String()
        msg.data = self.compute_state().value
        self.publisher.publish(msg)

    def set_recovery_service(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        if req.data == self.recovery_enabled:
            res.message = f"Recovery already {'enabled' if req.data else 'disabled'}."
        else:
            res.message = f"Recovery {'enabled' if req.data else 'disabled'}."
        
        self.recovery_enabled = req.data
        res.success = True
        self.get_logger().info(res.message + f" (ramp_enabled={self.ramp_enabled})")
        return res

    def set_ramp_service(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        if req.data == self.ramp_enabled:
            res.message = f"Ramp already {'enabled' if req.data else 'disabled'}."
        else:
            res.message = f"Ramp {'enabled' if req.data else 'disabled'}."
        
        self.ramp_enabled = req.data
        res.success = True
        self.get_logger().info(res.message + f" (recovery_enabled={self.recovery_enabled})")
        return res

def main() -> None:
    rclpy.init()
    node = RobotModePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
