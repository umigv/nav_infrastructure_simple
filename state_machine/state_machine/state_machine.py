from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import SetBool


class State(str, Enum):
    NORMAL = "normal"
    RAMP = "ramp"
    RECOVERY = "recovery"


class StateMachine(Node):
    def __init__(self) -> None:
        super().__init__("state_machine")

        self.publisher = self.create_publisher(
            String,
            "state",
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.last_state: State | None = None

        self.set_ramp_service = self.create_service(SetBool, "state/set_ramp", self.set_ramp_callback)
        self.ramp_enabled: bool = False

        self.set_recovery_service = self.create_service(SetBool, "state/set_recovery", self.set_recovery_callback)
        self.recovery_enabled: bool = False

        self.publish_state_if_changed(reason="init")

    def compute_state(self) -> State:
        if self.recovery_enabled:
            return State.RECOVERY

        if self.ramp_enabled:
            return State.RAMP

        return State.NORMAL

    def publish_state_if_changed(self, reason: str) -> None:
        state = self.compute_state()
        if state == self.last_state:
            return

        last_state_str = self.last_state.value if self.last_state is not None else "<none>"
        self.get_logger().info(f"Changed state from {last_state_str} to {state.value}, reason={reason}")

        self.last_state = state
        self.publisher.publish(String(data=state.value))

    def set_recovery_callback(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        if req.data == self.recovery_enabled:
            res.message = f"Recovery already {'enabled' if req.data else 'disabled'}."
        else:
            res.message = f"Recovery {'enabled' if req.data else 'disabled'}."

        self.recovery_enabled = req.data
        res.success = True
        self.get_logger().info(res.message + f" (ramp_enabled={self.ramp_enabled})")
        self.publish_state_if_changed(reason="state/set_recovery")
        return res

    def set_ramp_callback(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        if req.data == self.ramp_enabled:
            res.message = f"Ramp already {'enabled' if req.data else 'disabled'}."
        else:
            res.message = f"Ramp {'enabled' if req.data else 'disabled'}."

        self.ramp_enabled = req.data
        res.success = True
        self.get_logger().info(res.message + f" (recovery_enabled={self.recovery_enabled})")
        self.publish_state_if_changed(reason="state/set_ramp")
        return res


def main() -> None:
    rclpy.init()
    node = StateMachine()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
