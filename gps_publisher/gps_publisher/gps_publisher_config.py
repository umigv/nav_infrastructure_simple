from dataclasses import dataclass


@dataclass(frozen=True)
class GpsPublisherConfig:
    """Configuration for the GPS publisher node.

    Attributes:
        serial_port: The serial port device path for the GPS receiver.
        poll_period_s: The period in seconds between GPS polls.
        gps_frame_id: The TF frame ID to use in published GPS messages.
    """

    serial_port: str = "/dev/ttyACM0"
    poll_period_s: float = 0.1
    gps_frame_id: str = "gps_link"

    def __post_init__(self) -> None:
        if self.poll_period_s <= 0:
            raise ValueError("GpsPublisherConfig: poll_period_s must be > 0")
