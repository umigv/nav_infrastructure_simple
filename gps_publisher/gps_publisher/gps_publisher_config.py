from dataclasses import dataclass


@dataclass(frozen=True)
class GpsPublisherConfig:
    serial_port: str = "/dev/ttyACM0"
    poll_rate_hz: float = 100.0
    gps_frame_id: str = "gps_link"

    def __post_init__(self):
        if self.poll_rate_hz <= 0:
            raise ValueError("GpsPublisherConfig: poll_rate_hz must be > 0")
