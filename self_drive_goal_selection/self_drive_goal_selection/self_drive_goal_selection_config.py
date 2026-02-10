from dataclasses import dataclass


@dataclass(frozen=True)
class SelfDriveGoalSelectionConfig:
    """
    Parameters controlling self drive goal selection
    """

    """How often to run new goal detection/publish loop"""
    goal_publish_rate_hz: float = 0.5

    def __post_init__(self):
        if self.goal_publish_rate_hz <= 0:
            raise ValueError("SelfDriveGoalSelectionConfig: goal_publish_rate_hz must be > 0")
