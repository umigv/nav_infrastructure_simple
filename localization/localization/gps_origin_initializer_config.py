from dataclasses import dataclass


@dataclass(frozen=True)
class GpsOriginInitializerConfig:
    """Config for GpsOriginInitializer.

    Attributes:
        min_samples_required: Minimum number of valid GPS samples before setting datum.
        min_sample_duration_s: Minimum time (s) to collect samples.
        max_sample_duration_s: Maximum time (s) before setting datum regardless of sample count.
        max_horizontal_stdev_m: Maximum horizontal accuracy (m) for a sample to be accepted.
    """

    min_samples_required: int = 100
    min_sample_duration_s: float = 5.0
    max_sample_duration_s: float = 30.0
    max_horizontal_stdev_m: float = 1.0

    def __post_init__(self) -> None:
        if self.min_samples_required <= 0:
            raise ValueError("GpsOriginInitializerConfig: min_samples_required must be > 0")
        if self.min_sample_duration_s <= 0:
            raise ValueError("GpsOriginInitializerConfig: min_sample_duration_s must be > 0")
        if self.max_sample_duration_s <= 0:
            raise ValueError("GpsOriginInitializerConfig: max_sample_duration_s must be > 0")
        if self.min_sample_duration_s > self.max_sample_duration_s:
            raise ValueError("GpsOriginInitializerConfig: max_sample_duration_s must be >= min_sample_duration_s")
        if self.max_horizontal_stdev_m <= 0:
            raise ValueError("GpsOriginInitializerConfig: max_horizontal_stdev_m must be > 0")
