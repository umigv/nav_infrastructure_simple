from dataclasses import dataclass


@dataclass(frozen=True)
class GpsOriginInitializerConfig:
    min_samples_required: int = 100
    min_sample_duration_sec: float = 5.0
    max_sample_duration_sec: float = 30.0
    max_h_sigma_m: float = 1.0

    def __post_init__(self):
        if self.min_samples_required <= 0:
            raise ValueError("GpsOriginInitializerConfig: min_samples_required must be > 0")

        if self.min_sample_duration_sec <= 0:
            raise ValueError("GpsOriginInitializerConfig: min_sample_duration_sec must be > 0")

        if self.max_sample_duration_sec <= 0:
            raise ValueError("GpsOriginInitializerConfig: max_sample_duration_sec must be > 0")

        if self.min_sample_duration_sec > self.max_sample_duration_sec:
            raise ValueError("GpsOriginInitializerConfig: max_sample_duration_sec must be > min_sample_duration_sec")

        if self.max_h_sigma_m <= 0:
            raise ValueError("GpsOriginInitializerConfig: max_h_sigma_m must be > 0")
