"""ROS-independent navigation inputs and shared configuration (SI units)."""

from dataclasses import dataclass
import math
from typing import Tuple


@dataclass(frozen=True)
class Pose2D:
    """Planar pose or rigid transform."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class ScanData:
    """Immutable scan with the sensor-to-base transform captured at acquisition."""

    ranges: Tuple[float, ...]
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float
    sensor_pose: Pose2D = Pose2D()


@dataclass(frozen=True)
class VelocityCommand:
    """Forward and yaw velocity."""

    linear: float = 0.0
    angular: float = 0.0


@dataclass(frozen=True)
class NavigationConfig:
    """One set of local-control parameters for both comparison modes."""

    goal_tolerance: float = 0.20
    robot_radius: float = 0.33
    safety_margin: float = 0.10
    influence_distance: float = 1.50
    stop_distance: float = 0.45
    sector_width_deg: float = 5.0
    observed_fraction: float = 0.80
    obstacle_memory_seconds: float = 5.0
    obstacle_memory_resolution: float = 0.05
    histogram_threshold: float = 0.15
    max_linear_speed: float = 0.20
    max_angular_speed: float = 0.60
    goal_weight: float = 3.0
    heading_weight: float = 1.0
    smoothness_weight: float = 1.0
    switch_hysteresis: float = 0.35
    turn_side_penalty: float = 1.2
    angular_kp: float = 0.8
    angular_kd: float = 0.25
    angular_accel_limit: float = 0.8
    sensor_timeout: float = 0.5
    lookahead_distance: float = 0.60
    route_deviation: float = 0.50
    progress_window: float = 5.0
    minimum_progress: float = 0.15
    replan_interval: float = 1.0
    max_stuck_replans: int = 3
    free_threshold: int = 25

    @property
    def safe_radius(self):
        """Return the same inflated radius for mapping and local control."""
        return self.robot_radius + self.safety_margin

    def __post_init__(self):
        """Reject invalid settings before any motion is published."""
        for name, value in vars(self).items():
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        for name in ('robot_radius', 'influence_distance', 'sector_width_deg',
                     'sensor_timeout', 'lookahead_distance', 'progress_window',
                     'minimum_progress', 'replan_interval', 'max_stuck_replans',
                     'obstacle_memory_resolution'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive')
        if not 0 < self.observed_fraction <= 1:
            raise ValueError('observed_fraction must be in (0, 1]')
        if not 0 < self.histogram_threshold <= 1:
            raise ValueError('histogram_threshold must be in (0, 1]')
        if not 0 <= self.free_threshold <= 100:
            raise ValueError('free_threshold must be in [0, 100]')
        if self.stop_distance < self.safe_radius:
            raise ValueError('stop_distance must cover robot_radius + safety_margin')
        if self.influence_distance <= self.stop_distance:
            raise ValueError('influence_distance must exceed stop_distance')
