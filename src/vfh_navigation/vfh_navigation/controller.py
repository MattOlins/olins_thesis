"""Composable local VFH pipeline, usable without ROS or Gazebo."""

from dataclasses import dataclass
import math
from typing import Optional

from .direction_selector import DirectionSelector
from .geometry import inverse_transform, scan_points, transform_point, wrap_angle
from .histogram import HistogramBuilder, HistogramResult
from .models import NavigationConfig, VelocityCommand
from .velocity_controller import VelocityController


@dataclass(frozen=True)
class LocalResult:
    """Command and evidence for experiments or future adaptive policies."""

    command: VelocityCommand
    reason: str
    selected: Optional[float] = None
    histogram: Optional[HistogramResult] = None


class VFHController:
    """Use the exact same safety and steering components in both navigation modes."""

    def __init__(self, config=None):
        self.config = config or NavigationConfig()
        self.histogram = HistogramBuilder(self.config)
        self.selector = DirectionSelector(self.config)
        self.velocity = VelocityController(self.config)
        self.obstacle_memory = {}
        self.elapsed = 0.0

    def reset(self, clear_obstacles=True):
        """Reset all local-controller history."""
        self.selector.reset()
        self.velocity.reset()
        if clear_obstacles:
            self.obstacle_memory.clear()
            self.elapsed = 0.0

    def stop(self, reason):
        """Return a safe command without forgetting odometry-frame steering intent."""
        return LocalResult(self.velocity.stop(), reason)

    def step(self, scan, pose, yaw_rate, target, dt):
        """Return a command and diagnostics; all coordinates use odom except the scan."""
        if not all(math.isfinite(v) for v in (
                pose.x, pose.y, pose.yaw, yaw_rate, target[0], target[1], dt)) or dt <= 0:
            return self.stop('invalid_control_input')
        goal_angle = wrap_angle(math.atan2(target[1]-pose.y, target[0]-pose.x)-pose.yaw)
        try:
            self.elapsed += dt
            inverse = inverse_transform(pose)
            self.obstacle_memory = {
                cell: observation for cell, observation in self.obstacle_memory.items()
                if self.elapsed-observation[2] <= self.config.obstacle_memory_seconds and
                math.dist((pose.x, pose.y), observation[:2]) < self.config.influence_distance}
            _, _, points = scan_points(scan)
            for point in points:
                if math.hypot(*point) >= self.config.influence_distance:
                    continue
                world = transform_point(point, pose)
                key = tuple(round(value/self.config.obstacle_memory_resolution) for value in world)
                self.obstacle_memory[key] = (*world, self.elapsed)
            remembered = [transform_point(item[:2], inverse)
                          for item in self.obstacle_memory.values()]
            histogram = self.histogram.build(scan, remembered)
        except (ValueError, OverflowError):
            return self.stop('invalid_scan_geometry')
        selected = self.selector.choose(histogram, goal_angle, pose.yaw, dt)
        command, reason = self.velocity.command(selected, goal_angle, yaw_rate, histogram, dt)
        return LocalResult(command, reason, selected, histogram)
