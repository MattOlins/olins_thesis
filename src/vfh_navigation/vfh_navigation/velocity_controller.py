"""Shared forward-only velocity controller with final sensor safety checks."""

import math

import numpy as np

from .models import VelocityCommand


class VelocityController:
    """Damped angular steering with measured-time acceleration limits."""

    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        """Reset the previous command after an immediate stop."""
        self.last_angular_command = 0.0

    def stop(self):
        """Stop immediately; safety stops bypass acceleration limiting."""
        self.reset()
        return VelocityCommand()

    def command(self, selected, goal_angle, yaw_rate, histogram, dt):
        """Compute a command only from current local sensor evidence."""
        c = self.config
        if selected is None:
            return self.stop(), 'no_clear_sector'
        if not histogram.forward_observed:
            return self.stop(), 'unknown_forward_clearance'
        if not math.isfinite(yaw_rate) or not math.isfinite(dt) or dt <= 0:
            return self.stop(), 'invalid_control_time_or_velocity'
        omega = float(np.clip(c.angular_kp*selected-c.angular_kd*yaw_rate,
                              -c.max_angular_speed, c.max_angular_speed))
        if abs(selected) < math.radians(5) and abs(goal_angle) < math.radians(5):
            omega = 0.0
        limit = c.angular_accel_limit*dt
        omega = float(self.last_angular_command + np.clip(
            omega-self.last_angular_command, -limit, limit))
        self.last_angular_command = omega
        linear = c.max_linear_speed*max(0.0, 1.0-abs(selected)/0.45)
        forward = np.abs(histogram.angles) <= histogram.sector_width/2+1e-7
        facing_clear = bool(forward.any() and histogram.observed[forward].all() and
                            (histogram.density[forward] < c.histogram_threshold).all())
        if not facing_clear:
            linear = 0.0
        if histogram.front_distance < c.stop_distance:
            linear = 0.0
        reason = ('turning_to_clear_sector' if not facing_clear else
                  'front_obstacle' if linear == 0 and abs(selected) < 0.45 else 'ok')
        return VelocityCommand(linear, omega), reason
