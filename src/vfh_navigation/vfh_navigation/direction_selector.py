"""VFH steering costs and odometry-frame hysteresis."""

import math

import numpy as np

from .geometry import wrap_angle


class DirectionSelector:
    """Preserve steering intent as the robot's body frame rotates."""

    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        """Forget steering commitment at episode or final-goal boundaries."""
        self.previous_world_heading = None
        self.avoidance_side = 0
        self.clear_goal_time = 0.0

    def choose(self, histogram, goal_angle, yaw, dt):
        """Select an observed, sufficiently clear sector, or return None."""
        c = self.config
        clear = (histogram.density < c.histogram_threshold) & histogram.observed
        candidates = histogram.angles[clear]
        if not len(candidates):
            return None
        previous = (None if self.previous_world_heading is None else
                    wrap_angle(self.previous_world_heading-yaw))
        goal_error_all = np.abs(np.arctan2(np.sin(histogram.angles-goal_angle),
                                           np.cos(histogram.angles-goal_angle)))
        closest = int(np.argmin(goal_error_all))
        goal_in_fov = goal_error_all[closest] <= histogram.sector_width/2 + 1e-7
        goal_clear = bool(goal_in_fov and clear[closest])
        goal_blocked = bool(goal_in_fov and histogram.observed[closest] and
                            histogram.density[closest] >= c.histogram_threshold)
        self.clear_goal_time = (self.clear_goal_time+dt if goal_clear and
                                abs(goal_angle) < math.radians(20) else 0.0)
        if self.clear_goal_time >= 1.0-1e-9:
            self.avoidance_side = 0
        costs = np.array([
            c.goal_weight*abs(wrap_angle(a-goal_angle)) + c.heading_weight*abs(a) +
            c.smoothness_weight*(0 if previous is None else abs(wrap_angle(a-previous))) +
            (c.turn_side_penalty if goal_blocked and self.avoidance_side and
             np.sign(a) != self.avoidance_side else 0.0)
            for a in candidates])
        best = int(np.argmin(costs))
        if previous is not None:
            differences = np.abs(np.arctan2(np.sin(candidates-previous),
                                            np.cos(candidates-previous)))
            old = int(np.argmin(differences))
            if (differences[old] <= 1.5*histogram.sector_width and
                    costs[old] <= costs[best]+c.switch_hysteresis):
                best = old
        selected = float(candidates[best])
        self.previous_world_heading = wrap_angle(yaw+selected)
        if goal_blocked and not self.avoidance_side and abs(selected) > math.radians(15):
            self.avoidance_side = 1 if selected > 0 else -1
        return selected
