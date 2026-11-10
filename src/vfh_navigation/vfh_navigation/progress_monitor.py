"""Simulation-time progress and oscillation monitoring."""

from collections import deque
from dataclasses import dataclass
import math

from .geometry import wrap_angle


@dataclass(frozen=True)
class ProgressResult:
    """Window progress and cumulative steering reversals."""

    progress: float
    stuck: bool
    turn_reversals: int


class ProgressMonitor:
    """Measure decreasing distance-to-go, including route distance during detours."""

    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        """Clear episode history."""
        self.samples = deque()
        self.turn_reversals = 0
        self.last_turn = 0
        self.translation_reference = None
        self.last_translation_time = None

    def restart_window(self):
        """Restart distance measurements after installing a different route."""
        self.samples.clear()
        self.translation_reference = None
        self.last_translation_time = None

    def update(self, now, remaining, angular, active=True, alignment_error=0.0, yaw=None):
        """Return progress without counting waiting/planning time as active motion."""
        if self.samples and now < self.samples[-1][0]:
            self.reset()
        turn = 1 if angular > 0.05 else -1 if angular < -0.05 else 0
        if turn and self.last_turn and turn != self.last_turn:
            self.turn_reversals += 1
        if turn:
            self.last_turn = turn
        if not active:
            self.restart_window()
            return ProgressResult(0.0, False, self.turn_reversals)
        if self.translation_reference is None:
            self.translation_reference, self.last_translation_time = remaining, now
        if self.translation_reference-remaining >= self.config.minimum_progress:
            self.translation_reference, self.last_translation_time = remaining, now
        self.samples.append((now, remaining, abs(alignment_error), yaw))
        while len(self.samples) > 1 and now-self.samples[1][0] >= self.config.progress_window:
            self.samples.popleft()
        progress = self.samples[0][1]-remaining
        # Count bounded alignment improvement so a legitimate turn toward a goal
        # behind the 120-degree scan is not mistaken for a positional deadlock.
        # Report positional/route progress separately; oscillation yields no net gain.
        alignment_progress = self.config.safe_radius*max(
            0.0, self.samples[0][2]-abs(alignment_error))
        rotation_progress = 0.0
        if yaw is not None and self.samples[0][3] is not None:
            rotation_progress = self.config.safe_radius*abs(wrap_angle(yaw-self.samples[0][3]))
        # Allow a bounded normal turn before demanding translation. Net rotation
        # helps a real avoidance turn, while reversals do not count as net progress.
        turn_grace = self.config.progress_window + math.pi/max(self.config.max_angular_speed, 1e-6)
        turning = (max(alignment_progress, rotation_progress)
                   if now-self.last_translation_time <= turn_grace else 0.0)
        stuck = (now-self.samples[0][0] >= self.config.progress_window and
                 progress+turning < self.config.minimum_progress)
        return ProgressResult(progress, stuck, self.turn_reversals)
