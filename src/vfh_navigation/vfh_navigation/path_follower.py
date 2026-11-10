"""Monotonic route progress and collision-checked lookahead targets."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class FollowResult:
    """Local map-frame target and remaining route distance."""

    target: Optional[Tuple[float, float]]
    remaining: float
    deviation: float


class PathFollower:
    """Track path arc length without jumping across blocked corners."""

    def __init__(self, lookahead=0.60):
        self.lookahead = lookahead
        self.reset()

    def reset(self):
        """Discard the active route and arc-length cursor."""
        self.path = ()
        self.lengths = np.array([])
        self.progress = 0.0

    def set_path(self, path):
        """Install an immutable route and initialize tracking history."""
        self.path = tuple(tuple(p) for p in path)
        self.lengths = np.concatenate(([0.0], np.cumsum([
            math.dist(a, b) for a, b in zip(self.path, self.path[1:])])))
        self.progress = 0.0

    def point_at(self, arc):
        """Interpolate a coordinate along the polyline."""
        if len(self.path) == 1:
            return self.path[0]
        index = min(int(np.searchsorted(self.lengths, arc, side='right'))-1,
                    len(self.path)-2)
        index = max(0, index)
        length = self.lengths[index+1]-self.lengths[index]
        t = min(1.0, max(0.0, (arc-self.lengths[index])/length)) if length else 0.0
        a, b = self.path[index:index+2]
        return a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])

    def remaining_path(self):
        """Return the untraversed polyline for map-change validation."""
        if not self.path:
            return ()
        return (self.point_at(self.progress), *(
            p for p, s in zip(self.path, self.lengths) if s > self.progress+1e-9))

    def update(self, position, grid):
        """Find a nearby forward projection and a visible target up to lookahead away."""
        if not self.path:
            return FollowResult(None, math.inf, math.inf)
        projections = []
        for i, (a, b) in enumerate(zip(self.path, self.path[1:])):
            if self.lengths[i+1] < self.progress-1e-9:
                continue
            # Bound projection advancement to avoid jumping to another branch at a crossing.
            if self.lengths[i] > self.progress+2*self.lookahead:
                break
            a, b = np.asarray(a), np.asarray(b)
            vector = b-a
            t = float(np.clip(np.dot(np.asarray(position)-a, vector) /
                              max(np.dot(vector, vector), 1e-12), 0, 1))
            arc = max(self.progress, self.lengths[i]+t*(self.lengths[i+1]-self.lengths[i]))
            point = self.point_at(arc)
            if grid.segment_clear(position, point):
                projections.append((math.dist(position, point), arc))
        if projections:
            deviation, arc = min(projections)
            self.progress = max(self.progress, arc)
        else:
            deviation = math.dist(position, self.point_at(self.progress))
        limit = min(self.lengths[-1], self.progress+self.lookahead)
        candidates = [limit, *(s for s in self.lengths if self.progress < s < limit)]
        candidates.extend(np.linspace(self.progress, limit, 12))
        target = None
        for arc in sorted(set(candidates), reverse=True):
            point = self.point_at(arc)
            if grid.segment_clear(position, point):
                target = point
                break
        remaining = float(self.lengths[-1]-self.progress+deviation)
        return FollowResult(target, remaining, deviation)
