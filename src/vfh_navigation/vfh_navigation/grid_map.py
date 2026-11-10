"""Immutable occupancy-grid snapshots and conservative traversability."""

from dataclasses import dataclass
import hashlib
import math

import numpy as np

from .geometry import inverse_transform, transform_point
from .models import Pose2D


@dataclass(frozen=True)
class GridMap:
    """Inflated row-major grid; indices are (column, row)."""

    resolution: float
    origin: Pose2D
    blocked: np.ndarray
    fingerprint: str
    obstacles: np.ndarray
    safe_radius: float
    values: np.ndarray
    free_threshold: int

    @classmethod
    def from_data(cls, data, width, height, resolution, origin=Pose2D(),
                  safe_radius=0.43, free_threshold=25):
        """Copy and inflate all nonfree cells, including unknown cells and boundaries."""
        if (width <= 0 or height <= 0 or len(data) != width*height or
                not all(math.isfinite(v) for v in
                        (resolution, origin.x, origin.y, origin.yaw, safe_radius)) or
                resolution <= 0 or safe_radius < 0):
            raise ValueError('invalid map metadata or data size')
        values = np.array(data, dtype=float).reshape(height, width)
        if not np.isfinite(values).all() or ((values < -1) | (values > 100)).any():
            raise ValueError('invalid occupancy values')
        obstacle = (values < 0) | (values > free_threshold)
        blocked = obstacle.copy()
        # Include the occupied cell's extent. Continuous segment checks below
        # verify the actual swept circle, avoiding a second cell-size margin.
        radius = safe_radius + (math.sqrt(2)*resolution/2 if safe_radius else 0)
        cells = int(math.ceil(radius/resolution))
        for dy in range(-min(cells, height-1), min(cells, height-1)+1):
            for dx in range(-min(cells, width-1), min(cells, width-1)+1):
                if math.hypot(dx, dy)*resolution > radius+1e-12:
                    continue
                src_y = slice(max(0, -dy), min(height, height-dy))
                src_x = slice(max(0, -dx), min(width, width-dx))
                dst_y = slice(max(0, dy), min(height, height+dy))
                dst_x = slice(max(0, dx), min(width, width+dx))
                blocked[dst_y, dst_x] |= obstacle[src_y, src_x]
        yy, xx = np.indices((height, width))
        boundary_distance = np.minimum.reduce(
            [xx+0.5, yy+0.5, width-xx-0.5, height-yy-0.5])*resolution
        blocked |= boundary_distance < safe_radius+resolution/2
        blocked.setflags(write=False)
        obstacle.setflags(write=False)
        values.setflags(write=False)
        digest = hashlib.sha256()
        digest.update(repr((resolution, origin, width, height, safe_radius)).encode())
        digest.update(blocked.tobytes())
        digest.update(obstacle.tobytes())
        digest.update((values < 0).tobytes())
        return cls(float(resolution), origin, blocked, digest.hexdigest(),
                   obstacle, safe_radius, values, free_threshold)

    @property
    def width(self):
        """Return grid width in cells."""
        return self.blocked.shape[1]

    @property
    def height(self):
        """Return grid height in cells."""
        return self.blocked.shape[0]

    def world_to_grid(self, point):
        """Convert world coordinates through the inverse map-origin transform."""
        x, y = transform_point(point, inverse_transform(self.origin))
        return math.floor(x/self.resolution), math.floor(y/self.resolution)

    def grid_to_world(self, cell):
        """Return a cell center in the map frame."""
        return transform_point(((cell[0]+0.5)*self.resolution,
                                (cell[1]+0.5)*self.resolution), self.origin)

    def contains(self, cell):
        """Test bounds before indexing (negative indices must never wrap)."""
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, cell):
        """Treat outside-map coordinates as blocked."""
        return self.contains(cell) and not bool(self.blocked[cell[1], cell[0]])

    def cell_status(self, cell):
        """Explain endpoint failures instead of conflating unknown and occupied cells."""
        if not self.contains(cell):
            return 'outside_map'
        value = self.values[cell[1], cell[0]]
        if value < 0:
            return 'unknown'
        if value > self.free_threshold:
            return 'occupied_or_uncertain'
        return 'free' if self.is_free(cell) else 'insufficient_clearance'

    def segment_clear(self, start, end):
        """Check every crossed cell and both sides of diagonal grid corners."""
        a = np.asarray(transform_point(start, inverse_transform(self.origin)))/self.resolution
        b = np.asarray(transform_point(end, inverse_transform(self.origin)))/self.resolution
        cell = np.floor(a).astype(int)
        target = np.floor(b).astype(int)
        delta = b-a
        step = np.sign(delta).astype(int)
        interval = np.full(2, np.inf)
        crossing = np.full(2, np.inf)
        for axis in range(2):
            if step[axis]:
                interval[axis] = abs(1/delta[axis])
                edge = cell[axis]+(1 if step[axis] > 0 else 0)
                crossing[axis] = (edge-a[axis])/delta[axis]
        for _ in range(self.width+self.height+3):
            if not self.is_free(tuple(cell)):
                return False
            if np.array_equal(cell, target):
                return self._swept_circle_clear(a*self.resolution, b*self.resolution)
            if abs(crossing[0]-crossing[1]) < 1e-10:
                if (not self.is_free((cell[0]+step[0], cell[1])) or
                        not self.is_free((cell[0], cell[1]+step[1]))):
                    return False
                cell += step
                crossing += interval
            else:
                axis = int(np.argmin(crossing))
                cell[axis] += step[axis]
                crossing[axis] += interval[axis]
        return False

    def _swept_circle_clear(self, start, end):
        """Check continuous capsule-to-cell-square clearance in map-local metres."""
        radius, resolution = self.safe_radius, self.resolution
        if radius == 0:
            return True  # The supercover test already checked all intersected cells.
        endpoints = np.asarray((start, end))
        limits = np.array((self.width, self.height))*resolution
        if np.any(endpoints < radius) or np.any(endpoints > limits-radius):
            return False
        low = np.maximum(0, np.floor((endpoints.min(axis=0)-radius)/resolution).astype(int))
        high = np.minimum((self.width, self.height),
                          np.floor((endpoints.max(axis=0)+radius)/resolution).astype(int)+1)
        yy, xx = np.nonzero(self.obstacles[low[1]:high[1], low[0]:high[0]])
        if not len(xx):
            return True
        boxes_min = np.column_stack((xx+low[0], yy+low[1]))*resolution
        boxes_max = boxes_min+resolution
        delta = end-start
        # Slab intersection catches a segment crossing the interior of a cell.
        enter, leave = np.zeros(len(xx)), np.ones(len(xx))
        for axis in range(2):
            if abs(delta[axis]) < 1e-12:
                outside = (start[axis] < boxes_min[:, axis]) | (
                    start[axis] > boxes_max[:, axis])
                leave[outside] = -1
            else:
                first = (boxes_min[:, axis]-start[axis])/delta[axis]
                second = (boxes_max[:, axis]-start[axis])/delta[axis]
                enter = np.maximum(enter, np.minimum(first, second))
                leave = np.minimum(leave, np.maximum(first, second))
        if np.any(enter <= leave):
            return False
        for point in endpoints:
            separation = np.maximum(np.maximum(boxes_min-point, point-boxes_max), 0)
            if np.any(np.sum(separation**2, axis=1) <= radius**2):
                return False
        corners = np.stack((boxes_min, boxes_max,
                            np.column_stack((boxes_min[:, 0], boxes_max[:, 1])),
                            np.column_stack((boxes_max[:, 0], boxes_min[:, 1]))), axis=1)
        length_squared = float(np.dot(delta, delta))
        if length_squared > 1e-12:
            projection = np.clip(np.sum((corners-start)*delta, axis=2)/length_squared, 0, 1)
            distances_squared = np.sum((corners-(start+projection[:, :, None]*delta))**2, axis=2)
            if np.any(distances_squared <= radius**2):
                return False
        return True

    def path_clear(self, points):
        """Validate route vertices and the segments joining them."""
        return (bool(points) and all(self.is_free(self.world_to_grid(p)) for p in points)
                and all(self.segment_clear(a, b) for a, b in zip(points, points[1:])))
