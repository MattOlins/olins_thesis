"""Measured-FOV histograms with independent observation and obstacle masks."""

from dataclasses import dataclass
import math

import numpy as np

from .geometry import scan_points


@dataclass(frozen=True)
class HistogramResult:
    """Polar obstacle evidence and conservative forward-clearance evidence."""

    angles: np.ndarray
    density: np.ndarray
    observed: np.ndarray
    sector_width: float
    forward_observed: bool
    front_distance: float
    clearance: float


class HistogramBuilder:
    """Build sectors only over the sensor's physical angular coverage."""

    def __init__(self, config):
        self.config = config

    def build(self, scan, remembered_points=()):
        """Project hits using mounting geometry without expanding observation coverage."""
        c = self.config
        fov = scan.angle_max - scan.angle_min
        metadata = (scan.angle_min, scan.angle_max, scan.angle_increment,
                    scan.range_min, scan.range_max, scan.sensor_pose.x,
                    scan.sensor_pose.y, scan.sensor_pose.yaw)
        if (not all(math.isfinite(v) for v in metadata) or fov <= 0 or
                scan.angle_increment <= 0 or scan.range_min < 0 or
                scan.range_max <= scan.range_min):
            raise ValueError('invalid scan geometry')
        count = max(1, round(fov / math.radians(c.sector_width_deg)))
        width = fov / count
        sensor_angles = scan.angle_min + (np.arange(count) + 0.5)*width
        angles = sensor_angles + scan.sensor_pose.yaw
        bearings, valid, points = scan_points(scan)
        # Missing rays count against expected coverage, including truncated messages.
        expected_bearings = np.linspace(
            scan.angle_min, scan.angle_max,
            int(round(fov / scan.angle_increment)) + 1)
        expected_ids = np.clip(
            np.floor((expected_bearings - scan.angle_min)/width).astype(int), 0, count-1)
        expected = np.bincount(expected_ids, minlength=count)
        in_fov = (bearings >= scan.angle_min-1e-7) & (bearings <= scan.angle_max+1e-7)
        ids = np.clip(np.floor((bearings - scan.angle_min)/width).astype(int), 0, count-1)
        usable = np.bincount(ids[valid & in_fov], minlength=count)
        observed = (expected > 0) & (usable >= c.observed_fraction*expected)
        density = np.zeros(count)
        obstacles = np.vstack((points, np.asarray(remembered_points).reshape(-1, 2)))
        for point in obstacles:
            distance = float(np.linalg.norm(point))
            if distance >= c.influence_distance:
                continue
            bearing = math.atan2(point[1], point[0])
            error = np.abs(np.arctan2(np.sin(angles-bearing), np.cos(angles-bearing)))
            if distance <= c.safe_radius:
                density[:] = 1.0
                continue
            inflation = math.asin(min(1.0, c.safe_radius/distance))
            blocked = error <= inflation + width/2
            density[blocked] = np.maximum(
                density[blocked], (1-distance/c.influence_distance)**2)

        half_angle = math.atan2(c.safe_radius, c.stop_distance)
        # Account for lateral mounting offset when determining required sensor coverage.
        required_min = math.atan2(-c.safe_radius-scan.sensor_pose.y,
                                  max(c.stop_distance-scan.sensor_pose.x, 1e-6))
        required_max = math.atan2(c.safe_radius-scan.sensor_pose.y,
                                  max(c.stop_distance-scan.sensor_pose.x, 1e-6))
        ray_angles = bearings + scan.sensor_pose.yaw
        forward_rays = (ray_angles >= required_min) & (ray_angles <= required_max)
        overlap = ((angles+width/2 >= required_min) &
                   (angles-width/2 <= required_max))
        covers_arc = (scan.angle_min+scan.sensor_pose.yaw <= required_min and
                      scan.angle_max+scan.sensor_pose.yaw >= required_max)
        forward_observed = bool(
            covers_arc and overlap.any() and forward_rays.any() and
            observed[overlap].all() and valid[forward_rays].all() and
            scan.range_max >= c.stop_distance+c.safe_radius)
        distances = np.linalg.norm(obstacles, axis=1)
        point_angles = np.arctan2(obstacles[:, 1], obstacles[:, 0])
        front = distances[np.abs(point_angles) <= half_angle]
        front_distance = float(front.min()) if len(front) else math.inf
        observed_distances = np.linalg.norm(points, axis=1)
        clearance = (float(observed_distances.min()-c.robot_radius)
                     if len(observed_distances) else math.inf)
        return HistogramResult(angles, density, observed, width, forward_observed,
                               front_distance, clearance)
