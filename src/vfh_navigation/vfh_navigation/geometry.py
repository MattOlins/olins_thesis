"""Planar geometry without ROS dependencies."""

import math

import numpy as np

from .models import Pose2D


def wrap_angle(angle):
    """Wrap an angular difference to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(x, y, z, w):
    """Extract yaw from a finite, nonzero quaternion."""
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid quaternion')
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


def transform_point(point, transform):
    """Apply a source-to-target planar transform."""
    c, s = math.cos(transform.yaw), math.sin(transform.yaw)
    return (transform.x + c*point[0] - s*point[1],
            transform.y + s*point[0] + c*point[1])


def inverse_transform(transform):
    """Invert a planar rigid transform."""
    c, s = math.cos(transform.yaw), math.sin(transform.yaw)
    return Pose2D(-c*transform.x - s*transform.y,
                  s*transform.x - c*transform.y, -transform.yaw)


def scan_points(scan):
    """Return bearings, valid-ray flags, and finite hits in the base frame."""
    ranges = np.asarray(scan.ranges, dtype=float)
    bearings = scan.angle_min + np.arange(len(ranges))*scan.angle_increment
    valid = np.isposinf(ranges) | (
        np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max))
    valid &= (bearings >= scan.angle_min-1e-7) & (bearings <= scan.angle_max+1e-7)
    hits = valid & np.isfinite(ranges)
    angles = bearings[hits] + scan.sensor_pose.yaw
    points = np.column_stack((scan.sensor_pose.x + ranges[hits]*np.cos(angles),
                              scan.sensor_pose.y + ranges[hits]*np.sin(angles)))
    return bearings, valid, points
