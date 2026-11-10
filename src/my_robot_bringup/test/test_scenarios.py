"""Keep scenario goals, sensor limits, and packaged world fixtures consistent."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('manifest', sorted((ROOT/'config/scenarios').glob('*.yaml')))
def test_scenario_goal_matches_gazebo_marker(manifest):
    scenario = yaml.safe_load(manifest.read_text())
    world = ET.parse(ROOT/'worlds'/scenario['world']).getroot().find('world')
    marker = world.find("model[@name='goal_marker']/pose")
    position = [float(value) for value in marker.text.split()]
    assert [scenario['goal']['x'], scenario['goal']['y']] == pytest.approx(position[:2])
    assert scenario['goal']['frame'] == 'map'
    assert float(world.find('physics/real_time_factor').text) == 1.0


def test_physical_lidar_remains_120_degrees():
    robot = ET.parse(ROOT/'urdf/my_robot.urdf').getroot()
    horizontal = robot.find("gazebo[@reference='lidar']/sensor/lidar/scan/horizontal")
    fov = float(horizontal.find('max_angle').text)-float(horizontal.find('min_angle').text)
    assert math.degrees(fov) == pytest.approx(120.0, abs=.001)
