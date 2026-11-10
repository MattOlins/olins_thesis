"""Behavioral tests for partial-FOV local control and global planning."""

from dataclasses import replace
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from vfh_navigation.astar_planner import plan
from vfh_navigation.controller import VFHController
from vfh_navigation.geometry import inverse_transform, transform_point
from vfh_navigation.grid_map import GridMap
from vfh_navigation.histogram import HistogramBuilder
from vfh_navigation.models import NavigationConfig, Pose2D, ScanData, VelocityCommand
from vfh_navigation.navigation import Navigator
from vfh_navigation.path_follower import PathFollower
from vfh_navigation.progress_monitor import ProgressMonitor


def scan(ranges=None, sensor=Pose2D(), low=-math.pi/3, high=math.pi/3):
    """Create a hardware-FOV scan with 720 expected rays."""
    return ScanData(tuple(ranges if ranges is not None else [math.inf]*720),
                    low, high, (high-low)/719, .1, 10., sensor)


def grid(values=None, radius=0.0, origin=Pose2D(), resolution=.1):
    """Build a copied test map."""
    values = np.zeros((60, 100)) if values is None else np.asarray(values)
    return GridMap.from_data(values.ravel(), values.shape[1], values.shape[0],
                             resolution, origin, radius)


class ImmediateWorker:
    """Complete searches deterministically on the next poll."""

    def __init__(self):
        self.future = None
        self.completed = None

    def submit(self, snapshot, start, goal, token):
        if self.future is not None:
            return False
        self.future = True
        self.completed = token, plan(snapshot, start, goal)
        return True

    def poll(self):
        result = self.completed
        self.completed = self.future = None
        return result

    def close(self):
        pass


def test_pre_refactor_histogram_and_steering_regression():
    reference = json.loads((Path(__file__).parent/'fixtures/pre_refactor.json').read_text())
    config = replace(NavigationConfig(), robot_radius=.25, goal_weight=2.0)
    ranges = [math.inf]*720
    ranges[320:401] = [.8]*81
    controller = VFHController(config)
    histogram = controller.histogram.build(scan(ranges))
    np.testing.assert_allclose(histogram.density, reference['density'])
    np.testing.assert_array_equal(histogram.observed, reference['observed'])
    for step in reference['steps']:
        selected = controller.selector.choose(histogram, step['goal'], step['yaw'], .1)
        assert selected == pytest.approx(step['selected'])


def test_nominal_sector_geometry_and_no_blind_candidates():
    histogram = HistogramBuilder(NavigationConfig()).build(scan())
    assert len(histogram.angles) == 24
    assert histogram.observed.all()
    assert histogram.forward_observed
    assert np.degrees(histogram.angles[[0, -1]]) == pytest.approx([-57.5, 57.5])
    controller = VFHController()
    selected = controller.selector.choose(histogram, math.pi, 0., .1)
    assert abs(selected) <= math.pi/3
    assert controller.step(scan(), Pose2D(), 0, (-10, 0), .1).command.linear == 0


@pytest.mark.parametrize('value', [math.nan, -math.inf, -.1, .05, 11.])
def test_invalid_forward_ranges_stop(value):
    ranges = [math.inf]*720
    ranges[360] = value
    result = VFHController().step(scan(ranges), Pose2D(), 0., (10., 0.), .1)
    assert result.command == VelocityCommand()
    assert result.reason == 'unknown_forward_clearance'


@pytest.mark.parametrize('count', [0, 1, 100, 360])
def test_truncated_scan_cannot_authorize_forward_motion(count):
    result = VFHController().step(scan([math.inf]*count), Pose2D(), 0, (10, 0), .1)
    assert result.command.linear == 0
    assert not result.histogram.forward_observed


def test_partial_sector_validity_is_conservative():
    ranges = [math.inf]*720
    ranges[:25] = [math.nan]*25
    histogram = HistogramBuilder(NavigationConfig()).build(scan(ranges))
    assert not histogram.observed[0]
    assert histogram.observed[-1]


def test_empty_forward_sector_selection_is_not_vacuously_clear():
    histogram = HistogramBuilder(NavigationConfig()).build(
        scan(low=math.pi/2, high=math.pi))
    assert not histogram.forward_observed


def test_sensor_offset_changes_base_clearance_and_inflation():
    ranges = [math.inf]*720
    ranges[360] = .6
    builder = HistogramBuilder(NavigationConfig())
    centered = builder.build(scan(ranges))
    offset = builder.build(scan(ranges, Pose2D(.111125, 0, 0)))
    assert offset.front_distance-centered.front_distance == pytest.approx(.111125, abs=1e-5)
    assert offset.clearance > centered.clearance
    assert offset.density.max() < centered.density.max()


def test_safety_radius_blocks_intruding_obstacle():
    ranges = [math.inf]*720
    ranges[360] = .4
    result = VFHController().step(scan(ranges), Pose2D(), 0, (10, 0), .1)
    assert result.command == VelocityCommand()


def test_measured_dt_damping_and_reset():
    controller = VFHController()
    histogram = controller.histogram.build(scan())
    command, _ = controller.velocity.command(.8, .8, 0, histogram, .025)
    assert command.angular == pytest.approx(.02)
    command, _ = controller.velocity.command(.8, .8, 0, histogram, .2)
    assert command.angular == pytest.approx(.18)
    controller.reset()
    assert controller.selector.previous_world_heading is None
    assert controller.velocity.last_angular_command == 0
    damped, _ = controller.velocity.command(.2, .2, 1, histogram, 1)
    assert damped.angular < 0


def test_heading_memory_stays_in_odom():
    controller = VFHController()
    histogram = controller.histogram.build(scan())
    first = controller.selector.choose(histogram, .4, .2, .1)
    assert controller.selector.previous_world_heading == pytest.approx(.2+first)
    second = controller.selector.choose(histogram, .2, .4, .1)
    assert abs((.4+second)-(.2+first)) <= 2*histogram.sector_width


@pytest.mark.parametrize('kwargs', [
    {'sector_width_deg': 0}, {'robot_radius': math.nan},
    {'stop_distance': .1}, {'observed_fraction': 1.1}])
def test_invalid_configuration_rejected(kwargs):
    with pytest.raises(ValueError):
        NavigationConfig(**kwargs)


def test_rotated_grid_origin_and_immutable_copy():
    values = np.zeros((20, 20))
    origin = Pose2D(-2, 3, math.pi/2)
    snapshot = grid(values, origin=origin)
    for cell in ((0, 0), (10, 5), (19, 19)):
        assert snapshot.world_to_grid(snapshot.grid_to_world(cell)) == cell
    values[:] = 100
    assert snapshot.is_free((10, 10))
    with pytest.raises(ValueError):
        snapshot.blocked[10, 10] = True
    point = (4, -2)
    actual = transform_point(transform_point(point, origin), inverse_transform(origin))
    assert actual == pytest.approx(point)


def test_unknown_cells_and_boundary_inflation():
    values = np.zeros((40, 40))
    values[20, 20] = -1
    snapshot = grid(values, radius=.43)
    assert not snapshot.is_free((20, 20))
    assert not snapshot.is_free((24, 20))
    assert not snapshot.is_free((0, 10))
    assert not snapshot.is_free((-1, 10))
    assert snapshot.is_free((10, 10))


def test_diagonal_corner_cutting_and_line_shortcuts_are_rejected():
    values = np.zeros((2, 2))
    values[0, 1] = values[1, 0] = 100
    snapshot = grid(values, resolution=1)
    assert plan(snapshot, (.5, .5), (1.5, 1.5)).status == 'unreachable'
    assert not snapshot.segment_clear((.5, .5), (1.5, 1.5))


def test_astar_routes_around_u_and_preserves_goal():
    values = np.zeros((80, 120))
    values[20:60, 60] = 100
    values[20, 20:61] = values[59, 20:61] = 100
    snapshot = grid(values, radius=.43)
    result = plan(snapshot, (1., 4.), (10., 4.))
    assert result.status == 'success'
    assert result.path[0] == (1., 4.) and result.path[-1] == (10., 4.)
    assert snapshot.path_clear(result.path)
    assert any(y < 1.6 or y > 6.4 for x, y in result.path)
    assert result == plan(snapshot, (1., 4.), (10., 4.))


@pytest.mark.parametrize('start,goal,status', [
    ((-1., 2.), (2., 2.), 'start_outside_map'),
    ((2., 2.), (11., 2.), 'goal_outside_map'),
    ((2., 2.), (2.05, 3.05), 'goal_unknown'),
])
def test_explicit_endpoint_failures(start, goal, status):
    values = np.zeros((60, 100))
    values[30, 20] = -1
    assert plan(grid(values), start, goal).status == status


def test_invalid_map_does_not_start_search():
    assert plan(None, (0, 0), (1, 1)).status == 'invalid_map'
    with pytest.raises(ValueError):
        GridMap.from_data([0], 2, 2, .1)


def test_footprint_prevents_narrow_passage():
    values = np.zeros((80, 120))
    values[:, 50:55] = 100
    values[37:43, 50:55] = 0
    assert plan(grid(values), (2, 4), (8, 4)).status == 'success'
    assert plan(grid(values, radius=.43), (2, 4), (8, 4)).status == 'unreachable'


def test_follower_lookahead_does_not_cut_blocked_corner():
    values = np.zeros((50, 50))
    values[10:30, 20:30] = 100
    snapshot = grid(values)
    follower = PathFollower(2.0)
    follower.set_path(((1.5, 1.5), (1.5, 3.5), (3.5, 3.5)))
    result = follower.update((1.5, 2.0), snapshot)
    assert snapshot.segment_clear((1.5, 2.0), result.target)
    previous = follower.progress
    follower.update((1.5, 1.8), snapshot)
    assert follower.progress >= previous
    follower.set_path(((1., 1.), (2., 1.)))
    assert follower.progress == 0


def test_progress_uses_route_distance_for_legitimate_detour():
    monitor = ProgressMonitor(NavigationConfig())
    for i in range(61):
        result = monitor.update(i*.1, 10-i*.02, .1 if i < 20 else -.1)
    assert not result.stuck
    assert result.turn_reversals == 1
    monitor.reset()
    for i in range(51):
        result = monitor.update(i*.1, 10, 0)
    assert result.stuck
    assert not monitor.update(0, 10, 0).stuck


def test_vfh_goal_stuck_and_reset():
    navigator = Navigator(NavigationConfig(), worker=ImmediateWorker())
    assert navigator.step(scan(), Pose2D(), 0, (.1, 0), .1, 0).state == 'SUCCEEDED'
    navigator.reset()
    for i in range(51):
        result = navigator.step(scan(), Pose2D(), 0, (10, 0), .1, i*.1)
    assert result.state == 'FAILED'
    assert result.local.command == VelocityCommand()
    navigator.reset()
    assert navigator.step(scan(), Pose2D(), 0, (10, 0), .1, 6).state == 'TRACKING'


def test_stale_planner_result_is_discarded():
    worker = ImmediateWorker()
    navigator = Navigator(NavigationConfig(), 'vfh_astar', worker)
    navigator.set_map(grid())
    args = (scan(), Pose2D(1, 1), 0, (8, 1), .1)
    assert navigator.step(*args, 0, Pose2D(1, 1), (8, 1), Pose2D()).state == 'PLANNING'
    old_token = worker.completed[0]
    navigator.reset()
    assert old_token[0] != navigator.goal_revision
    result = navigator.step(*args, .1, Pose2D(1, 1), (8, 1), Pose2D())
    assert result.state == 'PLANNING'
    assert not navigator.follower.path


def test_unrelated_map_update_preserves_route_and_invalidating_update_stops():
    navigator = Navigator(NavigationConfig(), 'vfh_astar', ImmediateWorker())
    values = np.zeros((60, 100))
    navigator.set_map(grid(values))
    args = (scan(), Pose2D(1, 1), 0, (8, 1), .1)
    navigator.step(*args, 0, Pose2D(1, 1), (8, 1), Pose2D())
    navigator.step(*args, .1, Pose2D(1, 1), (8, 1), Pose2D())
    route = navigator.follower.path
    values[50, 50] = 100
    navigator.set_map(grid(values))
    navigator.step(*args, .2, Pose2D(1, 1), (8, 1), Pose2D())
    assert navigator.follower.path == route
    values[10, 40] = 100
    navigator.set_map(grid(values))
    result = navigator.step(*args, .3, Pose2D(1, 1), (8, 1), Pose2D())
    assert not navigator.follower.path
    assert result.local.command == VelocityCommand()


def test_map_revision_supersedes_search_result():
    navigator = Navigator(NavigationConfig(), 'vfh_astar', ImmediateWorker())
    navigator.set_map(grid())
    args = (scan(), Pose2D(1, 1), 0, (8, 1), .1)
    navigator.step(*args, 0, Pose2D(1, 1), (8, 1), Pose2D())
    values = np.zeros((60, 100))
    values[40, 40] = 100
    navigator.set_map(grid(values))
    navigator.step(*args, .1, Pose2D(1, 1), (8, 1), Pose2D())
    assert not navigator.follower.path


def test_repeated_unchanged_stuck_replans_latch_failure():
    navigator = Navigator(NavigationConfig(), 'vfh_astar', ImmediateWorker())
    navigator.set_map(grid())
    args = (scan(), Pose2D(1, 1), 0, (8, 1), .1)
    for i in range(210):
        result = navigator.step(*args, i*.1, Pose2D(1, 1), (8, 1), Pose2D())
        if result.state == 'FAILED':
            break
    assert result.state == 'FAILED'
    assert navigator.stuck_replans == 3
    assert result.local.command == VelocityCommand()


@pytest.mark.parametrize('goal', [(4., -.31), (-4., 0.), (0., 4.)])
def test_closed_loop_turns_toward_off_axis_and_rear_goals(goal):
    navigator = Navigator(NavigationConfig(), worker=ImmediateWorker())
    pose, yaw_rate = Pose2D(), 0.0
    for i in range(650):
        result = navigator.step(scan(), pose, yaw_rate, goal, .1, i*.1)
        if result.state in ('SUCCEEDED', 'FAILED'):
            break
        command = result.local.command
        pose = Pose2D(pose.x+.1*command.linear*math.cos(pose.yaw),
                      pose.y+.1*command.linear*math.sin(pose.yaw),
                      pose.yaw+.1*command.angular)
        yaw_rate = command.angular
    assert result.state == 'SUCCEEDED'
    assert math.dist((pose.x, pose.y), goal) < .20


def test_actual_u_world_has_a_route_with_full_safety_radius():
    world_file = (Path(__file__).resolve().parents[2]/'my_robot_bringup/worlds/'
                  'vfh_test_world.sdf')
    world = ET.parse(world_file).getroot().find('world')
    yy, xx = np.mgrid[-6:6:.05, -3:13:.05]
    xx, yy = xx+.025, yy+.025
    values = np.zeros(xx.shape)
    for model in world.findall('model'):
        box = model.find('link/collision/geometry/box/size')
        if box is None:
            continue
        pose = [float(v) for v in model.find('pose').text.split()]
        size = [float(v) for v in box.text.split()]
        dx, dy = xx-pose[0], yy-pose[1]
        c, s = math.cos(pose[5]), math.sin(pose[5])
        # Conservatively rasterize any cell intersecting a wall box.
        hit = ((abs(c*dx+s*dy) <= size[0]/2+.025) &
               (abs(-s*dx+c*dy) <= size[1]/2+.025))
        values[hit] = 100
    snapshot = grid(values, .43, Pose2D(-3, -6), .05)
    result = plan(snapshot, (0., 0.), (10., 0.))
    assert result.status == 'success'
    assert snapshot.path_clear(result.path)
    assert any(y > 2.2 for x, y in result.path)


def test_continuous_footprint_check_rejects_unsafe_subcell_endpoint():
    values = np.zeros((40, 40))
    values[20, 20] = 100
    snapshot = grid(values, radius=.41)
    # A free cell center is not permission to approach its obstacle-facing edge.
    assert snapshot.is_free(snapshot.world_to_grid((1.501, 2.05)))
    assert not snapshot.segment_clear((1.599, 2.05), (1.599, 2.05))
    assert plan(snapshot, (1., 1.), (1.599, 2.05)).status == 'goal_insufficient_clearance'


def test_nonfinite_navigation_input_stops_before_planning():
    navigator = Navigator(NavigationConfig(), 'vfh_astar', ImmediateWorker())
    result = navigator.step(scan(), Pose2D(math.nan, 0), 0, (8, 1), .1, 0)
    assert result.state == 'WAITING'
    assert result.local.command == VelocityCommand()


def test_turn_grace_is_bounded_and_oscillation_is_not_progress():
    monitor = ProgressMonitor(NavigationConfig())
    for i in range(120):
        result = monitor.update(i*.1, 10., .2, yaw=i*.02)
    assert result.stuck
    monitor.reset()
    for i in range(51):
        result = monitor.update(i*.1, 10., .2, yaw=.4*math.sin(i*.1*2*math.pi))
    assert result.stuck


def test_obstacle_memory_only_adds_constraints_and_resets_explicitly():
    controller = VFHController()
    ranges = [math.inf]*720
    ranges[360] = .7
    controller.step(scan(ranges), Pose2D(), 0, (4, 0), .1)
    assert controller.obstacle_memory
    result = controller.step(scan(), Pose2D(yaw=1.5), 0, (4, 0), .1)
    assert result.histogram.density.any()
    assert len(result.histogram.observed) == 24
    unknown = controller.step(scan([math.nan]*720), Pose2D(), 0, (4, 0), .1)
    assert not unknown.histogram.observed.any()
    assert unknown.command == VelocityCommand()
    controller.reset(clear_obstacles=False)
    assert controller.obstacle_memory
    controller.reset()
    assert not controller.obstacle_memory


def test_single_obstacle_closed_loop_preserves_clearance_and_returns_toward_goal():
    navigator = Navigator(NavigationConfig(), worker=ImmediateWorker())
    pose, rate = Pose2D(), 0.0
    bearings = np.linspace(-math.pi/3, math.pi/3, 720)
    minimum_distance = math.inf
    for i in range(500):
        directions = np.column_stack((np.cos(bearings+pose.yaw),
                                      np.sin(bearings+pose.yaw)))
        origin = np.array((pose.x+.111125*math.cos(pose.yaw),
                           pose.y+.111125*math.sin(pose.yaw)))
        with np.errstate(divide='ignore', invalid='ignore'):
            first = (np.array((1.75, -.25))-origin)/directions
            second = (np.array((2.25, .25))-origin)/directions
        near = np.minimum(first, second).max(axis=1)
        far = np.maximum(first, second).min(axis=1)
        ranges = np.where((near >= .1) & (near <= far) & (near <= 10), near, math.inf)
        observation = scan(ranges, Pose2D(.111125, 0, 0))
        result = navigator.step(observation, pose, rate, (4., 0.), .1, i*.1)
        separation = np.maximum(np.maximum(
            np.array((1.75, -.25))-(pose.x, pose.y),
            np.array((pose.x, pose.y))-(2.25, .25)), 0)
        minimum_distance = min(minimum_distance, float(np.linalg.norm(separation)))
        if result.state in ('FAILED', 'SUCCEEDED'):
            break
        command = result.local.command
        pose = Pose2D(pose.x+.1*command.linear*math.cos(pose.yaw),
                      pose.y+.1*command.linear*math.sin(pose.yaw),
                      pose.yaw+.1*command.angular)
        rate = command.angular
    assert result.state == 'SUCCEEDED'
    assert minimum_distance >= navigator.config.safe_radius
