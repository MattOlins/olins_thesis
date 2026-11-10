"""Small sequential trial runner using fresh Gazebo processes and saved SLAM maps."""

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


TOPICS = ['/clock', '/scan', '/odom', '/cmd_vel', '/map', '/tf', '/tf_static',
          '/goal_pose', '/global_path', '/local_goal', '/navigation_state']


def file_hash(path):
    """Hash experiment artifacts without relying on filenames alone."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stop_process(process):
    """Stop only this runner's process group, escalating if it does not exit."""
    if process is None:
        return
    for sig, timeout in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
        # Children can survive a launcher exit: signal remaining group members too.
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            break


class EpisodeObserver(Node):
    """Observe readiness and outcome without changing navigation decisions."""

    def __init__(self):
        super().__init__('comparison_observer', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.sim_time = self.first_clock = self.started = None
        self.state, self.reason = 'WAITING', ''
        self.seen_scan = self.seen_odom = self.seen_map = False
        self.last_position = None
        self.path_length = 0.0
        self.minimum_clearance = math.inf
        self.turn_reversals = self.stuck_events = 0
        self.last_reason = None
        self.trajectory = []
        self.diagnostics = []
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions_owned = [
            self.create_subscription(
                Clock, '/clock', self.clock_callback, qos_profile_sensor_data),
            self.create_subscription(
                LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data),
            self.create_subscription(Odometry, '/odom', self.odom_callback, 10),
            self.create_subscription(OccupancyGrid, '/map', self.map_callback, retained),
            self.create_subscription(
                DiagnosticArray, '/navigation_state', self.state_callback, 10),
        ]

    def clock_callback(self, message):
        """Use simulation time for episode timeouts and elapsed navigation time."""
        self.sim_time = message.clock.sec+message.clock.nanosec/1e9
        if self.first_clock is None:
            self.first_clock = self.sim_time

    def scan_callback(self, message):
        """Remember that the live sensor is producing data."""
        self.seen_scan = bool(message.ranges)

    def map_callback(self, message):
        """Require the same map/localization readiness for both comparison modes."""
        self.seen_map = bool(message.data)

    def odom_callback(self, message):
        """Accumulate the physical odometry trajectory, including detours."""
        self.seen_odom = True
        p = message.pose.pose.position
        position = p.x, p.y
        stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
        if self.last_position is not None and self.started is not None:
            self.path_length += math.dist(self.last_position, position)
        self.last_position = position
        self.trajectory.append({'sim_time': stamp, 'x': p.x, 'y': p.y})

    def state_callback(self, message):
        """Collect steering, progress, failures, and independently labeled clearance."""
        for status in message.status:
            if status.name != 'navigation':
                continue
            values = {entry.key: entry.value for entry in status.values}
            self.state, self.reason = values['state'], values['reason']
            stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
            if self.started is None and self.state in ('PLANNING', 'TRACKING', 'BLOCKED'):
                self.started = stamp
            clearance = float(values.get('observed_clearance', 'nan'))
            if math.isfinite(clearance):
                self.minimum_clearance = min(self.minimum_clearance, clearance)
            self.turn_reversals = max(self.turn_reversals, int(values.get('turn_reversals', 0)))
            if self.reason == 'stuck' and self.last_reason != 'stuck':
                self.stuck_events += 1
            if self.reason == 'stuck_without_global_planner' and self.last_reason != self.reason:
                self.stuck_events += 1
            self.last_reason = self.reason
            self.diagnostics.append({'sim_time': stamp, **values})

    def ready(self):
        """Require live sensor/map data and the complete navigation transform chain."""
        if not (self.seen_scan and self.seen_odom and self.seen_map):
            return False
        try:
            self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
            self.tf_buffer.lookup_transform('base_footprint', 'lidar', Time())
            return True
        except TransformException:
            return False


def write_csv(path, rows):
    """Write unioned diagnostic columns even when no sample was collected."""
    if not rows:
        path.write_text('')
        return
    columns = list(dict.fromkeys(key for row in rows for key in row))
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def best_effort_stop(observer):
    """Try to stop motion even if SIGINT is concurrently shutting down ROS."""
    try:
        for _ in range(3):
            if not rclpy.ok():
                break
            observer.stop_pub.publish(Twist())
            rclpy.spin_once(observer, timeout_sec=.05)
    except (RuntimeError, ExternalShutdownException, KeyboardInterrupt):
        pass  # Process cleanup must run even after a publisher context becomes invalid.


def run_episode(scenario_file, mode, trial, args):
    """Launch, record, observe, and terminate one isolated comparison episode."""
    scenario = yaml.safe_load(scenario_file.read_text())
    directory = args.output/scenario['name']/mode/f'trial_{trial:03d}'
    directory.mkdir(parents=True, exist_ok=False)
    share = Path(get_package_share_directory('my_robot_bringup'))
    world = Path(scenario['world'])
    if not world.is_absolute():
        world = share/'worlds'/world
    saved = Path(scenario['saved_pose_graph']).expanduser()
    if not saved.is_absolute():
        saved = args.maps_dir/saved
    graph_files = [Path(str(saved)+suffix) for suffix in ('.posegraph', '.data')]
    if not all(path.is_file() for path in graph_files):
        raise FileNotFoundError(f'Premap scenario first: expected {graph_files}')
    shutil.copy2(scenario_file, directory/'scenario.yaml')
    shutil.copy2(args.params_file, directory/'vfh_params.yaml')
    metadata = {
        'scenario': scenario['name'], 'mode': mode, 'trial': trial,
        'scenario_hash': file_hash(scenario_file), 'config_hash': file_hash(args.params_file),
        'world_hash': file_hash(world), 'robot_hash': file_hash(share/'urdf/my_robot.urdf'),
        'slam_config_hash': file_hash(share/'config/mapper_params_online_async.yaml'),
        'map_hashes': {str(path): file_hash(path) for path in graph_files},
        'collision_annotation': None,
        'collision_note': 'Not measured automatically; scan clearance is not collision truth.',
    }
    (directory/'metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
    observer = EpisodeObserver()
    launch = recorder = None
    started_wall = time.monotonic()
    ready = False
    outcome, reason = 'FAILED', 'startup_error'
    with (directory/'launch.log').open('w') as launch_log, (
            directory/'recorder.log').open('w') as recorder_log:
        try:
            recorder = subprocess.Popen(
                ['ros2', 'bag', 'record', '-o', str(directory/'bag'), *TOPICS],
                stdout=recorder_log, stderr=subprocess.STDOUT, start_new_session=True)
            launch = subprocess.Popen([
                'ros2', 'launch', 'my_robot_bringup', 'navigation.launch.py',
                f'scenario:={directory/"scenario.yaml"}', f'navigation_mode:={mode}',
                f'params_file:={directory/"vfh_params.yaml"}', 'slam_mode:=localization',
                f'saved_pose_graph:={saved}', 'rviz:=false',
                f'headless:={str(args.headless).lower()}',
            ], stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
            while rclpy.ok():
                rclpy.spin_once(observer, timeout_sec=.1)
                ready = ready or observer.ready()
                wall_elapsed = time.monotonic()-started_wall
                if launch.poll() is not None:
                    reason = 'launch_exited'
                    break
                if recorder.poll() is not None:
                    reason = 'recorder_exited'
                    break
                if observer.state in ('SUCCEEDED', 'FAILED'):
                    outcome, reason = observer.state, observer.reason
                    if outcome == 'SUCCEEDED' and not ready:
                        outcome, reason = 'FAILED', 'success_without_comparison_readiness'
                    break
                if not ready and wall_elapsed > args.readiness_timeout:
                    reason = 'readiness_timeout'
                    break
                initial = (observer.started if observer.started is not None else
                           observer.first_clock)
                if initial is not None and observer.sim_time is not None:
                    if observer.sim_time-initial >= scenario['timeout']:
                        outcome, reason = 'TIMEOUT', observer.reason or 'simulation_timeout'
                        break
                if wall_elapsed > scenario['timeout']*3+args.readiness_timeout:
                    outcome, reason = 'TIMEOUT', 'wall_watchdog_simulation_stalled'
                    break
            else:
                outcome, reason = 'INTERRUPTED', 'ros_context_shutdown'
        except KeyboardInterrupt:
            outcome, reason = 'INTERRUPTED', 'operator_interrupt'
            raise
        finally:
            end_sim_time = observer.sim_time
            best_effort_stop(observer)
            stop_process(launch)
            stop_process(recorder)
            elapsed = (end_sim_time-observer.started if observer.started is not None and
                       end_sim_time is not None else None)
            summary = {
                'scenario': scenario['name'], 'mode': mode, 'trial': trial,
                'outcome': outcome, 'reason': reason, 'ready': ready,
                'elapsed_sim_seconds': elapsed, 'trajectory_length_m': observer.path_length,
                'minimum_observed_clearance_m': (
                    observer.minimum_clearance if math.isfinite(observer.minimum_clearance)
                    else None),
                'turn_reversals': observer.turn_reversals, 'stuck_events': observer.stuck_events,
                'collision_annotation': None,
            }
            write_csv(directory/'trajectory.csv', observer.trajectory)
            write_csv(directory/'navigation.csv', observer.diagnostics)
            (directory/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
            observer.destroy_node()
    return summary


def main(argv=None):
    """Run repeated paired VFH/A* trials from premapped scenario manifests."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenarios', nargs='+', default=[
        'open_field', 'single_obstacle', 'corridor', 'u_world'])
    parser.add_argument('--modes', nargs='+', choices=['vfh', 'vfh_astar'],
                        default=['vfh', 'vfh_astar'])
    parser.add_argument('--trials', type=int, default=3)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--maps-dir', type=Path, default=Path.cwd(),
                        help='Resolve relative saved_pose_graph paths against this directory')
    parser.add_argument('--params-file', type=Path, default=Path(
        get_package_share_directory('vfh_navigation'))/'config/vfh_params.yaml')
    parser.add_argument('--readiness-timeout', type=float, default=60.)
    parser.add_argument('--headless', action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args(argv)
    if args.trials < 1 or args.readiness_timeout <= 0:
        parser.error('trials and readiness-timeout must be positive')
    args.output, args.maps_dir = args.output.resolve(), args.maps_dir.resolve()
    args.params_file = args.params_file.resolve()
    share = Path(get_package_share_directory('my_robot_bringup'))
    scenarios = [Path(name).resolve() if Path(name).is_file() else
                 share/'config/scenarios'/(name if name.endswith('.yaml') else name+'.yaml')
                 for name in args.scenarios]
    # Validate every required artifact before starting any simulation process.
    for path in scenarios:
        scenario = yaml.safe_load(path.read_text())
        saved = Path(scenario['saved_pose_graph']).expanduser()
        if not saved.is_absolute():
            saved = args.maps_dir/saved
        if not all(Path(str(saved)+suffix).is_file() for suffix in ('.posegraph', '.data')):
            parser.error(f'Premap {scenario["name"]} first: missing serialized map prefix {saved}')
    args.output.mkdir(parents=True, exist_ok=True)
    summaries = []
    rclpy.init()
    try:
        for scenario in scenarios:
            for trial in range(1, args.trials+1):
                # Alternate mode order to reduce ordering effects.
                modes = args.modes if trial % 2 else list(reversed(args.modes))
                for mode in modes:
                    summary = run_episode(scenario, mode, trial, args)
                    summaries.append(summary)
                    write_csv(args.output/'summary.csv', summaries)
                    print(json.dumps(summary), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
