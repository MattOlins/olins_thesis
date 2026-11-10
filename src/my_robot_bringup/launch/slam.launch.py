"""SLAM premapping or localization with the same simulator and sensor setup."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import yaml


def setup(context):
    """Configure SLAM lifecycle and optionally load a serialized pose graph."""
    def value(name):
        return LaunchConfiguration(name).perform(context)
    share = get_package_share_directory('my_robot_bringup')
    mode = value('slam_mode')
    if mode not in ('mapping', 'localization'):
        raise RuntimeError('slam_mode must be mapping or localization')
    initial = yaml.safe_load(value('initial_pose'))
    if not isinstance(initial, list) or len(initial) != 3 or not all(
            isinstance(x, (int, float)) and math.isfinite(x) for x in initial):
        raise RuntimeError('initial_pose must be a finite [x, y, yaw] list')
    saved = os.path.abspath(os.path.expanduser(value('saved_pose_graph')))
    if mode == 'localization' and not all(
            os.path.isfile(saved+suffix) for suffix in ('.posegraph', '.data')):
        raise RuntimeError(
            'localization requires saved_pose_graph prefix with .posegraph and .data')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'sim.launch.py')),
        launch_arguments={'world': value('world'), 'headless': value('headless'),
                          'spawn_x': str(initial[0]), 'spawn_y': str(initial[1]),
                          'spawn_yaw': str(initial[2])}.items())
    settings = {'use_sim_time': True, 'mode': mode}
    if mode == 'localization':
        settings.update(map_file_name=saved, map_start_pose=[float(x) for x in initial])
    slam_environment = dict(context.environment)
    # VS Code's Snap variables otherwise make a non-Snap SLAM process prepend
    # /var/snap/code/common even to absolute serialization/localization paths.
    if slam_environment.get('SNAP_NAME') == 'code':
        slam_environment.pop('SNAP_COMMON', None)
    slam = LifecycleNode(
        package='slam_toolbox', executable=(
            'localization_slam_toolbox_node' if mode == 'localization' else
            'async_slam_toolbox_node'), name='slam_toolbox', namespace='', output='screen',
        parameters=[os.path.join(share, 'config', 'mapper_params_online_async.yaml'), settings],
        env=slam_environment)
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_ACTIVATE))]))
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam),
        transition_id=Transition.TRANSITION_CONFIGURE))
    actions = [simulation, slam, activate, configure]
    if value('rviz').lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                            parameters=[{'use_sim_time': True}], arguments=[
                                '-d', os.path.join(share, 'config', 'navigation.rviz')]))
    return actions


def generate_launch_description():
    """Expose mapping/localization and simulator configuration."""
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='vfh_test_world.sdf'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('slam_mode', default_value='mapping'),
        DeclareLaunchArgument('saved_pose_graph', default_value=''),
        DeclareLaunchArgument('initial_pose', default_value='[0.0, 0.0, 0.0]'),
        OpaqueFunction(function=setup),
    ])
