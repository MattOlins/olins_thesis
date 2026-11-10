"""Scenario-aware VFH/A* navigation with data-driven startup readiness."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def setup(context):
    """Resolve a scenario and pass explicit common settings to both controller modes."""
    def value(name):
        return LaunchConfiguration(name).perform(context)
    bringup = get_package_share_directory('my_robot_bringup')
    navigation = get_package_share_directory('vfh_navigation')
    scenario_file = value('scenario')
    if not os.path.isfile(scenario_file):
        filename = scenario_file if scenario_file.endswith('.yaml') else scenario_file+'.yaml'
        scenario_file = os.path.join(bringup, 'config', 'scenarios', filename)
    with open(scenario_file) as stream:
        scenario = yaml.safe_load(stream)
    mode = value('navigation_mode')
    if mode not in ('vfh', 'vfh_astar'):
        raise RuntimeError('navigation_mode must be vfh or vfh_astar')
    initial = value('initial_pose') or str(scenario['initial_pose'])
    world = value('world') or scenario['world']
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'slam.launch.py')),
        launch_arguments={'world': world, 'initial_pose': initial,
                          'rviz': value('rviz'), 'headless': value('headless'),
                          'slam_mode': value('slam_mode'),
                          'saved_pose_graph': value('saved_pose_graph')}.items())
    actions = [slam]
    if value('navigation_enabled').lower() == 'true':
        params = value('params_file') or os.path.join(navigation, 'config', 'vfh_params.yaml')
        goal = scenario['goal']
        actions.append(Node(
            package='vfh_navigation', executable='vfh_node', name='vfh_node', output='screen',
            parameters=[params, {'use_sim_time': True, 'navigation_mode': mode,
                                 'require_map': True,
                                 'goal_frame': goal['frame'], 'goal_x': float(goal['x']),
                                 'goal_y': float(goal['y'])}]))
    return actions


def generate_launch_description():
    """Expose mode, scenario, world, map, initial pose, parameters, and visualization."""
    arguments = {
        'navigation_mode': 'vfh', 'navigation_enabled': 'true', 'scenario': 'u_world',
        'world': '', 'params_file': '', 'slam_mode': 'mapping', 'saved_pose_graph': '',
        'initial_pose': '', 'rviz': 'true', 'headless': 'false',
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in arguments.items()),
        OpaqueFunction(function=setup),
    ])
