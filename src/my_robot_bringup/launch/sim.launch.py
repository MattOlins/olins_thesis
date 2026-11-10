"""Launch the unchanged 120-degree robot in a selectable scenario world."""

import os
from shlex import quote

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    """Resolve world and spawn arguments before creating launch actions."""
    share = get_package_share_directory('my_robot_bringup')
    world = LaunchConfiguration('world').perform(context)
    if not os.path.isabs(world):
        world = os.path.join(share, 'worlds', world)
    if not os.path.isfile(world):
        raise RuntimeError(f'World does not exist: {world}')
    urdf = os.path.join(share, 'urdf', 'my_robot.urdf')
    with open(urdf) as stream:
        description = stream.read()
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    gazebo_args = '-r ' + ('-s --headless-rendering ' if headless else '') + quote(world)
    gazebo = IncludeLaunchDescription( PythonLaunchDescriptionSource(os.path.join( 
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gazebo_args}.items())
    state = Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': description, 'use_sim_time': True}])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', parameters=[{
        'config_file': os.path.join(share, 'config', 'bridge.yaml')}])
    spawn = Node(package='ros_gz_sim', executable='create', output='screen', arguments=[
        '-world', 'vfh_world', '-file', urdf, '-name', 'my_robot',
        '-x', LaunchConfiguration('spawn_x').perform(context),
        '-y', LaunchConfiguration('spawn_y').perform(context), '-z', '0.12',
        '-Y', LaunchConfiguration('spawn_yaw').perform(context)])
    # ros_gz_sim create waits for the world service; controller readiness is data-driven.
    return [gazebo, state, bridge, spawn]


def generate_launch_description():
    """Expose world, rendering, and initial-pose arguments."""
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='vfh_test_world.sdf'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        OpaqueFunction(function=setup),
    ])
