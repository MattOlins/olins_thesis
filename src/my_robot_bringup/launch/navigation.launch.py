import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    bringup_share = get_package_share_directory('my_robot_bringup')
    vfh_share = get_package_share_directory('vfh_navigation')

    # Start Gazebo, robot, SLAM and RViz
    slam_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'slam.launch.py')))

    # VFH parameter file
    vfh_params = os.path.join(vfh_share,'config','vfh_params.yaml')

    # Autonomous navigation node
    vfh_node = Node( package='vfh_navigation', executable='vfh_node', name='vfh_node', output='screen',
        parameters=[vfh_params, {'use_sim_time': True}] )

    return LaunchDescription([slam_launch,
        # The controller waits for valid LiDAR
        # and odometry before commanding motion.
        TimerAction(period=6.0, actions=[vfh_node])
    ])