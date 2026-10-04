import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # Path to the installed bringup package
    pkg_share = get_package_share_directory("my_robot_bringup")

    # Files belonging to our robot
    urdf_file = os.path.join(pkg_share, "urdf", "my_robot.urdf")
    world_file = os.path.join(pkg_share, "worlds", "obstacle_world.sdf")
    bridge_file = os.path.join(pkg_share, "config", "bridge.yaml")

    # Read the existing URDF
    with open(urdf_file, "r") as file:
        robot_description = file.read()

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
    )

    # Publish the robot's link transforms
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # Bridge Gazebo topics into ROS 2
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_file}],
    )

    # Insert the robot into the Gazebo world
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_my_robot",
        output="screen",
        arguments=[
            "-world",
            "vfh_world",
            "-file",
            urdf_file,
            "-name",
            "my_robot",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.01",
        ],
    )

    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            bridge,
            # Allow Gazebo to initialize before spawning.
            TimerAction(period=3.0, actions=[spawn_robot]),
        ]
    )
