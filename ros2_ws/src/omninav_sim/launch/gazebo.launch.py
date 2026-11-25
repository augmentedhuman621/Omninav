"""OmniNav Gazebo Classic simulation launch file.

Usage (Humble environment):
  ros2 launch omninav_sim gazebo.launch.py                 # with the Gazebo GUI
  ros2 launch omninav_sim gazebo.launch.py gui:=false      # headless (remote / low-spec)
  ros2 launch omninav_sim gazebo.launch.py world:=/path/to/xxx.world

After launch:
  /cmd_vel  -> the planar_move plugin moves the robot body directly (omnidirectional)
  /odom     -> published by the plugin (the odom frame -> base_link TF is published by the plugin)
  /scan     -> roof-mounted 2D lidar (360 rays, 10 Hz, 0.12-8 m)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('omninav_sim')
    gazebo_ros_share = FindPackageShare('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    spawn_z = LaunchConfiguration('spawn_z')

    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'omninav_sim.urdf.xacro'])
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, 'launch', 'gzserver.launch.py'])),
        launch_arguments={'world': world, 'verbose': 'true'}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, 'launch', 'gzclient.launch.py'])),
        condition=IfCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'omninav_sim',
            '-z', spawn_z,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([pkg_share, 'worlds', 'omninav_warehouse.world']),
            description='Path to the Gazebo world file'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Whether to start the Gazebo GUI'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time'),
        DeclareLaunchArgument(
            'spawn_z', default_value='0.075',
            description='Initial height of base_link (wheel radius 0.04 + chassis clearance 0.035)'),
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
    ])
