"""OmniNav Gazebo Classic simulation launch file.

Usage (Humble environment):
  ros2 launch omninav_sim gazebo.launch.py                 # with the Gazebo GUI
  ros2 launch omninav_sim gazebo.launch.py gui:=false      # headless (remote / low-spec)
  ros2 launch omninav_sim gazebo.launch.py world:=/path/to/xxx.world

RViz2 visualisation (works headless; useful when the VM has no 3D acceleration):
  ros2 launch omninav_sim gazebo.launch.py gui:=false gazebo_odom:=false rviz:=true
  The bundled config shows the URDF model, TF tree, /scan and the /odom trail; start
  stm32_bridge afterwards so that odom -> base_link exists (Fixed Frame = odom).

After launch:
  /cmd_vel  -> the planar_move plugin moves the robot body directly (omnidirectional)
  /odom     -> published by the plugin (the odom frame -> base_link TF is published by the plugin)
  /scan     -> roof-mounted 2D lidar (360 rays, 10 Hz, 0.12-8 m)

Odom ownership switch (bridge mode):
  ros2 launch omninav_sim gazebo.launch.py gui:=false gazebo_odom:=false
  With gazebo_odom:=false the plug-in only moves the body and no longer publishes /odom or the
  odom->base_link TF, so that stm32_bridge (with fake_stm32.py on a pty) is the single owner.
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
    gazebo_odom = LaunchConfiguration('gazebo_odom')

    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'omninav_sim.urdf.xacro'])
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' publish_odom:=', gazebo_odom,
                 ' publish_odom_tf:=', gazebo_odom]), value_type=str)

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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'omninav_robot.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
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
        DeclareLaunchArgument(
            'gazebo_odom', default_value='true',
            description='Let planar_move publish /odom + odom->base_link TF; set false when '
                        'stm32_bridge owns them (bridge simulation mode)'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Start rviz2 with the bundled omninav_robot.rviz config (model, TF, '
                        '/scan, /odom trail)'),
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        rviz,
    ])
