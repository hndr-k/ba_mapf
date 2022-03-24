#!/usr/bin/env python

from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Launch the example.launch.py launch file."""
    package_dir = get_package_share_directory('webots_ros2_robotino3')
    apriltag_dir = get_package_share_directory('apriltag_ros')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([package_dir, '/robot_launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([apriltag_dir, '/launch/tag_16h5_all.launch.py']),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '-1.58', '1.58', 'tag16h5:0', 'tag16h5:0_rotated']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '-1.58', '1.58', 'tag16h5:1', 'tag16h5:1_rotated']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '-1.58', '1.58', 'tag16h5:2', 'tag16h5:2_rotated']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '-1.58', '1.58', 'tag16h5:3', 'tag16h5:3_rotated']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '-1.58', '1.58', 'tag16h5:4', 'tag16h5:4_rotated']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '-1.58', '1.58', 'tag16h5:5', 'tag16h5:5_rotated']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['-5.83601', '0.0469383', '0.0469397', '0.0', '0.0', '0.0','1.0','world', 'map']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['-3.69', '-0.37', '1.08', '0.0', '0.0', '-1.0', '0.0', 'world', 'belt_start']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['-5.44', '0.03', '1.02', '0.0', '0.0', '0.0', '1.0', 'world', 'test_pos']
        ),
        Node(
            package='webots_ros2_robotino3',
            executable='frame_listener',
        ),
    ])