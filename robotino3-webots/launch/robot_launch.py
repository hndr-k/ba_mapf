#!/usr/bin/env python

"""Launch Webots Robotino driver."""

import os
import sys
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_core.webots_launcher import WebotsLauncher
from webots_ros2_core.utils import ControllerLauncher
from nav2_common.launch import RewrittenYaml


ARGUMENTS = [
     DeclareLaunchArgument(
            'world',
            default_value='rcll_world_test.wbt',
            description='Choose one of the world files from `/webots_ros2_robotino3/world` directory'
        ),
        DeclareLaunchArgument(
            'synchronization',
            default_value='false',
            description='If `False` robot.step() will be automatically called.'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Whether to publish transforms (tf)'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='robotino1',
            description='The name of the robot (has to be the same as in Webots)'
        ),
        DeclareLaunchArgument(
            'node',
            default_value='',
            description='The name of the node(has to be the same as in Webots)'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Whether to start GUI or not.'
        ),
         DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Whether to start GUI or not.'
        ),               
        DeclareLaunchArgument(
                    "log_level",
                    default_value=["debug"],
                    description="Logging level",
            ),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(get_package_share_directory('webots_ros2_robotino3'), 'params', 'webots_params.yaml'),
            description='Full path to the ROS2 parameters file to use',
            )
]




def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_robotino3')
    world = LaunchConfiguration('world')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    logger = LaunchConfiguration("log_level")
    gui = LaunchConfiguration('gui')
    mode = LaunchConfiguration('mode')
    publish_tf = LaunchConfiguration('publish_tf')
    robot_name = 'robotino1'
    node_name = 'robotino1'
    synchronization = LaunchConfiguration('synchronization')
    sys.stdout.flush()
    
    param_substitutions = {
    'use_sim_time': use_sim_time,
    'autostart': autostart}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]


            
    webots = WebotsLauncher(
    world=PathJoinSubstitution([package_dir, 'worlds', world]),
    mode=mode,
    gui=gui)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace='robotino1',
        remappings=[('/tf', 'tf'),('/tf_static', 'tf_static')],
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>',
            'use_sim_time': use_sim_time
        }],
        condition=launch.conditions.IfCondition(publish_tf)
    )
    controller = ControllerLauncher(
        package='webots_ros2_robotino3',
        executable='robotino3_driver',
        namespace='robotino1',
        additional_env={'WEBOTS_ROBOT_NAME' : 'robotino1'},
        parameters=[
            configured_params,
            {
                'synchronization': synchronization,
                'use_joint_state_publisher': publish_tf
            }],
        output='screen',
        arguments=[
            '--webots-robot-name', robot_name,
            '--webots-node-name', node_name
        ],
        remappings=[('/tf', 'tf'),('/tf_static', 'tf_static'), ('/robotino1/clock', '/clock')],
    )
    controller2 = ControllerLauncher(
        package='webots_ros2_robotino3',
        executable='robotino3_driver_new',
        namespace='robotino2',
        additional_env={'WEBOTS_ROBOT_NAME' : 'robotino2'},
        parameters=[
            configured_params,
            {
                'synchronization': synchronization,
                'use_joint_state_publisher': publish_tf
            }],
        output='screen',
        arguments=[
            '--webots-robot-name', 'robotino2',
            '--webots-node-name', 'robotino2'
        ],
        remappings=[('/tf', 'tf'),('/tf_static', 'tf_static')],
    )

    return LaunchDescription(ARGUMENTS +[
        webots,
        robot_state_publisher,
        controller,
        controller2
    ])
