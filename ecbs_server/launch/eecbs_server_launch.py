import os
from glob import glob
from setuptools import setup

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, launch_description
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    package_dir = get_package_share_directory('ecbs_server')
    
    params_path = os.path.join(package_dir, 'params', 'mapf_params.yaml')
    autostart = LaunchConfiguration('autostart')

    lifecycle_nodes = ['map_server_mapf', 'ecbs_server']

    print(os.path.join(package_dir, 'params', 'mapf_params.yaml'))

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(package_dir, 'params', 'mapf_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    return LaunchDescription([
        # Set env var to print messages to stdout immediately

        Node(
            package='ecbs_server',
            name='ecbs_server',
            executable= 'ecbs_server',
            output= 'screen',
            #prefix=['gdbserver localhost:3000'],
            parameters= [params_path]),

        # TODO: the following needs to be used properly
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(package_dir, 'maps', 'mapf.yaml'),
            description='Full path to map yaml file to load'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        Node(
            package='nav2_map_server' ,
            name = 'map_server_mapf',
            executable= 'map_server' ,
            parameters= [params_path]),
    
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapf',
            output='screen',
            parameters=[{'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),
        ])

#    start_mapf_server = Node(
#        package='mapf_server' ,
#        executable= 'mapf_server' ,
#        parameters= [ 
#            params_path
#        ]
#    )
#
#    start_map_server = Node(
#        package='nav2_map_server' ,
#        executable= 'map_server' ,
#        parameters= [ 
#            params_path
#        ]
#    )
#
    ld = LaunchDescription()

    ld.add_action(declare_params_file_cmddeclare_robot2_params_file_cmd)


    return ld
