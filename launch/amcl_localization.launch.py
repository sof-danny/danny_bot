#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Package directories
    mobile_dd_robot_dir = get_package_share_directory('danny_bot')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    
    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')
    
    declare_map_yaml_file_cmd = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=os.path.join(mobile_dd_robot_dir, 'maps', 'map_2_save.yaml'),
        description='Full path to map yaml file to load')
        
    
    # Launch the simulation environment
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            mobile_dd_robot_dir, 'launch', 'main.launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Start Map Server with the map
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml_file}
        ])
    
    # Lifecycle management for map server
    map_server_lifecycle = Node(
        package='nav2_util',
        executable='lifecycle_bringup',
        name='lifecycle_bringup',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']}
        ],
        arguments=['map_server']
        )
    
    # AMCL for localization
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ])
    
    # Lifecycle management for AMCL
    amcl_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['amcl']}
        ],
        arguments=['amcl']
        )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add declared launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_file_cmd)
    
    # Add simulation launch
    ld.add_action(simulation_launch)
    
    # Add Map Server and its lifecycle manager
    ld.add_action(map_server)
    ld.add_action(map_server_lifecycle)
    
    # Add AMCL localization and its lifecycle manager
    ld.add_action(amcl)
    ld.add_action(amcl_lifecycle)
    
    return ld 