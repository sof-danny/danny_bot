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
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')
    
    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')
    
    # Add declaration for map_yaml_file
    declare_map_yaml_file_cmd = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=os.path.join(mobile_dd_robot_dir, 'maps', 'map_1_save.yaml'),
        description='Full path to map yaml file to load'
    )
    
   
    declare_nav2_map_subscribe_for_amcl = DeclareLaunchArgument(
        'map_subscribe_transient_local',
        default_value='true',
        description='use transient local')
        
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launches

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_file_cmd) 
    ld.add_action(declare_nav2_map_subscribe_for_amcl)
    
    # Launch the navigation environment
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            mobile_dd_robot_dir, 'launch', 'nav2_nav.launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_subscribe_transient_local': map_subscribe_transient_local
        }.items()
    )
    
    ld.add_action(navigation_launch)
    
    
    return ld 