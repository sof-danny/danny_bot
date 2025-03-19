#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Package directories
    mobile_dd_robot_dir = get_package_share_directory('mobile_dd_robot')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_file_name = LaunchConfiguration('map_file_name')
    slam_mode = LaunchConfiguration('slam_mode')
    
    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')
        
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(mobile_dd_robot_dir, 'config', 'slam_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file for SLAM')
    
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file_name',
        default_value=os.path.join(mobile_dd_robot_dir, 'maps', 'map_1_serial'),
        description='Full path to serialized map file to load (without file extension)')
    
    declare_slam_mode_cmd = DeclareLaunchArgument(
        'slam_mode',
        default_value='mapping',
        description='SLAM mode: "mapping" or "localization"')
    
    # Launch the simulation environment
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            mobile_dd_robot_dir, 'launch', 'main.launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Create SLAM Toolbox node with direct parameter overrides to ensure they take precedence
    start_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            # Load the base params file first
            slam_params_file,
            # Direct command-line overrides applied AFTER loading the yaml file
            {
                'use_sim_time': use_sim_time,
                # The namespace matters - need to use the direct parameter name to override
                'mode': slam_mode,
                'map_file_name': map_file_name
            }
        ],
        remappings=[]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add declared launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_slam_mode_cmd)
    
    # Add simulation launch
    ld.add_action(simulation_launch)
    
    # Add SLAM node
    ld.add_action(start_slam_toolbox_node)
    
    return ld 