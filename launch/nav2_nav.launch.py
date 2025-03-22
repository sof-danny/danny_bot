#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    #for navigation
    danny_bot_dir = get_package_share_directory('danny_bot')
    #alternatively, we can copy the files: navigation_launch.py into our launch
    #directory and this will allow us to use our package here instead of using the 
    #underlay from the nav2 package as we did here

    nav2_params = os.path.join(danny_bot_dir, 'config','nav2_params.yaml')
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value= 'true',
        description='Use simulation clock if true')
    
    
    # Launch nav2 bringup
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(danny_bot_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params 
        }.items()
    )
    
    
    # Create launch description
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    
    
    # Add navigation launch (after SLAM)
    ld.add_action(navigation_launch)
    

    return ld 