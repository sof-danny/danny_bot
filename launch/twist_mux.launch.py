from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the twist_mux params file
    params_file = os.path.join(
        get_package_share_directory('danny_bot'),
        'config',
        'twist_mux.yaml'
    )

    # Create twist_mux node
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[params_file],
        remappings=[
            ('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')
        ],
        output='screen'
    )

    return LaunchDescription([
        twist_mux_node
    ])