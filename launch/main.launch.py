import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Robot name in the Xacro file
    robotXacroName = 'differential_drive_robot'
    
    # Package name
    namePackage = 'mobile_dd_robot'
    
    # Paths to files
    modelFileRelativePath = 'model/robot.xacro'
    worldFileRelativePath = 'model/empty_world.world'
    rvizConfigPath = os.path.join(get_package_share_directory(namePackage), 'rviz', 'robot_3.rviz')
    
    # Absolute paths
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)
    
    # Get robot description from xacro
    robotDescription = xacro.process_file(pathModelFile).toxml()
    
    # Gazebo launch
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments={'world': pathWorldFile}.items()
    )
    
    # Spawn robot in Gazebo
    spawnModelNode = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robotXacroName],
        output='screen'
    )
    
    # Robot State Publisher
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robotDescription,
            'use_sim_time': True
        }]
    )
    
    # RViz
    rvizNode = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rvizConfigPath],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Include joystick launch file
    joystickLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(namePackage), 'launch', 'joystick.launch.py')
        )
    )
    
    # Create launch description
    launchDescriptionObject = LaunchDescription()
    
    # Add actions
    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(spawnModelNode)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(rvizNode)
    launchDescriptionObject.add_action(joystickLaunch)
    
    return launchDescriptionObject 