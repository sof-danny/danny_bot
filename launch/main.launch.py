import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription , DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    # Robot name in the Xacro file
    robotXacroName = 'differential_drive_robot'
    
    # Package name
    namePackage = 'danny_bot'
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Paths to files
    modelFileRelativePath = 'model/robot.xacro'
    worldFileRelativePath = 'worlds/obstacles.world'
    rvizConfigPath = os.path.join(get_package_share_directory(namePackage), 'rviz', 'robot_2.rviz')
    
    # Absolute paths
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)
    
    # Get robot description from xacro
    robotDescription = xacro.process_file(pathModelFile).toxml()
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
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
    
    #adding this to use ros2_control
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robotDescription},
                   {'use_sim_time': use_sim_time},
                   os.path.join(get_package_share_directory(namePackage), 'config', 'my_controllers.yaml')],
        output='screen'
    )
    
    diff_drive_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],  # Fixed: removed quotes
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    joint_broad_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],  # Fixed: removed quotes
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Robot State Publisher
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robotDescription,
            'use_sim_time': use_sim_time
        }]
    )
    

    
    # RViz
    rvizNode = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rvizConfigPath],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    

    # Include joystick launch file
    joystickLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(namePackage), 'launch', 'joystick.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    #Include twist mux launch file
    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(namePackage), 'launch', 'twist_mux.launch.py')
        )
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add actions
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(gazeboLaunch)
    ld.add_action(spawnModelNode)
    ld.add_action(nodeRobotStatePublisher)
    ld.add_action(rvizNode)
    ld.add_action(joystickLaunch)
    ld.add_action(twist_mux_launch)
    ld.add_action(controller_manager_node)
    ld.add_action(diff_drive_spawner)
    ld.add_action(joint_broad_spawner)
    
    return ld