from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('simple_rc_car')
    xacro_file = os.path.join(pkg_share, 'urdf', 'rc_car.xacro')
    
    # Process xacro file to get robot description
    robot_description = Command(['xacro ', xacro_file])

    # Path to your world file
    world_file = os.path.join(pkg_share, 'worlds', 'rc_car_world.world')

    # Get Gazebo ROS package directory
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    
    return LaunchDescription([
        # Start Gazebo with your world file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': world_file,
                'verbose': 'true'
            }.items()
        ),
        
        # Robot state publisher - publishes robot_description topic
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }],
            output='screen'
        ),
        
        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'rc_car',
                '-topic', 'robot_description',
                '-x', '0',
                '-y', '0',
            ],
            output='screen'
        )
    ])