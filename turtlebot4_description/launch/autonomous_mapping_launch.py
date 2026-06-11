import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Package Directory Resolutions
    pkg_description = get_package_share_directory('turtlebot4_description')

    # 1. Include Base System (Fires up Gazebo Harmonic, Robot State Publisher, and RViz2)
    base_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'turtlebot4_lite.launch.py')
        )
    )

    # 2. Include SLAM Toolbox (Online Async Mode)
    # This single file replaces AMCL entirely. It dynamically maps the room
    # and provides high-precision localization by matching laser scans.
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    apf_explorer_node = Node(
        package='turtlebot4_description',  # Matches your project() name
        executable='apf_explorer',         # Matches the RENAME attribute in CMake
        name='apf_loop_corners_driver',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        base_system_launch,
        slam_toolbox_launch,
        apf_explorer_node
    ])