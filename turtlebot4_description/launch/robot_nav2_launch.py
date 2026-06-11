import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # -------------------------------------------------------------------------
    # 1. Paths to the packages and files
    # -------------------------------------------------------------------------
    turtlebot4_desc_dir = get_package_share_directory('turtlebot4_description')
    
    # Path to your custom map (sitting in the root of your workspace)
    map_yaml_file = os.path.abspath('./Autonomous_map.yaml')

    # Path to your custom nav2 parameters file
    # If it is sitting in the root of your workspace next to the map:
    custom_params_file = os.path.join(turtlebot4_desc_dir, 'config', 'nav2_params.yaml')

    # -------------------------------------------------------------------------
    # 2. Define the individual Launch Descriptions
    # -------------------------------------------------------------------------
    
    # First: Turtlebot4 Lite Description (Gazebo & RViz2)
    turtlebot4_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot4_desc_dir, 'launch', 'turtlebot4_lite.launch.py')
        )
    )

    # Second: Nav2 Localization
    nav2_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot4_desc_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': 'true',
            'params_file': custom_params_file  
        }.items()
    )

    # Third: Nav2 Navigation
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot4_desc_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map_subscribe_transient_local': 'true',
            'params_file': custom_params_file 
        }.items()
    )

    run_mission_launch = Node(
        package='turtlebot4_description',  # Matches your project() name
        executable='mission_coordinator',  # Matches the RENAME attribute in CMake
        name='mission_running',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    # -------------------------------------------------------------------------
    # 3. Return the single LaunchDescription hosting all three
    # -------------------------------------------------------------------------
    return LaunchDescription([
        turtlebot4_launch,
        nav2_localization_launch,
        nav2_navigation_launch,
        run_mission_launch,
    ])