import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    share_dir = get_package_share_directory('turtlebot4_description')
    # Get package paths
    pkg_name = 'turtlebot4_description'
    pkg_description = get_package_share_directory(pkg_name)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to standalone Harmonic URDF file
    urdf_file = os.path.join(pkg_description, 'urdf', 'turtlebot4_harmonic.urdf')

    # Read URDF file contents into memory
    with open(urdf_file, 'r') as infp:
        robot_description_raw = infp.read()

    # Node to publish robot transforms (TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }]
    )

    # Path to custom world file
    world_file = os.path.join(pkg_description, 'worlds', 'room_example.sdf')

    # Launch Gazebo Harmonic loading room layout
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Spawn the robot entity directly into the room environment
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot4_lite',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05'
        ],
        output='screen'
    )

    # ROS 2 to Gazebo Topic Parameter Bridge Configuration
    bridge_params = os.path.join(pkg_description, 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # --- RESOLVE CONFIGURATION PATHS ---
    twist_mux_config_path = os.path.join(pkg_description, 'config', 'twist_mux.yaml')

    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )


    # Node to launch twist_mux using package's yaml parameters
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config_path, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        rviz_node,
        twist_mux,
    ])