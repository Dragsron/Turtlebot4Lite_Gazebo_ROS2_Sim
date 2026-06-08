import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Points directly to the location of your coordinator script
    script_path = os.path.abspath('./src/turtlebot4_description/script/mission_coordinator.py')
    
    run_coordinator = ExecuteProcess(
        cmd=['python3', script_path],
        shell=False
    )

    return LaunchDescription([
        run_coordinator
    ])