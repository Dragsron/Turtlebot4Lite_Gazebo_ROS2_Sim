# Turtlebot4Lite_Gazebo_ROS2_Sim
https://github.com/turtlebot/turtlebot4, meshes based on turtlebot4 github. In this repository package for simulating turtlebot4 lite using ROS2, RVIZ2, and Gazebo. Put the package to the src folder of your ROS workspace. Colcon build and source install to launch robot_nav2_launch.py <br /><br />
Warning: This code is already set on a specific worlds, and ensure to get the map save file first <br /><br />
To make the map, you can do it autonomously by "ros2 launch turtlebot4_description autonomous_mapping_launch.py" Let it run until the whole map for the world is created, save and serialize the map inside the package folder inside savemap folder
