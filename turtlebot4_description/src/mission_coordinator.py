#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from action_msgs.msg import GoalStatusArray
import os
import time

class MissionCoordinator(Node):
    def __init__(self):
        super().__init__('mission_coordinator')
        
        # Publisher to update AMCL initial pose
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
            
        # Publisher to send navigation goals
        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10)
            
        # Subscriber to Nav2 Action Status (Replaces Odometry calculation)
        self.status_sub = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self.status_callback, 10)

        # Configuration Settings
        self.gazebo_world = "room_example_world"
        self.robot_name = "turtlebot4_lite"
        
        # State tracking
        self.current_mission = 1
        self.mission_started = False
        self.last_status = None
        
        # =========================================================================
        # YOUR MANUAL COORDINATES CONFIGURATION
        # Format: [Pickup_X, Pickup_Y, Pickup_OZ, Pickup_OW, Goal_X, Goal_Y, Goal_OZ, Goal_OW]
        # =========================================================================
        self.missions = {
            1: [
                1.18128, -1.14906, 0.999999, 0.0012041,     # First Initial Position
                -1.21403, -0.00953737, 0.999984, 0.00574414  # First Goal
            ],
            2: [
                -1.15631, 1.04711, 0.000786099, 1.0,        # Second Initial Position
                -1.07983, -1.0156, -0.711039, 0.703152      # Second Goal
            ]
        }

        # Setup sequence on a brief delay
        self.mission_timers = []
        self.mission_timers.append(self.create_timer(3.0, self.start_sequence))

    def status_callback(self, msg):
        if not self.mission_started or not msg.status_list:
            return

        # Get the latest goal status from the array
        latest_goal = msg.status_list[-1]
        status_code = latest_goal.status

        # Status Code 4 means GOAL_SUCCEEDED in ROS 2 Action messages
        if status_code == 4 and self.last_status != 4:
            self.get_logger().info(f'--- MISSION {self.current_mission} GOAL REACHED (NAV2 CONFIRMED)! ---')
            self.mission_started = False
            self.last_status = 4
            
            if self.current_mission == 1:
                self.current_mission = 2
                self.get_logger().info('Preparing to teleport for Mission 2 in 3 seconds...')
                time.sleep(3.0)  # Short pause at Goal 1 before resetting
                self.execute_current_mission()
            else:
                self.get_logger().info('*** ALL SIMULATION MISSIONS COMPLETED SUCCESSFULLY! ***')
                raise SystemExit
        
        # Track status changes so it doesn't double-trigger
        self.last_status = status_code

    def start_sequence(self):
        self.destroy_timer(self.mission_timers[0])
        self.execute_current_mission()

    def execute_current_mission(self):
        data = self.missions[self.current_mission]
        self.get_logger().info(f'Executing Pipeline Sequence for Mission {self.current_mission}...')

        # 1. Physical Teleportation via Protobuf Text format on Gazebo Sim Engine
        protobuf = f'name: "{self.robot_name}" position: {{ x: {data[0]} y: {data[1]} z: 0.05 }} orientation: {{ x: 0.0 y: 0.0 z: {data[2]} w: {data[3]} }}'
        cmd = f"gz service -s /world/{self.gazebo_world}/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 --req '{protobuf}'"
        os.system(cmd)
        self.get_logger().info(f'Physical robot model teleported to: X={data[0]}, Y={data[1]}')
        time.sleep(1.5)

        # 2. Publish AMCL Initial Pose alignment
        init_msg = PoseWithCovarianceStamped()
        init_msg.header.frame_id = 'map'
        init_msg.header.stamp = self.get_clock().now().to_msg()
        init_msg.pose.pose.position.x = data[0]
        init_msg.pose.pose.position.y = data[1]
        init_msg.pose.pose.orientation.z = data[2]
        init_msg.pose.pose.orientation.w = data[3]
        self.initial_pose_pub.publish(init_msg)
        self.get_logger().info('AMCL Costmap / Particle Cloud synchronized.')
        time.sleep(5.0)  # Allow costmaps to settle down

        # Reset last status tracker before firing the goal
        self.last_status = None

        # 3. Publish Nav Goal topic
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = data[4]
        goal_msg.pose.position.y = data[5]
        goal_msg.pose.orientation.z = data[6]
        goal_msg.pose.orientation.w = data[7]
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f'Navigation goal successfully sent: X={data[4]}, Y={data[5]}')
        
        # Activate state flag so status listener starts monitoring
        self.mission_started = True

def main(args=None):
    rclpy.init(args=args)
    node = MissionCoordinator()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()