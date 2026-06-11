#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class APFLoopCornersDriver(Node):
    def __init__(self):
        super().__init__('apf_loop_corners_driver')
        
        # Synchronize node clock with Gazebo simulation
        use_sim_time_param = rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)
        self.set_parameters([use_sim_time_param])
        
        # --- PATROL WAYPOINTS (Clockwise/Counter-Clockwise Corners) ---
        self.waypoints = [
            (1.2, 1.2),   # Quadrant 1 (Top-Right)
            (-1.2, 1.2),  # Quadrant 2 (Top-Left)
            (-1.2, -1.2), # Quadrant 3 (Bottom-Left)
            (1.2, -1.2)   # Quadrant 4 (Bottom-Right)
        ]
        self.current_waypoint_index = 0
        
        # Robot Position Variables
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # --- APF TUNED PARAMETERS ---
        self.k_att = 0.5   
        self.k_rep = 0.05  
        self.rho_0 = 0.30  # 40cm obstacle boundary line

        self.max_linear_speed = 0.15  
        self.max_angular_speed = 0.6  

        self.laser_ranges = []
        self.laser_angle_min = 0.0
        self.laser_angle_increment = 0.0

        # Set up tf2 Buffer and Listener for clean coordinate mapping
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS 2 Pubs/Subs
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Run loop at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop) 
        self.get_logger().info("🔄 Continuous Autonomous SLAM Patrol Controller Online!")
        self.print_current_target()

    def scan_callback(self, msg):
        self.laser_ranges = msg.ranges
        self.laser_angle_min = msg.angle_min
        self.laser_angle_increment = msg.angle_increment

    def print_current_target(self):
        tx, ty = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f"🎯 PATROL TARGET: Waypoint #{self.current_waypoint_index + 1} at ({tx}, {ty})")

    def update_robot_pose_from_tf(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', now)
            self.robot_x = trans.transform.translation.x
            self.robot_y = trans.transform.translation.y
            
            q = trans.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = np.arctan2(siny_cosp, cosy_cosp)
            return True
        except TransformException:
            return False

    def control_loop(self):
        if not self.update_robot_pose_from_tf() or not self.laser_ranges:
            return

        # Get current target coordinates
        target_x, target_y = self.waypoints[self.current_waypoint_index]
        dist_to_target = np.hypot(target_x - self.robot_x, target_y - self.robot_y)

        # --- INFINITE LOOP WAYPOINT TRANSITION LOGIC ---
        if dist_to_target < 0.15:
            self.get_logger().info(f"✅ Reached Corner #{self.current_waypoint_index + 1}!")
            
            # Increment index, then use modulo (%) to loop back to 0 automatically
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
            
            self.get_logger().warn("🔄 Looping patrol sequence...")
            self.print_current_target()
            return

        # 1. ATTRACTIVE FORCE
        f_att_x = self.k_att * (target_x - self.robot_x) / dist_to_target
        f_att_y = self.k_att * (target_y - self.robot_y) / dist_to_target

        # 2. REPULSIVE FORCE (Laser Scanner Field)
        f_rep_x = 0.0
        f_rep_y = 0.0
        active_obstacles_count = 0

        for i, distance in enumerate(self.laser_ranges):
            if np.isinf(distance) or np.isnan(distance) or distance <= 0.05 or distance > 10.0:
                continue
            if distance < self.rho_0:
                angle_world = self.laser_angle_min + (i * self.laser_angle_increment) + self.robot_yaw
                rep_factor = self.k_rep * ((1.0 / distance) - (1.0 / self.rho_0)) / (distance ** 2)
                f_rep_x -= rep_factor * np.cos(angle_world)
                f_rep_y -= rep_factor * np.sin(angle_world)
                active_obstacles_count += 1

        # 3. SUM FORCES & CALCULATE HEADING ERROR
        desired_force_x = f_att_x + f_rep_x
        desired_force_y = f_att_y + f_rep_y

        desired_heading = np.arctan2(desired_force_y, desired_force_x)
        heading_error = desired_heading - self.robot_yaw
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        # TERMINAL STREAMING LOG
        self.get_logger().info(
            f"WP: #{self.current_waypoint_index + 1} ({target_x}, {target_y}) | "
            f"Pose: X={self.robot_x:.2f}, Y={self.robot_y:.2f}, Yaw={np.degrees(self.robot_yaw):.1f}° | "
            f"Yaw Error: {np.degrees(heading_error):.1f}°",
            throttle_duration_sec=1.0  # Kept clean at 1s throttle
        )

        # 4. KINEMATIC RESOLUTION
        twist = Twist()
        if abs(heading_error) > 0.12:  # ~7 degrees threshold
            twist.angular.z = np.clip(1.5 * heading_error, -self.max_angular_speed, self.max_angular_speed)
            twist.linear.x = 0.02  # Tiny helper forward creep to break friction lock
        else:
            twist.linear.x = self.max_linear_speed
            twist.angular.z = np.clip(1.2 * heading_error, -self.max_angular_speed, self.max_angular_speed)

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = APFLoopCornersDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()