#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import math
import numpy as np

class TrajectoryPlannerNode(Node):
    """
    Advanced Trajectory Planner and Non-linear Tracking Controller.
    Generates a Lemniscate of Bernoulli reference trajectory and utilizes a 
    Lyapunov-based non-linear controller to output velocity commands (/cmd_vel).
    """
    def __init__(self):
        super().__init__('trajectory_planner_node')
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/reference_trajectory', 10)
        self.gt_sub = self.create_subscription(Odometry, '/ground_truth', self.gt_callback, 10)
        
        # Control loop timer (50 Hz for high fidelity)
        self.dt = 0.02 
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        # Lemniscate Trajectory Parameters
        self.a = 5.0  # Scale of the Lemniscate (meters)
        self.omega_ref = 0.1  # Angular frequency of the trajectory
        self.time_t = 0.0
        
        # Controller Gains (Tuned for critical damping)
        self.k_x = 2.0
        self.k_y = 10.0
        self.k_theta = 2.0
        
        # Robot's current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.state_initialized = False

        # Path visualization message
        self.ref_path = Path()
        self.ref_path.header.frame_id = 'odom'

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place for ROS geometry_msgs) to euler roll, pitch, yaw.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def gt_callback(self, msg):
        """
        Feedback callback to update the current state of the robot from ground truth.
        """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_theta = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.state_initialized = True

    def control_loop(self):
        """
        Main control loop: Computes reference state, calculates error, and applies 
        non-linear control law to generate velocity commands.
        """
        # 1. Compute Reference Trajectory (Lemniscate of Bernoulli)
        t = self.time_t
        sin_t = math.sin(self.omega_ref * t)
        cos_t = math.cos(self.omega_ref * t)
        denominator = 1 + sin_t**2
        
        # Reference Position
        x_r = (self.a * cos_t) / denominator
        y_r = (self.a * sin_t * cos_t) / denominator
        
        # Reference Derivatives (Analytical for precise feedforward)
        dx_r = -self.a * self.omega_ref * sin_t * (1 + 2 * cos_t**2 + sin_t**2) / (denominator**2)
        dy_r = self.a * self.omega_ref * (cos_t**2 - sin_t**2 - sin_t**4) / (denominator**2)
        
        # Reference Velocity and Heading
        v_r = math.sqrt(dx_r**2 + dy_r**2)
        theta_r = math.atan2(dy_r, dx_r)
        
        # Publish Reference Path for visualization
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = x_r
        pose.pose.position.y = y_r
        self.ref_path.poses.append(pose)
        self.path_pub.publish(self.ref_path)

        # Output message
        cmd_msg = Twist()

        if not self.state_initialized:
            # If no feedback yet, output zero velocities to wait for the simulation
            self.cmd_pub.publish(cmd_msg)
            return

        # 2. Compute Tracking Errors
        e_x_global = x_r - self.current_x
        e_y_global = y_r - self.current_y
        e_theta = theta_r - self.current_theta
        
        # Normalize angular error to [-pi, pi]
        e_theta = math.atan2(math.sin(e_theta), math.cos(e_theta))
        
        # Transform global error to robot's local frame
        e_x_local = math.cos(self.current_theta) * e_x_global + math.sin(self.current_theta) * e_y_global
        e_y_local = -math.sin(self.current_theta) * e_x_global + math.cos(self.current_theta) * e_y_global
        
        # 3. Apply Non-linear Lyapunov-based Tracking Control Law
        # Note: Feedforward angular velocity (omega_r) is approximated as 0 for simplicity in this step,
        # but the proportional terms will robustly drive the robot to the path.
        v_cmd = v_r * math.cos(e_theta) + self.k_x * e_x_local
        omega_cmd = 0.0 + v_r * (self.k_y * e_y_local + self.k_theta * math.sin(e_theta))
        
        # 4. Publish Velocity Commands
        cmd_msg.linear.x = v_cmd
        cmd_msg.angular.z = omega_cmd
        self.cmd_pub.publish(cmd_msg)
        
        # Increment time
        self.time_t += self.dt

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()