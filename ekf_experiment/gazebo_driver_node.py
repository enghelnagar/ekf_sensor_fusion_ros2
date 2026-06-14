#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class GazeboAdvancedDriver(Node):
    """
    Advanced Closed-Loop Trajectory Tracker for Gazebo.
    Uses a Non-linear Lyapunov-based Controller to force the robot 
    to track a perfect Lemniscate (Figure-8) despite Gazebo's physics 
    (friction, inertia, and wheel slip).
    """
    def __init__(self):
        super().__init__('gazebo_driver_node')
        
        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber to actual Gazebo odometry for position feedback
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Control loop execution rate: 50 Hz for smooth tracking
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        # Lemniscate Trajectory Parameters
        self.a = 3.0          # Scale of the Figure-8 (meters)
        self.omega_ref = 0.15 # Speed of the trajectory progression
        self.time_t = 0.0
        
        # Controller Gains (Tuned to overcome Gazebo physics)
        self.k_x = 1.5
        self.k_y = 5.0
        self.k_theta = 2.0
        
        # Robot's current state variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.state_initialized = False

    def euler_from_quaternion(self, quaternion):
        """ Converts quaternion to euler yaw angle """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def odom_callback(self, msg):
        """ Updates the current state based on Gazebo feedback """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_theta = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.state_initialized = True

    def control_loop(self):
        """ Calculates errors and publishes corrective velocity commands """
        cmd_msg = Twist()
        
        if not self.state_initialized:
            # Wait for the first odometry message from Gazebo
            self.cmd_pub.publish(cmd_msg)
            return

        # 1. Compute the Ideal Reference Trajectory (Lemniscate of Bernoulli)
        t = self.time_t
        sin_t = math.sin(self.omega_ref * t)
        cos_t = math.cos(self.omega_ref * t)
        denominator = 1 + sin_t**2
        
        x_r = (self.a * cos_t) / denominator
        y_r = (self.a * sin_t * cos_t) / denominator
        
        # Analytical derivatives to compute feedforward velocities
        dx_r = -self.a * self.omega_ref * sin_t * (1 + 2 * cos_t**2 + sin_t**2) / (denominator**2)
        dy_r = self.a * self.omega_ref * (cos_t**2 - sin_t**2 - sin_t**4) / (denominator**2)
        
        v_r = math.sqrt(dx_r**2 + dy_r**2)
        theta_r = math.atan2(dy_r, dx_r)
        
        # 2. Compute Tracking Errors (Global Frame)
        e_x_global = x_r - self.current_x
        e_y_global = y_r - self.current_y
        e_theta = theta_r - self.current_theta
        
        # Normalize angular error to the range [-pi, pi]
        e_theta = math.atan2(math.sin(e_theta), math.cos(e_theta))
        
        # Transform errors to the Robot's Local Frame
        e_x_local = math.cos(self.current_theta) * e_x_global + math.sin(self.current_theta) * e_y_global
        e_y_local = -math.sin(self.current_theta) * e_x_global + math.cos(self.current_theta) * e_y_global
        
        # 3. Apply Non-linear Control Law
        v_cmd = v_r * math.cos(e_theta) + self.k_x * e_x_local
        omega_cmd = v_r * (self.k_y * e_y_local + self.k_theta * math.sin(e_theta))
        
        # Publish the corrective commands
        cmd_msg.linear.x = v_cmd
        cmd_msg.angular.z = omega_cmd
        self.cmd_pub.publish(cmd_msg)
        
        # Advance time
        self.time_t += self.dt

def main(args=None):
    rclpy.init(args=args)
    node = GazeboAdvancedDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()