#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class KinematicSimNode(Node):
    """
    Advanced Kinematic Simulator for a Differential Drive Robot.
    Simulates both the true motion of the robot (with systematic hardware errors)
    and the raw odometry motion (ideal hardware assumption) to generate realistic drift.
    """
    def __init__(self):
        super().__init__('kinematic_sim_node')
        
        # Subscriptions and Publications
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.gt_pub = self.create_publisher(Odometry, '/ground_truth', 10)
        self.odom_pub = self.create_publisher(Odometry, '/sensors/odom_raw', 10)
        
        # Simulation frequency (50 Hz for smooth integration)
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.update_kinematics)
        
        # 1. Nominal Parameters (What the robot's computer thinks it has)
        self.r_nom = 0.1      # Nominal wheel radius (meters)
        self.L_nom = 0.5      # Nominal wheelbase (meters)
        
        # 2. Actual Parameters (The real physical hardware with manufacturing defects)
        # This creates the systematic drift that our EKF will have to fix!
        self.r_actual_right = 0.102  # Right wheel is slightly larger (+2mm)
        self.r_actual_left = 0.0985  # Left wheel is slightly smaller (-1.5mm)
        self.L_actual = 0.515        # Wheelbase is slightly wider (+15mm)
        
        # Velocity commands buffer
        self.v_cmd = 0.0
        self.omega_cmd = 0.0
        
        # State vectors [x, y, theta]
        self.state_gt = [0.0, 0.0, 0.0]     # Ground Truth State
        self.state_odom = [0.0, 0.0, 0.0]   # Odometry State
        
    def cmd_callback(self, msg):
        """ Receives velocity commands from the trajectory planner """
        self.v_cmd = msg.linear.x
        self.omega_cmd = msg.angular.z

    def quaternion_from_euler(self, roll, pitch, yaw):
        """ Helper function to convert Euler angles to Quaternion """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        class Quaternion:
            pass
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q
        
    def update_kinematics(self):
        """ Integrates the kinematic equations taking into account the hardware errors """
        
        # --- Step A: Inverse Kinematics (Controller output to wheel speeds) ---
        # The motor controller calculates required wheel speeds based on NOMINAL parameters
        omega_right = (self.v_cmd + (self.omega_cmd * self.L_nom / 2.0)) / self.r_nom
        omega_left = (self.v_cmd - (self.omega_cmd * self.L_nom / 2.0)) / self.r_nom
        
        # --- Step B: Forward Kinematics (Wheel speeds to Robot Motion) ---
        
        # 1. Calculate Raw Odometry (Using Nominal Parameters)
        v_odom = (self.r_nom / 2.0) * (omega_right + omega_left)
        w_odom = (self.r_nom / self.L_nom) * (omega_right - omega_left)
        
        self.state_odom[0] += v_odom * math.cos(self.state_odom[2]) * self.dt
        self.state_odom[1] += v_odom * math.sin(self.state_odom[2]) * self.dt
        self.state_odom[2] += w_odom * self.dt
        
        # 2. Calculate Ground Truth (Using Actual Defective Parameters)
        v_gt = (omega_right * self.r_actual_right + omega_left * self.r_actual_left) / 2.0
        w_gt = (omega_right * self.r_actual_right - omega_left * self.r_actual_left) / self.L_actual
        
        self.state_gt[0] += v_gt * math.cos(self.state_gt[2]) * self.dt
        self.state_gt[1] += v_gt * math.sin(self.state_gt[2]) * self.dt
        self.state_gt[2] += w_gt * self.dt
        
        # --- Step C: Publish the Data ---
        self.publish_odometry(self.odom_pub, self.state_odom, 'odom', 'base_link')
        self.publish_odometry(self.gt_pub, self.state_gt, 'world', 'base_link_gt')

    def publish_odometry(self, publisher, state, frame_id, child_frame_id):
        """ Formats and publishes the Odometry message """
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.child_frame_id = child_frame_id
        
        msg.pose.pose.position.x = state[0]
        msg.pose.pose.position.y = state[1]
        msg.pose.pose.position.z = 0.0
        
        q = self.quaternion_from_euler(0, 0, state[2])
        msg.pose.pose.orientation.x = q.x
        msg.pose.pose.orientation.y = q.y
        msg.pose.pose.orientation.z = q.z
        msg.pose.pose.orientation.w = q.w
        
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()