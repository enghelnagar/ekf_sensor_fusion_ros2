#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import random

class SensorSimNode(Node):
    """
    Advanced Hardware Sensor Simulator.
    Injects stochastic (Gaussian) noise and dynamic biases into the IMU 
    and Odometry measurements to mimic real-world low-cost sensors.
    These parameters can be dynamically altered via Jupyter Widgets.
    """
    def __init__(self):
        super().__init__('sensor_sim_node')
        
        # Subscriptions to perfect/raw data
        self.gt_sub = self.create_subscription(Odometry, '/ground_truth', self.gt_callback, 10)
        self.odom_raw_sub = self.create_subscription(Odometry, '/sensors/odom_raw', self.odom_raw_callback, 10)
        
        # Publishers for noisy sensor data
        self.imu_pub = self.create_publisher(Imu, '/sensors/imu', 10)
        self.odom_pub = self.create_publisher(Odometry, '/sensors/odom', 10)
        
        # Dynamic Parameters (Can be changed live from Jupyter Notebook)
        self.declare_parameter('imu_gyro_bias', 0.02)       # Constant drift in gyroscope (rad/s)
        self.declare_parameter('imu_gyro_noise', 0.015)     # Gaussian noise standard deviation for gyro
        self.declare_parameter('odom_slip_noise', 0.05)     # Gaussian noise representing wheel slip
        
        # Control loop frequency
        self.timer = self.create_timer(0.02, self.publish_sensors) # 50 Hz
        
        # State variables
        self.gt_msg = None
        self.odom_raw_msg = None

        # Integrated noisy odometry pose
        self.noisy_odom_x = 0.0
        self.noisy_odom_y = 0.0
        self.noisy_odom_theta = 0.0
        self.last_time = self.get_clock().now()

    def gt_callback(self, msg):
        self.gt_msg = msg
        
    def odom_raw_callback(self, msg):
        self.odom_raw_msg = msg

    def publish_sensors(self):
        if self.gt_msg is None or self.odom_raw_msg is None:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt <= 0:
            return

        # --- 1. Read Dynamic Parameters ---
        gyro_bias = self.get_parameter('imu_gyro_bias').value
        gyro_noise = self.get_parameter('imu_gyro_noise').value
        slip_noise = self.get_parameter('odom_slip_noise').value

        # --- 2. Simulate IMU Data ---
        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = 'base_link'
        
        # Ground truth angular velocity + Bias + Gaussian Noise
        true_omega = self.gt_msg.twist.twist.angular.z
        noisy_omega = true_omega + gyro_bias + random.gauss(0.0, gyro_noise)
        imu_msg.angular_velocity.z = noisy_omega
        
        # Simulated Accelerometer (Gravity + Noise)
        imu_msg.linear_acceleration.x = random.gauss(0.0, 0.1)
        imu_msg.linear_acceleration.y = random.gauss(0.0, 0.1)
        imu_msg.linear_acceleration.z = 9.81 + random.gauss(0.0, 0.1)
        
        # Covariance Matrix (Crucial for EKF)
        imu_msg.angular_velocity_covariance[8] = gyro_noise ** 2
        self.imu_pub.publish(imu_msg)

        # --- 3. Simulate Noisy Odometry (Wheel Slip) ---
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link_noisy'
        
        # Add slip noise to velocities
        v_raw = self.odom_raw_msg.twist.twist.linear.x
        w_raw = self.odom_raw_msg.twist.twist.angular.z
        
        v_noisy = v_raw + random.gauss(0.0, slip_noise)
        w_noisy = w_raw + random.gauss(0.0, slip_noise * 0.5)
        
        # Integrate noisy velocities to get drifting position
        self.noisy_odom_theta += w_noisy * dt
        self.noisy_odom_x += v_noisy * math.cos(self.noisy_odom_theta) * dt
        self.noisy_odom_y += v_noisy * math.sin(self.noisy_odom_theta) * dt
        
        odom_msg.pose.pose.position.x = self.noisy_odom_x
        odom_msg.pose.pose.position.y = self.noisy_odom_y
        
        # Convert Euler to Quaternion
        cy = math.cos(self.noisy_odom_theta * 0.5)
        sy = math.sin(self.noisy_odom_theta * 0.5)
        odom_msg.pose.pose.orientation.w = cy
        odom_msg.pose.pose.orientation.z = sy
        
        odom_msg.twist.twist.linear.x = v_noisy
        odom_msg.twist.twist.angular.z = w_noisy
        
        # Covariance Matrix
        odom_msg.pose.covariance[0] = 0.1    # Variance in X
        odom_msg.pose.covariance[7] = 0.1    # Variance in Y
        odom_msg.pose.covariance[35] = 0.05  # Variance in Yaw
        
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()