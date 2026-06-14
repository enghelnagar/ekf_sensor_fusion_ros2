#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
import math

class EKFFusionNode(Node):
    """
    Custom Extended Kalman Filter (EKF) Implementation.
    Fuses actual Gazebo Odometry and IMU data to estimate the true state of the robot.
    State Vector X: [x, y, theta, v, omega]^T
    """
    def __init__(self):
        super().__init__('ekf_fusion_node')
        
        # Subscriptions to actual Gazebo sensor topics
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/sensors/imu', self.imu_callback, 10)
        
        # Publisher for the filtered state
        self.filtered_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        
        # --- EKF Initialization ---
        # 1. State Vector [x, y, theta, v, omega]^T
        self.X = np.zeros((5, 1))
        
        # 2. State Covariance Matrix (P)
        self.P = np.eye(5) * 0.1
        
        # 3. Process Noise Covariance (Q)
        self.Q = np.diag([0.0001, 0.0001, 0.0001, 0.03, 0.03])
        
        # 4. Measurement Noise Covariances (R)
        self.R_odom = np.diag([0.05, 5])  # Variance in odometry [v, omega]
        self.R_imu = np.array([[0.05]])     # Variance in IMU [omega]
        
        self.last_time = self.get_clock().now()
        
        # Control loop at 50 Hz
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.odom_meas = None

    def predict(self, dt):

        if dt > 0.05 or dt <= 0.0:
            dt = 0.02

        """
        EKF Prediction Step based on non-linear kinematic model.
        """
        x = self.X[0, 0]
        y = self.X[1, 0]
        theta = self.X[2, 0]
        v = self.X[3, 0]
        omega = self.X[4, 0]
        
        # Non-linear State Prediction
        self.X[0, 0] = x + v * math.cos(theta) * dt
        self.X[1, 0] = y + v * math.sin(theta) * dt
        self.X[2, 0] = theta + omega * dt
        
        # Compute Jacobian Matrix (F)
        F = np.eye(5)
        F[0, 2] = -v * math.sin(theta) * dt
        F[0, 3] = math.cos(theta) * dt
        F[1, 2] = v * math.cos(theta) * dt
        F[1, 3] = math.sin(theta) * dt
        F[2, 4] = dt
        
        # Propagate Covariance
        self.P = F @ self.P @ F.T + self.Q

    def update_odom(self, v_meas, omega_meas):
        """
        EKF Update Step utilizing Odometry measurements.
        """
        Z = np.array([[v_meas], [omega_meas]])
        H = np.zeros((2, 5))
        H[0, 3] = 1.0  
        H[1, 4] = 1.0  
        
        Y = Z - (H @ self.X)
        S = H @ self.P @ H.T + self.R_odom
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # CRITICAL FIX: Prevent unobservable states (x, y, theta) from exploding 
        # due to massive Kalman gains. We decouple position correction from velocity residuals.
        K[0:3, :] = 0.0 
        
        self.X = self.X + (K @ Y)
        self.P = (np.eye(5) - (K @ H)) @ self.P
        
    def update_imu(self, omega_meas):
        """
        EKF Update Step utilizing IMU angular velocity.
        """
        Z = np.array([[omega_meas]])
        H = np.zeros((1, 5))
        H[0, 4] = 1.0  
        
        Y = Z - (H @ self.X)
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)
        K[0:3, :] = 0.0
        self.X = self.X + (K @ Y)
        self.P = (np.eye(5) - (K @ H)) @ self.P

    def odom_callback(self, msg):
        self.odom_meas = msg

    def imu_callback(self, msg):
        omega_meas = msg.angular_velocity.z
        self.update_imu(omega_meas)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """ Helper function to convert Euler angles to Quaternion """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        class Quaternion: pass
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0:
            return
            
        self.predict(dt)
        
        if self.odom_meas is not None:
            v_meas = self.odom_meas.twist.twist.linear.x
            omega_meas = self.odom_meas.twist.twist.angular.z
            self.update_odom(v_meas, omega_meas)
            self.odom_meas = None 
            
        msg = Odometry()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link_filtered'
        
        msg.pose.pose.position.x = float(self.X[0, 0])
        msg.pose.pose.position.y = float(self.X[1, 0])
        msg.pose.pose.position.z = 0.0
        
        q = self.quaternion_from_euler(0, 0, float(self.X[2, 0]))
        msg.pose.pose.orientation.x = q.x
        msg.pose.pose.orientation.y = q.y
        msg.pose.pose.orientation.z = q.z
        msg.pose.pose.orientation.w = q.w
        
        # Append covariances for plotting
        msg.pose.covariance[0] = float(self.P[0, 0])   
        msg.pose.covariance[7] = float(self.P[1, 1])   
        msg.pose.covariance[35] = float(self.P[2, 2])  
        
        self.filtered_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EKFFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()