# Extended Kalman Filter (EKF) Sensor Fusion for Differential Drive Robots in ROS2

## Author & Course Details
* **Author:** Hussein Elnaggar
* **Profession:** Mechatronics Engineer
* **Institution:** University of Genoa (Università degli Studi di Genova), Italy
* **Program:** Master's Degree in Robotics Engineering
* **Project:** State Estimation & Sensor Fusion Assignment (EKF Integration)

---

## Project Overview
This repository contains a robust ROS2 package (`ekf_experiment`) and an interactive Jupyter Notebook analysis dashboard designed to implement and evaluate a custom **Continuous-Discrete 5-State Extended Kalman Filter (EKF)**. The filter integrates data from wheel odometry (wheel encoders) and an Inertial Measurement Unit (IMU) to estimate the true state of a differential drive robot operating in a Gazebo simulation environment.

The package is explicitly engineered to demonstrate the real-world mathematical limitations of state estimation when a vehicle is subjected to systematic hardware manufacturing flaws and unmodeled sensor biases.

---

## Mathematical Architecture & Kinematic Model

The motion of the differential drive robot is tracked using a non-linear 5-state unicycle kinematic model. The continuous-time state vector X is formulated as:

X = [x, y, θ, v, ω]^T

Where:
* x, y: Coordinates representing the planar position of the robot in meters.
* θ: Robot heading orientation (yaw) in radians.
* v: Forward linear velocity in meters per second.
* ω: Body-frame angular velocity around the Z-axis in radians per second.

### Discrete-Time State Transition Model (Prediction Step)
At each sample interval dt, the state vector is projected forward using the following non-linear difference equations:

x_k = x_{k-1} + v_{k-1} * cos(θ_{k-1}) * dt
y_k = y_{k-1} + v_{k-1} * sin(θ_{k-1}) * dt
θ_k = θ_{k-1} + ω_{k-1} * dt
v_k = v_{k-1} + w_v
ω_k = ω_{k-1} + w_ω

Where w_v and w_ω are additive, zero-mean white Gaussian process noise components governed by the Process Noise Covariance Matrix (Q).

---

## Injected Real-World Hardware Flaws

To evaluate the mathematical robustness of the EKF tuning, two distinct deterministic hardware faults are explicitly injected into the Gazebo simulation background:

1. **Systematic Odometry Error (Kinematic Discrepancy):**
   * **Nominal Design:** Left and right wheel radii are structurally assumed to be equal (R = 0.1 m).
   * **Injected Reality (Ground Truth):** R_left = 0.095 m and R_right = 0.105 m (10 mm structural asymmetry in diameter).
   * **Kinematic Result:** Equal wheel angular velocity commands force the physical robot to curve involuntarily to the left (tighter Green circle). The raw odometry equations compute a wide, idealistic path (Red path), unaware of the asymmetric radius.

2. **Constant Gyroscope Bias (Inertial Fault):**
   * **Injected Reality:** A deterministic, unmodeled constant angular velocity drift of Bias = +0.02 rad/s is added to the IMU Z-axis gyroscope readings.
   * **Kinematic Result:** Temporal numerical integration of raw inertial readings yields a quadratically compounding error in orientation (θ), leading to catastrophic heading divergence if uncorrected.

---

## Extended Kalman Filter Formulation

### 1. Measurement Matrix (H) and Analytical Derivations
The measurement update handles incoming sensor observations asynchronously. The Jacobian observation matrix H maps internal state space variables to physical sensor measurements (Z_k).

#### Case A: Wheel Odometry Observation Update
The wheel encoders measure linear velocity (v) and angular velocity (ω) directly. The resulting mapping is linear, forming a 2x5 Jacobian matrix:

H_odom = 
[0  0  0  1  0]
[0  0  0  0  1]

#### Case B: IMU Gyroscope Observation Update
The IMU isolates and measures only the body-frame rotational velocity (ω). The mapping forms a 1x5 row vector:

H_imu = [0  0  0  0  1]

---

## Covariance Tuning Strategy (Q and R Design)

The EKF filter's behavior is dictated by the mathematical equilibrium established within the structural tuning matrices inside `ekf_fusion_node.py`:

```python
# State Covariance Initialization
self.P = np.eye(5) * 0.1

# 1. Process Noise Covariance (Q Matrix)
# Extremely low values constrain state variance, forcing high smoothness.
self.Q = np.diag([0.001, 0.001, 0.001, 0.01, 0.01])

# 2. Measurement Noise Covariance (R Matrices)
# R_odom: Moderately trusts linear velocity (0.05) but heavily penalizes 
# odometry angular updates (5.0) due to wheel radius asymmetry.
self.R_odom = np.diag([0.05, 5.0])  

# R_imu: High relative trust (0.05) to override odometry's angular errors.
self.R_imu = np.array([[0.05]])
