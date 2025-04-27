
# IMU Kalman Filter Package

## Introduction
A Kalman Filter is a recursive algorithm used to estimate the state of a dynamic system from noisy sensor measurements. This package implements a **6-dimensional Kalman Filter** for fusing IMU (Inertial Measurement Unit) data in ROS 2. It processes raw accelerometer (linear acceleration) and gyroscope (angular velocity) measurements to produce optimal filtered estimates.

### Why Kalman Filters?
- **Optimal Estimation**: Minimizes mean squared error under Gaussian noise assumptions
- **Recursive**: Computationally efficient (does not store past measurements)
- **Robustness**: Handles sensor noise and system uncertainties

---

## Mathematical Foundation

### 1. State-Space Representation
The Kalman Filter models a system using:
- **State Vector (xₖ)**: Contains variables to estimate (here: 6D IMU data)
- **Process Model**: Predicts how the state evolves over time
- **Measurement Model**: Relates measurements to the state

### 2. Gaussian Assumption
The filter assumes:
- Process noise (wₖ) ~ N(0, Qₖ)
- Measurement noise (vₖ) ~ N(0, Rₖ)
- Errors are Gaussian and uncorrelated

### 3. Core Equations

#### Prediction (Time Update):
```
x̂ₖ⁻ = Fₖ x̂ₖ₋₁    // State prediction
Pₖ⁻ = Fₖ Pₖ₋₁ Fₖᵀ + Qₖ    // Covariance prediction
```

#### Update (Measurement Update):
```
yₖ = zₖ − Hₖ x̂ₖ⁻    // Measurement residual
Sₖ = Hₖ Pₖ⁻ Hₖᵀ + Rₖ    // Residual covariance
Kₖ = Pₖ⁻ Hₖᵀ Sₖ⁻¹    // Kalman gain
x̂ₖ = x̂ₖ⁻ + Kₖ yₖ    // State update
Pₖ = (I − Kₖ Hₖ) Pₖ⁻    // Covariance update
```

---

## Theoretical Background

### 1. Linear System Assumption
The filter assumes linear system dynamics:
```math
xₖ = Fₖ xₖ₋₁ + wₖ
```
```math
zₖ = Hₖ xₖ + vₖ
```

### 2. Recursive Estimation
The algorithm alternates between:
- **Predict**: Project current state forward
- **Update**: Adjust prediction using new measurements

### 3. Optimality Conditions
The Kalman Filter is optimal when:
- System is linear
- Noise is Gaussian and white
- Accurate Q (process noise) and R (measurement noise) matrices

---

## IMU-Specific Implementation

### 1. State Vector
The 6D state vector combines accelerometer and gyroscope measurements:
```math
x = [aₓ, aᵧ, a_z, ωₓ, ωᵧ, ω_z]ᵀ
```

### 2. Matrix Configuration

#### State Transition Matrix (F)
```cpp
Eigen::MatrixXd::Identity(6, 6)
```
- Models constant state assumption between measurements
- Identity matrix implies no predicted state change

#### Observation Matrix (H)
```cpp
Eigen::MatrixXd::Identity(6, 6)
```
- All states directly correspond to measurements

### 3. Noise Covariances

| Parameter | Code Implementation | Purpose |
|-----------|----------------------|---------|
| Q (Process) | `MatrixXd::Identity(6,6) * 0.1` | Models system uncertainty |
| R (Measurement) | `MatrixXd::Identity(6,6) * 0.1` | Represents sensor noise |

---

## ROS Node Implementation

### 1. Architecture
```text
[IMU Sensor] --> /imu/data (raw) --> [Kalman Filter Node] --> /imu/filtered (cleaned)
```

### 2. Key Components
- **Subscriber**: Listens to `/imu/data` (sensor_msgs/Imu)
- **Publisher**: Outputs filtered data to `/imu/filtered`

**Callback Flow**:
- Convert ROS message → Eigen vector
- Predict state using previous estimate
- Update with new measurement
- Publish filtered data

### 3. Code Snippets

#### Prediction Step:
```cpp
kf_.predict(F);  // F = Identity matrix
```

#### Update Step:
```cpp
kf_.update(measurement, H);  // H = Identity matrix
```

---

## Tuning Guide

### Critical Parameters

- **Process Noise (Q)**:
    - Increase → Faster response to changes
    - Decrease → More smoothing
    - Default: `0.1 * I₆`
  
- **Measurement Noise (R)**:
    - Increase → Trust model more
    - Decrease → Trust sensor more
    - Default: `0.1 * I₆`

### Tuning Process

- Start with ```Q = 0.1 * I₆, R = 1.0 * I₆
- Record sensor data during typical operation
- Adjust diagonals based on:
    - Sensor noise characteristics (see datasheet)
    - Desired responsiveness vs. smoothing

---

## Limitations & Extensions

### Current Limitations
- Assumes linear system dynamics
- Constant noise parameters (Q, R)
- No cross-correlation between states

### Potential Enhancements
- **Adaptive Tuning**:
```cpp
Q = f(sensor_confidence), R = g(motion_activity)
```
- **Nonlinear Extension**:
    - Implement Extended Kalman Filter (EKF) for nonlinear dynamics
- **Orientation Integration**:
    - Fuse with magnetometer for full 9D pose estimation

---
