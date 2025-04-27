# ros2_kalman_imu
Implementation of Kalman Filter on MPU9250 IMU Linear Acceleration and Angular Velocity Data using ROS2

## 0. Run ROS2 IMU
Before proceeding to run any this node, ensure that the IMU data from your ESP32 device with an MPU sensor is properly integrated into your ROS 2 system. The step zero is to prepare and run the [https://github.com/syedmohiuddinzia/ros2_imu](https://github.com/syedmohiuddinzia/ros2_imu).

## 1. Prerequisites
- ROS 2 installed (e.g., Humble, Foxy, Iron)
- colcon build tool installed
- Git installed

## 2. Create a ROS 2 Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```
## 3. Clone the Repository
```bash
cd ~/ros2_ws/src
git clone https://github.com/<your_github_username>/imu_kalman_filter.git
```

## 4. Build the Workspace
```bash
cd ~/ros2_ws
source /opt/ros/<ros2_distro>/setup.bash
colcon build --packages-select imu_kalman_filter
```
Replace <ros2_distro> with your ROS 2 version (e.g., humble, foxy).

## 5. Source the Workspace
```bash
source install/setup.bash
```
```bash
source ~/ros2_ws/install/setup.bash
```
(Optional: Add it to your ~/.bashrc.)

## 6. Run the IMU Kalman Filter Node
```bash
ros2 run imu_kalman_filter imu_kalman_filter_node
```
![img1]()
Now run in another terminal ```ros2 topic list``` to see if **/imu/filtered** topics are available
![img2]()
And now run ```ros2 topic echo /imu/filtered``` to check if the data is available.
![img3]()
Now in another terminal run **rqt** to plot graphs between **imu/data** and **imu/filtered**. Therefore add topics ```imu/data/linear_acceleration``` and ```imu/filtered/linear_acceleration``` to differentiate between them.
![img4]()

## 7. Run the IMU Visualizer Node
In another terminal:
```bash
ros2 run imu_kalman_filter imu_visualizer_node
```
After running this node open **rviz2**
```
rviz2
```
Add **Imu** for **imu/data** and **imu/filtered** and add **Marker** </br>
Now move IMU to see the movements in rviz2.
![img5]()

## Notes

- Ensure your IMU is publishing data to /imu/data
- Visualize the /imu/filtered topic in RViz2 or by echoing:
- ros2 topic echo /imu/filtered
- Check serial permissions if using a USB IMU device.

## 1. Introduction

The **Kalman Filter** is an algorithm that estimates the true state of a system from noisy sensor measurements.  
It is **predictive** (it guesses ahead) and **corrective** (it fixes guesses based on new data).

You can think of it like:
- **Prediction**: "I think the position will be here based on physics."
- **Correction**: "Hmm, the sensor says I’m a little off, let’s adjust."

## 2. Why Kalman Filter?

In real-world systems like an **IMU (Inertial Measurement Unit)**, sensors measure:
- **Angular velocity** (rotation rate, e.g., °/s)
- **Linear acceleration** (change in speed, e.g., m/s²)

Both are noisy!

Kalman Filter **smooths** these measurements to give a **better estimate** of the true motion.

## 3. The Basic Idea

At every time step:
1. **Predict** where the system should be.
2. **Update** this prediction based on actual measurement.

This two-step cycle keeps running as data comes in.

## 4. Mathematical Foundation

We work with:
- **State vector**: what we are tracking (e.g., velocity, position)
- **Measurement**: what sensors tell us (noisy)
- **Matrices** that describe motion and noise.

### Key Variables
| Symbol | Meaning |
|:------:|:--------|
| x      | State (e.g., velocity, position) |
| P      | State uncertainty (covariance) |
| u      | Control input (e.g., acceleration) |
| z      | Measurement (sensor data) |
| F      | State transition model |
| H      | Measurement model |
| Q      | Process noise covariance |
| R      | Measurement noise covariance |
| K      | Kalman Gain (how much to trust sensor vs prediction) |

## 5. Kalman Filter Steps

### 5.1 Prediction

```math
x' = F * x + B * u
```
```math
P' = F * P * Fᵀ + Q
```

- `x'` is the predicted state.
- `P'` is the predicted uncertainty.

### 5.2 Update (Correction)

```math
K = P' * Hᵀ * (H * P' * Hᵀ + R)⁻¹
```
```math
x = x' + K * (z - H * x')
```
```math
P = (I - K * H) * P'
```

- `K` is the Kalman Gain.
- `z - H * x'` is the **innovation** (measurement residual).

## 6. IMU Example: Angular Velocity and Linear Acceleration

Suppose you have an IMU measuring:
- `ω` (angular velocity in rad/s)
- `a` (linear acceleration in m/s²)

You want to estimate:
- **Orientation** (angle)
- **Velocity**

### Define State

```math
x = [θ, v]ᵀ
```

where:
- θ = angle (from integrating angular velocity)
- v = velocity (from integrating acceleration)

### Measurement

```math
z = [ω^{measured}, a^{measured}]^T
```

### State Transition Model (F)

Assume constant velocity/angular velocity over small dt:

$$
H = \begin{bmatrix}
1 & dt \\
0 & 1
\end{bmatrix}
$$


### Control Input (u)

If external forces (optional):

```math
u = [0, 0]^T
```

### Measurement Model (H)

Since IMU gives direct measurements of angular velocity and acceleration:

$$
H = \begin{bmatrix}
1 & 0 \\
0 & 1
\end{bmatrix}
$$

### Process Noise Covariance (Q)

Models uncertainty in dynamics (e.g., slight modeling errors):

$$
\mathbf{Q} = \text{small constant matrix}
$$

### Measurement Noise Covariance (R)

Models sensor noise:

$$
\mathbf{R} = \text{based on sensor spec (e.g., standard deviation²)}
$$

## 7. Full Algorithm with IMU

At every time step:
1. **Predict**
    - Predict θ and v based on last estimates.
2. **Update**
    - Use new IMU measurements (ω, a) to correct estimates.

## 8. Advanced Topics

### 8.1 Tuning R and Q
- If R is too small → filter trusts sensors too much (noisy output).
- If Q is too big → filter thinks motion model is bad (overreacts to sensor).

You have to **tune** R and Q to get good performance!

### 8.2 Nonlinear Systems
When F or H is nonlinear, you need:
- **Extended Kalman Filter (EKF)** — linearizes around current estimate.
- **Unscented Kalman Filter (UKF)** — better but more complex.

IMU data often requires EKF because real motion is slightly nonlinear.

### 8.3 Sensor Fusion
If you add GPS or Magnetometer data, you can extend the Kalman Filter to fuse them with the IMU.

## 9. Visualization

- **Prediction** moves the estimate according to the model.
- **Correction** pulls it back towards the measurement.

Imagine walking blindfolded with a GPS that buzzes when you're off-course, you "predict" your next step and "correct" based on the buzz!
