# ros2_kalman_imu
Implementation of Kalman Filter on MPU9250 IMU Linear Acceleration and Angular Velocity Data using ROS2

# 📚 Mathematics Behind 3D Kalman Filter

## 1. Definitions

We are filtering a 3D state:

- **State vector** ($\mathbf{x}$):

$$
\mathbf{x} = \begin{bmatrix} x \\ y \\ z \end{bmatrix}
$$

- **Measurement vector** ($\mathbf{z}$):

$$
\mathbf{z} = \begin{bmatrix} z_x \\ z_y \\ z_z \end{bmatrix}
$$

- **State covariance** ($\mathbf{P}$) (3×3 matrix):  
  Represents the uncertainty in the estimate.

- **Process noise covariance** ($\mathbf{Q}$) (3×3 matrix):  
  Represents system disturbances.

- **Measurement noise covariance** ($\mathbf{R}$) (3×3 matrix):  
  Represents sensor noise.

- **Transition matrix** ($\mathbf{A}$) and **Observation matrix** ($\mathbf{H}$):  
  Here both are identity matrices ($\mathbf{I}_3$).

---

## 2. Kalman Filter Steps

### (A) Prediction Step

Predict the next state and its uncertainty:

$$
\mathbf{x}_{\text{pred}} = \mathbf{A} \mathbf{x}
$$

$$
\mathbf{P}_{\text{pred}} = \mathbf{A} \mathbf{P} \mathbf{A}^\top + \mathbf{Q}
$$

Since $\mathbf{A} = \mathbf{I}_3$, it simplifies to:

$$
\mathbf{x}_{\text{pred}} = \mathbf{x}
$$

$$
\mathbf{P}_{\text{pred}} = \mathbf{P} + \mathbf{Q}
$$

---

### (B) Update Step

Correct using the new measurement ($\mathbf{z}$).

1. **Compute Kalman Gain** ($\mathbf{K}$):

$$
\mathbf{K} = \mathbf{P}_{\text{pred}} \left( \mathbf{P}_{\text{pred}} + \mathbf{R} \right)^{-1}
$$

2. **Update the estimate**:

$$
\mathbf{x}_{\text{new}} = \mathbf{x}_{\text{pred}} + \mathbf{K} \left( \mathbf{z} - \mathbf{x}_{\text{pred}} \right)
$$

3. **Update the covariance**:

$$
\mathbf{P}_{\text{new}} = \left( \mathbf{I} - \mathbf{K} \right) \mathbf{P}_{\text{pred}}
$$

---

## 3. Matrix Sizes

| Symbol | Size        | Meaning                    |
|--------|-------------|-----------------------------|
| $\mathbf{x}$ | 3×1 | State (x, y, z)             |
| $\mathbf{P}$ | 3×3 | State covariance            |
| $\mathbf{A}$ | 3×3 | State transition matrix (Identity) |
| $\mathbf{H}$ | 3×3 | Measurement matrix (Identity) |
| $\mathbf{Q}$ | 3×3 | Process noise covariance    |
| $\mathbf{R}$ | 3×3 | Measurement noise covariance |
| $\mathbf{K}$ | 3×3 | Kalman gain matrix           |

---

### 4. Kalman Filter Flowchart

```
Previous State (x, P)
       ↓
  Prediction Step
       ↓
New Measurement (z)
       ↓
   Kalman Gain (K)
       ↓
Update Estimate (x, P)
```

---

### 5. Important Notes

- In this simple setup, the **state transition** assumes the system stays constant unless updated by measurements.
- The **process noise** ( \( \mathbf{Q} \) ) allows for small random changes.
- The **measurement noise** ( \( \mathbf{R} \) ) models sensor uncertainty.
- Extending this to **full IMU** filtering (position, velocity, orientation) would require a more complex \( \mathbf{A} \) and \( \mathbf{H} \) matrices.

---


