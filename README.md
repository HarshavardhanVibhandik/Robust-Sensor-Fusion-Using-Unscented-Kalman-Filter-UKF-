**Robust-Sensor-Fusion-Using-Unscented-Kalman-Filter-UKF-**

# **Project Objective:**

Designed and implemented a robust sensor fusion framework using the Unscented Kalman Filter (UKF) for precise state estimation in dynamic environments. The system integrates noisy and asynchronous sensor data, such as orientation, pose, and velocity, providing accurate and reliable estimates for autonomous robotics and navigation applications.

# **Project Overview:**

State estimation is a critical component for autonomous robots and vehicles, especially when operating in real-world environments with sensor noise and inconsistencies. This project leverages the UKF to address these challenges by predicting system states and refining them using sensor measurements. It is divided into two key tasks:

- **Part 1:** Estimating orientation and pose using IMU and pose measurements.
- **Part 2:** Estimating velocity through a combination of IMU and velocity data.

# **Key Contributions:**

- **Dynamic State Estimation:** Developed a UKF-based process model that predicts position, velocity, and orientation using IMU-derived motion dynamics.
- **Multi-Sensor Fusion:** Integrated asynchronous and noisy sensor data, including IMU acceleration, angular velocity, pose, and velocity measurements, for enhanced robustness.
- **Flexible Modular Design:** Created reusable prediction and update modules that can be adapted to various sensor fusion tasks.
- **Visualization and Error Analysis:** Implemented tools to visualize UKF estimates and compare them with ground truth, allowing for error analysis of position, velocity, and orientation.

# **Methodology:**

**1. Prediction Step:**

- Predicted the system state by propagating it through a non-linear process model using IMU inputs.
- Updated the covariance matrix to account for prediction uncertainties using the Unscented Transform.

**2. Update Step:**

- Refined the predicted state using Kalman gain and sensor measurements (pose, orientation, and velocity).
- Balanced trust between predictions and measurements by computing an optimal Kalman gain to minimize estimation errors.

**3. Unscented Transform:**

- Generated sigma points to propagate non-linear dynamics and maintain estimation accuracy, even for complex and highly dynamic systems.

# **Challenges and Solutions:**

- **Handling Non-Linear Dynamics:** Used the Unscented Transform to accurately handle the non-linearities in motion dynamics and sensor measurements.
- **Noisy Sensor Data:** Applied noise models within the covariance matrices to mitigate the effects of noisy and asynchronous sensor updates.
- **Scalability:** Designed a modular architecture that allows for easy integration of additional sensors, such as cameras or LIDAR.

# **Key Outcomes:**

- Achieved positional accuracy within ±0.1 meters in most scenarios, with minimal orientation error (<1 degree).
- Effectively handled sensor noise and asynchronous updates, ensuring stable and accurate state estimation.
- Demonstrated reliable performance even in dynamic conditions, making the system ideal for real-time robotics applications.

# **Technologies and Tools:**

Matlab

- **State Estimation:** Unscented Kalman Filter (UKF)
- **Programming and Visualization:** MATLAB
- **Sensor Data:** IMU (acceleration, angular velocity), pose, and velocity measurements
- **Data Structures:** Synchronized .mat datasets for testing and evaluation

# **Future Improvements:**

- **Incorporate Visual SLAM:** Add camera-based observations to further enhance spatial awareness.
- **Multi-Sensor Fusion:** Extend the system to integrate additional sensors like LIDAR and GPS for robust outdoor navigation.
- **Real-Time Optimization:** Optimize computational performance to meet the demands of real-time applications in UAVs and mobile robots.

This project delivers a robust and scalable solution for sensor fusion in autonomous systems, paving the way for future applications in robotics research, navigation systems, and dynamic control environments.
