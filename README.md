# IIFS

The Integrated IMU Fusion System (IIFS) library allows users to fuse multiple IMU sensors and convert their data into quaternions. This enables dynamic robot balancing and real-time sensor fusion for control systems.

## Features:
- Supports BMI160 IMUs and other compatible devices.
- Automatically selects devices via I2C multiplexing (e.g., TCA9548A).
- Applies filtering using Kalman filters for smooth sensor data.
- Converts raw IMU data to quaternions and fuses them to represent the robot's orientation.
