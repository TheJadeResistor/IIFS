#pragma once


class Filter {
private:
    // Kalman filter variables for accelerometer
    float accelX_estimate, accelY_estimate, accelZ_estimate;  // Estimate of position
    float accelX_P, accelY_P, accelZ_P;  // Estimate error covariance

    // Kalman filter variables for gyroscope
    float gyroX_estimate, gyroY_estimate, gyroZ_estimate;  // Estimate of angular velocity
    float gyroX_P, gyroY_P, gyroZ_P;  // Estimate error covariance

    // Kalman filter tuning parameters
    float Q;  // Process noise covariance
    float R;  // Measurement noise covariance

public:
    // Constructor: Initialize the filter with default parameters
    Filter(float processNoise = 0.001f, float measurementNoise = 0.1f);

    // Apply Kalman filter for accelerometer data
    float filterAccelX(float measurement);
    float filterAccelY(float measurement);
    float filterAccelZ(float measurement);

    // Apply Kalman filter for gyroscope data
    float filterGyroX(float measurement);
    float filterGyroY(float measurement);
    float filterGyroZ(float measurement);

    void filterData(float ax, float ay, float az, float gx, float gy, float gz);

    // Reset the filter (optional)
    void reset();
};







