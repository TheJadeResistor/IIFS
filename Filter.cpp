#include "Filter.h"


// Constructor: Initialize Kalman filter with process noise Q and measurement noise R
Filter::Filter(float processNoise, float measurementNoise) {
    // Initialize estimates and error covariance for both accelerometer and gyroscope
    accelX_estimate = accelY_estimate = accelZ_estimate = 0.0f;
    gyroX_estimate = gyroY_estimate = gyroZ_estimate = 0.0f;

    accelX_P = accelY_P = accelZ_P = 1.0f;
    gyroX_P = gyroY_P = gyroZ_P = 1.0f;

    Q = processNoise;  // Process noise covariance
    R = measurementNoise;  // Measurement noise covariance
}

// Kalman filter for accelerometer X axis
float Filter::filterAccelX(float measurement) {
    // Prediction step
    accelX_P = accelX_P + Q;

    // Measurement update
    float K = accelX_P / (accelX_P + R);  // Kalman gain
    accelX_estimate = accelX_estimate + K * (measurement - accelX_estimate);
    accelX_P = (1 - K) * accelX_P;

    return accelX_estimate;
}

// Kalman filter for accelerometer Y axis
float Filter::filterAccelY(float measurement) {
    accelY_P = accelY_P + Q;
    float K = accelY_P / (accelY_P + R);
    accelY_estimate = accelY_estimate + K * (measurement - accelY_estimate);
    accelY_P = (1 - K) * accelY_P;

    return accelY_estimate;
}

// Kalman filter for accelerometer Z axis
float Filter::filterAccelZ(float measurement) {
    accelZ_P = accelZ_P + Q;
    float K = accelZ_P / (accelZ_P + R);
    accelZ_estimate = accelZ_estimate + K * (measurement - accelZ_estimate);
    accelZ_P = (1 - K) * accelZ_P;

    return accelZ_estimate;
}

// Kalman filter for gyroscope X axis
float Filter::filterGyroX(float measurement) {
    gyroX_P = gyroX_P + Q;
    float K = gyroX_P / (gyroX_P + R);
    gyroX_estimate = gyroX_estimate + K * (measurement - gyroX_estimate);
    gyroX_P = (1 - K) * gyroX_P;

    return gyroX_estimate;
}

// Kalman filter for gyroscope Y axis
float Filter::filterGyroY(float measurement) {
    gyroY_P = gyroY_P + Q;
    float K = gyroY_P / (gyroY_P + R);
    gyroY_estimate = gyroY_estimate + K * (measurement - gyroY_estimate);
    gyroY_P = (1 - K) * gyroY_P;

    return gyroY_estimate;
}

// Kalman filter for gyroscope Z axis
float Filter::filterGyroZ(float measurement) {
    gyroZ_P = gyroZ_P + Q;
    float K = gyroZ_P / (gyroZ_P + R);
    gyroZ_estimate = gyroZ_estimate + K * (measurement - gyroZ_estimate);
    gyroZ_P = (1 - K) * gyroZ_P;

    return gyroZ_estimate;
}

// Optional: Reset the Kalman filter estimates and covariances
void Filter::reset() {
    accelX_estimate = accelY_estimate = accelZ_estimate = 0.0f;
    gyroX_estimate = gyroY_estimate = gyroZ_estimate = 0.0f;

    accelX_P = accelY_P = accelZ_P = 1.0f;
    gyroX_P = gyroY_P = gyroZ_P = 1.0f;
}







