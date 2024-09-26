#pragma once

#include <Wire.h>
#include <Arduino.h>
#include <Quaternion.h>  // Assuming you have a Quaternion class


class IMU {
public:
    // Virtual methods for generic IMU functions
    virtual bool begin() = 0;  // Initialize the IMU
    virtual void readData() = 0;  // Read sensor data
    virtual Quaternion getQuaternion() = 0;  // Get the orientation in quaternion form
    virtual float getRoll() = 0;  // Get roll
    virtual float getPitch() = 0;  // Get pitch
    virtual float getYaw() = 0;  // Get yaw

    virtual ~IMU() {}  // Virtual destructor for proper cleanup
};

class BMI160 : public IMU {
public:
    BMI160(uint8_t address = 0x69);  // Constructor with optional I2C address
    bool begin() override;
    void readData() override;
    Quaternion getQuaternion() override;
    float getRoll() override;
    float getPitch() override;
    float getYaw() override;

private:
    uint8_t _address;  // I2C address
    float accX, accY, accZ;  // Accelerometer readings
    float gyroX, gyroY, gyroZ;  // Gyroscope readings
    void calculateOrientation();  // Helper to calculate quaternion/orientation
    Quaternion currentQuaternion;
};

