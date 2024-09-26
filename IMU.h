#pragma once

#include <Wire.h>
#include <Arduino.h>
#include <BMI160Gen.h>  // Use the BMI160 library
#include "Quaternion.h"
#include "I2C_Manager.h"


class IMU {
public:
    IMU(uint8_t address = 0x68);  // Constructor with default I2C address
    ~IMU();
    bool begin(I2C_Manager i2cManager, int deviceChannel);
    void readData();
    Quaternion getQuaternion();
    void getEulerAngles(float roll, float pitch, float yaw);

    float accX, accY, accZ;  // Accelerometer readings
    float gyroX, gyroY, gyroZ;  // Gyroscope readings

private:
    uint8_t _address;  // I2C address
    void calculateOrientation();  // Helper to calculate quaternion/orientation
    Quaternion currentQuaternion;
};





