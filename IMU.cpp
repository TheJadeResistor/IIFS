#include "IMU.h"
#include <Wire.h>  // For I2C communication
#include <BMI160Gen.h>  // Assuming you use the BMI160 library


BMI160::BMI160(uint8_t address) : _address(address), accX(0), accY(0), accZ(0), gyroX(0), gyroY(0), gyroZ(0) {}

bool BMI160::begin() {
    Wire.begin();
    if (!BMI160.begin(_address)) {
        Serial.println("Failed to initialize BMI160!");
        return false;
    }
    Serial.println("BMI160 Initialized");
    return true;
}

void BMI160::readData() {
    // Read accelerometer and gyroscope data
    BMI160.readAcceleration(accX, accY, accZ);
    BMI160.readGyroscope(gyroX, gyroY, gyroZ);
    
    // Calculate orientation based on new data
    calculateOrientation();
}

void BMI160::calculateOrientation() {
    // Calculate the quaternion (this is a simplified version; use sensor fusion in real systems)
    float roll = atan2(accY, accZ);
    float pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ));
    float yaw = atan2(gyroZ, sqrt(gyroX * gyroX + gyroY * gyroY));

    // Convert to quaternion (this is a basic example)
    currentQuaternion = Quaternion(roll, pitch, yaw);  // Replace this with actual quaternion calculation
}

Quaternion BMI160::getQuaternion() {
    return currentQuaternion;
}

float BMI160::getRoll() {
    return currentQuaternion.getRoll();
}

float BMI160::getPitch() {
    return currentQuaternion.getPitch();
}

float BMI160::getYaw() {
    return currentQuaternion.getYaw();
}




