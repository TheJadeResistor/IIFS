#include "IMU.h"


IMU::IMU(uint8_t address) : _address(address), accX(0), accY(0), accZ(0), gyroX(0), gyroY(0), gyroZ(0) {}

IMU::~IMU() {}

bool IMU::begin(I2C_Manager i2cManager, int deviceChannel) {
    if ( !BMI160.begin(BMI160GenClass::I2C_MODE, i2cManager.getDeviceAddress(deviceChannel)) ) { 
        Serial.println("Failed to initialize BMI160!");
        return false;
    }
    Serial.println("BMI160 Initialized");
    return true;
}

void IMU::readData() {
    int aX, aY, aZ, gX, gY, gZ; //temp vars

    // Read accelerometer and gyroscope data
    BMI160.readAccelerometer(aX, aY, aZ);
    BMI160.readGyro(gX, gY, gZ);
    accX = static_cast<float>(aX); accY = static_cast<float>(aY); accZ = static_cast<float>(aZ); 
    gyroX = static_cast<float>(gX); gyroY = static_cast<float>(gY); gyroZ = static_cast<float>(gZ); 
}

void IMU::calculateOrientation() {
    // Calculate the quaternion (this is a simplified version; use sensor fusion in real systems)
    float roll = atan2(accY, accZ);
    float pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ));
    float yaw = atan2(gyroZ, sqrt(gyroX * gyroX + gyroY * gyroY));

    // Convert to quaternion
    currentQuaternion.fromEuler(roll, pitch, yaw);
}

Quaternion IMU::getQuaternion() {
    calculateOrientation();
    return currentQuaternion;
}

void IMU::getEulerAngles(float roll, float pitch, float yaw) {
    currentQuaternion.toEuler(roll, pitch, yaw);
}






