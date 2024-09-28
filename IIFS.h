#pragma once

#include "I2C_Manager.h"
#include "IMU.h"
#include "Filter.h"
#include "Quaternion.h"
#include "IMU_Fusion.h"

class IIFS {
private:
    I2C_Manager i2cManager;          // I2C Manager object
    IMU* imuDevices;                 // Array for IMU devices (dynamically allocated)
    Filter* filters;                 // Array of filters, one per IMU
    Quaternion* quaternions;         // Quaternions for each IMU
    IMU_Fusion imuFusion;            // IMU Fusion object
    Quaternion fusedQuaternion;      // Fused quaternion
    uint8_t numIMUs;                 // Number of IMUs
    uint8_t* imuAddresses;           // Array to store the I2C addresses for each IMU

public:
    // Constructor with number of IMUs and their addresses
    IIFS(uint8_t numIMUs, uint8_t tcaAddress, uint8_t* imuAddresses);

    // Destructor to clean up dynamically allocated arrays
    ~IIFS();

    // Method to initialize all IMUs
    void initializeIMUs();

    // Method to read and filter data from all IMUs
    void readAndFilterIMUData();

    // Method to fuse quaternions from multiple IMUs
    Quaternion fuseIMUQuaternions();
    
    // Getter for fused quaternion
    Quaternion getFusedQuaternion() const;
};








