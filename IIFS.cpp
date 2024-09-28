#include "IIFS.h"


// Constructor: Initializes the I2C Manager and IMU Fusion objects
IIFS::IIFS(uint8_t numIMUs, uint8_t tcaAddress, uint8_t* imuAddresses)
    : i2cManager(tcaAddress), imuFusion(numIMUs), numIMUs(numIMUs) {
    
    // Dynamically allocate memory for IMU devices, filters, and quaternions
    imuDevices = new IMU[numIMUs];
    filters = new Filter[numIMUs];
    quaternions = new Quaternion[numIMUs];
    
    // Store the IMU addresses
    this->imuAddresses = new uint8_t[numIMUs];
    for (uint8_t i = 0; i < numIMUs; i++) {
        this->imuAddresses[i] = imuAddresses[i];
    }
}

// Destructor: Clean up dynamically allocated memory
IIFS::~IIFS() {
    delete[] imuDevices;
    delete[] filters;
    delete[] quaternions;
    delete[] imuAddresses;
}

// Initialize IMUs
void IIFS::initializeIMUs() {
    i2cManager.begin();  // Start the I2C manager

    // Initialize each IMU on a specific channel
    for (uint8_t i = 0; i < numIMUs; i++) {
        i2cManager.addDevice(imuAddresses[i], i);  // Add IMU to the multiplexer
        if (i2cManager.selectDevice(i)) {
            imuDevices[i].begin(i2cManager, i);  // Initialize IMU with I2C manager
        }
    }
}

// Read and filter raw data from all IMUs
void IIFS::readAndFilterIMUData() {
    for (uint8_t i = 0; i < numIMUs; i++) {
        i2cManager.selectDevice(i);  // Select the IMU channel
        imuDevices[i].readData();    // Read IMU data

        // Apply filtering to IMU data
        filters[i].filterData(imuDevices[i].accX, imuDevices[i].accY, imuDevices[i].accZ, 
                              imuDevices[i].gyroX, imuDevices[i].gyroY, imuDevices[i].gyroZ);

        // Convert filtered data into quaternion
        quaternions[i] = imuDevices[i].getQuaternion();

        // Add quaternion to IMU Fusion system
        imuFusion.setIMUQuaternion(i, quaternions[i]);
    }
}

// Fuse quaternions from multiple IMUs into one
Quaternion IIFS::fuseIMUQuaternions() {
    fusedQuaternion = imuFusion.fuseIMUs();  // Fuse all quaternions
    return fusedQuaternion;
}

// Return the fused quaternion
Quaternion IIFS::getFusedQuaternion() const {
    return fusedQuaternion;
}








