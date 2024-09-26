#include "I2C_Manager.h"
#include "IMU.h"
#include "Filter.h"
#include "Quaternion.h"
#include "IMU_Fusion.h"

// Constants for device addresses
#define IMU_ADDR 0x68   // Default BMI160 address for all IMUs
#define TCA_ADDR 0x70   // Default TCA9548A multiplexer address

// Number of IMUs
#define NUM_IMUS 3

// Declare I2C manager object
I2C_Manager i2cManager(TCA_ADDR);

// Declare IMU objects (for multiple IMUs)
IMU imuDevices[NUM_IMUS];

// Declare quaternion objects for each IMU
Quaternion quaternions[NUM_IMUS];

// Declare filter objects (one per IMU)
Filter filters[NUM_IMUS];

// Declare IMU fusion object
IMU_Fusion imuFusion(NUM_IMUS);

// Array to store fused quaternion
Quaternion fusedQuaternion;

// Function to initialize the IMU devices
void initializeIMUs() {
    // Start the I2C manager (multiplexer)
    i2cManager.begin();

    // Initialize each IMU on a specific channel
    for (uint8_t i = 0; i < NUM_IMUS; i++) {
        i2cManager.addDevice(IMU_ADDR, i);  // Add each IMU to a specific channel on TCA9548A
        
        // Select the IMU channel and initialize the IMU
        if (i2cManager.selectDevice(i)) {
            imuDevices[i].begin(i2cManager, i);  // Initialize IMU with I2C manager
        }
    }
}

// Function to read and filter raw IMU data
void readAndFilterIMUData() {
    for (uint8_t i = 0; i < NUM_IMUS; i++) {
        // Select the IMU channel
        i2cManager.selectDevice(i);

        // Read raw sensor data (accelerometer, gyroscope)
        imuDevices[i].readData();

        // Apply filtering (e.g., Kalman filter)
        filters[i].filterData(imuDevices[i].accX, imuDevices[i].accY, imuDevices[i].accZ, imuDevices[i].gyroX, imuDevices[i].gyroY, imuDevices[i].gyroZ);

        // Convert filtered data into quaternion
        quaternions[i] = imuDevices[i].getQuaternion();
    }
}

// Function to fuse the quaternions from multiple IMUs into one
void fuseIMUQuaternions() {
    // Fuse all quaternions together
    imuFusion.fuseIMUs(quaternions, NUM_IMUS, fusedQuaternion);
}

void setup() {
    Serial.begin(115200);
    
    // Initialize all IMUs
    initializeIMUs();

    // Print message
    Serial.println("IIFS System Initialized");
}

void loop() {
    // Read and filter data from all IMUs
    readAndFilterIMUData();

    // Fuse the quaternion data into one fused quaternion
    fuseIMUQuaternions();

    // Use the fused quaternion (for balancing, control, etc.)
    // Example: print the fused quaternion for debugging
    Serial.print("Fused Quaternion: ");
    Serial.print(fusedQuaternion.w); Serial.print(", ");
    Serial.print(fusedQuaternion.x); Serial.print(", ");
    Serial.print(fusedQuaternion.y); Serial.print(", ");
    Serial.println(fusedQuaternion.z);

    delay(100);
}



