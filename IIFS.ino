#include "IIFS.h"


#define NUM_IMUS 2             // Define the number of IMUs
#define TCA_ADDR 0x70          // I2C address of the TCA9548A multiplexer
uint8_t imuAddresses[NUM_IMUS] = { 0x69, 0x69 };  // I2C addresses of the IMUs

// Create an instance of the IIFS class
IIFS iifsSystem(NUM_IMUS, TCA_ADDR, imuAddresses);


void setup() {
    Serial.begin(115200);
    
    iifsSystem.initializeIMUs();  // Initialize all IMUs

    // Print message
    Serial.println("IIFS System Initialized");
}

void loop() {
    iifsSystem.readAndFilterIMUData();   // Read and filter IMU data
    Quaternion fusedQuat = iifsSystem.fuseIMUQuaternions();  // Get fused quaternion

    // Example: Print the fused quaternion values
    Serial.print("Fused Quaternion: ");
    Serial.print(fusedQuat.w); Serial.print(", ");
    Serial.print(fusedQuat.x); Serial.print(", ");
    Serial.print(fusedQuat.y); Serial.print(", ");
    Serial.println(fusedQuat.z);

    // Example: Print the fused quaternion values as Euler
    float roll, pitch, yaw;
    fusedQuat.toEuler(roll, pitch, yaw);
    Serial.print("Fused Euler: ");
    Serial.print(roll); Serial.print(", ");
    Serial.print(pitch); Serial.print(", ");
    Serial.println(yaw);

    delay(1000);
}













