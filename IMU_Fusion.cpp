#include "IMU_Fusion.h"


// Constructor: Initialize the array for IMU quaternions
IMU_Fusion::IMU_Fusion(int numIMUs) : numIMUs(numIMUs) {
    imuQuaternions = new Quaternion[numIMUs];
    fusedQuaternion = Quaternion(1, 0, 0, 0);  // Initialize to identity quaternion
}

// Destructor: Clean up the dynamically allocated array
IMU_Fusion::~IMU_Fusion() {
    delete[] imuQuaternions;
}

// Set the quaternion for a specific IMU
void IMU_Fusion::setIMUQuaternion(int imuIndex, const Quaternion& quat) {
    if (imuIndex >= 0 && imuIndex < numIMUs) {
        imuQuaternions[imuIndex] = quat;
    }
}

// Fuse all the IMU quaternions into one quaternion
Quaternion IMU_Fusion::fuseIMUs() {
    fusedQuaternion = Quaternion(1, 0, 0, 0);  // Reset fused quaternion to identity

    // Sum all the quaternions (weighted or simple average can be applied here)
    for (int i = 0; i < numIMUs; i++) {
        fusedQuaternion = fusedQuaternion * imuQuaternions[i];  // Multiply quaternions
    }

    // Normalize the final quaternion to ensure it's a valid rotation
    fusedQuaternion = fusedQuaternion.normalize();

    return fusedQuaternion;
}

// Get the fused quaternion
Quaternion IMU_Fusion::getFusedQuaternion() const {
    return fusedQuaternion;
}

// Reset all IMU quaternions to the identity quaternion
void IMU_Fusion::resetIMUQuaternions() {
    for (int i = 0; i < numIMUs; i++) {
        imuQuaternions[i] = Quaternion(1, 0, 0, 0);  // Identity quaternion
    }
}





