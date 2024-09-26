#pragma once

#include "Quaternion.h"  // Include the Quaternion class


class IMU_Fusion {
private:
    Quaternion *imuQuaternions;  // Array of quaternions from each IMU
    int numIMUs;  // Number of IMUs being fused
    Quaternion fusedQuaternion;  // Resultant fused quaternion

public:
    // Constructor
    IMU_Fusion(int numIMUs);

    // Destructor
    ~IMU_Fusion();

    // Set the quaternion for a specific IMU
    void setIMUQuaternion(int imuIndex, const Quaternion& quat);

    // Fuse all the IMU quaternions into one quaternion
    Quaternion fuseIMUs();

    // Get the final fused quaternion
    Quaternion getFusedQuaternion() const;

    // Reset all IMU quaternions to the identity quaternion
    void resetIMUQuaternions();
};






