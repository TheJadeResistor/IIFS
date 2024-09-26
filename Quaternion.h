#pragma once


class Quaternion {
public:
    // Members for the quaternion components
    float w, x, y, z;

    // Constructors
    Quaternion();  // Default constructor
    Quaternion(float w, float x, float y, float z);  // Parameterized constructor

    // Quaternion Operations
    Quaternion operator*(const Quaternion& q) const;  // Quaternion multiplication
    Quaternion conjugate() const;  // Conjugate of the quaternion
    Quaternion normalize() const;  // Normalize the quaternion
    float magnitude() const;  // Get the magnitude of the quaternion

    // Conversion Functions
    void fromEuler(float roll, float pitch, float yaw);  // Convert Euler angles to quaternion
    void toEuler(float &roll, float &pitch, float &yaw) const;  // Convert quaternion to Euler angles

    // Utility Functions
    static Quaternion fromAxisAngle(float axisX, float axisY, float axisZ, float angle);  // Axis-angle to quaternion
};

