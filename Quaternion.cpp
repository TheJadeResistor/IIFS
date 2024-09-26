#include "Quaternion.h"
#include <math.h>  // For trigonometric and square root functions


// Default constructor initializes to a neutral quaternion (identity rotation)
Quaternion::Quaternion() : w(1), x(0), y(0), z(0) {}

// Parameterized constructor to set the quaternion values
Quaternion::Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

// Quaternion multiplication (this * q)
Quaternion Quaternion::operator*(const Quaternion& q) const {
    return Quaternion(
        w * q.w - x * q.x - y * q.y - z * q.z,  // w component
        w * q.x + x * q.w + y * q.z - z * q.y,  // x component
        w * q.y - x * q.z + y * q.w + z * q.x,  // y component
        w * q.z + x * q.y - y * q.x + z * q.w   // z component
    );
}

// Returns the conjugate of the quaternion
Quaternion Quaternion::conjugate() const {
    return Quaternion(w, -x, -y, -z);
}

// Returns the magnitude of the quaternion
float Quaternion::magnitude() const {
    return sqrt(w * w + x * x + y * y + z * z);
}

// Returns the normalized quaternion (unit quaternion)
Quaternion Quaternion::normalize() const {
    float mag = magnitude();
    if (mag == 0) return Quaternion(1, 0, 0, 0);  // Return identity if magnitude is 0
    return Quaternion(w / mag, x / mag, y / mag, z / mag);
}

// Converts Euler angles (in radians) to a quaternion
void Quaternion::fromEuler(float roll, float pitch, float yaw) {
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    w = cr * cp * cy + sr * sp * sy;
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;
}

// Converts a quaternion to Euler angles (in radians)
void Quaternion::toEuler(float &roll, float &pitch, float &yaw) const {
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);  // Use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
}

// Creates a quaternion from an axis-angle representation
Quaternion Quaternion::fromAxisAngle(float axisX, float axisY, float axisZ, float angle) {
    float halfAngle = angle * 0.5;
    float s = sin(halfAngle);
    return Quaternion(cos(halfAngle), axisX * s, axisY * s, axisZ * s);
}





