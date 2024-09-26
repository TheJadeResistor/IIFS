//class for managing I2C multiplexing (TCA9548A) and I2C added devices (BMI160)
//connect TCA9548A A0,A1,A2 to GND for I2C address 0x70
//A0 A1 A2 : address
//0 0 0 : 0x70
//1 0 0 :0x71
//0 1 0 :0x72
//1 1 0 :0x73
//0 0 1 :0x74
//1 0 1 :0x75
//0 1 1 :0x76
//1 1 1 :0x77

#include "I2C_Manager.h"

#define I2C_SDA 21
#define I2C_SCL 22

// Constructor to initialize the multiplexer address
I2C_Manager::I2C_Manager(uint8_t multiplexerAddr) : multiplexerAddress(multiplexerAddr), currentChannel(0xFF) {
}

// Initialize the I2C bus
void I2C_Manager::begin() {
    Wire.begin(I2C_SDA, I2C_SCL);  // Initialize I2C bus
}

// Private method to switch to a specific channel on the TCA9548A
bool I2C_Manager::selectChannel(uint8_t channel) {
    if (channel > 7 || channel == currentChannel) {
        return false;  // Invalid channel or already selected
    }

    // Write to the TCA9548A to select the desired channel
    Wire.beginTransmission(multiplexerAddress);
    Wire.write(1 << channel);  // Select the channel by setting the corresponding bit
    if (Wire.endTransmission() == 0) {
        currentChannel = channel;  // Update the current channel if successful
        return true;
    }
    return false;  // Failed to select channel
}

// Public method to select a device on a specific channel
bool I2C_Manager::selectDevice(uint8_t channel) {
    return selectChannel(channel);  // Call the private method to switch channels
}

// Dynamically add a device with an address to a specific channel
bool I2C_Manager::addDevice(uint8_t deviceAddress, uint8_t channel) {
    if (channel > 7) return false;  // Invalid channel
    devices.push_back({deviceAddress, channel});  // Add the device to the list
    return true;
}

// Get the channel associated with a specific device address
int8_t I2C_Manager::getDeviceChannel(uint8_t deviceAddress) {
    for (auto &device : devices) {
        if (device.first == deviceAddress) {
            return device.second;  // Return the channel number
        }
    }
    return -1;  // Device not found
}

// Get the I2C address associated with a specific device address
uint8_t I2C_Manager::getDeviceAddress(uint8_t channel){
    for (auto &device : devices) {
        if (device.second == channel) {
            return device.first;  // Return the device address
        }
    }
    return -1;  // Device not found
}

// Method to read a single byte from a specific register of an I2C device
uint8_t I2C_Manager::readByte(uint8_t deviceAddr, uint8_t regAddr) {
    int8_t channel = getDeviceChannel(deviceAddr);  // Get the channel for this device
    if (channel == -1 || !selectDevice(channel)) {  // Invalid channel or failed to select it
      Serial.println("Error selecting multiplexer channel: readByte");
      return 0;
    }

    Wire.beginTransmission(deviceAddr);
    Wire.write(regAddr);  // Specify the register to read
    if (Wire.endTransmission(false) != 0) {  // Send register and check for errors and keep the connection open
        return 0;  // Transmission failed
    }
    Wire.requestFrom(deviceAddr, (uint8_t)1);  // Request one byte of data
    return Wire.available() ? Wire.read() : 0;  // Return the byte received, or 0 if nothing is received
}

// Method to read multiple bytes from a specific register of an I2C device
void I2C_Manager::readBytes(uint8_t deviceAddr, uint8_t regAddr, uint8_t* data, uint8_t length) {
    int8_t channel = getDeviceChannel(deviceAddr);  // Get the channel for this device
    if (channel == -1 || !selectDevice(channel)) {  // Invalid channel or failed to select it
      Serial.println("Error selecting multiplexer channel: readBytes");
    }

    Wire.beginTransmission(deviceAddr);
    Wire.write(regAddr);  // Specify the register to start reading from
    Wire.endTransmission(false); // Send register and check for errors and keep the connection open
    Wire.requestFrom(deviceAddr, length);  // Request multiple bytes
    for (uint8_t i = 0; i < length && Wire.available(); i++) {
        data[i] = Wire.read();  // Read the data into the array
    }
}

// Method to write a single byte to a specific register of an I2C device
void I2C_Manager::writeByte(uint8_t deviceAddr, uint8_t regAddr, uint8_t data) {
    int8_t channel = getDeviceChannel(deviceAddr);  // Get the channel for this device
    if (channel == -1 || !selectDevice(channel)) {  // Invalid channel or failed to select it
      Serial.println("Error selecting multiplexer channel: writeByte");
    }

    Wire.beginTransmission(deviceAddr);
    Wire.write(regAddr);  // Specify the register to write to
    Wire.write(data);  // Send the data
    Wire.endTransmission();
}






