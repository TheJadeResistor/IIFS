#pragma once

#include <Wire.h>  // For I2C communication
#include <vector>
#include <utility>  // For std::pair


class I2C_Manager {
private:
    // TCA9548A I2C multiplexer address
    const uint8_t multiplexerAddress;

    // Current channel being used by the TCA9548A
    uint8_t currentChannel;

    // Vector to store devices mapped to channels: pairs of <deviceAddress, channel>
    std::vector<std::pair<uint8_t, uint8_t>> devices;

    // Private method to switch to a specific channel on the multiplexer
    bool selectChannel(uint8_t channel);

public:
    // Constructor: Initializes the I2C manager with the default multiplexer address
    I2C_Manager(uint8_t multiplexerAddr = 0x70);  // Default address for TCA9548A

    // Initialize the I2C bus
    void begin();

    // Method to select a device connected to a specific channel on the TCA9548A
    bool selectDevice(uint8_t channel);

    // Dynamically add a device with address to a specific channel
    bool addDevice(uint8_t deviceAddress, uint8_t channel);

    // Get the channel associated with a specific device address
    int8_t getDeviceChannel(uint8_t deviceAddress);

    // Get the I2C address associated with a specific device address
    uint8_t getDeviceAddress(uint8_t channel);

    // Read data from the connected I2C device (BMI160 or others)
    uint8_t readByte(uint8_t deviceAddr, uint8_t regAddr);
    void readBytes(uint8_t deviceAddr, uint8_t regAddr, uint8_t* data, uint8_t length);

    // Write data to the connected I2C device
    void writeByte(uint8_t deviceAddr, uint8_t regAddr, uint8_t data);
};








