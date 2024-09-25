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

#include "Wire.h"

void TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
  Serial.print(bus);
}