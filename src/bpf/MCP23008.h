#pragma once

class MCP23008 {
public:
#define MCP23008_ADDR  0x20  // MCP23008 with A1..A0 set to 0   https://ww1.microchip.com/downloads/en/DeviceDoc/21919e.pdf
  inline void SendRegister(uint8_t reg, uint8_t val){ i2c.begin(); i2c.beginTransmission(MCP23008_ADDR); i2c.write(reg); i2c.write(val); i2c.endTransmission(); }
  inline void init(){ SendRegister(0x09, 0x00); SendRegister(0x00, 0x00); } //GP0-7 to 0, GP0-7 as output
  inline void write(uint16_t data){ SendRegister(0x09, data); }  // output port cmd: write bits D7-D0 to GP7-GP0
};