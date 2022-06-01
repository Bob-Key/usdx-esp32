#pragma once

class PCA9536 {
public:
  #define PCA9536_ADDR  0x41  // PCA9536   https://www.ti.com/lit/ds/symlink/pca9536.pdf
  inline void SendRegister(uint8_t reg, uint8_t val){ i2c.begin(); i2c.beginTransmission(PCA9536_ADDR); i2c.write(reg); i2c.write(val); i2c.endTransmission(); }
  inline void init(){ SendRegister(0x03, 0x00); } // configuration cmd: IO0-IO7 as output
  inline void write(uint8_t data){ init(); SendRegister(0x01, data); }  // output port cmd: write bits D7-D0 to IO7-IO0
};