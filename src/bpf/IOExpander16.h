#pragma once

class IOExpander16 {
public:
#ifdef LPF_SWITCHING_DL2MAN_USDX_REV2_BETA
  #define IOEXP16_ADDR  0x74  // PCA9539 with A1..A0 set to 0     https://www.nxp.com/docs/en/data-sheet/PCA9539_PCA9539R.pdf
#endif
#ifdef LPF_SWITCHING_DL2MAN_USDX_REV2
  #define IOEXP16_ADDR  0x24  // TCA/PCA9555 with A2=1 A1..A0=0   https://www.ti.com/lit/ds/symlink/tca9555.pdf
#endif
#ifdef LPF_SWITCHING_DL2MAN_USDX_REV3
  #define IOEXP16_ADDR  0x20  // TCA/PCA9555 with A2=0 A1..A0=0   https://www.ti.com/lit/ds/symlink/tca9555.pdf
#endif
  inline void SendRegister(uint8_t reg, uint8_t val){ i2c.begin(); i2c.beginTransmission(IOEXP16_ADDR); i2c.write(reg); i2c.write(val); i2c.endTransmission(); }
  inline void init(){ write(0U); } //IO0, IO1 as input, IO0 to 0, IO0 as output, IO1 to 0, IO1 as output
  inline void write(uint16_t data){ SendRegister(0x07, 0xff);  SendRegister(0x06, 0xff);/*Common last!*/ SendRegister(0x02, data); SendRegister(0x06, 0x00);/*Common first!*/ SendRegister(0x03, data >> 8); SendRegister(0x07, 0x00); }  // output port cmd: write bits D15-D0 to IO1.7-0.0;
};