/**
 * TODO: Extract to a class
 */
#pragma once

/**
 * BPF selection
 **/

#ifdef LPF_SWITCHING_DL2MAN_USDX_REV1
PCA9536 ioext;

void set_latch(uint8_t io){ // reset all latches and set latch k to corresponding GPIO, all relays share a common (ground) GPIO
  #define LATCH_TIME  15   // set/reset time latch relay
  for(int i = 0; i != 8; i++){ ioext.write( (~(1 << i))| 0x01); delay(LATCH_TIME); } ioext.write(0x00); // reset all latches
  ioext.write((1 << io)| 0x00); delay(LATCH_TIME); ioext.write(0x00); // set latch wired to io port
}

static uint8_t prev_lpf_io = 0xff;
inline void set_lpf(uint8_t f){
  uint8_t lpf_io = (f >  8) ? 1 : (f > 4) ? 2 : /*(f <= 4)*/ 3; // cut-off freq in MHz to IO port of LPF relay
  if(prev_lpf_io != lpf_io){ prev_lpf_io = lpf_io; set_latch(lpf_io); };  // set relay
}
#endif  //LPF_SWITCHING_DL2MAN_USDX_REV1

#if defined(LPF_SWITCHING_DL2MAN_USDX_REV3) || defined(LPF_SWITCHING_DL2MAN_USDX_REV2) || defined(LPF_SWITCHING_DL2MAN_USDX_REV2_BETA)
IOExpander16 ioext;
enum gpioext_t { IO0_0, IO0_1, IO0_2, IO0_3, IO0_4, IO0_5, IO0_6, IO0_7, IO1_0, IO1_1, IO1_2, IO1_3, IO1_4, IO1_5, IO1_6, IO1_7 };

void set_latch(uint8_t io, uint8_t common_io, bool latch = true){ // reset all latches and set latch k to corresponding GPIO, all relays share a common (ground) GPIO
  #define LATCH_TIME  30   // set/reset time latch relay
  if(latch){
    ioext.write((1U << io)| 0x0000); delay(LATCH_TIME); ioext.write(0x0000); // set latch wired to io port
  } else {
    if(io == 0xff){ ioext.init(); for(int io = 0; io != 16; io++) set_latch(io, common_io, latch); } // reset all latches
    else { ioext.write( (~(1U << io))| (1U << common_io)); delay(LATCH_TIME); ioext.write(0x0000); } // reset latch wired to io port
  }
}

static uint8_t prev_lpf_io = 0xff; // inits and resets all latches
inline void set_lpf(uint8_t f){
#ifdef LPF_SWITCHING_DL2MAN_USDX_REV3
  uint8_t lpf_io = (f > 26) ? IO1_3 : (f > 20) ? IO1_4 : (f > 17) ? IO1_2 : (f > 12) ? IO1_5 : (f > 8) ? IO1_1 : (f > 5) ? IO1_6 : (f > 4) ? IO1_0 : /*(f <= 4)*/ IO1_7; // cut-off freq in MHz to IO port of LPF relay
#ifndef LPF_SWITCHING_DL2MAN_USDX_REV3_NOLATCH
  if(prev_lpf_io != lpf_io){ set_latch(prev_lpf_io, IO0_0, false); set_latch(lpf_io, IO0_0); prev_lpf_io = lpf_io; };  // set relay (latched)
#else
  if(prev_lpf_io != lpf_io){ ioext.write(1U << lpf_io); prev_lpf_io = lpf_io; };  // set relay (non-latched)
#endif //LPF_SWITCHING_DL2MAN_USDX_REV3_NOLATCH
#else //LPF_SWITCHING_DL2MAN_USDX_REV2 LPF_SWITCHING_DL2MAN_USDX_REV2_BETA
  uint8_t lpf_io = (f > 12) ? IO0_3 : (f > 8) ? IO0_5 : (f > 5) ? IO0_7 : (f > 4) ? IO1_1 : /*(f <= 4)*/ IO1_3; // cut-off freq in MHz to IO port of LPF relay
  if(prev_lpf_io != lpf_io){ set_latch(prev_lpf_io, IO0_1, false); set_latch(lpf_io, IO0_1); prev_lpf_io = lpf_io; };  // set relay

#endif
}
#endif  //LPF_SWITCHING_DL2MAN_USDX_REV3 LPF_SWITCHING_DL2MAN_USDX_REV2 REV2_BETA

#ifdef LPF_SWITCHING_WB2CBA_USDX_OCTOBAND
MCP23008 ioext;

static uint8_t prev_lpf_io = 0xff; // inits and resets all latches
inline void set_lpf(uint8_t f){
  uint8_t lpf_io = (f > 26) ? 7 : (f > 20) ? 6 : (f > 17) ? 5 : (f > 12) ? 4 : (f > 8) ? 3 : (f > 6) ? 2 : (f > 4) ? 1 : /*(f <= 4)*/ 0; // cut-off freq in MHz to IO port of LPF relay
  if(prev_lpf_io == 0xff){ ioext.init(); }
  if(prev_lpf_io != lpf_io){ ioext.write(1U << lpf_io); prev_lpf_io = lpf_io; };  // set relay (non-latched)
}
#endif  //LPF_SWITCHING_WB2CBA_USDX_OCTOBAND