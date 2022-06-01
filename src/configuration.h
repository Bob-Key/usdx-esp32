#pragma once

#define VERSION   "0.01"

// Configuration switches; remove/add a double-slash at line-start to enable/disable a feature; to save space disable e.g. CAT, DIAG, KEYER
#define DIAG             1   // Hardware diagnostics on startup (only disable when your rig is working)
#define KEYER            1   // CW keyer
#define CAT              1   // CAT-interface
#define F_XTAL    27005000   // 27MHz SI5351 crystal
//#define F_XTAL  25004000   // 25MHz SI5351 crystal  (enable for WB2CBA-uSDX, SI5351 break-out board or uSDXDuO)
//#define F_XTAL  25000000   // 25MHz SI5351 crystal  (enable for 25MHz TCXO)
//#define SWAP_ROTARY    1   // Swap rotary direction (enable for WB2CBA-uSDX)
//#define QCX            1   // Supports older (non-SDR) QCX HW modifications (QCX, QCX-SSB, QCX-DSP with I/Q alignment-feature)
//#define OLED_SSD1306   1   // OLED display (SSD1306 128x32 or 128x64), connect SDA (PD2), SCL (PD3)
//#define OLED_SH1106    1   // OLED display (SH1106 1.3" inch display), connect SDA (PD2), SCL (PD3), NOTE that this display is pretty slow
//#define LCD_I2C        1   // LCD with I2C (PCF8574 module          ), connect SDA (PD2), SCL (PD3), NOTE that this display is pretty slow
#define LPF_SWITCHING_DL2MAN_USDX_REV3           1   // Enable 8-band filter bank switching:     latching relays wired to a TCA/PCA9555 GPIO extender on the PC4/PC5 I2C bus; relays are using IO0.0 as common (ground), IO1.0..7 used by the individual latches K0-7 switching respectively LPFs for 10m, 15m, 17m, 20m, 30m, 40m, 60m, 80m
//#define LPF_SWITCHING_DL2MAN_USDX_REV3_NOLATCH 1   // Enable 8-band filter bank switching: non-latching relays wired to a TCA/PCA9555 GPIO extender on the PC4/PC5 I2C bus; relays are using IO0.0 as common (ground), IO1.0..7 used by the individual latches K0-7 switching respectively LPFs for 10m, 15m, 17m, 20m, 30m, 40m, 60m, 80m. Enable this if you are using 8-band non-latching version for the relays, the radio will draw extra 15mA current but will work ity any relay (Tnx OH2UDS/TA7W Baris)
//#define LPF_SWITCHING_DL2MAN_USDX_REV2         1   // Enable 5-band filter bank switching:     latching relays wired to a TCA/PCA9555 GPIO extender on the PC4/PC5 I2C bus; relays are using IO0.1 as common (ground), IO0.3, IO0.5, IO0.7, IO1.1, IO1.3 used by the individual latches K1-5 switching respectively LPFs for 20m, 30m, 40m, 60m, 80m
//#define LPF_SWITCHING_DL2MAN_USDX_REV2_BETA    1   // Enable 5-band filter bank switching:     latching relays wired to a PCA9539PW   GPIO extender on the PC4/PC5 I2C bus; relays are using IO0.1 as common (ground), IO0.3, IO0.5, IO0.7, IO1.1, IO1.3 used by the individual latches K1-5 switching respectively LPFs for 20m, 30m, 40m, 60m, 80m
//#define LPF_SWITCHING_DL2MAN_USDX_REV1         1   // Enable 3-band filter bank switching:     latching relays wired to a PCA9536D    GPIO extender on the PC4/PC5 I2C bus; relays are using IO0 as common (ground), IO1-IO3 used by the individual latches K1-3 switching respectively LPFs for 20m, 40m, 80m
//#define LPF_SWITCHING_WB2CBA_USDX_OCTOBAND     1   // Enable 8-band filter bank switching: non-latching relays wired to a MCP23008    GPIO extender on the PC4/PC5 I2C bus; relays are using GND as common (ground), GP0..7 used by the individual latches K1-8 switching respectively LPFs for 80m, 60m, 40m, 30m, 20m, 17m, 15m, 10m
//#define LPF_SWITCHING_PE1DDA_USDXDUO           14  // Enable 2-band filter bank switching: non-latching relay  wired to pin PD5 (pin 11); specify as value the frequency in MHz for which (and above) the relay should be altered (e.g. put 14 to enable the relay at 14MHz and above to use the 20m LPF).
#define SI5351_ADDR   0x60   // SI5351A I2C address: 0x60 for SI5351A-B-GT, Si5351A-B04771-GT, MS5351M; 0x62 for SI5351A-B-04486-GT; 0x6F for SI5351A-B02075-GT; see here for other variants: https://www.silabs.com/TimingUtility/timing-download-document.aspx?OPN=Si5351A-B02075-GT&OPNRevision=0&FileType=PublicAddendum
//#define F_MCU   16000000   // 16MHz ATMEGA328P crystal (enable for unmodified Arduino Uno/Nano boards with 16MHz crystal). You may change this value to any other crystal frequency (up to 28MHz may work)

// Advanced configuration switches
//#define CONDENSED      1   // Display in 4 line mode (for OLED and LCD2004 modules)
//#define CAT_EXT        1   // Extended CAT support: remote button and screen control commands over CAT
//#define CAT_STREAMING  1   // Extended CAT support: audio streaming over CAT, once enabled and triggered with CAT cmd, samplerate 7812Hz, 8-bit unsigned audio is sent over UART. The ";" is omited in the data-stream, and only sent to indicate the beginning and end of a CAT cmd.
#define CW_DECODER       1   // CW decoder
#define TX_ENABLE        1   // Disable this for RX only (no transmit), e.g. to support uSDX for kids idea: https://groups.io/g/ucx/topic/81030243#6276
#define KEY_CLICK        1   // Reduce key clicks by envelope shaping
#define SEMI_QSK         1   // Just after keying the transmitter, keeps the RX muted for a short amount of time in the anticipation for continued keying
#define RIT_ENABLE       1   // Receive-In-Transit alternates the receiving frequency with an user-defined offset to compensate for any necessary tuning needed on receive
#define VOX_ENABLE       1   // Voice-On-Xmit which is switching the transceiver into transmit as soon audio is detected (above noise gate level)
//#define MOX_ENABLE     1   // Monitor-On-Xmit which is audio monitoring on speaker during transmit
//#define FAST_AGC       1   // Adds fast AGC option (good for CW)
//#define VSS_METER      1   // Supports Vss measurement (as s-meter option), requires resistor of 1M between 12V and pin 26 (PC3)
//#define SWR_METER      1   // Supports SWR meter with bridge on A6/A7 (LQPF ATMEGA328P) by Alain, K1FM, see: https://groups.io/g/ucx/message/6262 and https://groups.io/g/ucx/message/6361
//#define ONEBUTTON      1   // Use single (encoder) button to control full the rig; optionally use L/R buttons to completely replace rotory encoder function
//#define DEBUG          1   // for development purposes only (adds debugging features such as CPU, sample-rate measurement, additional parameters)
//#define TESTBENCH      1   // Tests RX chain by injection of sine wave, measurements results are sent over serial
//#define CW_FREQS_QRP   1   // Defaults to CW QRP   frequencies when changing bands
//#define CW_FREQS_FISTS 1   // Defaults to CW FISTS frequencies when changing bands
#define CW_MESSAGE       1   // Transmits pre-defined CW messages on-demand (left-click menu item 4.2)
//#define CW_MESSAGE_EXT 1   // Additional CW messages
//#define TX_DELAY       1   // Enables a delay in the actual transmission to allow relay-switching to be completed before the power is applied (see also NTX, PTX definitions below for GPIO that can switch relay/PA)
//#define NTX            11  // Enables LOW  on TX, used as PTT out to enable external PAs (a value of 11 means PB3 is used)
//#define PTX            11  // Enables HIGH on TX, used as PTT out to enable external PAs (a value of 11 means PB3 is used)
//#define CLOCK          1   // Enables clock
#define CW_INTERMEDIATE  1   // CW decoder shows intermediate characters (only available for LCD and F_MCU at 20M), sequences like:  EIS[HV] EIUF EAW[JP] EARL TMO TMG[ZQ] TND[BX] TNK[YC], may be good to learn CW; a full list of possible sequences:  EISH5 EISV3 EIUF EIUU2 EAWJ1 EAWP EARL TMOO0 TMOO9 TMOO8 TMGZ7 TMGQ TNDB6 TNDX TNKY TNKC
//#define F_XTAL  20000000   // Enable this for uSDXDuO, 20MHz SI5351 crystal
//#define TX_CLK0_CLK1   1   // Enable this for uSDXDuO, i.e. when PA is driven by CLK0, CLK1 (not CLK2); NTX pin may be used for enabling the TX path (this is like RX pin, except that RX may also be used as attenuator)
//#define F_CLK2  12000000   // Enables a fixed CLK2 clock output of choice (only applicable when TX_CLK0_CLK1 is enabled), e.g. for up-converter or to clock UART USB device

// QCX pin defintions
// OLED
#define LCD_D4  0         //PD0    (pin 2)
#define LCD_D5  1         //PD1    (pin 3)
#define LCD_D6  2         //PD2    (pin 4)
#define LCD_D7  3         //PD3    (pin 5)
#define LCD_EN  4         //PD4    (pin 6)

// 
#define FREQCNT 5         //PD5    (pin 11)

// rotary encoder
#define ROT_A   6         //PD6    (pin 12)
#define ROT_B   7         //PD7    (pin 13)


#define RX      8         //PB0    (pin 14)
#define SIDETONE 9        //PB1    (pin 15)
#define KEY_OUT 10        //PB2    (pin 16)
#define SIG_OUT 11        //PB3    (pin 17)
#define DAH     12        //PB4    (pin 18)
#define DIT     13        //PB5    (pin 19)
#define AUDIO1  14        //PC0/A0 (pin 23)
#define AUDIO2  15        //PC1/A1 (pin 24)
#define DVM     16        //PC2/A2 (pin 25)
#define BUTTONS 17        //PC3/A3 (pin 26)
#define LCD_RS  18        //PC4    (pin 27)
#define SDA     18        //PC4    (pin 27)
#define SCL     19        //PC5    (pin 28)
//#define NTX   11        //PB3    (pin 17)
//#define PTX   11        //PB3    (pin 17)

#ifdef SWAP_ROTARY
#undef ROT_A
#undef ROT_B
#define ROT_A   7         //PD7    (pin 13)
#define ROT_B   6         //PD6    (pin 12)
#endif

#if (defined(OLED_SSD1306) || defined(OLED_SH1106))
#define OLED    1
#endif

#if (defined(CAT) || defined(TESTBENCH)) && !(OLED)
#define _SERIAL  1       // Coexistence support for serial port and LCD on the same pins
#endif

#ifdef LPF_SWITCHING_DL2MAN_USDX_REV3_NOLATCH
#define LPF_SWITCHING_DL2MAN_USDX_REV3         1
#endif

#ifdef TX_CLK0_CLK1
#ifdef F_CLK2
#define TX1RX0  0b11111000
#define TX1RX1  0b11111000
#define TX0RX1  0b11111000
#define TX0RX0  0b11111011
#else //!F_CLK2
#define TX1RX0  0b11111100
#define TX1RX1  0b11111100
#define TX0RX1  0b11111100
#define TX0RX0  0b11111111
#endif //F_CLK2
#else  //!TX_CLK0_CLK1
#define TX1RX0  0b11111011
#define TX1RX1  0b11111000
#define TX0RX1  0b11111100
#define TX0RX0  0b11111111
#endif //TX_CLK0_CLK1

#if defined(F_CLK2) && !defined(TX_CLK0_CLK1)
#error "TX_CLK0_CLK1 must be enabled in order to use F_CLK2."
#endif

#ifndef TX_ENABLE
#undef KEYER
#undef TX_DELAY
#undef SEMI_QSK
#undef RIT_ENABLE
#undef VOX_ENABLE
#undef MOX_ENABLE
#endif //!TX_ENABLE

#ifdef SWR_METER
float FWD;
float SWR;
float ref_V = 5 * 1.15;
static uint32_t stimer;
#define PIN_FWD  A6
#define PIN_REF  A7
#endif

/*
// UCX installation: On blank chip, use (standard Arduino Uno) fuse settings (E:FD, H:DE, L:FF), and use customized Optiboot bootloader for 20MHz clock, then upload via serial interface (with RX, TX and DTR lines connected to pin 1, 2, 3 respectively)
// UCX pin defintions
+#define SDA     3         //PD3    (pin 5)
+#define SCL     4         //PD4    (pin 6)
+#define ROT_A   6         //PD6    (pin 12)
+#define ROT_B   7         //PD7    (pin 13)
+#define RX      8         //PB0    (pin 14)
+#define SIDETONE 9        //PB1    (pin 15)
+#define KEY_OUT 10        //PB2    (pin 16)
+#define NTX     11        //PB3    (pin 17)
+#define DAH     12        //PB4    (pin 18)
+#define DIT     13        //PB5    (pin 19)
+#define AUDIO1  14        //PC0/A0 (pin 23)
+#define AUDIO2  15        //PC1/A1 (pin 24)
+#define DVM     16        //PC2/A2 (pin 25)
+#define BUTTONS 17        //PC3/A3 (pin 26)
// In addition set:
#define OLED  1
#define ONEBUTTON  1
#define ONEBUTTON_INV 1
#undef DEBUG
adjust I2C and I2C_ ports, 
ssb_cap=1; dsp_cap=2;
#define _DELAY() for(uint8_t i = 0; i != 5; i++) asm("nop");
#define F_XTAL 20004000
#define F_CPU F_XTAL
*/

#if(F_CPU != 16000000)
   #error "Unsupported clock frequency, Arduino IDE must specify 16MHz clock; alternate crystal frequencies may be specified with F_MCU."
#endif
#undef F_CPU
#define F_CPU 20007000  // Actual crystal frequency of 20MHz XTAL1, note that this declaration is just informative and does not correct the timing in Arduino functions like delay(); hence a 1.25 factor needs to be added for correction.
#ifndef F_MCU
#define F_MCU 20000000  // 20MHz ATMEGA328P crystal
#endif

#define F_SAMP_TX 4800 //4810 //4805 // 4402 // (Design) ADC sample-rate; is best a multiple of _UA and fits exactly in OCR2A = ((F_CPU / 64) / F_SAMP_TX) - 1 , should not exceed CPU utilization

#if(F_MCU != 20000000)
const int16_t _F_SAMP_TX = (F_MCU * 4800LL / 20000000);  // Actual ADC sample-rate; used for phase calculations
#else
#define _F_SAMP_TX  F_SAMP_TX
#endif
#define _UA  600 //=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision
#define MAX_DP  ((filt == 0) ? _UA : (filt == 3) ? _UA/4 : _UA/2)     //(_UA/2) // the occupied SSB bandwidth can be further reduced by restricting the maximum phase change (set MAX_DP to _UA/2).
#define CARRIER_COMPLETELY_OFF_ON_LOW  1    // disable oscillator on low amplitudes, to prevent potential unwanted biasing/leakage through PA circuit
#define MULTI_ADC  1  // multiple ADC conversions for more sensitive (+12dB) microphone input

//#define QUAD  1       // invert TX signal for phase changes > 180

//#define BLIND 1             // uSDX in head-less operation

#define MIC_ATTEN  0  // 0*6dB attenuation (note that the LSB bits are quite noisy)