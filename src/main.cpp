//  QCX-SSB.ino - https://github.com/threeme3/QCX-SSB
//
//  Copyright 2019, 2020, 2021   Guido PE1NNZ <pe1nnz@amsat.org>
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <Arduino.h>
#include "configuration.h"
#include "cw/keyer.h"
#include "phy/I2C_.h"
#include <si5351.h>
#include "bpf/IOExpander16.h"
#include "bpf/PCA9536.h"
#include "bpf/MCP23008.h"
/*
extern char __bss_end;
static int freeMemory(){ char* sp = reinterpret_cast<char*>(SP); return sp - &__bss_end; }  // see: http://www.nongnu.org/avr-libc/user-manual/malloc.html

#ifdef CAT_EXT
volatile uint8_t cat_key = 0;
uint8_t _digitalRead(uint8_t pin){  // reads pin or (via CAT) artificially overriden pins
  serialEvent();  // allows CAT update
  if(cat_key){ return (pin == BUTTONS) ? ((cat_key&0x07) > 0) : (pin == DIT) ? ~cat_key&0x10 : (pin == DAH) ? ~cat_key&0x20 : 0; } // overrides digitalRead(DIT, DAH, BUTTONS);
  return digitalRead(pin);
}
#else
#define _digitalRead(x) digitalRead(x)
#endif //CAT_EXT

//#define ONEBUTTON_INV 1 // Encoder button goes from PC3 to GND (instead PC3 to 5V, with 10k pull down)
#ifdef ONEBUTTON_INV
  uint8_t inv = 1;
#else
  uint8_t inv = 0;
#endif

static uint8_t practice = false;  // Practice mode

volatile uint8_t cat_active = 0;
volatile uint32_t rxend_event = 0;
volatile uint8_t vox = 0;

//#define _I2C_DIRECT_IO    1 // Enables communications that is not using the standard I/O pull-down approach with pull-up resistors, instead I/O is directly driven with 0V/5V
I2C_ Wire;
//#include <Wire.h>

uint8_t backlight = 8;
//#define RS_HIGH_ON_IDLE   1   // Experimental LCD support where RS line is high on idle periods to comply with SDA I2C standard.

// display declaration
#ifdef BLIND
Display<Blind> lcd;
#else
#ifdef OLED
Display<OLEDDevice> lcd;
#else
Display<LCD> lcd;     // highly-optimized LCD driver, OK for QCX supplied displays
//LCD_ lcd;  // slower LCD, suitable for non-QCX supplied displays
//#include <LiquidCrystal.h>
//LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
#endif
#endif

volatile int8_t encoder_val = 0;
volatile int8_t encoder_step = 0;
static uint8_t last_state;
ISR(PCINT2_vect){  // Interrupt on rotary encoder turn
  //noInterrupts();
  //PCMSK2 &= ~((1 << PCINT22) | (1 << PCINT23));  // mask ROT_A, ROT_B interrupts
  switch(last_state = (last_state << 4) | (_digitalRead(ROT_B) << 1) | _digitalRead(ROT_A)){ //transition  (see: https://www.allaboutcircuits.com/projects/how-to-use-a-rotary-encoder-in-a-mcu-based-project/  )
//#define ENCODER_ENHANCED_RESOLUTION  1
#ifdef ENCODER_ENHANCED_RESOLUTION // Option: enhance encoder from 24 to 96 steps/revolution, see: appendix 1, https://www.sdr-kits.net/documents/PA0KLT_Manual.pdf
    case 0x31: case 0x10: case 0x02: case 0x23: encoder_val++; break;
    case 0x32: case 0x20: case 0x01: case 0x13: encoder_val--; break;
#else
//    case 0x31: case 0x10: case 0x02: case 0x23: if(encoder_step < 0) encoder_step = 0; encoder_step++; if(encoder_step >  3){ encoder_step = 0; encoder_val++; } break;  // encoder processing for additional immunity to weared-out rotary encoders
//    case 0x32: case 0x20: case 0x01: case 0x13: if(encoder_step > 0) encoder_step = 0; encoder_step--; if(encoder_step < -3){ encoder_step = 0; encoder_val--; } break;
    case 0x23:  encoder_val++; break;
    case 0x32:  encoder_val--; break;
#endif
  }
  //PCMSK2 |= (1 << PCINT22) | (1 << PCINT23);  // allow ROT_A, ROT_B interrupts
  //interrupts();
}

void encoder_setup()
{
  pinMode(ROT_A, INPUT_PULLUP);
  pinMode(ROT_B, INPUT_PULLUP);
  PCMSK2 |= (1 << PCINT22) | (1 << PCINT23); // interrupt-enable for ROT_A, ROT_B pin changes; see https://github.com/EnviroDIY/Arduino-SDI-12/wiki/2b.-Overview-of-Interrupts
  PCICR |= (1 << PCIE2); 
  last_state = (_digitalRead(ROT_B) << 1) | _digitalRead(ROT_A);
  interrupts();
}

//#define log2(n) (log(n) / log(2))
uint8_t log2(uint16_t x){
  uint8_t y = 0;
  for(; x>>=1;) y++;
  return y;
}

I2C i2c;
static SI5351 si5351;

#if defined(LPF_SWITCHING_PE1DDA_USDXDUO)
inline void set_lpf(uint8_t f){
  pinMode(PD5, OUTPUT);
  digitalWrite(PD5, (f >= LPF_SWITCHING_PE1DDA_USDXDUO));
}
#endif  //LPF_SWITCHING_PE1DDA_USDXDUO

#if !defined(LPF_SWITCHING_DL2MAN_USDX_REV1) && !defined(LPF_SWITCHING_DL2MAN_USDX_REV2_BETA) && !defined(LPF_SWITCHING_DL2MAN_USDX_REV2) && !defined(LPF_SWITCHING_DL2MAN_USDX_REV3) && !defined(LPF_SWITCHING_WB2CBA_USDX_OCTOBAND) && !defined(LPF_SWITCHING_PE1DDA_USDXDUO)
inline void set_lpf(uint8_t f){} // dummy
#endif

/**
 * 
 **

#ifdef DEBUG
static uint32_t sr = 0;
static uint32_t cpu_load = 0;
volatile uint16_t param_a = 0;  // registers for debugging, testing and experimental purposes
volatile int16_t param_b = 0;
volatile int16_t param_c = 0;
#endif

enum dsp_cap_t { ANALOG, DSP, SDR };

#ifdef QCX
uint8_t dsp_cap = 0;
uint8_t ssb_cap = 0;
#else
 // force SSB and SDR capability
const uint8_t ssb_cap = 1;
const uint8_t dsp_cap = SDR;
#endif

enum mode_t { LSB, USB, CW, FM, AM };

volatile uint8_t mode = USB;
volatile uint16_t numSamples = 0;

volatile uint8_t tx = 0;
volatile uint8_t filt = 0;

inline void _vox(bool trigger)
{
  if(trigger){
    tx = (tx) ? 254 : 255; // hangtime = 255 / 4402 = 58ms (the time that TX at least stays on when not triggered again). tx == 255 when triggered first, 254 follows for subsequent triggers, until tx is off.
  } else {
    if(tx) tx--;
  }
}


inline int16_t arctan3(int16_t q, int16_t i)  // error ~ 0.8 degree
{ // source: [1] http://www-labs.iro.umontreal.ca/~mignotte/IFT2425/Documents/EfficientApproximationArctgFunction.pdf
//#define _atan2(z)  (_UA/8 + _UA/44) * z  // very much of a simplification...not accurate at all, but fast
#define _atan2(z)  (_UA/8 + _UA/22 - _UA/22 * z) * z  //derived from (5) [1]   note that atan2 can overflow easily so keep _UA low
//#define _atan2(z)  (_UA/8 + _UA/24 - _UA/24 * z) * z  //derived from (7) [1]
  int16_t r;
  if(abs(q) > abs(i))
    r = _UA / 4 - _atan2(abs(i) / abs(q));        // arctan(z) = 90-arctan(1/z)
  else
    r = (i == 0) ? 0 : _atan2(abs(q) / abs(i));   // arctan(z)
  r = (i < 0) ? _UA / 2 - r : r;                  // arctan(-z) = -arctan(z)
  return (q < 0) ? -r : r;                        // arctan(-z) = -arctan(z)
}

#define magn(i, q) (abs(i) > abs(q) ? abs(i) + abs(q) / 4 : abs(q) + abs(i) / 4) // approximation of: magnitude = sqrt(i*i + q*q); error 0.95dB

uint8_t lut[256];
volatile uint8_t amp;
#define MORE_MIC_GAIN   1       // adds more microphone gain, improving overall SSB quality (when speaking further away from microphone)
#ifdef MORE_MIC_GAIN
volatile uint8_t vox_thresh = (1 << 2);
#else
volatile uint8_t vox_thresh = (1 << 1); //(1 << 2);
#endif
volatile uint8_t drive = 2;   // hmm.. drive>2 impacts cpu load..why?

volatile uint8_t quad = 0;

inline int16_t ssb(int16_t in)
{
  static int16_t dc, z1;

  int16_t i, q;
  uint8_t j;
  static int16_t v[16];
  for(j = 0; j != 15; j++) v[j] = v[j + 1];
#ifdef MORE_MIC_GAIN
//#define DIG_MODE  // optimization for digital modes: for super flat TX spectrum, (only down < 100Hz to cut-off DC components)
#ifdef DIG_MODE
  int16_t ac = in;
  dc = (ac + (7) * dc) / (7 + 1);  // hpf: slow average
  v[15] = (ac - dc) / 2;           // hpf (dc decoupling)  (-6dB gain to compensate for DC-noise)
#else
  int16_t ac = in * 2;             //   6dB gain (justified since lpf/hpf is losing -3dB)
  ac = ac + z1;                    // lpf
  z1 = (in - (2) * z1) / (2 + 1);  // lpf: notch at Fs/2 (alias rejecting)
  dc = (ac + (2) * dc) / (2 + 1);  // hpf: slow average
  v[15] = (ac - dc);               // hpf (dc decoupling)
#endif //DIG_MODE
  i = v[7] * 2;  // 6dB gain for i, q  (to prevent quanitization issues in hilbert transformer and phase calculation, corrected for magnitude calc)
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 16) / 64 + (v[6] - v[8]); // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)

  uint16_t _amp = magn(i / 2, q / 2);  // -6dB gain (correction)
#else  // !MORE_MIC_GAIN
  //dc += (in - dc) / 2;       // fast moving average
  dc = (in + dc) / 2;        // average
  int16_t ac = (in - dc);   // DC decoupling
  //v[15] = ac;// - z1;        // high-pass (emphasis) filter
  v[15] = (ac + z1);// / 2;           // low-pass filter with notch at Fs/2
  z1 = ac;

  i = v[7];
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)

  uint16_t _amp = magn(i, q);
#endif  // MORE_MIC_GAIN

#ifdef CARRIER_COMPLETELY_OFF_ON_LOW
  _vox(_amp > vox_thresh);
#else
  if(vox) _vox(_amp > vox_thresh);
#endif
  //_amp = (_amp > vox_thresh) ? _amp : 0;   // vox_thresh = 4 is a good setting
  //if(!(_amp > vox_thresh)) return 0;

  _amp = _amp << (drive);
  _amp = ((_amp > 255) || (drive == 8)) ? 255 : _amp; // clip or when drive=8 use max output
  amp = (tx) ? lut[_amp] : 0;

  static int16_t prev_phase;
  int16_t phase = arctan3(q, i);

  int16_t dp = phase - prev_phase;  // phase difference and restriction
  //dp = (amp) ? dp : 0;  // dp = 0 when amp = 0
  prev_phase = phase;

  if(dp < 0) dp = dp + _UA; // make negative phase shifts positive: prevents negative frequencies and will reduce spurs on other sideband
#ifdef QUAD
  if(dp >= (_UA/2)){ dp = dp - _UA/2; quad = !quad; }
#endif

#ifdef MAX_DP
  if(dp > MAX_DP){ // dp should be less than half unit-angle in order to keep frequencies below F_SAMP_TX/2
    prev_phase = phase - (dp - MAX_DP);  // substract restdp
    dp = MAX_DP;
  }
#endif
  if(mode == USB)
    return dp * ( _F_SAMP_TX / _UA); // calculate frequency-difference based on phase-difference
  else
    return dp * (-_F_SAMP_TX / _UA);
}

volatile int8_t mox = 0;
volatile int8_t volume = 12;

// This is the ADC ISR, issued with sample-rate via timer1 compb interrupt.
// It performs in real-time the ADC sampling, calculation of SSB phase-differences, calculation of SI5351 frequency registers and send the registers to SI5351 over I2C.
static int16_t _adc;
void dsp_tx()
{ // jitter dependent things first
#ifdef MULTI_ADC  // SSB with multiple ADC conversions:
  int16_t adc;                         // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  adc = ADC;
  ADCSRA |= (1 << ADSC);
  //OCR1BL = amp;                        // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  si5351.SendPLLRegisterBulk();       // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
#ifdef QUAD
#ifdef TX_CLK0_CLK1
  si5351.SendRegister(16, (quad) ? 0x1f : 0x0f);  // Invert/non-invert CLK0 in case of a huge phase-change
  si5351.SendRegister(17, (quad) ? 0x1f : 0x0f);  // Invert/non-invert CLK1 in case of a huge phase-change
#else
  si5351.SendRegister(18, (quad) ? 0x1f : 0x0f);  // Invert/non-invert CLK2 in case of a huge phase-change
#endif
#endif //QUAD
  OCR1BL = amp;                      // submit amplitude to PWM register (takes about 1/32125 = 31us+/-31us to propagate) -> amplitude-phase-alignment error is about 30-50us
  adc += ADC;
ADCSRA |= (1 << ADSC);  // causes RFI on QCX-SSB units (not on units with direct biasing); ENABLE this line when using direct biasing!!
  int16_t df = ssb(_adc >> MIC_ATTEN); // convert analog input into phase-shifts (carrier out by periodic frequency shifts)
  adc += ADC;
  ADCSRA |= (1 << ADSC);
  si5351.freq_calc_fast(df);           // calculate SI5351 registers based on frequency shift and carrier frequency
  adc += ADC;
  ADCSRA |= (1 << ADSC);
  //_adc = (adc/4 - 512);
#define AF_BIAS   32
  _adc = (adc/4 - (512 - AF_BIAS));        // now make sure that we keep a postive bias offset (to prevent the phase swapping 180 degrees and potentially causing negative feedback (RFI)
#else  // SSB with single ADC conversion:
  ADCSRA |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
  //OCR1BL = amp;                        // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  si5351.SendPLLRegisterBulk();       // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
  OCR1BL = amp;                        // submit amplitude to PWM register (takes about 1/32125 = 31us+/-31us to propagate) -> amplitude-phase-alignment error is about 30-50us
  int16_t adc = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t df = ssb(adc >> MIC_ATTEN);  // convert analog input into phase-shifts (carrier out by periodic frequency shifts)
  si5351.freq_calc_fast(df);           // calculate SI5351 registers based on frequency shift and carrier frequency
#endif

#ifdef CARRIER_COMPLETELY_OFF_ON_LOW
  if(tx == 1){ OCR1BL = 0; si5351.SendRegister(SI_CLK_OE, TX0RX0); }   // disable carrier
  if(tx == 255){ si5351.SendRegister(SI_CLK_OE, TX1RX0); } // enable carrier
#endif

#ifdef MOX_ENABLE
  if(!mox) return;
  OCR1AL = (adc << (mox-1)) + 128;  // TX audio monitoring
#endif
}

volatile uint16_t acc;
volatile uint32_t cw_offset;
volatile uint8_t cw_tone = 1;
const uint32_t tones[] = { F_MCU * 700ULL / 20000000, F_MCU * 600ULL / 20000000, F_MCU * 700ULL / 20000000};

volatile int8_t p_sin = 0;     // initialized with A*sin(0) = 0
volatile int8_t n_cos = 448/4; // initialized with A*cos(t) = A
inline void process_minsky() // Minsky circle sample [source: https://www.cl.cam.ac.uk/~am21/hakmemc.html, ITEM 149]: p_sin+=n_cos*2*PI*f/fs; n_cos-=p_sin*2*PI*f/fs;
{
  int8_t alpha127 = tones[cw_tone]/*cw_offset* * 798 / _F_SAMP_TX;  // alpha = f_tone * 2 * pi / fs
  p_sin += alpha127 * n_cos / 127;
  n_cos -= alpha127 * p_sin / 127;
}

// CW Key-click shaping, ramping up/down amplitude with sample-interval of 60us. Tnx: Yves HB9EWY https://groups.io/g/ucx/message/5107
const uint8_t ramp[] PROGMEM = { 255, 254, 252, 249, 245, 239, 233, 226, 217, 208, 198, 187, 176, 164, 152, 139, 127, 115, 102, 90, 78, 67, 56, 46, 37, 28, 21, 15, 9, 5, 2 }; // raised-cosine(i) = 255 * sq(cos(HALF_PI * i/32))

void dummy()
{
}

void dsp_tx_cw()
{ // jitter dependent things first
#ifdef KEY_CLICK
  if(OCR1BL < lut[255]) { //check if already ramped up: ramp up of amplitude 
     for(uint16_t i = 31; i != 0; i--) {   // soft rising slope against key-clicks
        OCR1BL = lut[pgm_read_byte_near(&ramp[i-1])];
        delayMicroseconds(60);
     }
  }
#endif // KEY_CLICK
  OCR1BL = lut[255];
  
  process_minsky();
  OCR1AL = (p_sin >> (16 - volume)) + 128;
}

void dsp_tx_am()
{ // jitter dependent things first
  ADCSRA |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
  OCR1BL = amp;                        // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  int16_t adc = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t in = (adc >> MIC_ATTEN);
  in = in << (drive-4);
  //static int16_t dc;
  //dc += (in - dc) / 2;
  //in = in - dc;     // DC decoupling
  #define AM_BASE 32
  in=max(0, min(255, (in + AM_BASE)));
  amp=in;// lut[in];
}

void dsp_tx_fm()
{ // jitter dependent things first
  ADCSRA |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
  OCR1BL = lut[255];                   // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  si5351.SendPLLRegisterBulk();       // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
  int16_t adc = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t in = (adc >> MIC_ATTEN);
  in = in << (drive);
  int16_t df = in;
  si5351.freq_calc_fast(df);           // calculate SI5351 registers based on frequency shift and carrier frequency
}

#define EA(y, x, one_over_alpha)  (y) = (y) + ((x) - (y)) / (one_over_alpha); // exponental averaging [Lyons 13.33.1]
#define MLEA(y, x, L, M)  (y)  = (y) + ((((x) - (y)) >> (L)) - (((x) - (y)) >> (M))); // multiplierless exponental averaging [Lyons 13.33.1], with alpha=1/2^L - 1/2^M

#ifdef SWR_METER
volatile uint8_t swrmeter = 1;
#endif

volatile uint8_t menumode = 0;  // 0=not in menu, 1=selects menu item, 2=selects parameter value

#define F_SAMP_PWM (78125/1)
//#define F_SAMP_RX 78125  // overrun, do not use
#define F_SAMP_RX 62500
//#define F_SAMP_RX 52083
//#define F_SAMP_RX 44643
//#define F_SAMP_RX 39062
//#define F_SAMP_RX 34722
//#define F_SAMP_RX 31250
//#define F_SAMP_RX 28409
#define F_ADC_CONV (192307/2)  //was 192307/1, but as noted this produces clicks in audio stream. Slower ADC clock cures this (but is a problem for VOX when sampling mic-input simulatanously).

#ifdef FAST_AGC
volatile uint8_t agc = 2;
#else
volatile uint8_t agc = 1;
#endif
volatile uint8_t nr = 0;
volatile uint8_t att = 0;
volatile uint8_t att2 = 2;  // Minimum att2 increased, to prevent numeric overflow on strong signals
volatile uint8_t _init = 0;

// Old AGC algorithm which only increases gain, but does not decrease it for very strong signals.
// Maximum possible gain is x32 (in practice, x31) so AGC range is x1 to x31 = 30dB approx.
// Decay time is fine (about 1s) but attack time is much slower than I like. 
// For weak/medium signals it aims to keep the sample value between 1024 and 2048. 
static int16_t gain = 1024;
inline int16_t process_agc_fast(int16_t in)
{
  int16_t out = (gain >= 1024) ? (gain >> 10) * in : in;
  int16_t accum = (1 - abs(out >> 10));
  if((INT16_MAX - gain) > accum) gain = gain + accum;
  if(gain < 1) gain = 1;
  return out;
}

// Contribution by Alan, M0PUB: Experimental new AGC algorithm.
// ASSUMES: Input sample values are constrained to a maximum of +/-4096 to avoid integer overflow in earlier
// calculations.
//
// This algorithm aims to keep signals between a peak sample value of 1024 - 1536, with fast attack but slow
// decay.
//
// The variable centiGain actually represents the applied gain x 128 - i.e. the numeric gain applied is centiGain/128
//
// Since the largest valid input sample has a value of +/- 4096, centiGain should never be less than 32 (i.e.
// a 'gain' of 0.25). The maximum value for centiGain is 32767, and hence a gain of 255. So the AGC range
// is 0.25:255, or approx. 60dB.
//
// Variable 'slowdown' allows the decay time to be slowed down so that it is not directly related to the value
// of centiCount.

static int16_t centiGain = 128;
#define DECAY_FACTOR 400      // AGC decay occurs <DECAY_FACTOR> slower than attack.
static uint16_t decayCount = DECAY_FACTOR;
#define HI(x)  ((x) >> 8)
#define LO(x)  ((x) & 0xFF)

inline int16_t process_agc(int16_t in)
{
  static bool small = true;
  int16_t out;

  if(centiGain >= 128)
    out = (centiGain >> 5) * in;         // net gain >= 1
  else
    out = (centiGain >> 2) * (in >> 3);  // net gain < 1
  out >>= 2;

  if(HI(abs(out)) > HI(1536)){
    centiGain -= (centiGain >> 4);       // Fast attack time when big signal encountered (relies on CentiGain >= 16)
  } else {
    if(HI(abs(out)) > HI(1024))
      small = false;
    if(--decayCount == 0){               // But slow ramp up of gain when signal disappears
      if(small){                         // 400 samples below lower threshold - increase gain
        if(centiGain < (INT16_MAX-(INT16_MAX >> 4)))
          centiGain += (centiGain >> 4);
        else
          centiGain = INT16_MAX;
      }
      decayCount = DECAY_FACTOR;
      small = true;
    }
  }
  return out;
}

inline int16_t process_nr_old(int16_t ac)
{
  ac = ac >> (6-abs(ac));  // non-linear below amp of 6; to reduce noise (switchoff agc and tune-up volume until noise dissapears, todo:extra volume control needed)
  ac = ac << 3;
  return ac;
}

inline int16_t process_nr_old2(int16_t ac)
{  
  static int16_t ea1;
  //ea1 = MLEA(ea1, ac, 5, 6); // alpha=0.0156
  ea1 = EA(ea1, ac, 64); // alpha=1/64=0.0156
  //static int16_t ea2;
  //ea2 = EA(ea2, ea1, 64); // alpha=1/64=0.0156
 
  return ea1;
}

inline int16_t process_nr(int16_t in)
{ 
/*
  static int16_t avg;
  avg = EA(avg, abs(in), 64); // alpha=1/64=0.0156
param_c = avg;
*/

/*
  int32_t _avg = 64 * avg;
//  if(_avg > 4) _avg = 4;  // clip
//  uint16_t brs_avgsq = 1 << (_avg * _avg);
  if(_avg > 14) _avg = 14;  // clip
  uint16_t brs_avgsq = 1 << (_avg);

  
  int16_t inv_gain;
  if(brs_avgsq > 1) inv_gain = brs_avgsq / (brs_avgsq - 1);  // = 1 / (1 - 1/(1 << (1*avg*avg)) );
  else inv_gain = 32768;  */ /*

  static int16_t ea1;
  ea1 = EA(ea1, in, 1 << (nr-1) );
  //static int16_t ea2;
  //ea2 = EA(ea2, ea1, inv_gain);

  return ea1;
}
/*
inline int16_t process_nr(int16_t in)
{
  // Exponential moving average and variance (Lyons 13.36.2)
  param_b = EA(param_b, in, 1 << 4);  // avg
  param_c = EA(param_c, (in - param_b) * (in - param_b), 1 << 4);  // variance
}
*/ /*

#define N_FILT 7
//volatile uint8_t filt = 0;
uint8_t prev_filt[] = { 0 , 4 }; // default filter for modes resp. CW, SSB

/* basicdsp filter simulation:
  samplerate=7812
  za0=in
  p1=slider1*10
  p2=slider2*10
  p3=slider3*10
  p4=slider4*10
  zb0=(za0+2*za1+za2)/2-(p1*zb1+p2*zb2)/16
  zc0=(zb0+2*zb1+zb2)/4-(p3*zc1+p4*zc2)/16
  zc2=zc1
  zc1=zc0
  zb2=zb1
  zb1=zb0
  za2=za1
  za1=za0
  out=zc0

  samplerate=7812
  za0=in
  p1=slider1*100+100
  p2=slider2*100
  p3=slider3*100+100
  p4=slider4*100
  zb0=(za0+2*za1+za2)-(-p1*zb1+p2*zb2)/64
  zc0=(zb0-2*zb1+zb2)/8-(-p3*zc1+p4*zc2)/64
  zc2=zc1
  zc1=zc0
  zb2=zb1
  zb1=zb0
  za2=za1
  za1=za0
  out=zc0/8
*/ /*
inline int16_t filt_var(int16_t za0)  //filters build with www.micromodeler.com
{ 
  static int16_t za1,za2;
  static int16_t zb0,zb1,zb2;
  static int16_t zc0,zc1,zc2;
  
  if(filt < 4)
  {  // for SSB filters
    // 1st Order (SR=8kHz) IIR in Direct Form I, 8x8:16
    // M0PUB: There was a bug here, since za1 == zz1 at this point in the code, and the old algorithm for the 300Hz high-pass was:
    //    za0=(29*(za0-zz1)+50*za1)/64;
    //    zz2=zz1;
    //    zz1=za0;
    // After correction, this filter still introduced almost 6dB attenuation, so I adjusted the coefficients
    static int16_t zz1,zz2;
    //za0=(29*(za0-zz1)+50*za1)/64;                                //300-Hz
    zz2=zz1;
    zz1=za0;
    //za0=(30*(za0-zz2)+0*zz1)/32;                                 //300-Hz with very steep roll-off down to 0 Hz
    za0=(30*(za0-zz2)+25*zz1)/32;                                  //300-Hz

    // 4th Order (SR=8kHz) IIR in Direct Form I, 8x8:16
    switch(filt){
      case 1: zb0=(za0+2*za1+za2)/2-(13*zb1+11*zb2)/16; break;   // 0-2900Hz filter, first biquad section
      case 2: zb0=(za0+2*za1+za2)/2-(2*zb1+8*zb2)/16; break;     // 0-2400Hz filter, first biquad section
      //case 3: zb0=(za0+2*za1+za2)/2-(4*zb1+2*zb2)/16; break;     // 0-2400Hz filter, first biquad section
      case 3: zb0=(za0+2*za1+za2)/2-(0*zb1+4*zb2)/16; break;     //0-1800Hz  elliptic
      //case 3: zb0=(za0+7*za1+za2)/16-(-24*zb1+9*zb2)/16; break;  //0-1700Hz  elliptic with slope
    }
  
    switch(filt){
      case 1: zc0=(zb0+2*zb1+zb2)/2-(18*zc1+11*zc2)/16; break;     // 0-2900Hz filter, second biquad section
      case 2: zc0=(zb0+2*zb1+zb2)/4-(4*zc1+8*zc2)/16; break;       // 0-2400Hz filter, second biquad section
      //case 3: zc0=(zb0+2*zb1+zb2)/4-(1*zc1+9*zc2)/16; break;       // 0-2400Hz filter, second biquad section
      case 3: zc0=(zb0+2*zb1+zb2)/4-(0*zc1+4*zc2)/16; break;       //0-1800Hz  elliptic
      //case 3: zc0=(zb0+zb1+zb2)/16-(-22*zc1+47*zc2)/64; break;   //0-1700Hz  elliptic with slope
    }
   /*switch(filt){
      case 1: zb0=za0; break; //0-4000Hz (pass-through)
      case 2: zb0=(10*(za0+2*za1+za2)+16*zb1-17*zb2)/32; break;    //0-2500Hz  elliptic -60dB@3kHz
      case 3: zb0=(7*(za0+2*za1+za2)+48*zb1-18*zb2)/32; break;     //0-1700Hz  elliptic
    }
  
    switch(filt){
      case 1: zc0=zb0; break; //0-4000Hz (pass-through)
      case 2: zc0=(8*(zb0+zb2)+13*zb1-43*zc1-52*zc2)/64; break;   //0-2500Hz  elliptic -60dB@3kHz
      case 3: zc0=(4*(zb0+zb1+zb2)+22*zc1-47*zc2)/64; break;   //0-1700Hz  elliptic
    }*/ /*
  
    zc2=zc1;
    zc1=zc0;
  
    zb2=zb1;
    zb1=zb0;
  
    za2=za1;
    za1=za0;
    
    return zc0;
  } else { // for CW filters
    //   (2nd Order (SR=4465Hz) IIR in Direct Form I, 8x8:16), adding 64x front-gain (to deal with later division)
//#define FILTER_700HZ   1
#ifdef FILTER_700HZ
    if(cw_tone == 0){
      switch(filt){
        case 4: zb0=(za0+2*za1+za2)/2+(41L*zb1-23L*zb2)/32; break;   //500-1000Hz
        case 5: zb0=5*(za0-2*za1+za2)+(105L*zb1-58L*zb2)/64; break;   //650-840Hz
        case 6: zb0=3*(za0-2*za1+za2)+(108L*zb1-61L*zb2)/64; break;   //650-750Hz
        case 7: zb0=(2*za0-3*za1+2*za2)+(111L*zb1-62L*zb2)/64; break; //630-680Hz       
        //case 4: zb0=(0*za0+1*za1+0*za2)+(28*zb1-14*zb2)/16; break; //600Hz+-250Hz
        //case 5: zb0=(0*za0+1*za1+0*za2)+(28*zb1-15*zb2)/16; break; //600Hz+-100Hz
        //case 6: zb0=(0*za0+1*za1+0*za2)+(27*zb1-15*zb2)/16; break; //600Hz+-50Hz
        //case 7: zb0=(0*za0+1*za1+0*za2)+(27*zb1-15*zb2)/16; break; //630Hz+-18Hz
      }
    
      switch(filt){
        case 4: zc0=(zb0-2*zb1+zb2)/4+(105L*zc1-52L*zc2)/64; break;      //500-1000Hz
        case 5: zc0=((zb0+2*zb1+zb2)+97L*zc1-57L*zc2)/64; break;      //650-840Hz
        case 6: zc0=((zb0+zb1+zb2)+104L*zc1-60L*zc2)/64; break;       //650-750Hz
        case 7: zc0=((zb1)+109L*zc1-62L*zc2)/64; break;               //630-680Hz
        //case 4: zc0=(zb0-2*zb1+zb2)/1+(24*zc1-13*zc2)/16; break; //600Hz+-250Hz
        //case 5: zc0=(zb0-2*zb1+zb2)/4+(26*zc1-14*zc2)/16; break; //600Hz+-100Hz
        //case 6: zc0=(zb0-2*zb1+zb2)/16+(28*zc1-15*zc2)/16; break; //600Hz+-50Hz
        //case 7: zc0=(zb0-2*zb1+zb2)/32+(27*zc1-15*zc2)/16; break; //630Hz+-18Hz
      }
    }
    if(cw_tone == 1)
#endif
    {
      switch(filt){
        //case 4: zb0=(1*za0+2*za1+1*za2)+(90L*zb1-38L*zb2)/64; break; //600Hz+-250Hz
        //case 5: zb0=(1*za0+2*za1+1*za2)/2+(102L*zb1-52L*zb2)/64; break; //600Hz+-100Hz
        //case 6: zb0=(1*za0+2*za1+1*za2)/2+(107L*zb1-57L*zb2)/64; break; //600Hz+-50Hz
        //case 7: zb0=(0*za0+1*za1+0*za2)+(110L*zb1-61L*zb2)/64; break; //600Hz+-25Hz
        
        case 4: zb0=(0*za0+1*za1+0*za2)+(114L*zb1-57L*zb2)/64; break; //600Hz+-250Hz
        case 5: zb0=(0*za0+1*za1+0*za2)+(113L*zb1-60L*zb2)/64; break; //600Hz+-100Hz
        case 6: zb0=(0*za0+1*za1+0*za2)+(110L*zb1-62L*zb2)/64; break; //600Hz+-50Hz
        case 7: zb0=(0*za0+1*za1+0*za2)+(110L*zb1-61L*zb2)/64; break; //600Hz+-18Hz
        //case 8: zb0=(0*za0+1*za1+0*za2)+(110L*zb1-60L*zb2)/64; break; //591Hz+-12Hz

        /*case 4: zb0=(0*za0+1*za1+0*za2)+2*zb1-zb2+(-14L*zb1+7L*zb2)/64; break; //600Hz+-250Hz
        case 5: zb0=(0*za0+1*za1+0*za2)+2*zb1-zb2+(-15L*zb1+4L*zb2)/64; break; //600Hz+-100Hz
        case 6: zb0=(0*za0+1*za1+0*za2)+2*zb1-zb2+(-14L*zb1+2L*zb2)/64; break; //600Hz+-50Hz
        case 7: zb0=(0*za0+1*za1+0*za2)+2*zb1-zb2+(-14L*zb1+3L*zb2)/64; break; //600Hz+-18Hz
      }
    
      switch(filt){
        //case 4: zc0=(zb0-2*zb1+zb2)/4+(95L*zc1-44L*zc2)/64; break; //600Hz+-250Hz
        //case 5: zc0=(zb0-2*zb1+zb2)/8+(104L*zc1-53L*zc2)/64; break; //600Hz+-100Hz
        //case 6: zc0=(zb0-2*zb1+zb2)/16+(106L*zc1-56L*zc2)/64; break; //600Hz+-50Hz
        //case 7: zc0=(zb0-2*zb1+zb2)/32+(112L*zc1-62L*zc2)/64; break; //600Hz+-25Hz
        
        case 4: zc0=(zb0-2*zb1+zb2)/1+(95L*zc1-52L*zc2)/64; break; //600Hz+-250Hz
        case 5: zc0=(zb0-2*zb1+zb2)/4+(106L*zc1-59L*zc2)/64; break; //600Hz+-100Hz
        case 6: zc0=(zb0-2*zb1+zb2)/16+(113L*zc1-62L*zc2)/64; break; //600Hz+-50Hz
        case 7: zc0=(zb0-2*zb1+zb2)/32+(112L*zc1-62L*zc2)/64; break; //600Hz+-18Hz
        //case 8: zc0=(zb0-2*zb1+zb2)/64+(113L*zc1-63L*zc2)/64; break; //591Hz+-12Hz
        
        /*case 4: zc0=(zb0-2*zb1+zb2)/1+zc1-zc2+(31L*zc1+12L*zc2)/64; break; //600Hz+-250Hz
        case 5: zc0=(zb0-2*zb1+zb2)/4+2*zc1-zc2+(-22L*zc1+5L*zc2)/64; break; //600Hz+-100Hz
        case 6: zc0=(zb0-2*zb1+zb2)/16+2*zc1-zc2+(-15L*zc1+2L*zc2)/64; break; //600Hz+-50Hz
        case 7: zc0=(zb0-2*zb1+zb2)/16+2*zc1-zc2+(-16L*zc1+2L*zc2)/64; break; //600Hz+-18Hz
      } 
    }
    zc2=zc1;
    zc1=zc0;
  
    zb2=zb1;
    zb1=zb0;
  
    za2=za1;
    za1=za0;
    
    //return zc0 / 64; // compensate the 64x front-end gain
    return zc0 / 8; // compensate the front-end gain
  }
}

#define __UA   256
inline int16_t _arctan3(int16_t q, int16_t i)
{
  #define __atan2(z)  (__UA/8  + __UA/22) * z  // very much of a simplification...not accurate at all, but fast
  //#define __atan2(z)  (__UA/8 - __UA/22 * z + __UA/22) * z  //derived from (5) [1]
  int16_t r;
  if(abs(q) > abs(i))
    r = __UA / 4 - __atan2(abs(i) / abs(q));        // arctan(z) = 90-arctan(1/z)
  else
    r = (i == 0) ? 0 : __atan2(abs(q) / abs(i));   // arctan(z)
  r = (i < 0) ? __UA / 2 - r : r;                  // arctan(-z) = -arctan(z)
  return (q < 0) ? -r : r;                        // arctan(-z) = -arctan(z)
}

static uint32_t absavg256 = 0;
volatile uint32_t _absavg256 = 0;
volatile int16_t i, q;

inline int16_t slow_dsp(int16_t ac)
{
  static uint8_t absavg256cnt;
  if(!(absavg256cnt--)){ _absavg256 = absavg256; absavg256 = 0; } else absavg256 += abs(ac);

  if(mode == AM) {
    ac = magn(i, q);
    { static int16_t dc;   // DC decoupling
      dc += (ac - dc) / 2;
      ac = ac - dc; }
    } else if(mode == FM){
    static int16_t zi;
    ac = ((ac + i) * zi);  // -qh = ac + i
    zi =i;
    /*int16_t z0 = _arctan3(q, i);
    static int16_t z1;
    ac = z0 - z1; // Differentiator
    z1 = z0;*/
    /*static int16_t _q;
    _q = (_q + q) / 2;
    ac = i * _q;  // quadrature detector 
    //ac = ((q > 0) == !(i > 0)) ? 128 : -128; // XOR I/Q zero-cross detector
  }  // needs: p.12 https://www.veron.nl/wp-content/uploads/2014/01/FmDemodulator.pdf
  else { ; }  // USB, LSB, CW

#ifdef FAST_AGC
  if(agc == 2) {
    ac = process_agc(ac);
    ac = ac >> (16-volume);
  } else if(agc == 1){
    ac = process_agc_fast(ac);
    ac = ac >> (16-volume);
  }
#else
  if(agc == 1){
    ac = process_agc_fast(ac);
    ac = ac >> (16-volume);
#endif //!FAST_AGC
  } else {
    //ac = ac >> (16-volume);
    if(volume <= 13)    // if no AGC allow volume control to boost weak signals
      ac = ac >> (13-volume);
    else
      ac = ac << (volume-13);
  }
  if(nr) ac = process_nr(ac);

//  if(filt) ac = filt_var(ac) << 2;
  if(filt) ac = filt_var(ac);
/*
  if(mode == CW){
    if(cwdec){  // CW decoder enabled?
      char ch = cw(ac >> 0);
      if(ch){
        for(int i=0; i!=15;i++) out[i]=out[i+1];
        out[15] = ch;
        cw_event = true;
      }
    }
  }*/ /*
#ifdef CW_DECODER
  if(!(absavg256cnt % 64)){ _amp32 = amp32; amp32 = 0; } else amp32 += abs(ac);
#endif  //CW_DECODER
  //if(!(absavg256cnt--)){ _absavg256 = absavg256; absavg256 = 0; } else absavg256 += abs(ac);  //hack
  
  //static int16_t dc;
  //dc += (ac - dc) / 2;
  //dc = (15*dc + ac)/16;
  //dc = (15*dc + (ac - dc))/16;
  //ac = ac - dc;    // DC decoupling

  ac = min(max(ac, -512), 511);
  //ac = min(max(ac, -128), 127);
#ifdef QCX
  if(!dsp_cap) return 0;  // in QCX-SSB mode (no DSP), slow_dsp() should return 0 (in order to prevent upsampling filter to generate audio)
#endif
  return ac;

}

#ifdef TESTBENCH
// Sine table with 72 entries results in 868Hz sine wave at effective sampling rate of 31250 SPS
// for each of I and Q, since thay are sampled alternately, and hence I (for example) only
// gets 36 samples from this table before looping.
const int8_t sine[] = {
  11, 22, 33, 43, 54, 64, 73, 82, 90, 97, 104, 110, 115, 119, 123, 125, 127, 127, 127, 125, 123, 119, 115, 110, 104, 97, 90, 82, 73, 64, 54, 43, 33, 22, 11, 0, -11, -22, -33, -43, -54, -64, -73, -82, -90, -97, -104, -110, -115, -119, -123, -125, -127, -127, -127, -125, -123, -119, -115, -110, -104, -97, -90, -82, -73, -64, -54, -43, -33, -22, -11, 0
};

// Short Sine table with 36 entries results in 1736Hz sine wave at effective sampling rate of 62500 SPS.
/* const int8_t sine[] = {
  22, 43, 64, 82, 97, 110, 119, 125, 127, 125, 119, 110, 97, 82, 64, 43, 22, 0, -22, -43, -64, -82, -97, -110, -119, -125, -127, -125, -119, -110, -97, -82, -64, -43, -22, 0
}; */ /*

uint8_t ncoIdx = 0;
int16_t NCO_Q()
{
  ncoIdx++;
  if(ncoIdx >= sizeof(sine))
    ncoIdx = 0;
  return (int16_t(sine[ncoIdx])) << 2;
}

int16_t NCO_I()
{
  uint8_t i;

  ncoIdx++;
  if(ncoIdx >= sizeof(sine))
    ncoIdx = 0;

  i = ncoIdx + (sizeof(sine) / 4);  // Advance by 90 degrees
  if(i >= sizeof(sine))
    i -= sizeof(sine);
  return (int16_t(sine[i])) << 2;
}
#endif // TESTBENCH

volatile uint8_t cat_streaming = 0;
volatile uint8_t _cat_streaming = 0;

typedef void (*func_t)(void);
volatile func_t func_ptr;
#undef  R  // Decimating 2nd Order CIC filter
#define R 4  // Rate change from 62500/2 kSPS to 7812.5SPS, providing 12dB gain

//#define SIMPLE_RX  1
#ifndef SIMPLE_RX
volatile uint8_t admux[3];
volatile int16_t ocomb, qh;
volatile uint8_t rx_state = 0;

#pragma GCC push_options
#pragma GCC optimize ("Ofast")  // compiler-optimization for speed

// Non-recursive CIC Filter (M=2, R=4) implementation, so two-stages of (followed by down-sampling with factor 2):
// H1(z) = (1 + z^-1)^2 = 1 + 2*z^-1 + z^-2 = (1 + z^-2) + (2) * z^-1 = FA(z) + FB(z) * z^-1;
// with down-sampling before stage translates into poly-phase components: FA(z) = 1 + z^-1, FB(z) = 2
// Non-recursive CIC Filter (M=4) implementation (for second-stage only):
// H1(z) = (1 + z^-1)^4 = 1 + 4*z^-1 + 6*z^-2 + 4*z^-3 + z^-4 = 1 + 6*z^-2 + z^-4 + (4 + 4*z^-2) * z^-1 = FA(z) + FB(z) * z^-1;
// with down-sampling before stage translates into poly-phase components: FA(z) = 1 + 6*z^-1 + z^-2, FB(z) = 4 + 4*z^-1
// M=3 FA(z) = 1 + 3*z^-1, FB(z) = 3 + z^-1
// source: Lyons Understanding Digital Signal Processing 3rd edition 13.24.1

/* Basicdsp simulation:
# M=2 FA(z) = 1 + z^-1, FB(z) = 2
# M=3 FA(z) = 1 + 3*z^-1, FB(z) = 3 + z^-1
# M=4 FA(z) = 1 + 6*z^-1 + z^-2, FB(z) = 4 + 4*z^-1
samplerate=28000
x=x+1
clk1=mod1(x/2)*2
y=y+clk1
clk2=mod1(y/2)*2
#s1=clk1*fir(in, 1, 2, 1, 0)/16
#s2=clk2*fir(s1, 1, 0, 2, 0, 1, 0, 0)/16
#s1=clk1*fir(in, 1, 3, 3, 1, 0)/16
#s2=clk2*fir(s1, 1, 0, 3, 0, 3, 0, 1, 0, 0)/16
s1=clk1*fir(in, 1, 4, 6, 4, 1, 0)/16
s2=clk2*fir(s1, 1, 0, 4, 0, 6, 0, 4, 0, 1, 0, 0)/16
out=s2
 */ /*

#define NEW_RX  1   // Faster (3rd-order) CIC stage, with simultanuous processing capability
#ifdef NEW_RX
#define AF_OUT  1   // Enables audio output stage (can be disabled in conjunction with CAT_STREAMING to safe memory)

static uint8_t tc = 0;
void process(int16_t i_ac2, int16_t q_ac2)
{
  static int16_t ac3;
#ifdef CAT_STREAMING
  //UCSR0B &= ~(TXCIE0);  // disable USART TX interrupts
  //while (!( UCSR0A & (1<<UDRE0)));  // wait for empty buffer
  if(cat_streaming){ uint8_t out = ac3 + 128; if(out == ';') out++; Serial.write(out); }  //UDR0 = (uint8_t)(ac3 + 128);   // from:  https://www.xanthium.in/how-to-avr-atmega328p-microcontroller-usart-uart-embedded-programming-avrgcc
#endif // CAT_STREAMING
#ifdef AF_OUT
  static int16_t ozd1, ozd2;  // Output stage
  if(_init){ ac3 = 0; ozd1 = 0; ozd2 = 0; _init = 0; } // hack: on first sample init accumlators of further stages (to prevent instability)
  int16_t od1 = ac3 - ozd1; // Comb section
  ocomb = od1 - ozd2;
#endif //AF_OUT
#define OUTLET  1
#ifdef OUTLET
  if(tc++ == 0)   // prevent recursion
  //if(tc++ > 16)   // prevent recursion
#endif
  interrupts();  // hack, since slow_dsp process exceeds rx sample-time, allow subsequent 7 interrupts for further rx sampling while processing, prevent nested interrupts with tc
#ifdef AF_OUT
  ozd2 = od1;
  ozd1 = ac3;
#endif  //AF_OUT
  int16_t qh;
  {
    q_ac2 >>= att2;  // digital gain control
    static int16_t v[14];  // Process Q (down-sampled) samples
    // Hilbert transform, BasicDSP model:  outi= fir(inl,  0, 0, 0, 0, 0,  0,  0, 1,   0, 0,   0, 0,  0, 0, 0, 0); outq = fir(inr, 2, 0, 8, 0, 21, 0, 79, 0, -79, 0, -21, 0, -8, 0, -2, 0) / 128;
    qh = ((v[0] - q_ac2) + (v[2] - v[12]) * 4) / 64 + ((v[4] - v[10]) + (v[6] - v[8])) / 8 + ((v[4] - v[10]) * 5 - (v[6] - v[8]) ) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 43dB side-band rejection in 650..3400Hz (@8kSPS) when used in image-rejection scenario; (Hilbert transform require 4 additional bits)
    //qh = ((v[0] - q_ac2) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)
    v[0] = v[1]; v[1] = v[2]; v[2] = v[3]; v[3] = v[4]; v[4] = v[5]; v[5] = v[6]; v[6] = v[7]; v[7] = v[8]; v[8] = v[9]; v[9] = v[10]; v[10] = v[11]; v[11] = v[12]; v[12] = v[13]; v[13] = q_ac2;
  }
  i_ac2 >>= att2;  // digital gain control
  static int16_t v[7];  // Post processing I and Q (down-sampled) results
  i = i_ac2; q = q_ac2;   // tbd: this can be more efficient
  int16_t i = v[0]; v[0] = v[1]; v[1] = v[2]; v[2] = v[3]; v[3] = v[4]; v[4] = v[5]; v[5] = v[6]; v[6] = i_ac2;  // Delay to match Hilbert transform on Q branch
  ac3 = slow_dsp(-i - qh);  //inverting I and Q helps dampening a feedback-loop between PWM out and ADC inputs
#ifdef OUTLET
  tc--;
#endif
}

/*
// # M=3 .. = i0 + 3*(i2 + i3) + i1
int16_t i0, i1, i2, i3, i4, i5, i6, i7, i8;
int16_t q0, q1, q2, q3, q4, q5, q6, q7, q8;
#define M_SR  1

//#define EXPANDED_CIC
#ifdef EXPANDED_CIC
void sdr_rx_00(){         i0 = sdr_rx_common_i(); func_ptr = sdr_rx_01;   i4 = (i0 + (i2 + i3) * 3 + i1) >> M_SR; }
void sdr_rx_02(){         i1 = sdr_rx_common_i(); func_ptr = sdr_rx_03;   i8 = (i4 + (i6 + i7) * 3 + i5) >> M_SR; }
void sdr_rx_04(){         i2 = sdr_rx_common_i(); func_ptr = sdr_rx_05;   i5 = (i2 + (i0 + i1) * 3 + i3) >> M_SR; }
void sdr_rx_06(){         i3 = sdr_rx_common_i(); func_ptr = sdr_rx_07; }
void sdr_rx_08(){         i0 = sdr_rx_common_i(); func_ptr = sdr_rx_09;   i6 = (i0 + (i2 + i3) * 3 + i1) >> M_SR; }
void sdr_rx_10(){         i1 = sdr_rx_common_i(); func_ptr = sdr_rx_11;   i8 = (i6 + (i4 + i5) * 3 + i7) >> M_SR; }
void sdr_rx_12(){         i2 = sdr_rx_common_i(); func_ptr = sdr_rx_13;   i7 = (i2 + (i0 + i1) * 3 + i3) >> M_SR; }
void sdr_rx_14(){         i3 = sdr_rx_common_i(); func_ptr = sdr_rx_15; }
void sdr_rx_15(){         q0 = sdr_rx_common_q(); func_ptr = sdr_rx_00;   q4 = (q0 + (q2 + q3) * 3 + q1) >> M_SR; }
void sdr_rx_01(){         q1 = sdr_rx_common_q(); func_ptr = sdr_rx_02;   q8 = (q4 + (q6 + q7) * 3 + q5) >> M_SR; }
void sdr_rx_03(){         q2 = sdr_rx_common_q(); func_ptr = sdr_rx_04;   q5 = (q2 + (q0 + q1) * 3 + q3) >> M_SR; }
void sdr_rx_05(){         q3 = sdr_rx_common_q(); func_ptr = sdr_rx_06; process(i8, q8); }
void sdr_rx_07(){         q0 = sdr_rx_common_q(); func_ptr = sdr_rx_08;   q6 = (q0 + (q2 + q3) * 3 + q1) >> M_SR; }
void sdr_rx_09(){         q1 = sdr_rx_common_q(); func_ptr = sdr_rx_10;   q8 = (q6 + (q4 + q5) * 3 + q7) >> M_SR; }
void sdr_rx_11(){         q2 = sdr_rx_common_q(); func_ptr = sdr_rx_12;   q7 = (q2 + (q0 + q1) * 3 + q3) >> M_SR; }
void sdr_rx_13(){         q3 = sdr_rx_common_q(); func_ptr = sdr_rx_14; process(i8, q8); }
#else
void sdr_rx_00(){         i0 = sdr_rx_common_i(); func_ptr = sdr_rx_01;   i4 = (i0 + (i2 + i3) * 3 + i1) >> M_SR; }
void sdr_rx_02(){         i1 = sdr_rx_common_i(); func_ptr = sdr_rx_03;   i8 = (i4 + (i6 + i7) * 3 + i5) >> M_SR; }
void sdr_rx_04(){         i2 = sdr_rx_common_i(); func_ptr = sdr_rx_05;   i5 = (i2 + (i0 + i1) * 3 + i3) >> M_SR; }
void sdr_rx_06(){         i3 = sdr_rx_common_i(); func_ptr = sdr_rx_07;   i6 = i4; i7 = i5; q6 = q4; q7 = q5; }
void sdr_rx_07(){         q0 = sdr_rx_common_q(); func_ptr = sdr_rx_00;   q4 = (q0 + (q2 + q3) * 3 + q1) >> M_SR; }
void sdr_rx_01(){         q1 = sdr_rx_common_q(); func_ptr = sdr_rx_02;   q8 = (q4 + (q6 + q7) * 3 + q5) >> M_SR; }
void sdr_rx_03(){         q2 = sdr_rx_common_q(); func_ptr = sdr_rx_04;   q5 = (q2 + (q0 + q1) * 3 + q3) >> M_SR; }
void sdr_rx_05(){         q3 = sdr_rx_common_q(); func_ptr = sdr_rx_06; process(i8, q8); }
#endif
*/

 /*
static int16_t i_s0za1, i_s0zb0, i_s0zb1, i_s1za1, i_s1zb0, i_s1zb1;
static int16_t q_s0za1, q_s0zb0, q_s0zb1, q_s1za1, q_s1zb0, q_s1zb1, q_ac2;

#define M_SR  1  // CIC N=3
void sdr_rx_00(){ int16_t ac = sdr_rx_common_i(); func_ptr = sdr_rx_01;  int16_t i_s1za0 = (ac + (i_s0za1 + i_s0zb0) * 3 + i_s0zb1) >> M_SR; i_s0za1 = ac; int16_t ac2 = (i_s1za0 + (i_s1za1 + i_s1zb0) * 3 + i_s1zb1); i_s1za1 = i_s1za0; process(ac2, q_ac2); }
void sdr_rx_02(){ int16_t ac = sdr_rx_common_i(); func_ptr = sdr_rx_03;  i_s0zb1 = i_s0zb0; i_s0zb0 = ac; }
void sdr_rx_04(){ int16_t ac = sdr_rx_common_i(); func_ptr = sdr_rx_05;  i_s1zb1 = i_s1zb0; i_s1zb0 = (ac + (i_s0za1 + i_s0zb0) * 3 + i_s0zb1) >> M_SR; i_s0za1 = ac; }
void sdr_rx_06(){ int16_t ac = sdr_rx_common_i(); func_ptr = sdr_rx_07;  i_s0zb1 = i_s0zb0; i_s0zb0 = ac; }
void sdr_rx_01(){ int16_t ac = sdr_rx_common_q(); func_ptr = sdr_rx_02;  q_s0zb1 = q_s0zb0; q_s0zb0 = ac; }
void sdr_rx_03(){ int16_t ac = sdr_rx_common_q(); func_ptr = sdr_rx_04;  q_s1zb1 = q_s1zb0; q_s1zb0 = (ac + (q_s0za1 + q_s0zb0) * 3 + q_s0zb1) >> M_SR; q_s0za1 = ac; }
void sdr_rx_05(){ int16_t ac = sdr_rx_common_q(); func_ptr = sdr_rx_06;  q_s0zb1 = q_s0zb0; q_s0zb0 = ac; }
void sdr_rx_07(){ int16_t ac = sdr_rx_common_q(); func_ptr = sdr_rx_00;  int16_t q_s1za0 = (ac + (q_s0za1 + q_s0zb0) * 3 + q_s0zb1) >> M_SR; q_s0za1 = ac; q_ac2 = (q_s1za0 + (q_s1za1 + q_s1zb0) * 3 + q_s1zb1); q_s1za1 = q_s1za0; }
// */

/*
static int16_t i_s0za1, i_s0za2, i_s0zb0, i_s0zb1, i_s1za1, i_s1za2, i_s1zb0, i_s1zb1;
static int16_t q_s0za1, q_s0za2, q_s0zb0, q_s0zb1, q_s1za1, q_s1za2, q_s1zb0, q_s1zb1, q_ac2;

#define M_SR  0  // CIC N=2
void sdr_rx_00(){ int16_t ac = sdr_rx_common_i(); func_ptr = sdr_rx_01;  int16_t i_s1za0 = (ac + i_s0za1 + i_s0zb0 * 2 + i_s0zb1) >> M_SR; i_s0za1 = ac; int16_t ac2 = (i_s1za0 + i_s1za1 + i_s1zb0 * 2); i_s1za1 = i_s1za0; process(ac2, q_ac2); }
void sdr_rx_02(){ int16_t ac = sdr_rx_common_i(); func_ptr = sdr_rx_03;  i_s0zb0 = ac; }
void sdr_rx_04(){ int16_t ac = sdr_rx_common_i(); func_ptr = sdr_rx_05;  i_s1zb0 = (ac + i_s0za1 + i_s0zb0 * 2) >> M_SR; i_s0za1 = ac; }
void sdr_rx_06(){ int16_t ac = sdr_rx_common_i(); func_ptr = sdr_rx_07;  i_s0zb0 = ac; }
void sdr_rx_01(){ int16_t ac = sdr_rx_common_q(); func_ptr = sdr_rx_02;  q_s0zb0 = ac; }
void sdr_rx_03(){ int16_t ac = sdr_rx_common_q(); func_ptr = sdr_rx_04;  q_s1zb0 = (ac + q_s0za1 + q_s0zb0 * 2) >> M_SR; q_s0za1 = ac; }
void sdr_rx_05(){ int16_t ac = sdr_rx_common_q(); func_ptr = sdr_rx_06;  q_s0zb0 = ac; }
void sdr_rx_07(){ int16_t ac = sdr_rx_common_q(); func_ptr = sdr_rx_00;  int16_t q_s1za0 = (ac + q_s0za1 + q_s0zb0 * 2 + q_s0zb1) >> M_SR; q_s0za1 = ac; q_ac2 = (q_s1za0 + q_s1za1 + q_s1zb0 * 2); q_s1za1 = q_s1za0; }
*/ /*

static int16_t ozi1, ozi2;

inline int16_t sdr_rx_common_q(){
  ADMUX = admux[0]; ADCSRA |= (1 << ADSC); int16_t ac = ADC - 511;
/*ozi2 = ozi1 + ozi2;          // Integrator section - needed?
  ozi1 = ocomb + ozi1;
  OCR1AL = min(max(128 - (ozi2>>5) + 128, 0), 255); */ /*
  return ac;
}

inline int16_t sdr_rx_common_i()
{
  ADMUX = admux[1]; ADCSRA |= (1 << ADSC); int16_t adc = ADC - 511; 
  static int16_t prev_adc;
  int16_t ac = (prev_adc + adc) / 2; prev_adc = adc;
#ifdef AF_OUT
  if(_init){ ocomb=0; ozi1 = 0; ozi2 = 0; } // hack
  ozi2 = ozi1 + ozi2;          // Integrator section
  ozi1 = ocomb + ozi1;
  OCR1AL = min(max((ozi2>>5) + 128, 0), 255);
#endif // AF_OUT
  return ac;
}

 /*
#define M_SR  2  // CIC N=3
static uint8_t nested = false;

void sdr_rx()
{
#ifdef TESTBENCH
  int16_t adc = NCO_I();
#else
  ADMUX = admux[1];  // set MUX for next conversion
  ADCSRA |= (1 << ADSC);    // start next ADC conversion
  int16_t adc = ADC - 511; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
#endif
  func_ptr = sdr_rx_q;    // processing function for next conversion
  sdr_rx_common();
  
  static int16_t prev_adc;
  int16_t corr_adc = (prev_adc + adc) / 2;  // Only for I: correct I/Q sample delay by means of linear interpolation
  prev_adc = adc;
  adc = corr_adc;
  //static int16_t dc;
  //dc += (adc - dc) / 2;  // we lose LSB with this method
  //dc = (3*dc + adc)/4;
  //int16_t ac = adc - dc;     // DC decoupling
  int16_t ac = adc;

  static int16_t s0zb0, s0zb1;
  if(rx_state == 0 || rx_state == 4){  // stage s0: down-sample by 2
    static int16_t s0za1, s0za2;
    int16_t s1za0 = (ac + (s0za1 + s0zb0) * 3 + s0zb1) >> M_SR;           // FA + FB
    //int16_t s1za0 = (ac + s0za1 * 6 + s0za2 + s0zb0 + s0zb1);
    //s0za2 = s0za1;
    s0za1 = ac;
    static int16_t s1zb0, s1zb1;
    if(rx_state == 0){                   // stage s1: down-sample by 2
      static int16_t s1za1, s1za2;
      int16_t ac2 = (s1za0 + (s1za1 + s1zb0) * 3 + s1zb1) >> M_SR; // FA + FB $
      //int16_t ac2 = (s1za0 + s1za1 * 6 + s1za2 + s1zb0 + s1zb1); // FA + FB $
      //s1za2 = s1za1; // $
      s1za1 = s1za0;
      {
        rx_state++;

        static int16_t ac3;
        static int16_t ozd1, ozd2;  // Output stage
        if(_init){ ac3 = 0; ozd1 = 0; ozd2 = 0; _init = 0; } // hack: on first sample init accumlators of further stages (to prevent instability)
        int16_t od1 = ac3 - ozd1; // Comb section
        ocomb = od1 - ozd2;
        interrupts();
        ozd2 = od1;
        ozd1 = ac3;
        
        //if(nested){ return; } // fuse for too many nested interrupts (prevent stack overflow)
        //nested++;
        //interrupts();  // hack: post processing may be extend until next sample time: allow next sample to be processed while postprocessing        

        {
          q_ac2 >>= att2;  // digital gain control
          static int16_t v[14];  // Process Q (down-sampled) samples
          q = v[7];
          // Hilbert transform, BasicDSP model:  outi= fir(inl,  0, 0, 0, 0, 0,  0,  0, 1,   0, 0,   0, 0,  0, 0, 0, 0); outq = fir(inr, 2, 0, 8, 0, 21, 0, 79, 0, -79, 0, -21, 0, -8, 0, -2, 0) / 128;
          qh = ((v[0] - q_ac2) + (v[2] - v[12]) * 4) / 64 + ((v[4] - v[10]) + (v[6] - v[8])) / 8 + ((v[4] - v[10]) * 5 - (v[6] - v[8]) ) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 43dB side-band rejection in 650..3400Hz (@8kSPS) when used in image-rejection scenario; (Hilbert transform require 4 additional bits)
          //qh = ((v[0] - q_ac2) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)
          v[0] = v[1]; v[1] = v[2]; v[2] = v[3]; v[3] = v[4]; v[4] = v[5]; v[5] = v[6]; v[6] = v[7]; v[7] = v[8]; v[8] = v[9]; v[9] = v[10]; v[10] = v[11]; v[11] = v[12]; v[12] = v[13]; v[13] = q_ac2;
        }
        ac2 >>= att2;  // digital gain control
        static int16_t v[7];  // Post processing I and Q (down-sampled) results
        i = v[0]; v[0] = v[1]; v[1] = v[2]; v[2] = v[3]; v[3] = v[4]; v[4] = v[5]; v[5] = v[6]; v[6] = ac2;  // Delay to match Hilbert transform on Q branch
        ac3 = slow_dsp(i + qh);

        //nested--;
        return;
      }
    } else { s1zb1 = s1zb0; s1zb0 = s1za0; } // rx_state == 4 // *4
  } else { s0zb1 = s0zb0; s0zb0 = ac; }  // rx_state == 2 || rx_state == 6  // *4

  rx_state++;
}

void sdr_rx_q()
{
#ifdef TESTBENCH
  int16_t adc = NCO_Q();
#else
  ADMUX = admux[0];  // set MUX for next conversion
  ADCSRA |= (1 << ADSC);    // start next ADC conversion
  int16_t adc = ADC - 511; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
#endif
  func_ptr = sdr_rx;    // processing function for next conversion
  //sdr_rx_common();  //necessary? YES!... Maybe NOT!

  //static int16_t dc;
  //dc += (adc - dc) / 2;  // we lose LSB with this method
  //dc = (3*dc + adc)/4;
  //int16_t ac = adc - dc;     // DC decoupling
  int16_t ac = adc;

  static int16_t s0zb0, s0zb1;
  if(rx_state == 3 || rx_state == 7){  // stage s0: down-sample by 2
    static int16_t s0za1, s0za2;
    int16_t s1za0 = (ac + (s0za1 + s0zb0) * 3 + s0zb1) >> M_SR;           // FA + FB
    //int16_t s1za0 = (ac + s0za1 * 6 + s0za2 + s0zb0 + s0zb1);
    //s0za2 = s0za1;
    s0za1 = ac;
    static int16_t s1zb0, s1zb1;
    if(rx_state == 7){                   // stage s1: down-sample by 2
      static int16_t s1za1, s1za2;
      q_ac2 = (s1za0 + (s1za1 + s1zb0) * 3 + s1zb1) >> M_SR; // FA + FB $
      //q_ac2 = (s1za0 + s1za1 * 6 + s1za2 + s1zb0 + s1zb1); // FA + FB $
      //s1za2 = s1za1; // $
      s1za1 = s1za0;
      rx_state = 0; return;
    } else { s1zb1 = s1zb0; s1zb0 = s1za0; } // rx_state == 3  // *4
  } else { s0zb1 = s0zb0; s0zb0 = ac; }  // rx_state == 1 || rx_state == 5  // *4

  rx_state++;
}

inline void sdr_rx_common()
{
  static int16_t ozi1, ozi2;
  if(_init){ ocomb=0; ozi1 = 0; ozi2 = 0; } // hack
  // Output stage [25% CPU@R=4;Fs=62.5k]
#ifdef SECOND_ORDER_DUC
  ozi2 = ozi1 + ozi2;          // Integrator section
#endif
  ozi1 = ocomb + ozi1;
#ifdef SECOND_ORDER_DUC
  OCR1AL = min(max((ozi2>>5) + 128, 0), 255);  // OCR1AL = min(max((ozi2>>5) + ICR1L/2, 0), ICR1L);  // center and clip wrt PWM working range
#else
  OCR1AL = (ozi1>>5) + 128;
  // OCR1AL = min(max((ozi1>>5) + 128, 0), 255);  // OCR1AL = min(max((ozi2>>5) + ICR1L/2, 0), ICR1L);  // center and clip wrt PWM working range
#endif
}
*/ /*

#else // OLD_RX    //Orginal 2nd-order CIC:
//#define M4  1  // Enable to enable M=4 on second-stage (better alias rejection)

void sdr_rx()
{
  // process I for even samples  [75% CPU@R=4;Fs=62.5k] (excluding the Comb branch and output stage)
  ADMUX = admux[1];  // set MUX for next conversion
  ADCSRA |= (1 << ADSC);    // start next ADC conversion
  int16_t adc = ADC - 511; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  func_ptr = sdr_rx_q;    // processing function for next conversion
  sdr_rx_common();
  
  // Only for I: correct I/Q sample delay by means of linear interpolation
  static int16_t prev_adc;
  int16_t corr_adc = (prev_adc + adc) / 2;
  prev_adc = adc;
  adc = corr_adc;

  //static int16_t dc;
  //dc += (adc - dc) / 2;  // we lose LSB with this method
  //dc = (3*dc + adc)/4;
  //int16_t ac = adc - dc;     // DC decoupling
  int16_t ac = adc;

  int16_t ac2;
  static int16_t z1;
  if(rx_state == 0 || rx_state == 4){  // 1st stage: down-sample by 2
    static int16_t za1;
    int16_t _ac = ac + za1 + z1 * 2;           // 1st stage: FA + FB
    za1 = ac;
    static int16_t _z1;
    if(rx_state == 0){                   // 2nd stage: down-sample by 2
      static int16_t _za1;
      ac2 = _ac + _za1 + _z1 * 2;              // 2nd stage: FA + FB
      _za1 = _ac;
      {
        ac2 >>= att2;  // digital gain control
        // post processing I and Q (down-sampled) results
        static int16_t v[7];
        i = v[0]; v[0] = v[1]; v[1] = v[2]; v[2] = v[3]; v[3] = v[4]; v[4] = v[5]; v[5] = v[6]; v[6] = ac2;  // Delay to match Hilbert transform on Q branch
        
        int16_t ac = i + qh;
        ac = slow_dsp(ac);

        // Output stage
        static int16_t ozd1, ozd2;
        if(_init){ ac = 0; ozd1 = 0; ozd2 = 0; _init = 0; } // hack: on first sample init accumlators of further stages (to prevent instability)
#define SECOND_ORDER_DUC  1
#ifdef SECOND_ORDER_DUC
        int16_t od1 = ac - ozd1; // Comb section
        ocomb = od1 - ozd2;
        ozd2 = od1;
#else
        ocomb = ac - ozd1; // Comb section
#endif
        ozd1 = ac;
      }
    } else _z1 = _ac;
  } else z1 = ac;

  rx_state++;
}

void sdr_rx_q()
{
  // process Q for odd samples  [75% CPU@R=4;Fs=62.5k] (excluding the Comb branch and output stage)
#ifdef TESTBENCH
  int16_t adc = NCO_Q();
#else
  ADMUX = admux[0];  // set MUX for next conversion
  ADCSRA |= (1 << ADSC);    // start next ADC conversion
  int16_t adc = ADC - 511; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
#endif
  func_ptr = sdr_rx;    // processing function for next conversion
#ifdef SECOND_ORDER_DUC
//  sdr_rx_common();  //necessary? YES!... Maybe NOT!
#endif

  //static int16_t dc;
  //dc += (adc - dc) / 2;  // we lose LSB with this method
  //dc = (3*dc + adc)/4;
  //int16_t ac = adc - dc;     // DC decoupling
  int16_t ac = adc;

  int16_t ac2;
  static int16_t z1;
  if(rx_state == 3 || rx_state == 7){  // 1st stage: down-sample by 2
    static int16_t za1;
    int16_t _ac = ac + za1 + z1 * 2;           // 1st stage: FA + FB
    za1 = ac;
    static int16_t _z1;
    if(rx_state == 7){                   // 2nd stage: down-sample by 2
      static int16_t _za1;
      ac2 = _ac + _za1 + _z1 * 2;              // 2nd stage: FA + FB
      _za1 = _ac;
      {
        ac2 >>= att2;  // digital gain control
        // Process Q (down-sampled) samples
        static int16_t v[14];
        q = v[7];
  // Hilbert transform, BasicDSP model:  outi= fir(inl,  0, 0, 0, 0, 0,  0,  0, 1,   0, 0,   0, 0,  0, 0, 0, 0); outq = fir(inr, 2, 0, 8, 0, 21, 0, 79, 0, -79, 0, -21, 0, -8, 0, -2, 0) / 128;
        qh = ((v[0] - ac2) + (v[2] - v[12]) * 4) / 64 + ((v[4] - v[10]) + (v[6] - v[8])) / 8 + ((v[4] - v[10]) * 5 - (v[6] - v[8]) ) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 43dB side-band rejection in 650..3400Hz (@8kSPS) when used in image-rejection scenario; (Hilbert transform require 4 additional bits)
        //qh = ((v[0] - ac2) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)
        for(uint8_t j = 0; j != 13; j++) v[j] = v[j + 1]; v[13] = ac2;
        //v[0] = v[1]; v[1] = v[2]; v[2] = v[3]; v[3] = v[4]; v[4] = v[5]; v[5] = v[6]; v[6] = v[7]; v[7] = v[8]; v[8] = v[9]; v[9] = v[10]; v[10] = v[11]; v[11] = v[12]; v[12] = v[13]; v[13] = ac2;
      }
      rx_state = 0; return;
    } else _z1 = _ac;
  } else z1 = ac;

  rx_state++;
}

inline void sdr_rx_common()
{
  static int16_t ozi1, ozi2;
  if(_init){ ocomb=0; ozi1 = 0; ozi2 = 0; } // hack
  // Output stage [25% CPU@R=4;Fs=62.5k]
#ifdef SECOND_ORDER_DUC
  ozi2 = ozi1 + ozi2;          // Integrator section
#endif
  ozi1 = ocomb + ozi1;
#ifdef SECOND_ORDER_DUC
  OCR1AL = min(max((ozi2>>5) + 128, 0), 255);  // OCR1AL = min(max((ozi2>>5) + ICR1L/2, 0), ICR1L);  // center and clip wrt PWM working range
#else
  OCR1AL = (ozi1>>5) + 128;
  OCR1AL = min(max((ozi1>>5) + 128, 0), 255);  // OCR1AL = min(max((ozi2>>5) + ICR1L/2, 0), ICR1L);  // center and clip wrt PWM working range
#endif
}
#endif //OLD_RX

#endif  //!SIMPLE_RX

#ifdef SIMPLE_RX
volatile uint8_t admux[3];
static uint8_t rx_state = 0;

static struct rx {
  int16_t z1;
  int16_t za1;
  int16_t _z1;
  int16_t _za1;
} rx_inst[2];

void sdr_rx()
{
  static int16_t ocomb;
  static int16_t qh;

  uint8_t b = !(rx_state & 0x01);
  rx* p = &rx_inst[b];
  uint8_t _rx_state;
  int16_t ac;
  if(b){  // rx_state == 0, 2, 4, 6 -> I-stage
    ADMUX = admux[1];  // set MUX for next conversion
    ADCSRA |= (1 << ADSC);    // start next ADC conversion
    ac = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH

    //sdr_common
    static int16_t ozi1, ozi2;
    if(_init){ ocomb=0; ozi1 = 0; ozi2 = 0; } // hack
    // Output stage [25% CPU@R=4;Fs=62.5k]
    #define SECOND_ORDER_DUC 1
    #ifdef SECOND_ORDER_DUC
    ozi2 = ozi1 + ozi2;          // Integrator section
    #endif
    ozi1 = ocomb + ozi1;
    #ifdef SECOND_ORDER_DUC
    OCR1AL = min(max((ozi2>>5) + 128, 0), 255);  // OCR1AL = min(max((ozi2>>5) + ICR1L/2, 0), ICR1L);  // center and clip wrt PWM working range
    #else
    OCR1AL = (ozi1>>5) + 128;
    //OCR1AL = min(max((ozi1>>5) + 128, 0), 255);  // OCR1AL = min(max((ozi2>>5) + ICR1L/2, 0), ICR1L);  // center and clip wrt PWM working range
    #endif
    // Only for I: correct I/Q sample delay by means of linear interpolation
    static int16_t prev_adc;
    int16_t corr_adc = (prev_adc + ac) / 2;
    prev_adc = ac;
    ac = corr_adc;
    _rx_state = ~rx_state;
  } else {
    ADMUX = admux[0];  // set MUX for next conversion
    ADCSRA |= (1 << ADSC);    // start next ADC conversion
    ac = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
    _rx_state = rx_state;
  }
    
  if(_rx_state & 0x02){  // rx_state == I: 0, 4  Q: 3, 7  1st stage: down-sample by 2
    int16_t _ac = ac + p->za1 + p->z1 * 2;           // 1st stage: FA + FB
    p->za1 = ac;
    if(_rx_state & 0x04){                   // rx_state == I: 0  Q:7   2nd stage: down-sample by 2
      int16_t ac2 = _ac + p->_za1 + p->_z1 * 2;              // 2nd stage: FA + FB
      p->_za1 = _ac;
      if(b){
        // post processing I and Q (down-sampled) results
        ac2 >>= att2;  // digital gain control
        // post processing I and Q (down-sampled) results
        static int16_t v[7];
        i = v[0]; v[0] = v[1]; v[1] = v[2]; v[2] = v[3]; v[3] = v[4]; v[4] = v[5]; v[5] = v[6]; v[6] = ac2;  // Delay to match Hilbert transform on Q branch

        int16_t ac = i + qh;
        ac = slow_dsp(ac);

        // Output stage
        static int16_t ozd1, ozd2;
        if(_init){ ac = 0; ozd1 = 0; ozd2 = 0; _init = 0; } // hack: on first sample init accumlators of further stages (to prevent instability)
        #ifdef SECOND_ORDER_DUC
        int16_t od1 = ac - ozd1; // Comb section
        ocomb = od1 - ozd2;
        ozd2 = od1;
        #else
        ocomb = ac - ozd1; // Comb section
        #endif
        ozd1 = ac;
      } else {
        ac2 >>= att2;  // digital gain control
        // Process Q (down-sampled) samples
        static int16_t v[14];
        q = v[7];
        qh = ((v[0] - ac2) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)
        for(uint8_t j = 0; j != 13; j++) v[j] = v[j + 1]; v[13] = ac2;
      }
    } else p->_z1 = _ac;
  } else p->z1 = ac;  // rx_state == I: 2, 6  Q: 1, 5

  rx_state++;
}
//#pragma GCC push_options
//#pragma GCC optimize ("Ofast")  // compiler-optimization for speed
//#pragma GCC pop_options  // end of DSP section
// */ /*
#endif //SIMPLE_RX

ISR(TIMER2_COMPA_vect)  // Timer2 COMPA interrupt
{
  func_ptr();
#ifdef DEBUG
  numSamples++;
#endif
}

#pragma GCC pop_options  // end of DSP section

/*ISR (TIMER2_COMPA_vect  ,ISR_NAKED) {
asm("push r24         \n\t"
    "lds r24,  0\n\t"
    "sts 0xB4, r24    \n\t"
    "pop r24          \n\t"
    "reti             \n\t");
}*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Below a radio-specific implementation based on the above components (seperation of concerns)
//
// Feel free to replace it with your own custom radio implementation :-)

//void inline lcd_blanks(){ lcd.print(F("        ")); }






volatile bool change = true;
volatile int32_t freq = 14000000;
static int32_t vfo[] = { 7074000, 14074000 };
static uint8_t vfomode[] = { USB, USB };
enum vfo_t { VFOA=0, VFOB=1, SPLIT=2 };
volatile uint8_t vfosel = VFOA;
volatile int16_t rit = 0;

void start_rx()
{
  _init = 1;
  rx_state = 0;
  func_ptr = sdr_rx_00;  //enable RX DSP/SDR
  adc_start(2, true, F_ADC_CONV*4); admux[2] = ADMUX;  // Note that conversion-rate for TX is factors more
  if(dsp_cap == SDR){
//#define SWAP_RX_IQ 1    // Swap I/Q ADC inputs, flips RX sideband
#ifdef SWAP_RX_IQ
    adc_start(1, !(att == 1)/*true*/, F_ADC_CONV); admux[0] = ADMUX;
    adc_start(0, !(att == 1)/*true*/, F_ADC_CONV); admux[1] = ADMUX;
#else
    adc_start(0, !(att == 1)/*true*/, F_ADC_CONV); admux[0] = ADMUX;
    adc_start(1, !(att == 1)/*true*/, F_ADC_CONV); admux[1] = ADMUX;
#endif //SWAP_RX_IQ
  } else { // ANALOG, DSP
    adc_start(0, false, F_ADC_CONV); admux[0] = ADMUX; admux[1] = ADMUX;
  }
  timer1_start(F_SAMP_PWM);
  timer2_start(F_SAMP_RX);  
  TCCR1A &= ~(1 << COM1B1); digitalWrite(KEY_OUT, LOW); // disable KEY_OUT PWM
}

int16_t _centiGain = 0;

uint8_t txdelay = 0;
uint8_t semi_qsk = false;
uint32_t semi_qsk_timeout = 0;

// void switch_rxtx(uint8_t tx_enable){
//   TIMSK2 &= ~(1 << OCIE2A);  // disable timer compare interrupt
//   //delay(1);
//   delayMicroseconds(20); // wait until potential RX interrupt is finalized
//   noInterrupts();
// #ifdef TX_DELAY
// #ifdef SEMI_QSK
//   if(!(semi_qsk_timeout))
// #endif
//     if((txdelay) && (tx_enable) && (!(tx)) && (!(practice))){  // key-up TX relay in advance before actual transmission
//       digitalWrite(RX, LOW); // TX (disable RX)
// #ifdef NTX
//       digitalWrite(NTX, LOW);  // TX (enable TX)
// #endif //NTX
// #ifdef PTX
//       digitalWrite(PTX, HIGH);  // TX (enable TX)
// #endif //PTX
//       lcd.setCursor(15, 1); lcd.print('D');  // note that this enables interrupts again.
//       interrupts();    //hack.. to allow delay()
//       delay(F_MCU / 16000000 * txdelay);
//       noInterrupts();  //end of hack
//     }
// #endif //TX_DELAY
//   tx = tx_enable;
//   if(tx_enable){  // tx
//     _centiGain = centiGain;  // backup AGC setting
// #ifdef SEMI_QSK
//     semi_qsk_timeout = 0;
// #endif
//     switch(mode){
//       case USB:
//       case LSB: func_ptr = dsp_tx; break;
//       case CW:  func_ptr = dsp_tx_cw; break;
//       case AM:  func_ptr = dsp_tx_am; break;
//       case FM:  func_ptr = dsp_tx_fm; break;
//     }
//   } else {  // rx
//     if((mode == CW) && (!(semi_qsk_timeout))){
// #ifdef SEMI_QSK
// #ifdef KEYER
//       semi_qsk_timeout = millis() + ditTime * 8;
// #else
//       semi_qsk_timeout = millis() + 8 * 8;  // no keyer? assume dit-time of 20 WPM
// #endif //KEYER
// #endif //SEMI_QSK
//       if(semi_qsk) func_ptr = dummy; else func_ptr = sdr_rx_00;
//     } else {
//       centiGain = _centiGain;  // restore AGC setting
// #ifdef SEMI_QSK
//       semi_qsk_timeout = 0;
// #endif
//       func_ptr = sdr_rx_00;
//     }
//   }
//   if((!dsp_cap) && (!tx_enable) && vox) func_ptr = dummy; //hack: for SSB mode, disable dsp_rx during vox mode enabled as it slows down the vox loop too much!
//   interrupts();
//   if(tx_enable) ADMUX = admux[2];
//   else _init = 1;
//   rx_state = 0;
// #ifdef CW_DECODER
//   if((cwdec) && (mode == CW)){ filteredstate = tx_enable; dec2(); }
// #endif  //CW_DECODER
  
//   if(tx_enable){ // tx
//     if(practice){
//       digitalWrite(RX, LOW); // TX (disable RX)
//       lcd.setCursor(15, 1); lcd.print('P');
//       si5351.SendRegister(SI_CLK_OE, TX0RX0);
//       // Do not enable PWM (KEY_OUT), do not enble CLK2
//     } else
//     {
//       digitalWrite(RX, LOW); // TX (disable RX)
// #ifdef NTX
//       digitalWrite(NTX, LOW);  // TX (enable TX)
// #endif //NTX
// #ifdef PTX
//       digitalWrite(PTX, HIGH);  // TX (enable TX)
// #endif //PTX
//       lcd.setCursor(15, 1); lcd.print('T');
//       if(mode == CW){ si5351.freq_calc_fast(-cw_offset); si5351.SendPLLRegisterBulk(); } // for CW, TX at freq
// #ifdef RIT_ENABLE
//       else if(rit){ si5351.freq_calc_fast(0); si5351.SendPLLRegisterBulk(); }
// #endif //RIT_ENABLE
//       si5351.SendRegister(SI_CLK_OE, TX1RX0);
//       OCR1AL = 0x80; // make sure SIDETONE is set at 2.5V
//       if((!mox) && (mode != CW)) TCCR1A &= ~(1 << COM1A1); // disable SIDETONE, prevent interference during SSB TX
//       TCCR1A |= (1 << COM1B1);  // enable KEY_OUT PWM
// #ifdef _SERIAL
//       if(cat_active){ DDRC &= ~(1<<2); } // disable PC2, so that ADC2 can be used as mic input
// #endif
//     }
//   } else {  // rx
// #ifdef KEY_CLICK
//       if(OCR1BL != 0) {
//        for(uint16_t i = 0; i != 31; i++) {   // ramp down of amplitude: soft falling edge to prevent key clicks
//          OCR1BL = lut[pgm_read_byte_near(&ramp[i])];
//           delayMicroseconds(60);
//        }
//       }
// #endif //KEY_CLICK
//       TCCR1A |= (1 << COM1A1);  // enable SIDETONE (was disabled to prevent interference during ssb tx)
//       TCCR1A &= ~(1 << COM1B1); digitalWrite(KEY_OUT, LOW); // disable KEY_OUT PWM, prevents interference during RX
//       OCR1BL = 0; // make sure PWM (KEY_OUT) is set to 0%
// #ifdef QUAD
// #ifdef TX_CLK0_CLK1
//       si5351.SendRegister(16, 0x0f);  // disable invert on CLK0
//       si5351.SendRegister(17, 0x0f);  // disable invert on CLK1
// #else
//       si5351.SendRegister(18, 0x0f);  // disable invert on CLK2
// #endif  //TX_CLK0_CLK1
// #endif //QUAD
//       si5351.SendRegister(SI_CLK_OE, TX0RX1);
// #ifdef SEMI_QSK
//       if((!semi_qsk_timeout) || (!semi_qsk))   // enable RX when no longer in semi-qsk phase; so RX and NTX/PTX outputs are switching only when in RX mode
// #endif //SEMI_QSK
//       {
//         digitalWrite(RX, !(att == 2)); // RX (enable RX when attenuator not on)
// #ifdef NTX
//         digitalWrite(NTX, HIGH);  // RX (disable TX)
// #endif //NTX
// #ifdef PTX
//         digitalWrite(PTX, LOW);   // TX (disable TX)
// #endif //PTX
//       }
// #ifdef RIT_ENABLE
//       si5351.freq_calc_fast(rit); si5351.SendPLLRegisterBulk();  // restore original PLL RX frequency
// #else
//       si5351.freq_calc_fast(0); si5351.SendPLLRegisterBulk();  // restore original PLL RX frequency
// #endif //RIT_ENABLE
// #ifdef SWR_METER
//       if(swrmeter > 0) { show_banner(); lcd.print("                "); }
// #endif
//       lcd.setCursor(15, 1); lcd.print((vox) ? 'V' : 'R');
// #ifdef _SERIAL
//       if(!vox) if(cat_active){ DDRC |= (1<<2); } // enable PC2, so that ADC2 is pulled-down so that CAT TX is not disrupted via mic input
// #endif
//   }
//   OCR2A = ((F_CPU / 64) / ((tx_enable) ? F_SAMP_TX : F_SAMP_RX)) - 1;
//   TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt TIMER2_COMPA_vect
// }

uint8_t rx_ph_q = 90;



uint8_t prev_bandval = 3;
uint8_t bandval = 3;
#define N_BANDS 11

#ifdef CW_FREQS_QRP
uint32_t band[N_BANDS] = { /*472000,*/ 1810000, 3560000, 5351500, 7030000, 10106000, 14060000, 18096000, 21060000, 24906000, 28060000, 50096000/*, 70160000, 144060000*/ };  // CW QRP freqs
#else
#ifdef CW_FREQS_FISTS
uint32_t band[N_BANDS] = { /*472000,*/ 1818000, 3558000, 5351500, 7028000, 10118000, 14058000, 18085000, 21058000, 24908000, 28058000, 50058000/*, 70158000, 144058000*/ };  // CW FISTS freqs
#else
uint32_t band[N_BANDS] = { /*472000,*/ 1840000, 3573000, 5357000, 7074000, 10136000, 14074000, 18100000, 21074000, 24915000, 28074000, 50313000/*, 70101000, 144125000*/ };  // FT8 freqs
#endif
#endif



// void powerDown()
// { // Reduces power from 110mA to 70mA (back-light on) or 30mA (back-light off), remaining current is probably opamp quiescent current
//   lcd.setCursor(0, 1); lcd.print(F("Power-off 73 :-)")); lcd_blanks();

//   MCUSR = ~(1<<WDRF);  // MSY be done before wdt_disable()
//   wdt_disable();   // WDTON Fuse High bit need to be 1 (0xD1), if NOT it will override and set WDE=1; WDIE=0, meaning MCU will reset when watchdog timer is zero, and this seems to happen when wdt_disable() is called
  
//   timer2_stop();
//   timer1_stop();
//   adc_stop();

//   si5351.powerDown();

//   delay(1500);

//   // Disable external interrupts INT0, INT1, Pin Change
//   PCICR = 0;
//   PCMSK0 = 0;
//   PCMSK1 = 0;
//   PCMSK2 = 0;
//   // Disable internal interrupts
//   TIMSK0 = 0;
//   TIMSK1 = 0;
//   TIMSK2 = 0;
//   WDTCSR = 0;
//   // Enable BUTTON Pin Change interrupt
//   *digitalPinToPCMSK(BUTTONS) |= (1<<digitalPinToPCMSKbit(BUTTONS));
//   *digitalPinToPCICR(BUTTONS) |= (1<<digitalPinToPCICRbit(BUTTONS));

//   // Power-down sub-systems
//   PRR = 0xff;

//   lcd.noDisplay();
//   PORTD &= ~0x08; // disable backlight

//   set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//   sleep_enable();
//   interrupts();
//   sleep_bod_disable();
//   //MCUCR |= (1<<BODS) | (1<<BODSE);  // turn bod off by settings BODS, BODSE; note BODS is reset after three clock-cycles, so quickly go to sleep before it is too late
//   //MCUCR &= ~(1<<BODSE);  // must be done right before sleep
//   sleep_cpu();  // go to sleep mode, wake-up by either INT0, INT1, Pin Change, TWI Addr Match, WDT, BOD
//   sleep_disable();

//   //void(* reset)(void) = 0; reset();   // soft reset by calling reset vector (does not reset registers to defaults)
//   do { wdt_enable(WDTO_15MS); for(;;); } while(0);  // soft reset by trigger watchdog timeout
// }

// void show_banner(){
//   lcd.setCursor(0, 0);
// #ifdef QCX
//   lcd.print(F("QCX"));
//   const char* cap_label[] = { "SSB", "DSP", "SDR" };
//   if(ssb_cap || dsp_cap){ lcd.print('-'); lcd.print(cap_label[dsp_cap]); }
// #else
//   lcd.print(F("uSDX"));
// #endif //QCX
//   lcd.print('\x01'); lcd_blanks(); lcd_blanks();
// }

const char* vfosel_label[] = { "A", "B"/*, "Split"*/ };
const char* mode_label[5] = { "LSB", "USB", "CW ", "FM ", "AM " };

// inline void display_vfo(int32_t f){
//   lcd.setCursor(0, 1);
//   lcd.print((rit) ? ' ' : ((vfosel%2)|((vfosel==SPLIT) & tx)) ? '\x07' : '\x06');  // RIT, VFO A/B

//   int32_t scale=10e6;
//   if(rit){
//     f = rit;
//     scale=1e3;  // RIT frequency
//     lcd.print(F("RIT ")); lcd.print(rit < 0 ? '-' : '+');
//   } else {
//     if(f/scale == 0){ lcd.print(' '); scale/=10; }  // Initial space instead of zero
//   }
//   for(; scale!=1; f%=scale, scale/=10){
//     lcd.print(abs(f/scale));
//     if(scale == (int32_t)1e3 || scale == (int32_t)1e6) lcd.print(',');  // Thousands separator
//   }
  
//   lcd.print(' '); lcd.print(mode_label[mode]); lcd.print(' ');
//   lcd.setCursor(15, 1); lcd.print((vox) ? 'V' : 'R');
// }

volatile uint8_t event;
//volatile uint8_t menumode = 0;  // 0=not in menu, 1=selects menu item, 2=selects parameter value
volatile uint8_t prev_menumode = 0;
volatile int8_t menu = 0;  // current parameter id selected in menu

#define pgm_cache_item(addr, sz) byte _item[sz]; memcpy_P(_item, addr, sz);  // copy array item from PROGMEM to SRAM
#define get_version_id() ((VERSION[0]-'1') * 2048 + ((VERSION[2]-'0')*10 + (VERSION[3]-'0')) * 32 +  ((VERSION[4]) ? (VERSION[4] - 'a' + 1) : 0) * 1)  // converts VERSION string with (fixed) format "9.99z" into uint16_t (max. values shown here, z may be removed) 

uint8_t eeprom_version;
#define EEPROM_OFFSET 0x150  // avoid collision with QCX settings, overwrites text settings though
int eeprom_addr;

// Support functions for parameter and menu handling
enum action_t { UPDATE, UPDATE_MENU, NEXT_MENU, LOAD, SAVE, SKIP, NEXT_CH };

// // output menuid in x.y format
// void printmenuid(uint8_t menuid){
//   static const char seperator[] = {'.', ' '};
//   uint8_t ids[] = {(uint8_t)(menuid >> 4), (uint8_t)(menuid & 0xF)};
//   for(int i = 0; i < 2; i++){
//     uint8_t id = ids[i];
//     if(id >= 10){
//       id -= 10;
//       lcd.print('1');
//     }
//     lcd.print(char('0' + id));
//     lcd.print(seperator[i]);
//   }
// }

// void printlabel(uint8_t action, uint8_t menuid, const __FlashStringHelper* label){
//   if(action == UPDATE_MENU){
//     lcd.setCursor(0, 0);
//     printmenuid(menuid);
//     lcd.print(label); lcd_blanks(); lcd_blanks();
//     lcd.setCursor(0, 1); // value on next line
//     if(menumode >= 2) lcd.print('>');
//   } else { // UPDATE (not in menu)
//     lcd.setCursor(0, 1); lcd.print(label); lcd.print(F(": "));
//   }
// }

// void initPins(){
//   // initialize
//   digitalWrite(SIG_OUT, LOW);
//   digitalWrite(RX, HIGH);
//   digitalWrite(KEY_OUT, LOW);
//   digitalWrite(SIDETONE, LOW);

//   // pins
//   pinMode(SIDETONE, OUTPUT);
//   pinMode(SIG_OUT, OUTPUT);
//   pinMode(RX, OUTPUT);
//   pinMode(KEY_OUT, OUTPUT);
// #ifdef ONEBUTTON
//   pinMode(BUTTONS, INPUT_PULLUP);  // rotary button
// #else
//   pinMode(BUTTONS, INPUT);  // L/R/rotary button
// #endif
//   pinMode(DIT, INPUT_PULLUP);
//   pinMode(DAH, INPUT);  // pull-up DAH 10k via AVCC
//   //pinMode(DAH, INPUT_PULLUP); // Could this replace D4? But leaks noisy VCC into mic input!

//   digitalWrite(AUDIO1, LOW);  // when used as output, help can mute RX leakage into AREF
//   digitalWrite(AUDIO2, LOW);
//   pinMode(AUDIO1, INPUT);
//   pinMode(AUDIO2, INPUT);

// #ifdef NTX
//   digitalWrite(NTX, HIGH);
//   pinMode(NTX, OUTPUT);
// #endif //NTX
// #ifdef PTX
//   digitalWrite(PTX, LOW);
//   pinMode(PTX, OUTPUT);
// #endif //PTX
// #ifdef SWR_METER
//   pinMode(PIN_FWD, INPUT);
//   pinMode(PIN_REF, INPUT);
// #endif
// #ifdef OLED  // assign unused LCD pins
//   pinMode(PD4, OUTPUT);
//   pinMode(PD5, OUTPUT);
// #endif
// }

// void fatal(const __FlashStringHelper* msg, int value = 0, char unit = '\0') {
//   lcd.setCursor(0, 1);
//   lcd.print('!'); lcd.print('!');
//   lcd.print(msg);
//   if(unit != '\0') {
//     lcd.print('=');
//     lcd.print(value);
//     lcd.print(unit);
//   }
//   lcd_blanks();
//   delay(1500);
//   wdt_reset();
// }

// //refresh LUT based on pwm_min, pwm_max
// void build_lut()
// {
//   for(uint16_t i = 0; i != 256; i++)    // refresh LUT based on pwm_min, pwm_max
//     lut[i] = (i * (pwm_max - pwm_min)) / 255 + pwm_min;
//     //lut[i] = min(pwm_max, (float)106*log(i) + pwm_min);  // compressed microphone output: drive=0, pwm_min=115, pwm_max=220
// }

**
 * 
 **/
void setup()
{
  digitalWrite(KEY_OUT, LOW);  // for safety: to prevent exploding PA MOSFETs, in case there was something still biasing them.
  si5351.powerDown();  // disable all CLK outputs (especially needed for si5351 variants that has CLK2 enabled by default, such as Si5351A-B04486-GT)

  //wdt_enable(WDTO_4S);  // TODO: Enable watchdog at the end of the project
  //set_debug();          // TODO: Rewrite debug for ESP32
  // disable external interrupts
  //PCMSK0 = 0;           // TODO: Rewrite it for ESP32
  //PCMSK1 = 0;           // TODO: Rewrite it for ESP32
  //PCMSK2 = 0;           // TODO: Rewrite it for ESP32

  encoder_setup();        // TODO: Rewrite it for ESP32

  initPins();             // TODO: Rewrite it for ESP32
    /* 
  delay(100);             // at least 40ms after power rises above 2.7V before sending commands
  lcd.begin(16, 4);       // Init LCD // TODO: Rewrite it for ESP32
#ifndef OLED
  for(i = 0; i != N_FONTS; i++){  // Init fonts
    pgm_cache_item(fonts[i], 8);
    lcd.createChar(0x01 + i, _item);
  }
#endif

  show_banner();
  lcd.setCursor(7, 0); lcd.print(F(" R")); lcd.print(F(VERSION)); lcd_blanks();

  set_QCX();               // TODO: Rewrite it for ESP32         

  drive = 4;  // Init settings
#ifdef QCX                // TODO: Rewrite it for ESP32
  if(!ssb_cap){ vfomode[0] = CW; vfomode[1] = CW; filt = 4; stepsize = STEP_500; }
  if(dsp_cap != SDR) pwm_max = 255; // implies that key-shaping circuit is probably present, so use full-scale
  if(dsp_cap == DSP) volume = 10;
  if(!dsp_cap) cw_tone = 2;   // use internal 700Hz QCX filter, so use same offset and keyer tone
#endif //QCX

  cw_offset = tones[cw_tone];
  //freq = bands[band];
  
  // Load parameters from EEPROM, reset to factory defaults when stored values are from a different version
  paramAction(LOAD, VERS);
  if((eeprom_version != get_version_id()) || _digitalRead(BUTTONS)){  // EEPROM clean: if rotary-key pressed or version signature in EEPROM does NOT corresponds with this firmware
    eeprom_version = get_version_id();
    //for(int n = 0; n != 1024; n++){ eeprom_write_byte((uint8_t *) n, 0); wdt_reset(); } //clean EEPROM
    //eeprom_write_dword((uint32_t *)EEPROM_OFFSET/3, 0x000000);
    paramAction(SAVE);  // save default parameter values
    lcd.setCursor(0, 1); lcd.print(F("Reset settings.."));
    delay(500); wdt_reset();
  } else {
    paramAction(LOAD);  // load all parameters
  }
  //if(abs((int32_t)F_XTAL - (int32_t)si5351.fxtal) > 50000){ si5351.fxtal = F_XTAL; }  // if F_XTAL frequency deviates too much with actual setting -> use default
  si5351.iqmsa = 0;  // enforce PLL reset
  change = true;
  prev_bandval = bandval;
  vox = false;  // disable VOX
  nr = 0; // disable NR
  rit = false;  // disable RIT
  freq = vfo[vfosel%2];
  mode = vfomode[vfosel%2];

#ifdef TX_ENABLE
  build_lut();
#endif

  show_banner();  // remove release number

  */
  start_rx();     // TODO: Rewrite it for ESP32

  set_key();      // TODO: Rewrite it for ESP32

static int32_t _step = 0;
}
/**
 ****************************************************************************************************************************
 **/
void loop()
{
//  run_vox();            // TODO: Rewrite it for ESP32

//  run_cw_decode();      // TODO: Rewrite it for ESP32

//  run_cw_keyer();       // TODO: Rewrite it for ESP32

//  run_control();        // TODO: Rewrite it for ESP32

/* BACKLOG:
code definitions and re-use for comb, integrator, dc decoupling, arctan
refactor main()
agc based on rms256, agc/smeter after filter
noisefree integrator (rx audio out) in lower range
raised cosine tx amp for cw, 4ms tau seems enough: http://fermi.la.asu.edu/w9cf/articles/click/index.html
32 bin fft
dynamic range cw
att extended agc
Split
undersampling, IF-offset
K2/TS480 CAT control
faster RX-TX switch to support CW
usdx API demo code
scan
move last bit of arrays into flash? https://web.archive.org/web/20180324010832/https://www.microchip.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_rom_array.html
u-law in RX path?: http://dystopiancode.blogspot.com/2012/02/pcm-law-and-u-law-companding-algorithms.html
Arduino library?
1. RX bias offset correction by measurement avg, 2. charge decoupling cap. by resetting to 0V and setting 5V for a certain amount of (charge) time
add 1K (500R?) to GND at TTL RF output to keep zero-level below BS170 threshold
additional PWM output for potential BOOST conversion
squelch gating
more buttons
s-meter offset vs DC bal.
keyer with interrupt-driven timers (to reduce jitter)

Analyse assembly:
/home/guido/Downloads/arduino-1.8.10/hardware/tools/avr/bin/avr-g++ -S -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10810 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR -I/home/guido/Downloads/arduino-1.8.10/hardware/arduino/avr/cores/arduino -I/home/guido/Downloads/arduino-1.8.10/hardware/arduino/avr/variants/standard /tmp/arduino_build_483134/sketch/QCX-SSB.ino.cpp -o /tmp/arduino_build_483134/sketch/QCX-SSB.ino.cpp.txt

Rewire/code I/Q clk pins so that a Div/1 and Div/2 scheme is used instead of 0 and 90 degrees phase shift
10,11,13,12   10,11,12,13  (pin)
Q- I+ Q+ I-   Q- I+ Q+ I-
90 deg.shift  div/2@S1(pin2)

50MHz LSB OK, USB NOK

atmega328p signature: https://forum.arduino.cc/index.php?topic=341799.15   https://www.eevblog.com/forum/microcontrollers/bootloader-on-smd-atmega328p-au/msg268938/#msg268938 https://www.avrfreaks.net/forum/undocumented-signature-row-contents

Alain k1fm AGC sens issue:  https://groups.io/g/ucx/message/3998   https://groups.io/g/ucx/message/3999
txdelay when vox is on (disregading the tx>0 state due to ssb() overrule, instead use RX-digitalinput)
Adrian: issue #41, set cursor just after writing 'R' when smeter is off, and (menumode == 0)
Konstantinos: backup/restore vfofilt settings when changing vfo.
Bob: 2mA for clk0/1 during RX
Uli: accuracate voltages during diag

agc behind filter
vcc adc extend. power/curr measurement
swr predistort eff calc
block ptt while in vox mode

adc bias error and potential error correction
noise burst on tx
https://groups.io/g/ucx/topic/81030243#6265

for (size_t i = 0; i < 9; i++) id[i] = boot_signature_byte_get(0x0E + i + (i > 5));

// https://www.ti.com/lit/ds/symlink/ina226.pdf
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
#include <Adafruit_INA219.h>
Adafruit_INA219 ina219;
float pwr;
float Eff;
float Vinc, Vref = 0, SWR;
float k = 0.85;
float busvoltage = 0;
float current_mA = 0;
float power_mW = 0;

void setup() {
  ina219.begin();
  lcd.init();
  lcd.backlight();
}
void loop() {
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  Vinc = analogRead(3);
  Vref = analogRead(2);
  SWR = (Vinc + Vref) / (Vinc - Vref);
  Vinc = ((Vinc * 5.0) / 1024.0) + 0.5;
  pwr = ((((Vinc) * (Vinc)) - 0.25 ) * k);
  Eff = (pwr) / ((power_mW) / 1000) * 100;
  if (pwr > 0 ) (pwr = pwr + 0.25);
  lcd.setCursor(0, 0);
  lcd.print("SWR     :1 / P     W");
  lcd.setCursor(4, 0);
  lcd.print(SWR);
  lcd.setCursor(15, 0);
  lcd.print(pwr);

  lcd.setCursor (0, 2);
  //lcd.print ("Vss = ");
  lcd.print(busvoltage);
  lcd.print("V  ");

  lcd.setCursor(8, 2);
  lcd.print (-((current_mA) / 1000));
  lcd.print("A  ");

  lcd.setCursor(15, 2);
  lcd.print((power_mW) / 1000);
  lcd.print("W   ");

  lcd.setCursor(0, 1);
  lcd.print("Efficiency = ");
  lcd.print(Eff);
  lcd.setCursor(17, 1);
  lcd.print("%   ");

  delay(300);
}
*/
