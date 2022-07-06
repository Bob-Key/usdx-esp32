// void actionCommon(uint8_t action, uint8_t *ptr, uint8_t size){
//   //uint8_t n;
//   switch(action){
//     case LOAD:
//       //for(n = size; n; --n) *ptr++ = eeprom_read_byte((uint8_t *)eeprom_addr++);
//       eeprom_read_block((void *)ptr, (const void *)eeprom_addr, size);
//       break;
//     case SAVE:
//       //noInterrupts();
//       //for(n = size; n; --n){ wdt_reset(); eeprom_write_byte((uint8_t *)eeprom_addr++, *ptr++); }
//       eeprom_write_block((const void *)ptr, (void *)eeprom_addr, size);
//       //interrupts();
//       break;
//     case SKIP:
//       //eeprom_addr += size;
//       break;
//   }
//   eeprom_addr += size;
// }

template<typename T> void paramAction(uint8_t action, volatile T& value, uint8_t menuid, const __FlashStringHelper* label, const char* enumArray[], int32_t _min, int32_t _max, bool continuous){
  switch(action){
    case UPDATE:
    case UPDATE_MENU:
      if(((int32_t)value + encoder_val) < _min) value = (continuous) ? _max : _min;
      else if(((int32_t)value + encoder_val) > _max) value = (continuous) ? _min : _max;
      else value = (int32_t)value + encoder_val;
      encoder_val = 0;

      lcd.noCursor();
      printlabel(action, menuid, label);  // print normal/menu label
      if(enumArray == NULL){  // print value
        if((_min < 0) && (value >= 0)) lcd.print('+');  // add + sign for positive values, in case negative values are supported
        lcd.print(value);
      } else {
        lcd.print(enumArray[value]);
      }
      lcd_blanks(); lcd_blanks(); //lcd.setCursor(0, 1);
      //if(action == UPDATE) paramAction(SAVE, value, menuid, label, enumArray, _min, _max, continuous, init_val);
      break;
    default:
      actionCommon(action, (uint8_t *)&value, sizeof(value));
      break;
  }
}

#ifdef MENU_STR
static uint8_t pos = 0;
void paramAction(uint8_t action, char* value, uint8_t menuid, const __FlashStringHelper* label, uint8_t size){
  const uint8_t _min = ' '; const uint8_t _max = 'Z';
  switch(action){
    case NEXT_CH:
      if(pos < size) pos++;  // allow to go to next character when string size allows and when current character is not string end
      action = UPDATE_MENU; //fall-through next case
    case UPDATE:
    case UPDATE_MENU:
      if(menumode != 3) pos = 0;
      if(menumode == 2) menumode = 3; // hack: for strings enter in edit mode
      if(((value[pos] + encoder_val) < _min) || ((value[pos] + encoder_val) == 0)) value[pos] = _min;
      else if((value[pos] + encoder_val) > _max) value[pos] = _max;
      else value[pos] = value[pos] + encoder_val;
      encoder_val = 0;

      printlabel(action, menuid, label);  // print normal/menu label
      for(int i = 0; i != 13; i++){ char ch = value[(pos / 8) * 8 + i]; if(ch) lcd.print(ch); else break; } // print value
      //lcd.print(&value[(pos / 8) * 8]); // print value
      lcd.print('\x01');  // print terminator
      lcd_blanks();
      lcd.setCursor((pos % 8) + (menumode >= 2), 1); lcd.cursor();
      break;
    case SAVE:
      for(uint8_t i = size; i > 0; i--){
        if((value[i-1] == ' ') || (value[i-1] == 0)) value[i-1] = 0;  // remove trailing spaces
        else break; // stop once content found
      }
      //fall-through next case
    default:
      actionCommon(action, (uint8_t *)value, size);
      break;
  }
}
#endif //MENU_STR

static uint32_t save_event_time = 0;
static uint8_t vox_tx = 0;
static uint8_t vox_sample = 0;
static uint16_t vox_adc = 0;

static uint8_t pwm_min = 0;    // PWM value for which PA reaches its minimum: 29 when C31 installed;   0 when C31 removed;   0 for biasing BS170 directly
#ifdef QCX
static uint8_t pwm_max = 255;  // PWM value for which PA reaches its maximum: 96 when C31 installed; 255 when C31 removed;
#else
static uint8_t pwm_max = 128;  // PWM value for which PA reaches its maximum:                                              128 for biasing BS170 directly
#endif

const char* offon_label[2] = {"OFF", "ON"};
#if(F_MCU > 16000000)
const char* filt_label[N_FILT+1] = { "Full", "3000", "2400", "1800", "500", "200", "100", "50" };
#else
const char* filt_label[N_FILT+1] = { "Full", "2400", "2000", "1500", "500", "200", "100", "50" };
#endif
const char* band_label[N_BANDS] = { "160m", "80m", "60m", "40m", "30m", "20m", "17m", "15m", "12m", "10m", "6m" };
const char* stepsize_label[] = { "10M", "1M", "0.5M", "100k", "10k", "1k", "0.5k", "100", "10", "1" };
const char* att_label[] = { "0dB", "-13dB", "-20dB", "-33dB", "-40dB", "-53dB", "-60dB", "-73dB" };
#ifdef CLOCK
const char* smode_label[] = { "OFF", "dBm", "S", "S-bar", "wpm", "Vss", "time" };
#else
#ifdef VSS_METER
const char* smode_label[] = { "OFF", "dBm", "S", "S-bar", "wpm", "Vss" };
#else
const char* smode_label[] = { "OFF", "dBm", "S", "S-bar", "wpm" };
#endif
#endif
#ifdef SWR_METER
const char* swr_label[] = { "OFF", "FWD-SWR", "FWD-REF", "VFWD-VREF" };
#endif
const char* cw_tone_label[] = { "700", "600" };
#ifdef KEYER
const char* keyer_mode_label[] = { "Iambic A", "Iambic B","Straight" };
#endif
const char* agc_label[] = { "OFF", "Fast", "Slow" };

#define _N(a) sizeof(a)/sizeof(a[0])

#define N_PARAMS 44  // number of (visible) parameters

#define N_ALL_PARAMS (N_PARAMS+5)  // number of parameters

enum params_t {_NULL, VOLUME, MODE, FILTER, BAND, STEP, VFOSEL, RIT, AGC, NR, ATT, ATT2, SMETER, SWRMETER, CWDEC, CWTONE, CWOFF, SEMIQSK, KEY_WPM, KEY_MODE, KEY_PIN, KEY_TX, VOX, VOXGAIN, DRIVE, TXDELAY, MOX, CWINTERVAL, CWMSG1, CWMSG2, CWMSG3, CWMSG4, CWMSG5, CWMSG6, PWM_MIN, PWM_MAX, SIFXTAL, IQ_ADJ, CALIB, SR, CPULOAD, PARAM_A, PARAM_B, PARAM_C, BACKL, FREQA, FREQB, MODEA, MODEB, VERS, ALL=0xff};


int8_t paramAction(uint8_t action, uint8_t id = ALL)  // list of parameters
{
  if((action == SAVE) || (action == LOAD)){
    eeprom_addr = EEPROM_OFFSET;
    for(uint8_t _id = 1; _id < id; _id++) paramAction(SKIP, _id);
  }
  if(id == ALL) for(id = 1; id != N_ALL_PARAMS+1; id++) paramAction(action, id);  // for all parameters
  
  switch(id){    // Visible parameters
    case VOLUME:  paramAction(action, volume, 0x11, F("Volume"), NULL, -1, 16, false); break;
    case MODE:    paramAction(action, mode, 0x12, F("Mode"), mode_label, 0, _N(mode_label) - 1, false); break;
    case FILTER:  paramAction(action, filt, 0x13, F("Filter BW"), filt_label, 0, _N(filt_label) - 1, false); break;
    case BAND:    paramAction(action, bandval, 0x14, F("Band"), band_label, 0, _N(band_label) - 1, false); break;
    case STEP:    paramAction(action, stepsize, 0x15, F("Tune Rate"), stepsize_label, 0, _N(stepsize_label) - 1, false); break;
    case VFOSEL:  paramAction(action, vfosel, 0x16, F("VFO Mode"), vfosel_label, 0, _N(vfosel_label) - 1, false); break;
#ifdef RIT_ENABLE
    case RIT:     paramAction(action, rit, 0x17, F("RIT"), offon_label, 0, 1, false); break;    
#endif
#ifdef FAST_AGC
    case AGC:     paramAction(action, agc, 0x18, F("AGC"), agc_label, 0, _N(agc_label) - 1, false); break;
#else
    case AGC:     paramAction(action, agc, 0x18, F("AGC"), offon_label, 0, 1, false); break;
#endif // FAST_AGC
    case NR:      paramAction(action, nr, 0x19, F("NR"), NULL, 0, 8, false); break;
    case ATT:     paramAction(action, att, 0x1A, F("ATT"), att_label, 0, 7, false); break;
    case ATT2:    paramAction(action, att2, 0x1B, F("ATT2"), NULL, 0, 16, false); break;
    case SMETER:  paramAction(action, smode, 0x1C, F("S-meter"), smode_label, 0, _N(smode_label) - 1, false); break;
#ifdef SWR_METER
    case SWRMETER:  paramAction(action, swrmeter, 0x1D, F("SWR Meter"), swr_label, 0, _N(swr_label) - 1, false); break;
#endif
#ifdef CW_DECODER
    case CWDEC:   paramAction(action, cwdec, 0x21, F("CW Decoder"), offon_label, 0, 1, false); break;
#endif
#ifdef FILTER_700HZ
    case CWTONE:  if(dsp_cap) paramAction(action, cw_tone, 0x22, F("CW Tone"), cw_tone_label, 0, 1, false); break;
#endif
#ifdef QCX
    case CWOFF:   paramAction(action, cw_offset, 0x23, F("CW Offset"), NULL, 300, 2000, false); break;
#endif
#ifdef SEMI_QSK
    case SEMIQSK: paramAction(action, semi_qsk,  0x24, F("Semi QSK"), offon_label, 0, 1, false); break;
#endif
#if defined(KEYER) || defined(CW_MESSAGE)
    case KEY_WPM:  paramAction(action, keyer_speed, 0x25, F("Keyer Speed"), NULL, 1, 60, false); break;
#endif
#ifdef KEYER
    case KEY_MODE: paramAction(action, keyer_mode,  0x26, F("Keyer Mode"), keyer_mode_label, 0, 2, false); break;
    case KEY_PIN:  paramAction(action, keyer_swap,  0x27, F("Keyer Swap"), offon_label, 0, 1, false); break;
#endif
    case KEY_TX:   paramAction(action, practice,    0x28, F("Practice"), offon_label, 0, 1, false); break;
#ifdef VOX_ENABLE
    case VOX:     paramAction(action, vox,        0x31, F("VOX"), offon_label, 0, 1, false); break;
    case VOXGAIN: paramAction(action, vox_thresh, 0x32, F("Noise Gate"), NULL, 0, 255, false); break;
#endif
    case DRIVE:   paramAction(action, drive,   0x33, F("TX Drive"), NULL, 0, 8, false); break;
#ifdef TX_DELAY
    case TXDELAY: paramAction(action, txdelay, 0x34, F("TX Delay"), NULL, 0, 255, false); break;
#endif
#ifdef MOX_ENABLE
    case MOX:     paramAction(action, mox, 0x35, F("MOX"), NULL, 0, 2, false); break;
#endif
#ifdef CW_MESSAGE
    case CWINTERVAL: paramAction(action, cw_msg_interval, 0x41, F("CQ Interval"), NULL, 0, 60, false); break;
    case CWMSG1:    paramAction(action, cw_msg[0],   0x42, F("CQ Message"), sizeof(cw_msg)); break;
#ifdef CW_MESSAGE_EXT
    case CWMSG2:    paramAction(action, cw_msg[1],   0x43, F("CW Message 2"), sizeof(cw_msg)); break;
    case CWMSG3:    paramAction(action, cw_msg[2],   0x44, F("CW Message 3"), sizeof(cw_msg)); break;
    case CWMSG4:    paramAction(action, cw_msg[3],   0x45, F("CW Message 4"), sizeof(cw_msg)); break;
    case CWMSG5:    paramAction(action, cw_msg[4],   0x46, F("CW Message 5"), sizeof(cw_msg)); break;
    case CWMSG6:    paramAction(action, cw_msg[5],   0x47, F("CW Message 6"), sizeof(cw_msg)); break;
#endif
#endif
    case PWM_MIN: paramAction(action, pwm_min, 0x81, F("PA Bias min"), NULL, 0, pwm_max - 1, false); break;
    case PWM_MAX: paramAction(action, pwm_max, 0x82, F("PA Bias max"), NULL, pwm_min, 255, false); break;
    case SIFXTAL: paramAction(action, si5351.fxtal, 0x83, F("Ref freq"), NULL, 14000000, 28000000, false); break;
    case IQ_ADJ:  paramAction(action, rx_ph_q, 0x84, F("IQ Phase"), NULL, 0, 180, false); break;
#ifdef CAL_IQ
    case CALIB:   if(dsp_cap != SDR) paramAction(action, cal_iq_dummy, 0x85, F("IQ Test/Cal."), NULL, 0, 0, false); break;
#endif
#ifdef DEBUG
    case SR:      paramAction(action, sr, 0x91, F("Sample rate"), NULL, INT32_MIN, INT32_MAX, false); break;
    case CPULOAD: paramAction(action, cpu_load, 0x92, F("CPU load %"), NULL, INT32_MIN, INT32_MAX, false); break;
    case PARAM_A: paramAction(action, param_a, 0x93, F("Param A"), NULL, 0, UINT16_MAX, false); break;
    case PARAM_B: paramAction(action, param_b, 0x94, F("Param B"), NULL, INT16_MIN, INT16_MAX, false); break;
    case PARAM_C: paramAction(action, param_c, 0x95, F("Param C"), NULL, INT16_MIN, INT16_MAX, false); break;
#endif
    case BACKL:   paramAction(action, backlight, 0xA1, F("Backlight"), offon_label, 0, 1, false); break;   // workaround for varying N_PARAM and not being able to overflowing default cases properly
    // Invisible parameters
    case FREQA:   paramAction(action, vfo[VFOA], 0, NULL, NULL, 0, 0, false); break;
    case FREQB:   paramAction(action, vfo[VFOB], 0, NULL, NULL, 0, 0, false); break;
    case MODEA:   paramAction(action, vfomode[VFOA], 0, NULL, NULL, 0, 0, false); break;
    case MODEB:   paramAction(action, vfomode[VFOB], 0, NULL, NULL, 0, 0, false); break;
    case VERS:    paramAction(action, eeprom_version, 0, NULL, NULL, 0, 0, false); break;

    // Non-parameters
    case _NULL:   menumode = 0; show_banner(); change = true; break;
    default:      if((action == NEXT_MENU) && (id != N_PARAMS)) id = paramAction(action, max(1 /*0*/, min(N_PARAMS, id + ((encoder_val > 0) ? 1 : -1))) ); break;  // keep iterating util menu item found
  }
  return id;
}