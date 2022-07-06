// We measure the average amplitude of the signal (see slow_dsp()) but the S-meter should be based on RMS value.
// So we multiply by 0.707/0.639 in an attempt to roughly compensate, although that only really works if the input
// is a sine wave
uint8_t smode = 1;
uint32_t max_absavg256 = 0;
int16_t dbm;

static int16_t smeter_cnt = 0;

int16_t smeter(int16_t ref = 0)
{
  max_absavg256 = max(_absavg256, max_absavg256); // peak

  if((smode) && ((++smeter_cnt % 2048) == 0)){   // slowed down display slightly
    float rms = (float)max_absavg256 * (float)(1 << att2);
    if(dsp_cap == SDR) rms /= (256.0 * 1024.0 * (float)R * 8.0 * 500.0 * 1.414 / (0.707 * 1.1));   // = -98.8dB  1 rx gain stage: rmsV = ADC value * AREF / [ADC DR * processing gain * receiver gain * "RMS compensation"]
    else               rms /= (256.0 * 1024.0 * (float)R * 2.0 * 100.0 * 120.0 / (1.750 * 5.0));   // = -94.6dB
    dbm = 10 * log10((rms * rms) / 50) + 30 - ref; //from rmsV to dBm at 50R

    lcd.noCursor(); 
    if(smode == 1){ // dBm meter
      lcd.setCursor(9, 0); lcd.print((int16_t)dbm); lcd.print(F("dBm "));
    }
    if(smode == 2){ // S-meter
      uint8_t s = (dbm < -63) ? ((dbm - -127) / 6) : (((uint8_t)(dbm - -73)) / 10) * 10;  // dBm to S (modified to work correctly above S9)
      lcd.setCursor(14, 0); if(s < 10){ lcd.print('S'); } lcd.print(s);
    }
    if(smode == 3){ // S-bar
      int8_t s = (dbm < -63) ? ((dbm - -127) / 6) : (((uint8_t)(dbm - -73)) / 10) * 10;  // dBm to S (modified to work correctly above S9)
      char tmp[5];
      for(uint8_t i = 0; i != 4; i++){ tmp[i] = max(2, min(5, s + 1)); s = s - 3; } tmp[4] = 0;
      lcd.setCursor(12, 0); lcd.print(tmp);
    }
#ifdef CW_DECODER
    if(smode == 4){ // wpm-indicator
      lcd.setCursor(14, 0); if(mode == CW) lcd.print(wpm); lcd.print("  ");
    }
#endif  //CW_DECODER
#ifdef VSS_METER
    if(smode == 5){ // Supply-voltage indicator; add resistor of value R_VSS (see below) between 12V supply input and pin 26 (PC3)   Contribution by Jeff WB4LCG: https://groups.io/g/ucx/message/4470
#define R_VSS   1000 // for 1000kOhm from VSS to PC3 (and 10kOhm to GND). Correct this value until VSS is matching
      uint8_t vss10 = (uint32_t)analogSafeRead(BUTTONS, true) * (R_VSS + 10) * 11 / (10 * 1024);   // use for a 1.1V ADC range VSS measurement
      //uint8_t vss10 = (uint32_t)analogSafeRead(BUTTONS, false) * (R_VSS + 10) * 50 / (10 * 1024);  // use for a 5V ADC range VSS measurement (use for 100k value of R_VSS)
      lcd.setCursor(10, 0); lcd.print(vss10/10); lcd.print('.'); lcd.print(vss10%10); lcd.print("V ");
    }
#endif //VSS_METER
#ifdef CLOCK
    if(smode == 6){ // clock-indicator
      uint32_t _s = (millis() * 16000000ULL / F_MCU) / 1000;
      uint8_t h = (_s / 3600) % 24;
      uint8_t m = (_s / 60) % 60;
      uint8_t s = (_s) % 60;
      lcd.setCursor(8, 0); lcd.print(h / 10); lcd.print(h % 10); lcd.print(':'); lcd.print(m / 10); lcd.print(m % 10); lcd.print(':'); lcd.print(s / 10); lcd.print(s % 10); lcd.print("  ");
    }
#endif //CLOCK
    stepsize_showcursor();
    max_absavg256 /= 2;  // Implement peak hold/decay for all meter types    
  }
  return dbm;
}