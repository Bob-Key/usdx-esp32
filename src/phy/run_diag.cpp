void run_diag() {
 #ifdef DIAG
  // Measure VDD (+5V); should be ~5V
  si5351.SendRegister(SI_CLK_OE, TX0RX0); // Mute QSD
  digitalWrite(KEY_OUT, LOW);
  digitalWrite(RX, LOW);  // mute RX
  delay(100); // settle
  float vdd = 2.0 * (float)analogRead(AUDIO2) * 5.0 / 1024.0;
  digitalWrite(RX, HIGH);
  if(!(vdd > 4.8 && vdd < 5.2)){
    fatal(F("V5.0"), vdd, 'V');
  }

  // Measure VEE (+3.3V); should be ~3.3V
  float vee = (float)analogRead(SCL) * 5.0 / 1024.0;
  if(!(vee > 3.2 && vee < 3.8)){
    fatal(F("V3.3"), vee, 'V');
  }

  // Measure AVCC via AREF and using internal 1.1V reference fed to ADC; should be ~5V
  analogRead(6); // setup almost proper ADC readout
  bitSet(ADMUX, 3); // Switch to channel 14 (Vbg=1.1V)
  delay(1); // delay improves accuracy
  bitSet(ADCSRA, ADSC);
  for(; bit_is_set(ADCSRA, ADSC););
  float avcc = 1.1 * 1023.0 / ADC;
  if(!(avcc > 4.6 && avcc < 5.2)){
    fatal(F("Vavcc"), avcc, 'V');
  }

  // Report no SSB capability
  if(!ssb_cap){
    fatal(F("No MIC input..."));
  }

  // Test microphone polarity
  /*if((ssb_cap) && (!_digitalRead(DAH))){
    fatal(F("MIC in rev.pol"));
  }*/

  // Measure DVM bias; should be ~VAREF/2
#ifdef _SERIAL
    DDRC &= ~(1<<2);  // disable PC2, so that ADC2 can be used as mic input
#else
    PORTD |= 1<<1; DDRD |= 1<<1;  // keep PD1 HIGH so that in case diode is installed to PC2 it is kept blocked (otherwise ADC2 input is pulled down!)
#endif
  delay(10);
#ifdef TX_ENABLE
  float dvm = (float)analogRead(DVM) * 5.0 / 1024.0;
  if((ssb_cap) && !(dvm > 1.8 && dvm < 3.2)){
    fatal(F("Vadc2"), dvm, 'V');
  }
#endif

  // Measure AUDIO1, AUDIO2 bias; should be ~VAREF/2
  if(dsp_cap == SDR){
    float audio1 = (float)analogRead(AUDIO1) * 5.0 / 1024.0;
    if(!(audio1 > 1.8 && audio1 < 3.2)){
      fatal(F("Vadc0"), audio1, 'V');
    }
    float audio2 = (float)analogRead(AUDIO2) * 5.0 / 1024.0;
    if(!(audio2 > 1.8 && audio2 < 3.2)){
      fatal(F("Vadc1"), audio2, 'V');
    }
  }
  
#ifdef TX_ENABLE
  // Measure I2C Bus speed for Bulk Transfers
  //si5351.freq(freq, 0, 90);
  wdt_reset();
  t0 = micros();
  for(i = 0; i != 1000; i++) si5351.SendPLLRegisterBulk();
  t1 = micros();
  uint32_t speed = (1000000 * 8 * 7) / (t1 - t0); // speed in kbit/s
  if(false) {
    fatal(F("i2cspeed"), speed, 'k');
  }

  // Measure I2C Bit-Error Rate (BER); should be error free for a thousand random bulk PLLB writes
  si5351.freq(freq, 0, 90);  // freq needs to be set in order to use freq_calc_fast()
  wdt_reset();
  uint16_t i2c_error = 0;  // number of I2C byte transfer errors
  for(i = 0; i != 1000; i++){
    si5351.freq_calc_fast(i);
    //for(int j = 0; j != 8; j++) si5351.pll_regs[j] = rand();
    si5351.SendPLLRegisterBulk();
    #define SI_SYNTH_PLL_A 26
#ifdef NEW_TX
    for(int j = 4; j != 8; j++) if(si5351.RecvRegister(SI_SYNTH_PLL_A + j) != si5351.pll_regs[j]) i2c_error++;
#else
    for(int j = 3; j != 8; j++) if(si5351.RecvRegister(SI_SYNTH_PLL_A + j) != si5351.pll_regs[j]) i2c_error++;
#endif //NEW_TX
  }
  wdt_reset();
  if(i2c_error){
    fatal(F("BER_i2c"), i2c_error, ' ');
  }
#endif //TX_ENABLE
#endif  // DIAG
}   