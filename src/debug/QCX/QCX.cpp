void set_QCX() {
#ifdef QCX
  // Test if QCX has DSP/SDR capability: SIDETONE output disconnected from AUDIO2
  si5351.SendRegister(SI_CLK_OE, TX0RX0); // Mute QSD
  digitalWrite(RX, HIGH);  // generate pulse on SIDETONE and test if it can be seen on AUDIO2
  delay(1); // settle
  digitalWrite(SIDETONE, LOW);
  int16_t v1 = analogRead(AUDIO2);
  digitalWrite(SIDETONE, HIGH);
  int16_t v2 = analogRead(AUDIO2);
  digitalWrite(SIDETONE, LOW);
  dsp_cap = !(abs(v2 - v1) > (0.05 * 1024.0 / 5.0));  // DSP capability?
  if(dsp_cap){  // Test if QCX has SDR capability: AUDIO2 is disconnected from AUDIO1  (only in case of DSP capability)
    delay(400); wdt_reset(); // settle:  the following test only works well 400ms after startup
    v1 = analogRead(AUDIO1);
    digitalWrite(AUDIO2, HIGH);   // generate pulse on AUDIO2 and test if it can be seen on AUDIO1
    pinMode(AUDIO2, OUTPUT);
    delay(1);
    digitalWrite(AUDIO2, LOW); 
    delay(1);
    digitalWrite(AUDIO2, HIGH);
    v2 = analogRead(AUDIO1);
    pinMode(AUDIO2, INPUT);
    if(!(abs(v2 - v1) > (0.125 * 1024.0 / 5.0))) dsp_cap = SDR;  // SDR capacility?
  }
  // Test if QCX has SSB capability: DAH is connected to DVM
  delay(1); // settle
  pinMode(DAH, OUTPUT);
  digitalWrite(DAH, LOW);
  v1 = analogRead(DVM);
  digitalWrite(DAH, HIGH);
  v2 = analogRead(DVM);
  digitalWrite(DAH, LOW);
  //pinMode(DAH, INPUT_PULLUP);
  pinMode(DAH, INPUT);
  ssb_cap = (abs(v2 - v1) > (0.05 * 1024.0 / 5.0));  // SSB capability?

  //ssb_cap = 0; dsp_cap = ANALOG;  // force standard QCX capability
  //ssb_cap = 1; dsp_cap = ANALOG;  // force SSB and standard QCX-RX capability
  //ssb_cap = 1; dsp_cap = DSP;     // force SSB and DSP capability
  //ssb_cap = 1; dsp_cap = SDR;     // force SSB and SDR capability
#endif // QCX
}



#ifdef QCX
#define CAL_IQ 1
#ifdef CAL_IQ
int16_t cal_iq_dummy = 0;
// RX I/Q calibration procedure: terminate with 50 ohm, enable CW filter, adjust R27, R24, R17 subsequently to its minimum side-band rejection value in dB
void calibrate_iq()
{
  smode = 1;
  lcd.setCursor(0, 0); lcd_blanks(); lcd_blanks();
  digitalWrite(SIG_OUT, true); // loopback on
  si5351.freq(freq, 0, 90);  // RX in USB  
  si5351.SendRegister(SI_CLK_OE, TX1RX1);
  float dbc;
  si5351.freqb(freq+700); delay(100);
  dbc = smeter();
  si5351.freqb(freq-700); delay(100);
  lcd.setCursor(0, 1); lcd.print("I-Q bal. 700Hz"); lcd_blanks();
  for(; !_digitalRead(BUTTONS);){ wdt_reset(); smeter(dbc); } for(; _digitalRead(BUTTONS);) wdt_reset();
  si5351.freqb(freq+600); delay(100);
  dbc = smeter();
  si5351.freqb(freq-600); delay(100);
  lcd.setCursor(0, 1); lcd.print("Phase Lo 600Hz"); lcd_blanks();
  for(; !_digitalRead(BUTTONS);){ wdt_reset(); smeter(dbc); } for(; _digitalRead(BUTTONS);) wdt_reset();
  si5351.freqb(freq+800); delay(100);
  dbc = smeter();
  si5351.freqb(freq-800); delay(100);
  lcd.setCursor(0, 1); lcd.print("Phase Hi 800Hz"); lcd_blanks();
  for(; !_digitalRead(BUTTONS);){ wdt_reset(); smeter(dbc); } for(; _digitalRead(BUTTONS);) wdt_reset();

  lcd.setCursor(9, 0); lcd_blanks();  // cleanup dbmeter
  digitalWrite(SIG_OUT, false); // loopback off
  si5351.SendRegister(SI_CLK_OE, TX0RX1);
  change = true;  //restore original frequency setting
}
#endif
#endif //QCX