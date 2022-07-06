void set_key() {
    #ifdef KEYER
  keyerState = IDLE;
  keyerControl = IAMBICB;      // Or 0 for IAMBICA
  loadWPM(keyer_speed);        // Fix speed at 15 WPM
    #endif //KEYER

  for(; !_digitalRead(DIT) || ((mode == CW && keyer_mode != SINGLE) && (!_digitalRead(DAH)));){ fatal(F("Check PTT/key")); }// wait until DIH/DAH/PTT is released to prevent TX on startup
}
