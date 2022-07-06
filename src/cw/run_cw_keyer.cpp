run_cw_keyer() {
    #ifdef KEYER  //Keyer
  if(mode == CW && keyer_mode != SINGLE){  // check DIT/DAH keys for CW

    switch(keyerState){ // Basic Iambic Keyer, keyerControl contains processing flags and keyer mode bits, Supports Iambic A and B, State machine based, uses calls to millis() for timing.
    case IDLE: // Wait for direct or latched paddle press
        if((_digitalRead(DAH) == LOW) ||
            (_digitalRead(DIT) == LOW) ||
            (keyerControl & 0x03))
        {
#ifdef CW_MESSAGE
            cw_msg_event = 0;  // clear cw message event
#endif //CW_MESSAGE
            update_PaddleLatch();
            keyerState = CHK_DIT;
        }
        break;
    case CHK_DIT: // See if the dit paddle was pressed
        if(keyerControl & DIT_L) {
            keyerControl |= DIT_PROC;
            ktimer = ditTime;
            keyerState = KEYED_PREP;
        } else {
            keyerState = CHK_DAH;
        }
        break;
    case CHK_DAH: // See if dah paddle was pressed
        if(keyerControl & DAH_L) {
            ktimer = ditTime*3;
            keyerState = KEYED_PREP;
        } else {
            keyerState = IDLE;
        }
        break;
    case KEYED_PREP: // Assert key down, start timing, state shared for dit or dah
        Key_state = HIGH;
        switch_rxtx(Key_state);
        ktimer += millis();                 // set ktimer to interval end time
        keyerControl &= ~(DIT_L + DAH_L);   // clear both paddle latch bits
        keyerState = KEYED;                 // next state
        break;
    case KEYED: // Wait for timer to expire
        if(millis() > ktimer) {            // are we at end of key down ?
            Key_state = LOW;
            switch_rxtx(Key_state);
            ktimer = millis() + ditTime;    // inter-element time
            keyerState = INTER_ELEMENT;     // next state
        } else if(keyerControl & IAMBICB) {
            update_PaddleLatch();           // early paddle latch in Iambic B mode
        }
        break;
    case INTER_ELEMENT:
        // Insert time between dits/dahs
        update_PaddleLatch();               // latch paddle state
        if(millis() > ktimer) {            // are we at end of inter-space ?
            if(keyerControl & DIT_PROC) {             // was it a dit or dah ?
                keyerControl &= ~(DIT_L + DIT_PROC);   // clear two bits
                keyerState = CHK_DAH;                  // dit done, check for dah
            } else {
                keyerControl &= ~(DAH_L);              // clear dah latch
                keyerState = IDLE;                     // go idle
            }
        }
        break;
    }

  } else {
#endif //KEYER
#ifdef TX_ENABLE
  uint8_t pin = ((mode == CW) && (keyer_swap)) ? DAH : DIT;
  if(!vox_tx)  //  ONLY if VOX not active, then check DIT/DAH (fix for VOX to prevent RFI feedback through EMI on DIT or DAH line)
   if(!_digitalRead(pin)){  // PTT/DIT keys transmitter
#ifdef CW_MESSAGE
    cw_msg_event = 0;  // clear cw message event
#endif //CW_MESSAGE
    switch_rxtx(1);
    do {
      wdt_reset();
      delay((mode == CW) ? 10 : 100);  // keep the tx keyed for a while before sensing (helps against RFI issues on DAH/DAH line)
#ifdef SWR_METER
      if(smeter > 0 && mode == CW && millis() >= stimer) { readSWR(); stimer = millis() + 500; }
#endif
      if(inv ^ _digitalRead(BUTTONS)) break;  // break if button is pressed (to prevent potential lock-up)
    } while(!_digitalRead(pin)); // until released
    switch_rxtx(0);
   }
#endif //TX_ENABLE
#ifdef KEYER
  }
#endif //KEYER
  }