void control()
{
#ifdef SEMI_QSK
  if((semi_qsk_timeout) && (millis() > semi_qsk_timeout)){ switch_rxtx(0); }  // delayed QSK RX
#endif
  enum event_t { BL=0x10, BR=0x20, BE=0x30, SC=0x01, DC=0x02, PL=0x04, PLC=0x05, PT=0x0C }; // button-left, button-right and button-encoder; single-click, double-click, push-long, push-and-turn
  if(inv ^ _digitalRead(BUTTONS)){   // Left-/Right-/Rotary-button (while not already pressed)
    if(!((event & PL) || (event & PLC))){  // hack: if there was long-push before, then fast forward
      uint16_t v = analogSafeRead(BUTTONS);
#ifdef CAT_EXT
      if(cat_key){ v = (cat_key&0x04) ? 512 : (cat_key&0x01) ? 870 : (cat_key&0x02) ? 1024 : 0; }  // override analog value exercised by BUTTONS press
#endif //CAT_EXT
      event = SC;
      int32_t t0 = millis();
      for(; inv ^ _digitalRead(BUTTONS);){ // until released or long-press
        if((millis() - t0) > 300){ event = PL; break; }
        wdt_reset();
      }
      delay(10); //debounce
      for(; (event != PL) && ((millis() - t0) < 500);){ // until 2nd press or timeout
        if(inv ^ _digitalRead(BUTTONS)){ event = DC; break; }
        wdt_reset();
      }
      for(; inv ^ _digitalRead(BUTTONS);){ // until released, or encoder is turned while longpress
        if(encoder_val && event == PL){ event = PT; break; }
#ifdef ONEBUTTON
        if(event == PL) break;  // do not lock on longpress, so that L and R buttons can be used for tuning
#endif
        wdt_reset();
      }  // Max. voltages at ADC3 for buttons L,R,E: 3.76V;4.55V;5V, thresholds are in center
      event |= (v < (uint16_t)(4.2 * 1024.0 / 5.0)) ? BL : (v < (uint16_t)(4.8 * 1024.0 / 5.0)) ? BR : BE; // determine which button pressed based on threshold levels
    } else {  // hack: fast forward handling
      event = (event&0xf0) | ((encoder_val) ? PT : PLC/*PL*/);  // only alternate between push-long/turn when applicable
    }
    switch(event){
#ifndef ONEBUTTON
      case BL|PL:  // Called when menu button pressed
      case BL|PLC: // or kept pressed
        menumode = 2;
        break;
      case BL|PT:
        menumode = 1;
        //if(menu == 0) menu = 1;
        break;
      case BL|SC:
#ifdef CW_MESSAGE
        if((menumode == 1) && (menu >= CWMSG1) && (menu <= CWMSG6)){
          cw_msg_event = millis();
          cw_msg_id = menu - CWMSG1;
          menumode = 0;
          break;
        }
#endif //CW_MESSAGE
        int8_t _menumode;
        if(menumode == 0){ _menumode = 1; if(menu == 0) menu = 1; }  // short left-click while in default screen: enter menu mode
        if(menumode == 1){ _menumode = 2; }                          // short left-click while in menu: enter value selection screen
        if(menumode >= 2){ _menumode = 0; paramAction(SAVE, menu); } // short left-click while in value selection screen: save, and return to default screen
        menumode = _menumode;
        break;
      case BL|DC:
        break;
      case BR|SC:
        if(!menumode){
          int8_t prev_mode = mode;
          if(rit){ rit = 0; stepsize = prev_stepsize[mode == CW]; change = true;  break; }
          mode += 1;
          //encoder_val = 1;
          //paramAction(UPDATE, MODE); // Mode param //paramAction(UPDATE, mode, NULL, F("Mode"), mode_label, 0, _N(mode_label), true);
//#define MODE_CHANGE_RESETS  1
#ifdef MODE_CHANGE_RESETS
          if(mode != CW) stepsize = STEP_1k; else stepsize = STEP_500; // sets suitable stepsize
#endif
          if(mode > CW) mode = LSB;  // skip all other modes (only LSB, USB, CW)
#ifdef MODE_CHANGE_RESETS
          if(mode == CW) { filt = 4; nr  = 0; } else filt = 0;  // resets filter (to most BW) and NR on mode change
#else
          if(mode == CW) { nr  = 0; }
          prev_stepsize[prev_mode == CW] = stepsize; stepsize = prev_stepsize[mode == CW]; // backup stepsize setting for previous mode, restore previous stepsize setting for current selected mode; filter settings captured for either CQ or other modes.
          prev_filt[prev_mode == CW] = filt; filt = prev_filt[mode == CW];  // backup filter setting for previous mode, restore previous filter setting for current selected mode; filter settings captured for either CQ or other modes.
#endif
          //paramAction(UPDATE, MODE);
          vfomode[vfosel%2] = mode;
          paramAction(SAVE, (vfosel%2) ? MODEB : MODEA);  // save vfoa/b changes
          paramAction(SAVE, MODE);
          paramAction(SAVE, FILTER);
          si5351.iqmsa = 0;  // enforce PLL reset
#ifdef CW_DECODER
          if((prev_mode == CW) && (cwdec)) show_banner();
#endif
          change = true;
        } else {
          if(menumode == 1){ menumode = 0; }  // short right-click while in menu: enter value selection screen
          if(menumode >= 2){ menumode = 1; change = true; paramAction(SAVE, menu); } // short right-click while in value selection screen: save, and return to menu screen
        }
        break;
      case BR|DC:
        filt++;
        _init = true;
        if(mode == CW && filt > N_FILT) filt = 4;
        if(mode == CW && filt == 4) stepsize = STEP_500; // reset stepsize for 500Hz filter
        if(mode == CW && (filt == 5 || filt == 6) && stepsize < STEP_100) stepsize = STEP_100; // for CW BW 200, 100      -> step = 100 Hz
        if(mode == CW && filt == 7 && stepsize < STEP_10) stepsize = STEP_10;                  // for CW BW 50 -> step = 10 Hz
        if(mode != CW && filt > 3) filt = 0;
        encoder_val = 0; 
        paramAction(UPDATE, FILTER);
        paramAction(SAVE, FILTER);
        wdt_reset(); delay(1500); wdt_reset();
        change = true; // refresh display
        break;
      case BR|PL:
#ifdef SIMPLE_RX
        // Experiment: ISR-less sdr_rx():
        smode = 0;
        TIMSK2 &= ~(1 << OCIE2A);  // disable timer compare interrupt
        delay(100);
        lcd.setCursor(15, 1); lcd.print('X');
        static uint8_t x = 0;
        uint32_t next = 0;
        for(;;){
          func_ptr();
        #ifdef DEBUG
          numSamples++;
        #endif
          if(!rx_state){
            x++;
            if(x > 16){
              loop();
              //lcd.setCursor(9, 0); lcd.print((int16_t)100); lcd.print(F("dBm   "));  // delays are taking too long!
              x= 0;
            }
          }
          //for(;micros() < next;);  next = micros() + 16;   // sync every 1000000/62500=16ms (or later if missed)
        } //
#endif //SIMPLE_RX
#ifdef RIT_ENABLE
        rit = !rit;
        stepsize = (rit) ?  STEP_10 : prev_stepsize[mode == CW];
        if(!rit){  // after RIT comes VFO A/B swap
#else
        {
#endif //RIT_ENABLE
          vfosel = !vfosel;
          freq = vfo[vfosel%2];  // todo: share code with menumode
          mode = vfomode[vfosel%2];
          // make more generic: 
          if(mode != CW) stepsize = STEP_1k; else stepsize = STEP_500;
          if(mode == CW) { filt = 4; nr = 0; } else filt = 0;
        }
        change = true;
        break;
//#define TUNING_DIAL  1
#ifdef TUNING_DIAL
      case BR|PLC:  // while pressed long continues
      case BE|PLC:
        freq = freq + ((_step > 0) ? 1 : -1) * pow(2, abs(_step)); change=true;
        break;
      case BR|PT:
        _step += encoder_val; encoder_val = 0;
        lcd.setCursor(0, 0); lcd.print(_step); lcd_blanks();
        break;
#endif //TUNING_DIAL
      case BE|SC:
        if(!menumode){
          stepsize_change(+1);
        } else {
          int8_t _menumode;
          if(menumode == 1){ _menumode = 2; }  // short encoder-click while in menu: enter value selection screen
          if(menumode == 2){ _menumode = 1; change = true; paramAction(SAVE, menu); } // short encoder-click while in value selection screen: save, and return to menu screen
#ifdef MENU_STR
          if(menumode == 3){ _menumode = 3; paramAction(NEXT_CH, menu); } // short encoder-click while in string edit mode: change position to next character
#endif
          menumode = _menumode;
        }
        break;
      case BE|DC:
        //delay(100);
        bandval++;
        //if(bandval >= N_BANDS) bandval = 0;
        if(bandval >= (N_BANDS-1)) bandval = 1;  // excludes 6m, 160m
        stepsize = STEP_1k;
        change = true;
        break;
      case BE|PL: stepsize_change(-1); break;
      case BE|PT:
          for(; _digitalRead(BUTTONS);){ // process encoder changes until released
          wdt_reset();
          if(encoder_val){
            paramAction(UPDATE, VOLUME);
            if(volume < 0){ volume = 10; paramAction(SAVE, VOLUME); powerDown(); }  // powerDown when volume < 0
            paramAction(SAVE, VOLUME);
          }
        }
        change = true; // refresh display
        break;
#else //ONEBUTTON
      case BE|SC:
        int8_t _menumode;
        if(menumode == 0){ _menumode = 1; if(menu == 0) menu = 1; }  // short enc-click while in default screen: enter menu mode
        if(menumode == 1){ _menumode = 2; }                          // short enc-click while in menu: enter value selection screen
        if(menumode == 2){ _menumode = 0; paramAction(SAVE, menu); } // short enc-click while in value selection screen: save, and return to default screen
#ifdef MENU_STR
        if(menumode == 3){ _menumode = 3; paramAction(NEXT_CH, menu); } // short encoder-click while in string edit mode: change position to next character
#endif
        menumode = _menumode;
        break;
      case BE|DC:
        if(!menumode){
          int8_t prev_mode = mode;
          if(rit){ rit = 0; stepsize = prev_stepsize[mode == CW]; change = true;  break; }
          mode += 1;
          //encoder_val = 1;
          //paramAction(UPDATE, MODE); // Mode param //paramAction(UPDATE, mode, NULL, F("Mode"), mode_label, 0, _N(mode_label), true);
//#define MODE_CHANGE_RESETS  1
#ifdef MODE_CHANGE_RESETS
          if(mode != CW) stepsize = STEP_1k; else stepsize = STEP_500; // sets suitable stepsize
#endif
          if(mode > CW) mode = LSB;  // skip all other modes (only LSB, USB, CW)
#ifdef MODE_CHANGE_RESETS
          if(mode == CW) { filt = 4; nr  = 0; } else filt = 0;  // resets filter (to most BW) and NR on mode change
#else
          if(mode == CW) { nr  = 0; }
          prev_stepsize[prev_mode == CW] = stepsize; stepsize = prev_stepsize[mode == CW]; // backup stepsize setting for previous mode, restore previous stepsize setting for current selected mode; filter settings captured for either CQ or other modes.
          prev_filt[prev_mode == CW] = filt; filt = prev_filt[mode == CW];  // backup filter setting for previous mode, restore previous filter setting for current selected mode; filter settings captured for either CQ or other modes.
#endif
          //paramAction(UPDATE, MODE);
          vfomode[vfosel%2] = mode;
          paramAction(SAVE, (vfosel%2) ? MODEB : MODEA);  // save vfoa/b changes
          paramAction(SAVE, MODE);
          paramAction(SAVE, FILTER);
          si5351.iqmsa = 0;  // enforce PLL reset
          if((prev_mode == CW) && (cwdec)) show_banner();
          change = true;
        } else {
          if(menumode == 1){ menumode = 0; }  // short right-click while in menu: enter value selection screen
          if(menumode >= 2){ menumode = 1; change = true; paramAction(SAVE, menu); } // short right-click while in value selection screen: save, and return to menu screen
        }
        break;
      case BE|PL:
        stepsize += 1;
        if(stepsize < STEP_1k) stepsize = STEP_10;
        if(stepsize > STEP_10) stepsize = STEP_1k;
        stepsize_showcursor();
        break;
      case BE|PLC: // or kept pressed
        menumode = 2;
        break;
      case BE|PT:
        menumode = 1;
        //if(menu == 0) menu = 1;
        break;
      case BL|SC:
      case BL|DC:
      case BL|PL:
      case BL|PLC:
        encoder_val++;
        break;
      case BR|SC:
      case BR|DC:
      case BR|PL:
      case BR|PLC:
        encoder_val--;
        break;
#endif //ONEBUTTON
    }
  } else event = 0;  // no button pressed: reset event

  if((menumode) || (prev_menumode != menumode)){  // Show parameter and value
    int8_t encoder_change = encoder_val;
    if((menumode == 1) && encoder_change){
      menu += encoder_val;   // Navigate through menu
#ifdef ONEBUTTON
      menu = max(0, min(menu, N_PARAMS));
#else
      menu = max(1 /* 0 */, min(menu, N_PARAMS));
#endif
      menu = paramAction(NEXT_MENU, menu);  // auto probe next menu item (gaps may exist)
      encoder_val = 0;
    }
    if(encoder_change || (prev_menumode != menumode)) paramAction(UPDATE_MENU, (menumode) ? menu : 0);  // update param with encoder change and display
    prev_menumode = menumode;
    if(menumode == 2){
      if(encoder_change){
        lcd.setCursor(0, 1); lcd.cursor();  // edits menu item value; make cursor visible
        if(menu == MODE){ // post-handling Mode parameter
          vfomode[vfosel%2] = mode;
          paramAction(SAVE, (vfosel%2) ? MODEB : MODEA);  // save vfoa/b changes
          change = true;
          si5351.iqmsa = 0;  // enforce PLL reset
          // make more generic: 
          if(mode != CW) stepsize = STEP_1k; else stepsize = STEP_500;
          if(mode == CW) { filt = 4; nr = 0; } else filt = 0;
        }
        if(menu == BAND){
          change = true;
        }
        //if(menu == NR){ if(mode == CW) nr = false; }
        if(menu == VFOSEL){
          freq = vfo[vfosel%2];
          mode = vfomode[vfosel%2];
          // make more generic: 
          if(mode != CW) stepsize = STEP_1k; else stepsize = STEP_500;
          if(mode == CW) { filt = 4; nr = 0; } else filt = 0;
          change = true;
        }
#ifdef RIT_ENABLE
        if(menu == RIT){
          stepsize = (rit) ?  STEP_10 : STEP_500;
          change = true;
        }
#endif
        //if(menu == VOX){ if(vox){ vox_thresh-=1; } else { vox_thresh+=1; }; }
        if(menu == ATT){ // post-handling ATT parameter
          if(dsp_cap == SDR){
            noInterrupts();
          #ifdef SWAP_RX_IQ
            adc_start(1, !(att & 0x01)/*true*/, F_ADC_CONV); admux[0] = ADMUX;
            adc_start(0, !(att & 0x01)/*true*/, F_ADC_CONV); admux[1] = ADMUX;
          #else
            adc_start(0, !(att & 0x01)/*true*/, F_ADC_CONV); admux[0] = ADMUX;
            adc_start(1, !(att & 0x01)/*true*/, F_ADC_CONV); admux[1] = ADMUX;
          #endif //SWAP_RX_IQ
            interrupts();
          }
          digitalWrite(RX, !(att & 0x02)); // att bit 1 ON: attenuate -20dB by disabling RX line, switching Q5 (antenna input switch) into 100k resistence
          pinMode(AUDIO1, (att & 0x04) ? OUTPUT : INPUT); // att bit 2 ON: attenuate -40dB by terminating ADC inputs with 10R
          pinMode(AUDIO2, (att & 0x04) ? OUTPUT : INPUT);
        }
        if(menu == SIFXTAL){
          change = true;
        }
#ifdef TX_ENABLE
        if((menu == PWM_MIN) || (menu == PWM_MAX)){
          build_lut();
        }
#endif
        if(menu == CWTONE){
          if(dsp_cap){ cw_offset = (cw_tone == 0) ? tones[0] : tones[1]; paramAction(SAVE, CWOFF); }
        }
        if(menu == IQ_ADJ){
          change = true;
        }
#ifdef CAL_IQ
        if(menu == CALIB){
          if(dsp_cap != SDR) calibrate_iq(); menu = 0;
        }
#endif
#ifdef KEYER
        if(menu == KEY_WPM){
          loadWPM(keyer_speed);
        }
        if(menu == KEY_MODE){
          if(keyer_mode == 0){ keyerControl = IAMBICA; }
          if(keyer_mode == 1){ keyerControl = IAMBICB; }
          if(keyer_mode == 2){ keyerControl = SINGLE; }
        }
#endif //KEYER
#ifdef TX_DELAY
        if(menu == TXDELAY){
          semi_qsk = (txdelay > 0);
        }
#endif //TX_DELAY
      }
#ifdef DEBUG
      if(menu == SR){          // measure sample-rate
        numSamples = 0;
        delay(F_MCU * 500UL / 16000000);   // delay 0.5s (in reality because F_CPU=20M instead of 16M, delay() is running 1.25x faster therefore we need to multiply with 1.25)
        sr = numSamples * 2;   // samples per second
        paramAction(UPDATE_MENU, menu); // refresh
      }
      if(menu == CPULOAD){     // measure CPU-load
        uint32_t i = 0;
        uint32_t prev_time = millis();
        for(i = 0; i != 300000; i++) wdt_reset(); // fixed CPU-load 132052*1.25us delay under 0% load condition; is 132052*1.25 * 20M = 3301300 CPU cycles fixed load
        cpu_load = 100 - 132 * 100 / (millis() - prev_time);
        paramAction(UPDATE_MENU, menu); // refresh
      }
      if((menu == PARAM_A) || (menu == PARAM_B) || (menu == PARAM_C)){
        delay(300);
        paramAction(UPDATE_MENU, menu); // refresh
      }
#endif
    }
  }

  if(menumode == 0){
    if(encoder_val){  // process encoder tuning steps
      process_encoder_tuning_step(encoder_val);
      encoder_val = 0;
    }
  }

  if((change) && (!tx) && (!vox_tx)){  // only change if TX is OFF, prevent simultaneous I2C bus access
    change = false;
    if(prev_bandval != bandval){ freq = band[bandval]; prev_bandval = bandval; }
    vfo[vfosel%2] = freq;
    save_event_time = millis() + 1000;  // schedule time to save freq (no save while tuning, hence no EEPROM wear out)
 
    if(menumode == 0){
      display_vfo(freq);
      stepsize_showcursor();
#ifdef CAT
      //Command_GETFreqA();
#endif

      // The following is a hack for SWR measurement:
      //si5351.alt_clk2(freq + 2400);
      //si5351.SendRegister(SI_CLK_OE, TX1RX1);
      //digitalWrite(SIG_OUT, HIGH);  // inject CLK2 on antenna input via 120K
    }

    //noInterrupts();
    uint8_t f = freq / 1000000UL;
    set_lpf(f);
    bandval = (f > 32) ? 10 : (f > 26) ? 9 : (f > 22) ? 8 : (f > 20) ? 7 : (f > 16) ? 6 : (f > 12) ? 5 : (f > 8) ? 4 : (f > 6) ? 3 : (f > 4) ? 2 : (f > 2) ? 1 : 0;  prev_bandval = bandval; // align bandval with freq

    if(mode == CW){
      si5351.freq(freq + cw_offset, rx_ph_q, 0/*90, 0*/);  // RX in CW-R (=LSB), correct for CW-tone offset
    } else
    if(mode == LSB)
      si5351.freq(freq, rx_ph_q, 0/*90, 0*/);  // RX in LSB
    else
      si5351.freq(freq, 0, rx_ph_q/*0, 90*/);  // RX in USB, ...
#ifdef RIT_ENABLE
    if(rit){ si5351.freq_calc_fast(rit); si5351.SendPLLRegisterBulk(); }
#endif //RIT_ENABLE
    //interrupts();
  }
  
  if((save_event_time) && (millis() > save_event_time)){  // save freq when time has reached schedule
    paramAction(SAVE, (vfosel%2) ? FREQB : FREQA);  // save vfoa/b changes
    save_event_time = 0;
    //lcd.setCursor(15, 1); lcd.print('S'); delay(100); lcd.setCursor(15, 1); lcd.print('R');
  }

#ifdef CW_MESSAGE
  if((mode == CW) && (cw_msg_event) && (millis() > cw_msg_event)){  // if it is time to send a CW message 
    if((cw_tx(cw_msg[cw_msg_id]) == 0) && ((cw_msg[cw_msg_id][0] == 'C') && (cw_msg[cw_msg_id][1] == 'Q')) && cw_msg_interval) cw_msg_event = millis() + 1000 * cw_msg_interval; else cw_msg_event = 0;  // then send message, if not interrupted and its a CQ msg and there is an interval set, then schedule new event
  }
#endif //CW_MESSAGE

  wdt_reset();

  //{ lcd.setCursor(0, 0); lcd.print(freeMemory()); lcd.print(F("    ")); }
}
}