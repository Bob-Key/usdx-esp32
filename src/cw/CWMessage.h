/**
 * TODO: Extract to a class
 */
#pragma once

const char m2c[] PROGMEM = "~ ETIANMSURWDKGOHVF*L*PJBXCYZQ**54S3***2**+***J16=/***H*7*G*8*90************?_****\"**.****@***'**-********;!*)*****,****:****";

#ifdef CW_MESSAGE
#define MENU_STR  1

uint8_t delayWithKeySense(uint32_t ms){
  uint32_t event = millis() + ms;
  for(; millis() < event;){
    wdt_reset();
    if(inv ^ digitalRead(BUTTONS) || !digitalRead(DAH) || !digitalRead(DIT)){
      for(; inv ^ digitalRead(BUTTONS);) wdt_reset();  // wait until buttons released  
      return 1;  // stop when button/key pressed
    }
  }
  return 0;
}
#ifdef CW_MESSAGE_EXT
char cw_msg[6][48] = { "CQ PE1NNN +", "CQ CQ DE PE1NNN PE1NNN +", "GE TKS 5NN 5NN NAME IS GUIDO GUIDO HW?", "FB RPTR TX 5W 5W ANT INV V 73 CUAGN", "73 TU E E", "PE1NNN" };
#else
char cw_msg[1][48] = { "CQ PE1NNN +" };
#endif
uint8_t cw_msg_interval = 5; // number of seconds CW message is repeated
uint32_t cw_msg_event = 0;
uint8_t cw_msg_id = 0; // selected message

int cw_tx(char ch){    // Transmit message in CW
  char sym;
  for(uint8_t j = 0; (sym = pgm_read_byte_near(m2c + j)); j++){  // lookup msg[i] in m2c, skip if not found
    if(sym == ch){  // found -> transmit CW character j
      wdt_reset();
      uint8_t k = 0x80; for(; !(j & k); k >>= 1); k >>= 1; // shift start of cw code to MSB
      if(k == 0) delay(ditTime * 4); // space -> add word space
      else {
        for(; k; k >>= 1){ // send dit/dah one by one, until everythng is sent
          switch_rxtx(1);  // key-on  tx
          if(delayWithKeySense(ditTime * ((j & k) ? 3 : 1))){ switch_rxtx(0); return 1; } // symbol: dah or dih length
          switch_rxtx(0);  // key-off tx
          if(delayWithKeySense(ditTime)) return 1;   // add symbol space
        }
        if(delayWithKeySense(ditTime * 2)) return 1; // add letter space
      }
      break; // next character
    }
  }
  return 0;
}

int cw_tx(char* msg){
  for(uint8_t i = 0; msg[i]; i++){  // loop over message
    lcd.setCursor(0, 0); lcd.print(i); lcd.print("    ");
    if(cw_tx(msg[i])) return 1;
  }
  return 0;
}
#endif // CW_MESSAGE