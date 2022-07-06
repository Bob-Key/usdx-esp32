void run_cw_decode() {
    #ifdef CW_DECODER
  //if((mode == CW) && cwdec) cw_decode();  // if(!(semi_qsk_timeout)) cw_decode(); else dec2();
  if((mode == CW) && cwdec && ((!tx) && (!semi_qsk_timeout))) cw_decode();  // CW decoder only active during RX
#endif  //CW_DECODER

  if(menumode == 0){ // in main
#ifdef CW_DECODER
    if(cw_event){
      uint8_t offset = (uint8_t[]){ 0, 7, 3, 5, 3, 7, 8 }[smode]; // depending on smeter more/less cw-text
      lcd.noCursor();
#ifdef OLED
      //cw_event = false; for(int i = 0; out[offset + i] != '\0'; i++){ lcd.setCursor(i, 0); lcd.print(out[offset + i]); if((!tx) && (!semi_qsk_timeout)) cw_decode(); }   // like 'lcd.print(out + offset);' but then in parallel calling cw_decoding() to handle long OLED writes
      //uint8_t i = cw_event - 1; if(out[offset + i]){ lcd.setCursor(i, 0); lcd.print(out[offset + i]); cw_event++; } else cw_event = false;  // since an oled string write would hold-up reliable decoding/keying, write only a single char each time and continue
      uint8_t i = cw_event - 1; if(15 - offset - i + 1){ lcd.setCursor(15 - offset - i, 0); lcd.print(out[15 - i]); cw_event++; } else cw_event = false;  // since an oled string write would hold-up reliable decoding/keying, write only a single char each time and continue
#else
      cw_event = false;
      lcd.setCursor(0, 0); lcd.print(out + offset);
#endif
      stepsize_showcursor();
    }
    else
#endif  //CW_DECODER
      if((!semi_qsk_timeout) && (!vox_tx))
        smeter();
  }
}