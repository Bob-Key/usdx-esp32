#pragma once

class OLEDDevice: public Print {  // https://www.buydisplay.com/download/manual/ER-OLED0.91-3_Series_Datasheet.pdf
public:
  #define OLED_ADDR    0x3C  // Slave address
  #define OLED_PAGES   4
  #define OLED_COMMAND 0x00
  #define OLED_DATA    0x40
  uint8_t oledX = 0, oledY = 0;
  uint8_t renderingFrame = 0xB0;
  bool wrap = false;
  void cmd(uint8_t b){
    Wire.beginTransmission(OLED_ADDR); Wire.write(OLED_COMMAND);
    Wire.write(b);
    Wire.endTransmission();
  }
  void begin(uint8_t cols, uint8_t rows, uint8_t charsize = 0){
    Wire.begin();
    Wire.beginTransmission(OLED_ADDR); Wire.write(OLED_COMMAND);
    for (uint8_t i = 0; i < sizeof(oled_init_sequence); i++) {
      Wire.write(pgm_read_byte(&oled_init_sequence[i]));
    }
    Wire.endTransmission();
    delayMicroseconds(100);
#ifdef CONDENSED
    for(uint8_t y = 0; y != rows; y++) for(uint8_t x = 0; x != cols; x++){ setCursor(x, y); write(' '); }  // clear
#endif
  }
  bool curs = false;
  void noCursor(){ curs = false; }
  void cursor(){ curs = true; }
  void noDisplay(){ cmd(0xAE); }
  void createChar(uint8_t l, uint8_t glyph[]){}

  void _setCursor(uint8_t x, uint8_t y) {
    oledX = x; oledY = y;
    Wire.beginTransmission(OLED_ADDR); Wire.write(OLED_COMMAND);
    Wire.write(renderingFrame | (oledY & 0x07));
    uint8_t _oledX = oledX;
#ifdef OLED_SH1106
    _oledX += 2; // SH1106 is a 132x64 controller.  Use middle 128 columns.
#endif
    Wire.write(0x10 | ((_oledX & 0xf0) >> 4));
    Wire.write(_oledX & 0x0f);
    Wire.endTransmission();
  }
  void drawCursor(bool en){
    //_setCursor(oledX, oledY + (FONT_W/(FONT_STRETCHH+1)));
    Wire.beginTransmission(OLED_ADDR); Wire.write(OLED_DATA);
    Wire.write((en) ? 0xf0 : 0x00);  // horizontal line
    Wire.endTransmission();
  }
  void setCursor(uint8_t x, uint8_t y) {
    if(curs){ drawCursor(false); } _setCursor(x * FONT_W, y * FONT_H); if(curs){ drawCursor(true); _setCursor(oledX, oledY); }
  }
    
  void newLine() {
    oledY+=FONT_H;
    if(oledY > OLED_PAGES - FONT_H) {
      oledY = OLED_PAGES - FONT_H;
    }
    _setCursor(0, oledY);
  }
    
  size_t write(byte c) {
    if((c == '\n') || (oledX > ((uint8_t)128 - FONT_W)) ) {
      if(wrap)  newLine();
      return 1;
    }
    //if(oledY > OLED_PAGES - FONT_H) return; //needed?
    c = ((c < 9) ? (c + '~') : c) - ' ';
    
    uint16_t offset = ((uint16_t)c) * FONT_W/(FONT_STRETCHH+1) * FONT_H;
    uint8_t line = FONT_H;
    do
    {
      if(FONT_STRETCHV) offset = ((uint16_t)c) * FONT_W/(FONT_STRETCHH+1) * FONT_H/(2*FONT_STRETCHV);
      Wire.beginTransmission(OLED_ADDR); Wire.write(OLED_DATA);
      for (uint8_t i = 0; i < (FONT_W/(FONT_STRETCHH+1)); i++) {
        uint8_t b = pgm_read_byte(&(font[offset++]));
        if(FONT_STRETCHV){
          uint8_t b2 = 0;
          if(line > 1) for(int i = 0; i!=4; i++) b2 |=/* ! */(b & (1<<i)) ? (1<<(i*2)) | (1<<((i*2)+1)): 0x00;
          else         for(int i = 0; i!=4; i++) b2 |=/* ! */(b & (1<<(i+4))) ? (1<<(i*2)) | (1<<((i*2)+1)): 0x00;
          Wire.write(b2);
          if(FONT_STRETCHH) Wire.write(b2);
        } else { Wire.write(b); if(FONT_STRETCHH) Wire.write(b); }
      }
      Wire.endTransmission();
      if(FONT_H == 1) {
        oledX+=FONT_W;
      }
      else {
        if(line > 1) {
          _setCursor(oledX, oledY + 1);
        }
        else {
          _setCursor(oledX + FONT_W, oledY - (FONT_H - 1));
        }
      }
    }
    while (--line);
    return 1;
  }

  void bitmap(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t bitmap[]) {
    uint16_t j = 0;
    for (uint8_t y = y0; y < y1; y++) {
      _setCursor(x0, y);
      Wire.beginTransmission(OLED_ADDR); Wire.write(OLED_DATA);
      for (uint8_t x = x0; x < x1; x++) {
        Wire.write(pgm_read_byte(&bitmap[j++]));
      }
      Wire.endTransmission();
    }
    setCursor(0, 0);
  }
};