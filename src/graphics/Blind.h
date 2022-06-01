#pragma once

class Blind : public Print {  // This class is a dummy LCD replacement
public:
  size_t write(uint8_t b){}
  void setCursor(uint8_t _x, uint8_t _y){}
  void cursor(){}
  void noCursor(){}
  void begin(uint8_t x = 0, uint8_t y = 0){}
  void noDisplay(){}
  void createChar(uint8_t l, uint8_t glyph[]){}
};