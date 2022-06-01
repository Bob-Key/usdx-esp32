#pragma once

template<class parent>class Display : public parent {  // This class spoofs display contents and cursor state
public:
#ifdef CAT_EXT
  uint8_t x, y;
  bool curs;
  char text[2*16+1];
  Display() : parent() { clear(); };
  size_t write(uint8_t b){ if((x<16) && (y<2)){ text[y*16+x] = ((b < 9) ? "> :*#AB"[b-1] /*(b + 0x80 - 1)*/ /*(uint8_t[]){ 0xAF, 0x20, 0xB0, 0xB1, 0xB2, 0xA6, 0xE1, 0x20 }[b-1]*/ : b); x++; } return parent::write(b); }
  void setCursor(uint8_t _x, uint8_t _y){ x = _x; y = _y; parent::setCursor(_x, _y); }
  void cursor(){ curs = true; parent::cursor(); }
  void noCursor(){ curs = false; parent::noCursor(); }
  void clear(){ for(uint8_t i = 0; i != 2*16; i++) text[i] = ' ' ; text[2*16] = '\0'; x = 0; y = 0; }
#endif //CAT_EXT
};