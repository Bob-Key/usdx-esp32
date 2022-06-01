/**
 * TODO: Extract to a class
 */
#pragma once

#ifdef CW_DECODER
volatile uint8_t cwdec = 1;
static int32_t avg = 256;
static uint8_t sym;
static uint32_t amp32 = 0;
volatile uint32_t _amp32 = 0;
static char out[] = "                ";
volatile uint8_t cw_event = false;

void printsym(bool submit = true){
  if(sym<128){ char ch=pgm_read_byte_near(m2c + sym); if(ch != '*'){
#ifdef CW_INTERMEDIATE
    out[15] = ch; cw_event = true; if(submit){ for(int i=0; i!=15;i++){ out[i]=out[i+1]; } out[15] = ' '; }   // update LCD, only shift when submit is true, otherwise update last char only
#else
    for(int i=0; i!=15;i++) out[i]=out[i+1]; out[15] = ch; cw_event = true;   // update LCD
#endif
  } }
  if(submit) sym=1;
}

bool realstate = LOW;
bool realstatebefore = LOW;
bool filteredstate = LOW;
bool filteredstatebefore = LOW;
uint8_t nbtime = 16;  // 6 // ms noise blanker
uint32_t starttimehigh;
uint32_t highduration;
uint32_t hightimesavg;
uint32_t lowtimesavg;
uint32_t startttimelow;
uint32_t lowduration;
uint32_t laststarttime = 0;
uint8_t wpm = 25;

inline void cw_decode()
{
  int32_t in = _amp32;
  EA(avg, in, (1 << 8));
  realstate = (in>(avg*1/2));  // threshold

  // here we clean up the state with a noise blanker
  if(realstate != realstatebefore){
    laststarttime = millis();
  }
//#define NB_SCALED_TO_WPM    1   // Scales noise-blanker timing the actual CW speed; this should reduce errors from noise at low speeds; this may have side-effect with fast speed changes that fast CW will be filtered out
#ifdef NB_SCALED_TO_WPM
  if((millis() - laststarttime) > min(1200/(20*2), max(1200/(40*2), hightimesavg/6))){
#else
  if((millis() - laststarttime) > nbtime){
#endif
    if(realstate != filteredstate){
      filteredstate = realstate;
      //dec2();
    }
  } else avg += avg/100; // keep threshold above noise spikes (increase threshold with 1%)

  dec2();
  realstatebefore = realstate;
}

//#define NEW_CW  1   // CW decoder portions from by Hjalmar Skovholm Hansen OZ1JHM, source: http://www.skovholm.com/decoder11.ino
#ifdef NEW_CW
void dec2()
{
 // Then we do want to have some durations on high and low
 if(filteredstate != filteredstatebefore){
 if(menumode == 0){ lcd.noCursor(); lcd.setCursor(15, 1); lcd.print((filteredstate) ? 'R' : ' '); stepsize_showcursor(); }

  if(filteredstate == HIGH){
    starttimehigh = millis();
    lowduration = (millis() - startttimelow);
  }

  if(filteredstate == LOW){
    startttimelow = millis();
    highduration = (millis() - starttimehigh);
    if(highduration < (2*hightimesavg) || hightimesavg == 0){
      hightimesavg = (highduration+hightimesavg+hightimesavg)/3;     // now we know avg dit time ( rolling 3 avg)
    }
    if(highduration > (5*hightimesavg)){
      hightimesavg = highduration/3;     // if speed decrease fast ..      
      //hightimesavg = highduration+hightimesavg;     // if speed decrease fast ..
    }
  }
 }

 // now we will check which kind of baud we have - dit or dah, and what kind of pause we do have 1 - 3 or 7 pause, we think that hightimeavg = 1 bit
 if(filteredstate != filteredstatebefore){
  if(filteredstate == LOW){  //// we did end a HIGH
#define FAIR_WEIGHTING    1
#ifdef FAIR_WEIGHTING
    if(highduration < (hightimesavg + hightimesavg/2) && highduration > (hightimesavg*6/10)){ /// 0.6 filter out false dits
#else
    if(highduration < (hightimesavg*2) && highduration > (hightimesavg*6/10)){ /// 0.6 filter out false dits
#endif
      sym=(sym<<1)|(0);        // insert dit (0)
    }
#ifdef FAIR_WEIGHTING
    if(highduration > (hightimesavg + hightimesavg/2) && highduration < (hightimesavg*6)){
#else
    if(highduration > (hightimesavg*2) && highduration < (hightimesavg*6)){
#endif
      sym=(sym<<1)|(1);        // insert dah (1)
      wpm = (wpm + (1200/((highduration)/3) * 4/3))/2;
    }
  }
 
   if(filteredstate == HIGH){  // we did end a LOW 
     uint16_t lacktime = 10;
     if(wpm > 25)lacktime=10; // when high speeds we have to have a little more pause before new letter or new word 
     if(wpm > 30)lacktime=12;
     if(wpm > 35)lacktime=15;

#ifdef FAIR_WEIGHTING
     if(lowduration > (hightimesavg*(lacktime*1/10)) && lowduration < hightimesavg*(lacktime*5/10)){ // letter space
#else
     if(lowduration > (hightimesavg*(lacktime*7/80)) && lowduration < hightimesavg*(lacktime*5/10)){ // letter space
     //if(lowduration > (hightimesavg*(lacktime*2/10)) && lowduration < hightimesavg*(lacktime*5/10)){ // letter space
#endif
       printsym();
   }
   if(lowduration >= hightimesavg*(lacktime*5/10)){ // word space
     printsym();
     printsym();  // print space
   }
  }
 }

 // write if no more letters
  if((millis() - startttimelow) > (highduration * 6) && (sym > 1)){
    printsym();
  }

  filteredstatebefore = filteredstate;
}

#else // OLD_CW

void dec2()
{
 if(filteredstate != filteredstatebefore){ // then we do want to have some durations on high and low
  if(menumode == 0){ lcd.noCursor(); lcd.setCursor(15, 1); lcd.print((filteredstate) ? 'R' : ' '); stepsize_showcursor(); }

  if(filteredstate == HIGH){
    starttimehigh = millis();
    lowduration = (millis() - startttimelow);
    //highduration = 0;

    if((sym > 1) && lowduration > (hightimesavg*2)/* && lowduration < hightimesavg*(5*lacktime)*/){ // letter space
      printsym();
      wpm = (1200/hightimesavg * 4/3);
      //if(lowduration >= hightimesavg*(5)){ sym=1; printsym(); } // (print additional space) word space
    }
    if(lowduration >= hightimesavg*(5)){ sym=1; printsym(); } // (print additional space) word space
  }

  if(filteredstate == LOW){
    startttimelow = millis();
    highduration = (millis() - starttimehigh);
    //lowduration = 0;
    if(highduration < (2*hightimesavg) || hightimesavg == 0){
      hightimesavg = (highduration+hightimesavg+hightimesavg)/3;     // now we know avg dit time (rolling 3 avg)
    }
    if(highduration > (5*hightimesavg)){
      hightimesavg = highduration/3;     // if speed decrease fast ..      
      //hightimesavg = highduration+hightimesavg;     // if speed decrease fast ..
    }
    if(highduration > (hightimesavg/2)){ sym=(sym<<1)|(highduration > (hightimesavg*2));       // dit (0) or dash (1)
#if defined(CW_INTERMEDIATE) && !defined(OLED) && !defined(LCD_I2C) && (F_MCU >= 20000000)
      printsym(false);
#endif
    }
  }
 }
 
 if(((millis() - startttimelow) > hightimesavg*(6)) && (sym > 1)){
 //if(((millis() - startttimelow) > hightimesavg*(12)) && (sym > 1)){
   //if(sym == 2) sym = 1; else // skip E E E E E
   printsym();  // write if no more letters
   //sym=0; printsym(); // print special char
   //startttimelow = millis();
 }
 
 filteredstatebefore = filteredstate;
}
#endif //OLD_CW
#endif  //CW_DECODER