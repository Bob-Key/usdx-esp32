
#ifndef VSS_METER
int analogSafeRead(uint8_t pin, bool ref1v1 = false){  // performs classical analogRead with default Arduino sample-rate and analog reference setting; restores previous settings
  noInterrupts();
  for(;!(ADCSRA & (1 << ADIF)););  // wait until (a potential previous) ADC conversion is completed
  uint8_t adcsra = ADCSRA;
  uint8_t admux = ADMUX;
  ADCSRA &= ~(1 << ADIE);  // disable interrupts when measurement complete
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128 prescaler for 9.6kHz
  if(ref1v1) ADMUX &= ~(1 << REFS0);  // restore reference voltage AREF (1V1)
  else ADMUX = (1 << REFS0);  // restore reference voltage AREF (5V)
  delay(1);  // settle
  int val = analogRead(pin);
  ADCSRA = adcsra;
  ADMUX = admux;
  interrupts();
  return val;
}
#else //VSS_METER
uint16_t analogSafeRead(uint8_t adcpin, bool ref1v1 = false){
   noInterrupts();
   uint8_t oldmux = ADMUX;
   ADMUX = (3 & 0x0f) | ((ref1v1) ? (1 << REFS1) : 0) | (1 << REFS0);  // set MUX for next conversion   note: hardcoded for BUTTONS adcpin
   for(;!(ADCSRA & (1 << ADIF)););  // wait until (a potential previous) ADC conversion is completed
   delayMicroseconds(16);  // settle
   ADCSRA |= (1 << ADSC);    // start next ADC conversion
   for(;!(ADCSRA & (1 << ADIF)););  // wait until ADC conversion is completed
   ADMUX = oldmux;
   uint16_t adc = ADC;
   interrupts();
   return adc;
}
#endif