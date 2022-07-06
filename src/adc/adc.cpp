//void adc_start(uint8_t adcpin, bool ref1v1, uint32_t fs)
//{
  //DIDR0 |= (1 << adcpin); // disable digital input 
 // ADCSRA = 0;             // clear ADCSRA register
 // ADCSRB = 0;             // clear ADCSRB register
//  ADMUX = 0;              // clear ADMUX register
//  ADMUX |= (adcpin & 0x0f);    // set analog input pin
 // ADMUX |= ((ref1v1) ? (1 << REFS1) : 0) | (1 << REFS0);  // If reflvl == true, set AREF=1.1V (Internal ref); otherwise AREF=AVCC=(5V)
 // ADCSRA |= ((uint8_t)log2((uint8_t)(F_CPU / 13 / fs))) & 0x07;  // ADC Prescaler (for normal conversions non-auto-triggered): ADPS = log2(F_CPU / 13 / Fs) - 1; ADSP=0..7 resulting in resp. conversion rate of 1536, 768, 384, 192, 96, 48, 24, 12 kHz
  //ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
 // ADCSRA |= (1 << ADEN);  // enable ADC
  //ADCSRA |= (1 << ADSC);  // start ADC measurements
//#ifdef ADC_NR
//  set_sleep_mode(SLEEP_MODE_ADC);  // ADC NR sleep destroys the timer2 integrity, therefore Idle sleep is better alternative (keeping clkIO as an active clock domain)
//  set_sleep_mode(SLEEP_MODE_IDLE);
//  sleep_enable();
//#endif
//}

//void adc_stop()
//{
  //ADCSRA &= ~(1 << ADATE); // disable auto trigger
//  ADCSRA &= ~(1 << ADIE);  // disable interrupts when measurement complete
 // ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128 prescaler for 9.6kHz
//#ifdef ADC_NR
//  sleep_disable();
//#endif
//  ADMUX = (1 << REFS0);  // restore reference voltage AREF (5V)
//}

// void timer1_start(uint32_t fs)
// {  // Timer 1: OC1A and OC1B in PWM mode
//   TCCR1A = 0;
//   TCCR1B = 0;
//   TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11); // Clear OC1A/OC1B on compare match, set OC1A/OC1B at BOTTOM (non-inverting mode)
//   TCCR1B |= (1 << CS10) | (1 << WGM13) | (1 << WGM12); // Mode 14 - Fast PWM;  CS10: clkI/O/1 (No prescaling)
//   ICR1H = 0x00;
//   ICR1L = min(255, F_CPU / fs);  // PWM value range (fs>78431):  Fpwm = F_CPU / [Prescaler * (1 + TOP)]
//   //TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10); // Clear OC1A/OC1B on compare match, set OC1A/OC1B at BOTTOM (non-inverting mode)
//   //TCCR1B |= (1 << CS10) | (1 << WGM12); // Mode 5 - Fast PWM, 8-bit;  CS10: clkI/O/1 (No prescaling)
//   OCR1AH = 0x00;
//   OCR1AL = 0x00;  // OC1A (SIDETONE) PWM duty-cycle (span defined by ICR).
//   OCR1BH = 0x00;
//   OCR1BL = 0x00;  // OC1B (KEY_OUT) PWM duty-cycle (span defined by ICR).
// }

// void timer1_stop()
// {
//   OCR1AL = 0x00;
//   OCR1BL = 0x00;
// }

// void timer2_start(uint32_t fs)
// {  // Timer 2: interrupt mode
//   ASSR &= ~(1 << AS2);  // Timer 2 clocked from CLK I/O (like Timer 0 and 1)
//   TCCR2A = 0;
//   TCCR2B = 0;
//   TCNT2 = 0;
//   TCCR2A |= (1 << WGM21); // WGM21: Mode 2 - CTC (Clear Timer on Compare Match)
//   TCCR2B |= (1 << CS22);  // Set C22 bits for 64 prescaler
//   TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt TIMER2_COMPA_vect
//   uint8_t ocr = ((F_CPU / 64) / fs) - 1;   // OCRn = (F_CPU / pre-scaler / fs) - 1;
//   OCR2A = ocr;
// }

// void timer2_stop()
// { // Stop Timer 2 interrupt
//   TIMSK2 &= ~(1 << OCIE2A);  // disable timer compare interrupt
//   delay(1);  // wait until potential in-flight interrupts are finished
// }