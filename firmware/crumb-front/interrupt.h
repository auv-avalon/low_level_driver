#ifndef _INTERUPT_H
#define _INTERUPT_H   1
#include "ADS1100.h"


void enableTriggerTimer();
void disableTriggerTimer();

extern volatile uint16_t ads1112Front[3];
extern volatile uint16_t ads1112Back[3];
extern volatile uint16_t ads1100[5];
extern volatile uint16_t adcDirect[8];
extern volatile int currendADCPort;
extern volatile uint8_t  emergency;
extern volatile uint8_t	triggerEnabled;
extern volatile uint8_t laserEnabled;
void initTimer();
void initADC();
void keepAlive();
ISR(ADC_vect);
ISR(TIMER1_COMPA_vect);
ISR(TIMER3_COMPA_vect);
void initServoTimer();
void calcTacts();
void setLaserRate(uint8_t rate);

#endif
