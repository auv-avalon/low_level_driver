#ifndef DEPTH_SENSOR_H
#define DEPTH_SENSOR_H
#include <avr/interrupt.h>

extern volatile int32_t calibration_value;
void calibrate_depth_sensor();
int32_t get_current_depth();
void init_depth_sensor();
ISR(TIMER0_COMP_vect);
#endif
