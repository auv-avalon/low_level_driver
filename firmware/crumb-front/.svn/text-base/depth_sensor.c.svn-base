#include <avr/io.h>

#include "depth_sensor.h"
#include "ADS1100.h"

volatile int32_t calibration_value=0;
static int32_t last_depth_value=0;

void calibrate_depth_sensor() {
	calibration_value = last_depth_value;
}

int32_t get_current_depth() {
	if(calibration_value==0)
		calibration_value = last_depth_value;
	return last_depth_value - calibration_value;
}

void init_depth_sensor() {
	TCCR0		=	0;
	TIMSK		|=	_BV(OCIE0);
	TCCR0		=	_BV(CS00) | _BV(CS01) | _BV(CS02);
	TIFR		|=	_BV(OCF0);
	OCR0		=	250;
	//emergency	=	0;
}

ISR(TIMER0_COMP_vect) {
	selectPort(0);
	last_depth_value = readADS1112(0);
	setADS1112(0,0);

}
