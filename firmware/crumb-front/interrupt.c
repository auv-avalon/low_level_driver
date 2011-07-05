#include <avr/interrupt.h>
#include <avr/io.h>
#include "definitions.h"
#include "interrupt.h"
#include <avr/wdt.h>
#include "depth_sensor.h"

#define SENSOR_OC_STEP		200
#define CAM_LASER_OC_STEP	100
#define LASER_PORT			PORTB
#define LASER_PIN			(_BV(5) | _BV(7))
//#define MINWAITBETWEENSTARTS	(12000)	
#define MINWAITBETWEENSTARTS	(0) //TODO mal richtigen wert rausfinden und nicht einfch abfangen	
#define MINWAITTIME		20*147456 //20*10ms=200ms          //589824 // 40ms			147456 // 10ms
#define MIN_DEPTH_FOR_LASER (-0.5/-0.00228881835938)
#define PRESCALER               1//8//64//256
#define STATE_SHORT1		1
#define STATE_SHORT2		2
#define STATE_LONG		3
#define STATE_WAIT		4
#define STATE_SAFETY		5
#define MAX_TIMER_VALUE		65535

volatile uint8_t	state;
volatile uint16_t	adcDirect[8];
volatile uint8_t	emergency;
volatile uint16_t	ads1112Front[3];
volatile uint16_t	ads1112Back[3];
volatile uint8_t	ads1112loop;
volatile uint16_t	ads1100[5];
volatile int		ads1112cnt=0;
volatile int		currendADCPort=0;
volatile uint16_t	timer0counter;
volatile uint32_t	timer1counter;
volatile uint16_t	frameCounter;
volatile uint32_t	shortExposure=200000;//50000; // microseconds
volatile uint32_t	longExposure=200000;//500000; // microseconds
volatile uint32_t	oldShortExposure;
volatile uint32_t	oldLongExposure;
volatile uint32_t	shortExposureTacts;
volatile uint32_t	longExposureTacts;
volatile uint32_t	clockTactsPerSecond;
volatile uint32_t       waitingTime = 0; // microseconds
volatile uint32_t	oldWaitingTime;
volatile uint32_t	waitingTimeTacts;
volatile uint8_t	stopping=0;
volatile uint8_t 	ignore_laser_depth=0;
//volatile uint16_t	skipper=0;
volatile uint8_t	triggerEnabled;
extern volatile unsigned char ledState;
extern volatile unsigned char laserEnabled=1;

void enableTriggerTimer() {
	state=STATE_SHORT1;
	triggerEnabled=1;
	ledState &= ~_BV(6);	
	//TCNT1 		= 0;
	//frameCounter= 0;
	//OCR1A		= 1;
	//OCR1B		= shortExposure;
	//ICR1		= shortExposure + MINWAITTONEXTFRAME;
/*	TIFR		&= ~(_BV(OCF1A) | _BV(OCF1B));
	ETIFR		|= _BV(OCF1C);
	//TCCR1B		= _BV(CS11) |_BV(CS10) | _BV(WGM13) | _BV(WGM12);
	//TIMSK &= ~_BV(TICIE1);	
*/	//TCNT1 		= 0;
	TIMSK		|=  _BV(OCIE1A) | _BV(OCIE1B);	//Intrrupt on Compare Match
	ETIMSK		|= _BV(OCIE1C);
	
//	skipper=3;
}

void disableTriggerTimer() {
	ledState |= _BV(6);	
	triggerEnabled=0;
	//TIMSK &= ~_BV(TICIE1);
	stopping=1;
}

void calcTacts() {
	oldShortExposure = shortExposure;
        oldLongExposure = longExposure;
	oldWaitingTime = waitingTime;

	clockTactsPerSecond = (uint32_t)round((double)F_CPU/PRESCALER);

	shortExposureTacts = (uint32_t)round((double)shortExposure/1000000*clockTactsPerSecond);
	longExposureTacts = (uint32_t)round((double)longExposure/1000000*clockTactsPerSecond);
	waitingTimeTacts = (uint32_t)round((double)waitingTime/1000000*clockTactsPerSecond);
}

void initTimer() {
	calcTacts();
	timer1counter = 0;

	//WDTCR|=	_BV(WDP2) | _BV(WDP1) | _BV(WDE);
	
	//wdt_enable(WDTO_2S);

	
	//_BV(PORTB6);
	//PORTB&= ~_BV(PORTB5);
	TCNT1 		= 0;
	
	TCCR1A		= 0;//_BV(COM1A1) | _BV(COM1A0);
	TCCR1B		= 0;
	TCCR1C		= 0;

	//TIMSK		|=  _BV(OCIE1A) | _BV(OCIE1B);	//Intrrupt on Compare Match
	//ETIMSK		|= _BV(OCIE1C);
	//TIFR		|= _BV(OCF1A) |  _BV(OCF1B);	//Compare A,B an
	//ETIFR		|= _BV(OCF1C);

	//#define CAMTIME (3000*8)
	ICR1		=	2304; // 0.15625ms
	OCR1A		=	2304; // CAM1
	OCR1B		=	3000; // not needed	
	OCR1C		=	3000; // not needed
	//DDRA |= 1;
	//PORTA &= ~1;
	//frameCounter=0;
	//TCCR1B          = _BV(CS12) | _BV(WGM13) | _BV(WGM12); // CK/256
	//TCCR1B		=  _BV(CS11) | _BV(CS10) | _BV(WGM13) | _BV(WGM12); // CK/64
	//TCCR1B		= _BV(CS11) | _BV(WGM13) | _BV(WGM12); // CK/8
	TCCR1B		= _BV(CS10) | _BV(WGM13) | _BV(WGM12); // CK/1 (no prescaler)
	enableTriggerTimer();
	DDRB |= _BV(PORTB5) | _BV(PORTB6) | _BV(PORTB7);
	PORTB&= ~_BV(PORTB6);
	PORTB&= ~_BV(PORTB5);
}

void keepAlive() {
	emergency		=	0;
	TCNT0			=	0;			// Hier stand TCNT2=0, das kann imho nicht stimmen
	timer0counter	=	0;
}

static volatile uint16_t laser_counter = 0;

ISR(TIMER1_COMPA_vect) {
	/*if(timer1counter == 0) {
		PORTB |= _BV(PORTB6); // cam on
		timer1counter = 1;
	} else {
		PORTB&= ~_BV(PORTB6); // cam off
		timer1counter = 0;
	}*/
	++timer1counter;
	switch(state) {
		case STATE_SHORT1:
		{
			if(PORTB == _BV(PORTB5) && (timer1counter*ICR1 < shortExposureTacts)) { // if laser on
	                        state = STATE_SAFETY;
                                break;
                        }
			if(timer1counter == 1) { // first iteration
				PORTB |= _BV(PORTB6); // cam on
				if((shortExposure != oldShortExposure) ||
					(longExposure != oldLongExposure) ||
					(waitingTime != oldWaitingTime)) // new exposure time
					calcTacts();
			} else if(timer1counter*ICR1 >= shortExposureTacts+MINWAITTIME) {
				timer1counter = 0;
				state = STATE_SHORT2;
			} else if(timer1counter*ICR1 >= shortExposureTacts) {
				PORTB&= ~_BV(PORTB6); // cam off
        	                //PORTB |= _BV(PORTB5); // laser on
			}
                        break;
		}
		case STATE_SHORT2:
		{
			if(timer1counter == 1) { // first iteration
				PORTB |= _BV(PORTB6); // cam on
			} else if(timer1counter*ICR1 >= shortExposureTacts+MINWAITTIME) {
				timer1counter = 0;
				state = STATE_LONG;
			} else if(timer1counter*ICR1 >= shortExposureTacts) {
				PORTB&= ~_BV(PORTB6); // cam off
				PORTB&= ~_BV(PORTB5); // laser off
			}
			break;
		}
		case STATE_LONG:
		{
			if(PORTB == _BV(PORTB5)) { // if laser on
                                state = STATE_SAFETY;
                                break;
                        }
			if(timer1counter == 1) {
				PORTB |= _BV(PORTB6); // cam on
			} else if(timer1counter*ICR1 >= longExposureTacts+MINWAITTIME) {
				timer1counter = 0;
				if(stopping) {
				        TIMSK           &=  ~(_BV(OCIE1A));
			        	TIMSK           &=  ~(_BV(OCIE1B));
				        PORTB &= ~(_BV(PORTB5) | _BV(PORTB6));
				        stopping=0;
				} else {
			        	if(waitingTimeTacts>0)
				        	state = STATE_WAIT;
					else
				        	state = STATE_SHORT1;
				}
			} else if(timer1counter*ICR1 >= longExposureTacts) {
				PORTB&= ~_BV(PORTB6); // cam off
			}
			break;
		}
		case STATE_WAIT:
		{
			if(PORTB == _BV(PORTB5)) { // if laser on
                                state = STATE_SAFETY;
                                break;
                        }
			if(timer1counter*ICR1 >= waitingTimeTacts) {
				state = STATE_SHORT1;
				timer1counter = 0;
			}
			break;
		}
		case STATE_SAFETY:
		{
			PORTB&= ~_BV(PORTB5); // laser off
			PORTB&= ~_BV(PORTB6); // cam off
		}
	}
}

ISR(TIMER1_COMPB_vect) {
}

ISR(TIMER1_COMPC_vect) {
}

void initADC() {
	ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) | _BV(ADIE);
  	ADMUX = _BV(REFS0); // using AVCC
  	ADMUX = (ADMUX & 0b11100000) | currendADCPort; // choosing channel
}

ISR(ADC_vect) {
	adcDirect[currendADCPort] = ADCL + (ADCH << 8);

	while(ADCSRA & _BV(ADIF));		// warum? wird das überhaupt jemals gecleared, solange der interrupt noch läuft?
	ADMUX = (ADMUX & 0b11100000) | currendADCPort; // choosing channel
	
	if( !(ADCSRA & _BV(ADSC)) ) {
		ADCSRA |= _BV(ADSC); // start conversion
	}

	if( currendADCPort==7 ) {
		currendADCPort=0;
	} else {
		currendADCPort++;
	}
}
