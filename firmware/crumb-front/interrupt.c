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
#define MINWAITTIME		3000
#define MIN_DEPTH_FOR_LASER (-0.5/-0.00228881835938)

volatile uint16_t	adcDirect[8];
volatile uint8_t	emergency;
volatile uint16_t	ads1112Front[3];
volatile uint16_t	ads1112Back[3];
volatile uint8_t	ads1112loop;
volatile uint16_t	ads1100[5];
volatile int		ads1112cnt=0;
volatile int		currendADCPort=0;
volatile uint16_t	timer0counter;
volatile uint16_t	frameCounter;
volatile uint16_t	shortExposure=15000;
volatile uint16_t	longExposure=23000; //26000
volatile uint32_t       waitUntilNextFrame;
volatile uint8_t        maxFramesPerSecond = 2;
volatile uint8_t        framesPerSecond;
volatile uint8_t	stopping=0;
volatile uint8_t 	ignore_laser_depth=0;
//volatile uint16_t	skipper=0;
volatile uint8_t	triggerEnabled;
extern volatile unsigned char ledState;
extern volatile unsigned char laserEnabled=1;

void enableTriggerTimer() {
	triggerEnabled=1;
	ledState &= ~_BV(6);	
	TCNT1 		= 0;
	frameCounter= 0;// TODO doppelt
	OCR1A		= 1; // TODO doppelt
	//OCR1B		= shortExposure;
	//ICR1		= shortExposure + MINWAITTONEXTFRAME;
	TIFR		&= ~(_BV(OCF1A) | _BV(OCF1B));
	ETIFR		|= _BV(OCF1C);
	//TCCR1B		= _BV(CS11) |_BV(CS10) | _BV(WGM13) | _BV(WGM12);
	//TIMSK &= ~_BV(TICIE1);	
	TCNT1 		= 0; // TODO doppelt
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

void initTimer() {
	uint32_t clockTicks = 16000000/64; // 16MHz and CK/64
        uint32_t minClockTicksPerCycle = 2*shortExposure + longExposure + 3*MINWAITTIME + 3;
        framesPerSecond = clockTicks/minClockTicksPerCycle;
        if(framesPerSecond > maxFramesPerSecond)
                framesPerSecond = maxFramesPerSecond;
        uint32_t clockTicksPerCycle = clockTicks/framesPerSecond;
        waitUntilNextFrame = clockTicksPerCycle-minClockTicksPerCycle;

	//WDTCR|=	_BV(WDP2) | _BV(WDP1) | _BV(WDE);
	
	//wdt_enable(WDTO_2S);

	
	//_BV(PORTB6);
	PORTB&= ~_BV(PORTB5); // TODO doppelt
	TCNT1 		= 0;
	
	TCCR1A		= 0;//_BV(COM1A1) | _BV(COM1A0);
	TCCR1B		= 0;
	TCCR1C		= 0;

	//TIMSK		|=  _BV(OCIE1A) | _BV(OCIE1B);	//Intrrupt on Compare Match
	//ETIMSK		|= _BV(OCIE1C);
	//TIFR		|= _BV(OCF1A) |  _BV(OCF1B);	//Compare A,B an
	//ETIFR		|= _BV(OCF1C);

	//#define CAMTIME (3000*8)
	ICR1		=	1000; //Any Value don't matterMINWAITBETWEENSTARTS;   //
	OCR1A		=	1; //CAMTIME/2;	    //Laser ein
	OCR1B		=	longExposure;//((CAMTIME/4))-5000;  //CAM1	
	OCR1C		=	MINWAITTIME-500;	            //CAM2
	//DDRA |= 1;
	//PORTA &= ~1;
	frameCounter=0; // TODO doppelt
	TCCR1B		= _BV(CS11) |_BV(CS10) | _BV(WGM13) | _BV(WGM12);
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
	//wdt_reset();	

//	frameCounter++;
	//PORTB=255;
//	if(skipper){
//		return;
//	}
	frameCounter++;
	PORTB |= _BV(PORTB6);

	
	if(frameCounter==3){
		OCR1B = longExposure;
		ICR1 = longExposure + MINWAITTIME + waitUntilNextFrame;
	}else{
		OCR1B = shortExposure;
		//if(shortExposure+MINWAITTIME < MINWAITBETWEENSTARTS)
		//	ICR1 = shortExposure + MINWAITBETWEENSTARTS; //TODO kleineren wert reintickern
		//else
		ICR1 = shortExposure + MINWAITTIME;	// TODO: Das ist == MINWAITTONEXTFRAME, das kann nicht stimmen, oder?
		//ICR1 = shortExposure + MINWAITTONEXTFRAME;
	}
}

ISR(TIMER1_COMPB_vect) {
	PORTB&= ~_BV(PORTB6);
	PORTB&= ~_BV(PORTB5);
	if(frameCounter==3){
		frameCounter=0;
		if(stopping){
			TIMSK		&=  ~(_BV(OCIE1A));
			TIMSK		&=  ~(_BV(OCIE1B));
			PORTB &= ~(_BV(PORTB5) | _BV(PORTB6));
			stopping=0;
		}else{
			//if((ignore_laser_depth || get_current_depth() > MIN_DEPTH_FOR_LASER) && laserEnabled){
			if(laserEnabled){
				PORTB |= _BV(PORTB5);
				ledState |= _BV(5);	
			}else{
				ledState &= ~_BV(5);	
				//char buff[500];
				//avalonSendMessage(1,snprintf(buff,500,"Curr Depth: %i < %i (offset: %i)\n",get_current_depth(),MIN_DEPTH_FOR_LASER,calibration_value,buff));
				}
		}
		frameCounter=0;
	}
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
