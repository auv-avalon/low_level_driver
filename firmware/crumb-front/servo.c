#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>


#define SERVO_TIMER_TCCR3A (_BV(COM3A1))							// OC1A löschen, wenn OCR1A beim Hochzählen erreicht wird,
																	// OC1A setzen, wenn OCR1A beim Runterzählen erreicht wird
#define SERVO_TIMER_TCCR3B (_BV(CS30) | _BV(WGM33))		// Prescaler = 8, phase & frequency correct pwm mode
#define SERVO_TIMER_TCCR3C 0										// Unbenutzt, weil PWM-Mode benutzt wird
#define SERVO_50HZ_ICR	18432										// Bei "Frequency and phase correct PWM" und 14745600Hz Clock 
																	// entspricht das einer Periodenlänge von 20ms entspr. 50Hz
#define SERVO_LOW		  0L										// Wert für OCR1x: 1.0ms (0,9993ms)
#define SERVO_HIGH		 65500L										// Wert für OCR1x: 2.0ms (1,9998ms)
//#define SERVO_PITCH		((SERVO_HIGH-SERVO_LOW)-150)					// Steigung der PWM-Kurve (um wieviel steigt sie von der Mittelstellung bis
																	// zum Maximalausschlag?)
#define SERVO_MAX_ANGLE	180L											// Maximaler Ausschlag des Servos in Grad
#define SERVO_FIXPOINT	10L											// Position des Festkommas für die Gradangabe des Servos, (10 bedeutet, dass 
																	// position=123 genau 12,3° bedeutet)

// Die Auflösung für den Laser beträgt also 921 Schritte, bei Untersetzung 2:1 
// und (theoretischem) Aktionsradius von 0° bis 180° und Spielfreiheit des 
// Servos entspricht das einer Auflösung von 0,0977°

void setServo(uint16_t position) {
	// Position berechnen (Teilen erst nach allen Multiplikationen, um keine Probleme mit 
	// Ganzzahlarithmetik zu bekommen)
	int32_t servo_value = position; //SERVO_LOW + ((uint32_t)position * SERVO_PITCH) / (SERVO_MAX_ANGLE * SERVO_FIXPOINT);

	
	// Bereich begrenzen
	if( servo_value < SERVO_LOW )
		servo_value = SERVO_LOW;
	else if( servo_value > SERVO_HIGH )
		servo_value = SERVO_HIGH;
	if(servo_value == 0){
		TCCR3A = 0;
		TCCR3B = 0;
		PORTE = 0;
	}else{
		TCNT3   = 	0;
		TCCR3A	=	SERVO_TIMER_TCCR3A;
		TCCR3B  =       SERVO_TIMER_TCCR3B;
		// Neue Servoposition übernehmen
		OCR3A = (uint16_t)(servo_value&0xffff);
	}
}

void initServo() {
	TCCR3A	=	0;//SERVO_TIMER_TCCR3A;
	TCCR3B	=	0;//SERVO_TIMER_TCCR3B;
	PORTE 	= 	0;
	TCCR3C	=	SERVO_TIMER_TCCR3C;
	ICR3	=	SERVO_50HZ_ICR;
	OCR3A	=	0; //Init to 0   //(SERVO_HIGH-SERVO_LOW)/2;
	ETIMSK	&= 	~(_BV(TICIE3) | _BV(OCIE3A) | _BV(OCIE3B) | _BV(TOIE3));	// Sämtliche Interruptquellen für Timer 1 abschalten
	ETIMSK	&= 	~(_BV(OCIE3C));
	TCNT3	= 	0;														// Servo zurücksetzen
	DDRE	|= 	_BV(DDE3) | _BV(DDE4) | _BV(DDE5);												// PORTB:5 (OC1A) als Output benutzen (und damit PWM auch tatsächlich rausgeben)
}
