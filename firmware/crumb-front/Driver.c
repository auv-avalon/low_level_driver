#include <avr/io.h>
#include <inttypes.h>
#include <stdio.h>
#include <avr/interrupt.h>

#include "i2cmaster.h"
#include "interrupt.h"
#include "lm75.h"
#include "ADS1100.h"
#include "uart.h"
#include "servo.h"
#include "definitions.h"
#include "depth_sensor.h"
#include <string.h>
#include <avr/wdt.h>

#define Reset_AVR() wdt_enable(WDTO_30MS); while(1) {}
#define					UART_BAUD_RATE 			115200

enum MessageType{
	  DepthValue=0,
	  TemperatureValue,
	  SetLEDValue,
	  String,
	  LaserOverride,
	  SetLongExposure,
	  SetShortExposure,
	  SetWaitingTime,
	  SetServoValue,
	  Reset,
	  CalibrateDepth
	};

//#define DEBUG 1
#define FIFOSIZE 500
uint8_t	buffer[FIFOSIZE]; //FIFO buffer
uint8_t readPos,writePos;
char buff[200];

//extern volatile uint8_t	emergency;

extern volatile uint8_t ignore_laser_depth;
volatile unsigned char ledState=0;
extern volatile uint16_t shortExposure;
extern volatile uint32_t longExposure;
extern volatile uint32_t waitingTime;
extern volatile int32_t calibration_value; //Depth offset



#ifdef DEBUG
	writeDebug(char *c, uint8_t i, uint16_t value){
		char buff[300];
		sprintf(buff,"%s%i: %i      ",c,i,value);
		uart1_puts(buff);
		uart_puts(buff);
	}
#endif

void uart_send(uint8_t *data, unsigned int size){
  for(int i=0;i<size;i++){
    uart1_putc((char)data[i]);
    uart_putc((char)data[i]);
  }
}


#if 0
int avalon_getc() {
	unsigned int byte = uart1_getc();

	char tmp[20];
	if(byte == UART_NO_DATA){
		unsigned int byte2 = uart_getc();
		if(byte2 == UART_NO_DATA){
			return AVALON_PROTOCOL_NO_DATA;
				
		}else if(byte2 == UART_BUFFER_OVERFLOW) {
			avalonSendMessage(1, snprintf(tmp, 20, "BUFFEROVERRUN"), (uint8_t*)tmp);
			return AVALON_PROTOCOL_NO_DATA;
		}else{
			return byte & 0xff;
		}
	}else if(byte == UART_BUFFER_OVERFLOW) {
		avalonSendMessage(1, snprintf(tmp, 20, "BUFFEROVERRUN"), (uint8_t*)tmp);
		return AVALON_PROTOCOL_NO_DATA;
	} else {
		return byte & 0xff;
	}
}

void avalon_send_raw(const uint8_t *buffer, int len) {
	int i;
	for(i = 0; i < len; i++){
		uart1_putc(buffer[i]);
		uart_putc(buffer[i]);
	}
}
#endif

void sendString(char *c){
	char hello[100];
	int len = sprintf(hello+3,"%s",c);
	hello[0]='#';
	hello[1]=len+4;
	hello[2]=String;
	hello[len+3]='\n';
	uart_send(hello,len+4);
}


void set_leds(uint8_t leds) {
	PORTC = (PORTC & 0xC0) | (leds & 0x3F);
	PORTG = (PORTG & 0xFC) | (leds >> 6);
}
	

void processMessage(uint8_t *buffer){
  
  switch(buffer[2]){
    case SetLEDValue:
    {
      set_leds(buffer[3]);
      sprintf(buff,"Setting LED Value %i",buffer[3]);
      sendString(buff);
      break;
    }
    case LaserOverride:
    {
      ignore_laser_depth = buffer[3];
      break;
    }
    case SetLongExposure:
    {
      uint32_t oldExposure = longExposure;
      longExposure = buffer[6];
      longExposure <<= 8;
      longExposure |= buffer[5];
      longExposure <<= 8;
      longExposure |= buffer[4];
      longExposure <<= 8;
      longExposure |= buffer[3];
      //longExposure = (buffer[3] | (buffer[4]<<8) | (buffer[5]<<16) | (buffer[6]<<24));
      
      if(shortExposure > longExposure){
	sprintf(buff,"Long Exposure below Short Exposure, skipping %u %lu", shortExposure, longExposure);
	sendString(buff);
	longExposure = oldExposure;
      }
      
      uint8_t buf[8];
      buf[0] = '#';
      buf[1] = 8;
      buf[2] = SetLongExposure;
      buf[3] = buffer[3];
      buf[4] = buffer[4];
      buf[5] = buffer[5];
      buf[6] = buffer[6];
      buf[7] = '\n';
      uart_send(buf,8);

      break;
    }
    case SetShortExposure:
    {
      uint16_t oldExposure = shortExposure;
      shortExposure = buffer[3] | (buffer[4]<<8);
      if(shortExposure < 280){
	      sprintf(buff,"Short Exposure below 250 is not allowed, skipping");
	      sendString(buff);
      }
      if(shortExposure > longExposure){
	sprintf(buff,"Short Exposure above Long Exposure, skipping %u %lu", shortExposure, longExposure);
	sendString(buff);
	shortExposure = oldExposure;
      }
      uint8_t buf[6];
      buf[0] = '#';
      buf[1] = 6;
      buf[2] = SetShortExposure;
      buf[3] = buffer[3];
      buf[4] = buffer[4];
      buf[5] = '\n';
      uart_send(buf,6);

      break;
    }
    case SetWaitingTime:
    {
      waitingTime = buffer[6];
      waitingTime <<= 8;
      waitingTime |= buffer[5];
      waitingTime <<= 8;
      waitingTime |= buffer[4];
      waitingTime <<= 8;
      waitingTime |= buffer[3];

      uint8_t buf[8];
      buf[0] = '#';
      buf[1] = 8;
      buf[2] = SetWaitingTime;
      buf[3] = buffer[3];
      buf[4] = buffer[4];
      buf[5] = buffer[5];
      buf[6] = buffer[6];
      buf[7] = '\n';
      uart_send(buf,8);
      break;
    }
    case SetServoValue:
    {
      setServo(buffer[3] | (buffer[4]<<8));
      //Send response
      uint8_t buf[6];
      buf[0] = '#';
      buf[1] = 6;
      buf[2] = SetServoValue;
      buf[3] = buffer[3];
      buf[4] = buffer[4];
      buf[5] = '\n';
      uart_send(buf,6);
      break;
    }
    case Reset:
    {
    	Reset_AVR();
	break;
    }
    case CalibrateDepth:
    {
    	calibrate_depth_sensor();	
	break;
    }

    default:{
      char buff[200];
      sprintf(buff,"Cannot Parse message Type %i,%i,%i,%i",buffer[0],buffer[1],buffer[2],buffer[3]);
      sendString(buff);
    }
  }
}


uint8_t bytesAvailible(){
  if(readPos < writePos){
    return writePos-readPos;
  }else if(readPos==writePos){
    return 0;
  }else{
    return (FIFOSIZE-readPos) + writePos;
  }
}

void processBuffer(void){
  while(buffer[readPos] != '#'){
    if(bytesAvailible() > 1){
      readPos++;
    }else{
      //sendString("cannot find start");
      return;
    }
  }
  
  if(bytesAvailible() < 2){
    //char buff[200];
    //sprintf(buff,"Not enouth bytes: %i",bytesAvailible());
    //sendString(buff);
    return; //No data here yet
  }
  
  uint8_t *p = buffer+readPos;
  if(p[0] == '#'){ //Check Header
    if(bytesAvailible() >= p[1]){ //Check Length
      if(p[p[1]-1] == '\n'){ //check Message End
	processMessage(&buffer[readPos]); //Process Message
	readPos+=p[1]-2;
	readPos%=FIFOSIZE;
	return;
      }else{
	
	char buff[200];
	sprintf(buff,"Broken Data: (%i,%i,%i,%i,%i) size: (%i) (BA: %i)",p[0],p[1],p[2],p[3],p[4],p[1]-1,bytesAvailible());
	sendString(buff);
	
	readPos++; //Message seems broken
	readPos%=FIFOSIZE;
	return;
      }
    }
  }
}


int main(void){
	//DDRB =255;
	//PORTB = 0;
	//

	uart1_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );

	initServo();
	initTimer();
	//initADC();
	

	selectPort(1);
	initADS(6);
	init_depth_sensor();

	DDRC = 0xff;
	DDRG = 0xff;
	readPos=0;
	writePos=0;
	//initServoTimer();
	selectPort(0);
	

	if (!(ADCSRA & _BV(ADSC))){
		ADCSRA |= _BV(ADSC); // start conversion
	}

	sei();

	ads1100[0]=0;

	DDRA= 255;
	PORTA |= 1;
	DDRC = 255;
	DDRB=255;
	//PORTB=255;
	DDRF=255;
	PORTF = 255;
      for(int i=0;i<FIFOSIZE;i++)
	 buffer[i]=0;
	//PORTB = 255;

	//avalonInit(avalon_callbacks, sizeof(avalon_callbacks)/sizeof(ReceiveCallback*));

	//avalonSendMessage(1, 13,(uint8_t const*) "crumb startet");
	int32_t old_depth = 0xEFFFFFFF;
	//avalonSendMessage(4, 0, 0); //sent reset msg

	sendString("Crumb started");
	
	for(;;) {
	  	unsigned int byte = uart1_getc();
		while(byte != UART_NO_DATA){
		  buffer[writePos] = byte;
		  writePos++;
		  writePos%=FIFOSIZE;
		  if(readPos==writePos){
		    readPos++; //Buffer voll
		    sendString("Buffer Overflow");
		  }
		  readPos%=FIFOSIZE;
		  byte = uart1_getc();
		}
	  	byte = uart_getc();
		while(byte != UART_NO_DATA){
		  buffer[writePos] = byte;
		  writePos++;
		  writePos%=FIFOSIZE;
		  if(readPos==writePos){
		    readPos++; //Buffer voll
		    sendString("Buffer Overflow");
		  }
		  readPos%=FIFOSIZE;
		  byte = uart_getc();
		}

		processBuffer();
		
		
		
		
	#if 0
			uint16_t tempbuff[8];
			int i;

			//TIMSK &= ~_BV(OCIE1A); //Intrrupt off to prevent Chanel switching
			for(i=0;i<8;i++)
				tempbuff[i] = readTemp(i);
			//TIMSK |= _BV(OCIE1A); //Intrrupt on

			for(i=0;i<3;i++) buff[i] =  ads1112Front[i];
			for(i=0;i<5;i++) buff[i+3]= ads1100[i];

			for(i=0;i<8;i++) uart1_puti(buff[i]);
			for(i=0;i<8;i++) uart1_puti(tempbuff[i] + '\0');
			for(i=0;i<8;i++) uart1_puti(adcDirect[i]);
			uart1_putc('\r');
			uart1_putc('\n');

			loop=0;
	#endif
			
				
			int32_t depth_reading = get_current_depth();
			if(depth_reading != old_depth) {
				uint8_t buffer[8];
				buffer[0] = '#';
				buffer[1] = 8;
				buffer[2] = DepthValue;
				memcpy(&buffer[3],(const void*)&depth_reading,4);
				buffer[7] = '\n';
				old_depth = depth_reading;
				uart_send(buffer,8);
			}

#if 0
		if(emergency){
			uart_puts(allOff);
			keepAlive();
		}
		while(recived != UART_NO_DATA){
			keepAlive();
		
			if(recived != 'P'){
				if(recived == 'S') {
					recived2 = uart1_getc();
					recived3 = uart1_getc();
					if(recived2 != UART_NO_DATA && recived3 != UART_NO_DATA){
						uint16_t pos = recived2<<8 | recived3;	// uint, damit es keine Probleme mit signed shift gibt
						setServo(pos);							// und dann einfach ganz frech als signed int benutzen
					}
					/*
				}else if(recived == 'K'){
					PORTB=0;
				}else if(recived == 'L'){
					PORTB=255;
					*/
				}else{
				uart_putc(recived);
				}
			}
			recived = uart1_getc();
		}
#endif
	}
}


