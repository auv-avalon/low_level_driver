#include <inttypes.h>
#include "i2cmaster.h"
#include "uart.h"
#include <stdlib.h>
#include "lm75.h"

#define DEBUG 0

uint16_t readTemp(int sensorID){
		sensorID*=2;
		sensorID+=144;
//		char buffer[30];
		uint16_t ret=0;
		i2c_start(sensorID+1);
		//uint16_t i =0,j=0;
		ret = i2c_readAck();	
		
		//sprintf(buffer,"T%u:%u",sensorID,ret);
		//uart1_puts(buffer);
		ret +=  i2c_readNak() << 8;
		
		
		//if(ret>>7==1)
		//	uart1_puts(".5 ");
		//else
		//	uart1_puts(".0 ");

		i2c_stop(); 
		i2c_stop(); 		
		return ret;
}
