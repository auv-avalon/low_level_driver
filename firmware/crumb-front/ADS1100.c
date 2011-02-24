#include <inttypes.h>
#include "i2cmaster.h"
#include "uart.h"

#define DEBUG 1


void initADS(int sensorID){
	sensorID*=2;
	sensorID+=144;
	i2c_start(sensorID);
	if(i2c_write(0x04)) //bit 4 continues conversion
			if(DEBUG)
				uart_puts("Write Failed");
	i2c_stop();

}

uint16_t readADS(int sensorID){
	sensorID*=2;
	sensorID+=144;
	uint16_t erg=0;
	i2c_start(sensorID+1);
	erg = i2c_readAck();
	erg+= i2c_readAck()<<8;
	i2c_stop();
	return erg;

}

uint16_t readADS1112(int id){
	uint16_t ret=0;
	id = 0x91 + 2*id;
//	int i,j;

	//for(i=0; i!=1000;i++)
//		for(j=0;j!=105;j++);

	i2c_start(id);
	ret = i2c_readAck()<<8;
	ret+= i2c_readAck();
	i2c_stop();
	return ret;
}

void setADS1112(int id, int port){
	id = 0x90 + 2*id;
	i2c_start(id);
	i2c_write(port==0?172:port==1?204:236);
	i2c_stop();
	//TODO Workaround without it will not work
	i2c_start(id);
	i2c_write(port==0?172:port==1?204:236);
	i2c_stop();

}

void selectPort(int port){
	i2c_start_wait(224);
	i2c_write(port!=0?5:4);
	i2c_stop();
}
