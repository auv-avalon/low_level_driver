#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "../../../firmware/crumb-front/avalon_protocol.h"

//static FILE *serialPortWrite, *serialPortRead;
int readFD,writeFD;

void init_avalon_protocol_functions(int _writeFD, int _readFD){
	if(_writeFD < 0 || _readFD <0){
		fprintf(stderr,"DAMM my Crumb is closed: writeFD: %i, readFD: %i. Exiting...\n",_writeFD,_readFD);
		exit(-3);
	}
	readFD = _readFD;
	writeFD = _writeFD;
	//serialPortRead = fdopen(readFD, "rb");
	//serialPortWrite = fdopen(writeFD, "wb");
}

int avalon_getc() {
	if(writeFD < 0 || readFD <0){
		fprintf(stderr,"DAMM my Crumb is closed: writeFD: %i, readFD: %i. Exiting...\n",writeFD,readFD);
		exit(-3);
	}
	int buff[10];
	int size = read(readFD,buff,1);
	while(size < 1){
		usleep(10);
		size = read(readFD,buff,1);
	}
	return buff[0];
	//return fgetc(serialPortRead);
}

void avalon_send_raw(const uint8_t *buffer, int len) {
	int i;
	//printf("starting write: ");
	if(writeFD < 0 || readFD <0){
		fprintf(stderr,"DAMM my Crumb is closed: writeFD: %i, readFD: %i. Exiting...\n",writeFD,readFD);
		exit(-3);
	}

	/*for(i = 0; i < len; i++) {
		printf("%02x", buffer[len]);
	}*/
	int count = write(writeFD,buffer,len);
	//std::cout << "sending data to the crumb...............  " << count << std::endl;
	//fwrite(buffer, 1, len, serialPortWrite);
	//fflush(serialPortWrite);
	//printf("  //ending write\n");
}

