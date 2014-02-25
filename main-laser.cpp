/**
 * Author: Matthias Goldhoorn (matthias.goldhoorn@dfki.de)
 * Company: Deutsches Forschungszentrum für Künstliche Intelligenz - Robotics Innovation Center (DFKI RIC)
 * Year 2010
 * Desc:
 *
*/
#include <stdio.h>
#include "lowlevel_processor.h"
#include <stdlib.h>
#include <string.h>

int main(int argc, char* argv[]) {
  LowLevelProcessor llpc;

  if(argc == 3){
	uint8_t value = atoi(argv[2]);
	if(value > 0){
  		printf("Turning the laser on\n");
	}else{
		printf("Turning the laser off\n");
	
	}
  	llpc.init(std::string(argv[1]));
	llpc.setLaserRate(value);
  }else{
  	fprintf(stderr,"Please enter device name and the laser rate (0-100)\n");
	return -1;
  }
  return 0;
}
