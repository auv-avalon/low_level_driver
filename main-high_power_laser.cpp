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
	if(value !=  666){
  		printf("You don't know what you are doing do Away, only for fulltime employees not for students\n");
	        return -1;
	}else{
	    printf("Turning the laser on\n");
  	    llpc.init(std::string(argv[1]));
            while(true){
        	llpc.keepHighPowerLaserActive();
                usleep(5000);
            }
	}
  }else{
  	printf("You don't know what you are doing do Away, only for fulltime employees not for students\n");
	return -1;
  }
  return 0;
}
