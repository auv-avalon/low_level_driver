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
	uint16_t value = atoi(argv[2]);
	if(value !=  666){
  		printf("You don't know what you are doing do Away, only for fulltime employees not for students\n");
	        return -1;
	}else{
	    printf("Laser should be off\n");
  	    llpc.init(std::string(argv[1]));
            bool bla = true;
            while(bla){
                llpc.getData();
                /*
                for(int i=0;i<10;i++){
                    usleep(1000);
                    llpc.getData();
                }
                */
                printf("Activating laser\n");
                for(int i=0;i<100000000;i++){
                    llpc.getData();
        	    llpc.keepHighPowerLaserActive();
                    sleep(1);
                }
                /*
                printf("Deactivating laser\n");
        	llpc.deactivateHighPowerLaser();
                for(int i=0;i<10;i++){
                    llpc.getData();
                    usleep(1000);
                }
                printf("Activating laser\n");
                for(int i=0;i<10;i++){
                    llpc.getData();
        	    llpc.keepHighPowerLaserActive();
                    usleep(100);
                }
                printf("Waiting for laser Timeout");
                for(int i=0;i<20;i++){
                    llpc.getData();
                    usleep(1000);
                }
                
                bla=false;
//                printf("Start from beginning");
                */
            }
	}
  }else{
  	printf("You don't know what you are doing do Away, only for fulltime employees not for students\n");
	return -1;
  }
  return 0;
}
