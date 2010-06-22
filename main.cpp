#include <stdio.h>
#include "lowlevel_processor.h"
#include <stdlib.h>
 
int main(int argc, char* argv[]) {
  LowLevelProcessor llpc;
  llpc.init(std::string(argv[1]));
  
  if(argc == 5){
      llpc.setShortExposure(atoi(argv[2]));
      llpc.setLongExposure(atoi(argv[3]));
    if(argv[4][0] == '1'){
      llpc.setLaserOverride(true);
    }else{
      llpc.setLaserOverride(false);
    }
  }
//  else
  {
  
  
    while(1){
      double depth;
      uint8_t leds=0;
      try {
	if(llpc.getData(depth)){
	  fprintf(stdout,"Depth: %f\r",depth);
	}
	//llpc.setLEDs(leds++);
      }catch(timeout_error t) {
	printf("Timeout\n");
  ;

      }

    }
  }
}
