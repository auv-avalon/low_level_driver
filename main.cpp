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
  
    printf("trzzing\n"); 
    while(1){
      uint8_t leds=0;
      base::Time last_depth_time;
      try {
	llpc.getData();
	//if (last_depth_time != llpc.depthTime)
	//{
	  fprintf(stdout,"Depth: %f\r",llpc.depthValue);
	//}
	//llpc.setLEDs(leds++);
      }catch(timeout_error t) {
	printf("Timeout\n");
      }

    }
  }
}
