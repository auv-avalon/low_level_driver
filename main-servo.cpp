#include <stdio.h>
#include "lowlevel_processor.h"
#include <stdlib.h>
#include <string.h>

int main(int argc, char* argv[]) {
  LowLevelProcessor llpc;
  uint16_t value = 0;
  if(argc == 3){
	value = atoi(argv[2]);
  	llpc.init(std::string(argv[1]));
	llpc.setServoValue(value);
  }else{
  	fprintf(stderr,"Please enter device name and PWM Value\n");
	return -1;
  }
    while(1){
      uint8_t leds=0;
      base::Time last_depth_time;
      try {
	llpc.getData();
	if (last_depth_time != llpc.depthTime)
	{
	  fprintf(stdout,"Depth: %f\r",llpc.depthValue);
	}
	//llpc.setLEDs(leds++);
      }catch(timeout_error t) {
	printf("Timeout\n");
      }
  }
  return 0;
}
