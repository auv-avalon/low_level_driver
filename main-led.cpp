#include <stdio.h>
#include "lowlevel_processor.h"
#include <stdlib.h>
#include <string.h>

int main(int argc, char* argv[]) {
  LowLevelProcessor llpc;
  uint8_t on = 0;
  if(argc == 3){
	if(strcmp(argv[2],"on") == 0){
  		on = 255;
		printf("Turning the light on\n");
	}else{
		printf("Turning the light off\n");
	
	}
  	llpc.init(std::string(argv[1]));
	llpc.setLEDs(on);
  }else{
  	fprintf(stderr,"Please enter device name and on or nothing for off\n");
	return -1;
  }
  return 0;
}
