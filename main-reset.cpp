#include <stdio.h>
#include "lowlevel_processor.h"
#include <stdlib.h>
 
int main(int argc, char* argv[]) {
  LowLevelProcessor llpc;
  
  if(argc == 2){
  	llpc.init(std::string(argv[1]));
	llpc.reset();
  }else{
  	fprintf(stderr,"Please enter device name\n");
	return -1;
  }
  return 0;
}
