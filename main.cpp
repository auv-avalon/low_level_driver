#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <QCoreApplication>
#include <QApplication>
#include <DataSpace.h>
#include <dataitems/ThrusterControlCommand.h>
#include <ADRFApplication.h>
#include "lowlevel_processor.h"
#include <IPCCommandProcessor.h>
//#include "serial_protocol.h"

int main(int argc, char* argv[]) {

	unsigned char data_in[10] = {1,2,3,4,5,6,7,8,9,0};
	unsigned char *data_out;
	uint16_t length_out;
	bool laserOverride =false;
	bool laserOff =false;
	if(argc>=2){
		if(!strcmp(argv[1],"laser-on")){
			laserOverride=true;
			std::cout << "i Will start with laser overrive (so it's ever on)" << std::endl;
			sleep(1);
		}else if(!strcmp(argv[1],"laser-off")){
			laserOff=true;
			std::cout << "i Will start without laser (so it's ever off)" << std::endl;
			sleep(1);
		}
	}
    /*
	encode(34, 10, data_in, &length_out, &data_out);

	for(int i = 0; i < length_out; i++)
		printf("%02x", data_out[i]);
	printf("\n");
    */

	std::cout << argc;
	ADRFApplication app (argc, argv);
	QApplication* app2 = dynamic_cast<QApplication*>(app.getQApplication());

	LowLevelProcessor thruster(app.getDPManager(),laserOverride,laserOff);
	std::cout << argc;
	app.getDPManager()->addProcessor(&thruster);

	thruster.startSearching();
	app.start();
	std::cout << "entering main loop" << std::endl;
	return app2->exec();
}
