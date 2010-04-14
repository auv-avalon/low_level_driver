#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include "lowlevel_processor.h"
#include <arpa/inet.h>
#include <math.h>
#include <iomanip>
#include <string.h>


LowLevelProcessor::LowLevelProcessor():
  IODriver(MAX_PACKET_SIZE,false)
{
	depthValue=0;
#if 0
	for(int i=0;i<TemperatureReading::TEMP_MAX;i++){
		oldtemprature[i]=0;
		temprature[i] = 1;
	}

	cpuTempFile = fopen("/proc/acpi/thermal_zone/THRM/temperature","r");
	if(cpuTempFile == 0){
		std::cerr << "Konnte CPU Port nicht öffnen\n";
	}
#endif


}

LowLevelProcessor::~LowLevelProcessor(){
}

bool LowLevelProcessor::init(std::string const &port){
	bool v = openSerial(port,115200);
	if(v){
		calibrateDepth();
		return true;
	}
	return false;
}

int LowLevelProcessor::getReadFD() {
	return getFileDescriptor();
}

void LowLevelProcessor::reset(){
  static const int len=4;
  uint8_t buff[len];
  buff[0]='#';
  buff[1]=len;
  buff[2]=Reset;
  buff[3]='\n';
  writePacket(buff,len,200);
}

void LowLevelProcessor::setLaserOverride(bool v)
{
  static const int len=4;
  uint8_t buff[len];
  buff[0]='#';
  buff[1]=len;
  buff[2]=LaserOverride;
  buff[3]=v?1:0;
  buff[4]='\n';
  writePacket(buff,len,200);
}


void LowLevelProcessor::calibrateDepth(){
  static const int len=4;
  uint8_t buff[len];
  buff[0]='#';
  buff[1]=len;
  buff[2]=CalibrateDepth;
  buff[3]='\n';
  writePacket(buff,len,200);

}

void LowLevelProcessor::setLEDs(const uint8_t& value)
{
  static const int len=5;
  uint8_t buff[len];
  buff[0]='#';
  buff[1]=len;
  buff[2]=SetLEDValue;
  buff[3]=value;
  buff[4]='\n';
  writePacket(buff,len,200);
}


void LowLevelProcessor::setLongExposure(uint16_t value)
{
  static const int len=5;
  uint8_t buff[len];
  buff[0]='#';
  buff[1]=len;
  buff[2]=SetLongExposure;
  memcpy(&buff[3],&value,2);
  buff[5]='\n';
  writePacket(buff,len,200);

}

void LowLevelProcessor::setShortExposure(uint16_t value)
{
  static const int len=5;
  uint8_t buff[len];
  buff[0]='#';
  buff[1]=len;
  buff[2]=SetShortExposure;
  memcpy(&buff[3],&value,2);
  buff[5]='\n';
  printf("Writing packed: \n");
  for(int i=0;i<len;i++)printf("%i ",buff[i]);
  printf("\n");
  writePacket(buff,len,500);
}

void LowLevelProcessor::setServoValue(uint16_t value)
{
  static const int len=6;
  uint8_t buff[len];
  buff[0]='#';
  buff[1]=len;
  buff[2]=SetServoValue;
  memcpy(&buff[3],&value,2);
  buff[5]='\n';
  writePacket(buff,len,200);
}




bool LowLevelProcessor::getData(double &depth){
  int bufsize = MAX_PACKET_SIZE;
  uint8_t packed[bufsize];
  if(readPacket(packed,bufsize,10000,10000)){
    switch(packed[2]){
      case DepthValue:
      {
	int32_t value;
	memcpy(&value,packed+3,4);
	depth = value*DEPTHFACTOR;
	break;
      }
      case TemperatureValue:
      {
	int temp= (packed[3] | packed[4] << 8);
	break;
      }
      case SetLEDValue:
      {
	
	break;
      }
      case String:
      {
	char c[packed[1]-3];
	memcpy(c,&packed[3],packed[1]-4);
	c[packed[1]-4]=0;
	printf("Debug message: %s \n\n",c);
	
	break;
      }
      default:
      {
	fprintf(stderr,"Cannot Handle Packed from type: %i\n",packed[2]);
	return false;
      }
    }
    return true;
  }else{
    return false;
  }
}

int LowLevelProcessor::extractPacket(uint8_t const* buffer, size_t buffer_size) const {
	int skip = 0;
	while(skip < buffer_size && buffer[skip] != '#'){
		skip++; //Skipping data as far we don't detect an statring packed
	}
	if(skip > 0){
		return -skip; //Packed start is not at beginning
	}
	if(buffer_size<3){
		return 0; //Packed start is corret but not complete yet
	}
	if(buffer_size<buffer[1]){
	  return 0;
	}
	
	if(buffer[0] == '#' && buffer[buffer[1]-1] == '\n'){
		return buffer[1]; 
	}else{
		fprintf(stdout,"OOps?\n");
		return -1; //Unsure what we can skip, but the current one is not the starting one...
	}
}

#if 0
int LowLevelProcessor::do_temperatures(const sensors_chip_name* name) {
	std::string prf(name->prefix);
	if(prf == "coretemp") {
		double cur;
		sensors_get_feature(*name, SENSORS_CORETEMP_TEMP1, &cur);
		int16_t val = cur;
		int tempname = TemperatureReading::TEMP_CORE_0+name->addr;;
		temp->setTemperature((TemperatureReading::TemperatureSensors)tempname,val);
	} else if(prf == "w83627ehf") {
		double cur;
		TemperatureReading::TemperatureSensors temp_names[] = {TemperatureReading::TEMP_SYS, TemperatureReading::TEMP_CPU, TemperatureReading::TEMP_AUX};
		for(int i = 0; i < 3; i++) {
			sensors_get_feature(*name, SENSORS_W83627EHF_TEMP1+i, &cur);
			temp->setTemperature(temp_names[i], cur);
		}
	}
}

void LowLevelProcessor::do_voltages(const sensors_chip_name* name) {
	if(std::string(name->prefix) == "w83627ehf") {
		double cur;
		const char* temp_names[] = {"VCore", "in1", "AVCC", "3VCC", "in4", "in5", "in6", "VSB", "VBAT", "in9"};
		for(int i = 0; i < 10; i++) {
			sensors_get_feature(*name, SENSORS_W83627EHF_IN0+i, &cur);
			std::cout << Timer::getCurrentTime()/1000 << temp_names[i] << ": " << cur << std::endl;
		}
	}
}

void LowLevelProcessor::do_fans(const sensors_chip_name* name) {
	if(std::string(name->prefix) == "w83627ehf") {
		double cur;
		const char* temp_names[] = {"","CPUFAN"};
		for(int i = 1; i < 2; i++) {
			sensors_get_feature(*name, SENSORS_W83627EHF_FAN1+i, &cur);
			std::cout << Timer::getCurrentTime()/1000 << temp_names[i] << ": " << cur << std::endl;
		}
	}
}

void LowLevelProcessor::read_sensors() {
	int nr=0;
	const sensors_chip_name *name;
	do {
		name = sensors_get_detected_chips(&nr);
		if(name) {
			do_temperatures(name);
			do_voltages(name);
			do_fans(name);
		}
	} while(name);
}

void LowLevelProcessor::cleanup_lmsensors() {
	sensors_cleanup();
}

int LowLevelProcessor::initialize_lmsensors() {
	FILE* cfgfile = fopen("/etc/sensors.conf", "r");
	if(!cfgfile) {
		std::cerr << "Unable to open sensors.conf" << std::endl;
		return -1;
	}
	int e = sensors_init(cfgfile);
	if(e) {
		std::cerr << "Unable to initialize lm_sensors" << std::endl;
		return -2;
	}
	fclose(cfgfile);
}

#endif
