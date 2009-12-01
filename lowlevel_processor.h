#ifndef LOWLEVEL_PROCESSOR_H
#define LOWLEVEL_PROCESSOR_H

#include <string>
#include <iodrivers_base.hh>



#if 0
struct sensors_chip_name;
#endif

class LowLevelProcessor : public IODriver {

public:
	enum MessageType{
	  DepthValue=0,
	  TemperatureValue,
	  LEDValue,
	  String,
	  LaserOverride
	};
	
	LowLevelProcessor();
	~LowLevelProcessor();
	int getReadFD();
	bool init(std::string const &port);
	bool getData(double &depth);
	void setLEDs(uint8_t const &value);
	void setLaserOverride(bool v);
	

  private:
	double depthValue;
	static const int MAX_PACKET_SIZE = 256;
	virtual int extractPacket(uint8_t const* buffer, size_t buffer_size) const;
#if 0
	FILE *cpuTempFile;
	
	
	void read_sensors(); 
	void cleanup_lmsensors();
	int initialize_lmsensors();
	void sendExposure();	
	
	int do_temperatures(const sensors_chip_name* name);
	void do_voltages(const sensors_chip_name* name);
	void do_fans(const sensors_chip_name* name);
#endif
    

	


};


#endif
