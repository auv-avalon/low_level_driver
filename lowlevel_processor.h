/**
 * Author: Matthias Goldhoorn (matthias.goldhoorn@dfki.de)
 * Company: Deutsches Forschungszentrum für Künstliche Intelligenz - Robotics Innovation Center (DFKI RIC)
 * Year 2010
 * Desc:
 *
*/
#ifndef LOWLEVEL_PROCESSOR_H
#define LOWLEVEL_PROCESSOR_H

#include <string>
#include <iodrivers_base.hh>
#include <base/time.h>

#define DEPTHFACTOR -0.00432



#if 0
struct sensors_chip_name;
#endif

class LowLevelProcessor : public IODriver {

public:
	enum MessageType{
	  DepthValue=0,
	  TemperatureValue,
	  SetLEDValue,
	  String,
	  LaserOverride,
	  SetLongExposure,
	  SetShortExposure,
	  SetWaitingTime,
	  SetServoValue,
	  Reset,
	  CalibrateDepth,
	  SetLaserRate,
	};
	
	LowLevelProcessor();
	~LowLevelProcessor();
	int getReadFD();
	bool init(std::string const &port);
	bool getData(bool reRequest=false);
	void setLEDs(uint8_t const &value);
	void setLaserOverride(bool v);
	void reset();
	void calibrateDepth();
	void setShortExposure(uint32_t value);
	void setLongExposure(uint32_t value);
	void setWaitingTime(uint32_t value);
	void setServoValue(uint16_t value);
	void setLaserRate(uint8_t value);
	double depthValue;
	base::Time depthTime;

  private:
	static const int MAX_PACKET_SIZE = 256;
	virtual int extractPacket(uint8_t const* buffer, size_t buffer_size) const;
	uint32_t targetShortExposure;
	uint32_t targetLongExposure;
	uint32_t targetWaitingTime;
	uint16_t targetServoValue;
	uint32_t crumbShortExposure;
	uint32_t crumbLongExposure;
	uint32_t crumbWaitingTime;
	uint16_t crumbServoValue;
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
