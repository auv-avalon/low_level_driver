#ifndef LOWLEVEL_PROCESSOR_H
#define LOWLEVEL_PROCESSOR_H


#define RESYNC_INTERVAL 50
#define CRUMB_RESEND_INTERVAL 1000
//#define IMUPORT "/dev/ttyUSB0" 
//#define CRUMBPORT "/dev/null"
#define DEFAULT_LONG_EXPOSURE 6000
#define DEFAULT_SHORT_EXPOSURE 500
#define DEPTHOFFSET 0
//#define DEPTHFACTOR -0.004577776421399578845
#define DEPTHFACTOR -0.00228881835938
#define IMUPORT "/dev/ttyUSB2"
#define CRUMBPORT "/dev/ttyUSB0"

#include <EventDataProcessor.h>
#include <TypeDefinitions.h>
#include <SearchItem.h>
#include <DataSpace.h>
#include <dataitems/ThrusterControlCommand.h>
#include <dataitems/TemperatureReading.h>
#include <dataitems/DepthReading.h>
#include <dataitems/BatteryCharge.h>
#include <dataitems/LaserControl.h>
#include <dataitems/DebugMessage.h>
#include <dataitems/WaterReading.h>
#include <dataitems/Calibration.h>
#include <dataitems/IMUReading.h>
#include <dataitems/ExtTriggerRequest.h>
#include <DataSpace.h>
#include <DPManager.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <QMutex>
#include <QTimer>
#include <sys/time.h>
#include <Calibration.h>
#include <StateEstimator.h>
#include "body.h"
#include <Configuration.h>
#include <StaticConfiguration.h>
#include <termios.h>
#include <timer.h>
#include <unistd.h>
#include "../../../../../carlton-imu-code/protocol/protocol.h"
#include <fstream>
//pre-declaration of low level processor class
class LowLevelProcessor; 
class XsensThread;
#include "XsensThread.h"
struct sensors_chip_name;

class CrumbSerialThread : public QThread {
	Q_OBJECT
public:
	CrumbSerialThread(LowLevelProcessor *parent);
	virtual void run();
private:
	LowLevelProcessor *parent;
};

class IMUSerialThread : public QThread {
	Q_OBJECT
public:
	IMUSerialThread(FILE*, LowLevelProcessor*);
	virtual void run();
private:
	FILE *serialPort,calibProc;
	QMutex mutex;
    LowLevelProcessor *parent;
};


class LowLevelProcessor: public EventDataProcessor, public Protocol::Serial {
	Q_OBJECT
	friend class IMUSerialThread;
	friend class CrumbSerialThread;


public:
	LowLevelProcessor(DPManager *parent,bool laserOverride,bool laserOff);
	~LowLevelProcessor();
        int send(void *buf, size_t count);
	int recv(void *buf, size_t count);
	void startSearching();
	void searchResponse(SearchItem *item, QList<DataItem*> *itemList);
	//void gotNewCrumbData(unsigned char*);
	void newDepthReading(int32_t depth);
	void setCrumbTriggerState(uint8_t);
	void laser(bool onoff);
	void sendServo();
	void gotNewIMUDataXsens(double acc[3], double gyro[3], double mag[3], double quat[4], double euler[3]);
	void setTrigger(bool);
	void processDepthOffset(int32_t);
	void startResetTimer();
private:
	bool laserOverride,laserOff;
	std::ofstream logfile;
	XsensThread *xsensThread;
	void read_sensors(); 
	void cleanup_lmsensors();
	Protocol::CANSerial::CANSerial *protokoll;
	int initialize_lmsensors();
	void sendExposure();	
	FILE *output;
	uint16_t servoPosition;
	int do_temperatures(const sensors_chip_name* name);
	void do_voltages(const sensors_chip_name* name);
	void do_fans(const sensors_chip_name* name);
	bool laserOn;
	bool crumbTriggerOn;
	void initIMU();
    int16_t oldtemprature[TemperatureReading::TEMP_MAX];
	int16_t temprature[TemperatureReading::TEMP_MAX];
	CrumbSerialThread *serialreader;
	void nsleep(int nanoSecounds);
	DataSpace *dataSpace;
	int serialPortW;
	int serialPortR;
	FILE *cpuTempFile;
	bool laserPulsed;
	TemperatureReading *temp;
	BatteryCharge *batt;
	DepthReading *depth;
	DataItems::Calibration *calibrationItem;
	DebugMessage *debug;
	WaterReading *water;
	QTimer *timerLaser;
	void processLaserControl(LaserControl* control);
	void submitDebug(QString);
	//Timer askTime;
	QMutex mutexw;
	QMutex mutexr;
	QTimer resyncTimer;
	QTimer timer;
	QTimer crumbResendTimer;
	uint16_t shortExposure;
	uint16_t longExposure;
	uint32_t syncFrameID;
	/*IMU Stuff */
	FILE *imuSerialPort;
	IMUReading *imuReading;
	IMUSerialThread *imuSerialReader;
	void gotNewIMUData(int16_t*,int16_t*,int16_t*);
	int precounter;
	//int16_t convertData(int,unsigned char*);
	Eigen::Matrix<xymreal,3,1>  a_rmeas, omega_rmeas, m_rmeas;
	void updateIMUCalibration(DataItems::Calibration *c);
	ExtTriggerRequest *extTrigger;
	/*General Calibration Stuff */
	Configuration *config;
	bool stateEstimatorInitialised;
    StateEstimator* stateEstimator;
    Calibration *calibration;
	//Hack time call from searchResponse dosn't work
	bool doResync;
	int32_t depthOffset;
	double depthValue;
	void getCrumbDepth();
	QTimer resyncFunctionTimer;
	QTimer triggerOffTimer;
	QTimer triggerOnTimer;
	QTimer resetTimer;
	bool newDepthOffsetMarker;
	Timer askTime; //Use it only for  getting time
public slots:
	void laserPWM();
	void reader();
	void resyncProsilica();
	void processCrumbReset();
	void resendCrumb();
	void setTriggerOff();
	void setTriggerOn();
};


#endif
