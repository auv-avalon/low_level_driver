#include <stdio.h>
#include <sensors/sensors.h>
#include <sensors/chips.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include "lowlevel_processor.h"
#include <arpa/inet.h>
#include <math.h>
#include <Eigen/Geometry>
#include <dataitems/ExtTriggerRequest.h>
#include <dataitems/ProsilicaInfo.h>
#include "avalon_protocol_functions.h"
#include "avalon_protocol_callbacks.h"
#include "../../../firmware/crumb-front/avalon_protocol.h"
#include <iomanip>

//#include <valgrind/memcheck.h>
//#define SafeRAWDATA 1
//#include <valgrind/memcheck.h>

//#define IGNORE_IMU

#define PROTECTMOTCON 1
#define SERVO_OFFSET 900
#define DEFAULT_SERVO 50

//#define DEBUG
//

void avalonSendMessageSafe(uint8_t type, uint8_t length, uint8_t* data) {
	static QMutex send_mutex;
	QMutexLocker locker(&send_mutex);
	char buff[200];
	sprintf(buff,"Sending command: %i",type);
	
//	std::cout << buff << std::endl;

	avalonSendMessage(type, length, data);
}

LowLevelProcessor::LowLevelProcessor(DPManager *parent, bool laserOverride, bool laserOff) : EventDataProcessor(parent), 
	laserOverride(laserOverride),
	laserOff(laserOff),
	logfile("ImuDataItem.log")
{
	logfile << "#Imu Data Item Data:\n";
	logfile << std::setw(15) << "time" << " " 
			<< std::setw(10) << "depth" << " " 
			<< std::setw(10) << "orientation" << " " 
			<< std::setw(10) << "quat.x" << " " 
			<< std::setw(10) << "quat.y" << " " 
			<< std::setw(10) << "quat.z" << " " 
			<< std::setw(10) << "quat.w" << " " 
			<< std::setw(10) << "gyro.x" << " " 
			<< std::setw(10) << "gyro.y" << " " 
			<< std::setw(10) << "gyro.z" << " " 
			<< std::setw(10) << "acc.x" << " " 
			<< std::setw(10) << "acc.y" << " " 
			<< std::setw(10) << "acc.z" << " " 
			<< std::setw(10) << "mag.x" << " " 
			<< std::setw(10) << "mag.y" << " " 
			<< std::setw(10) << "mag.z" << "\n"; 
	protokoll = new Protocol::CANSerial::CANSerial(this);
//	calibProc = popen("/usr/local/imu_calib","w");
	dataSpace = parent->getDataSpace(dstPercept);
	depthValue=0;
	depthOffset=0;
	precounter=10;
	resetTimer.setInterval(10);
	resetTimer.setSingleShot(false);
	triggerOnTimer.setInterval(10);
	triggerOnTimer.setSingleShot(false);
	triggerOffTimer.setInterval(10);
	triggerOffTimer.setSingleShot(false);
	resyncFunctionTimer.setInterval(10);
	resyncFunctionTimer.setSingleShot(false);
	temp = new TemperatureReading();
    temp->acquireItemUsage();
    batt = new BatteryCharge();
    batt->acquireItemUsage();
    depth = new DepthReading();
    depth->acquireItemUsage();
	laserOn=false;
    water = new WaterReading();
    water->acquireItemUsage();
    timerLaser = new QTimer();
    timerLaser->setInterval(100);
    timerLaser->setSingleShot(false);

	resyncTimer.setInterval(RESYNC_INTERVAL);
	resyncTimer.setSingleShot(false);
	resyncTimer.start();

   	extTrigger = new ExtTriggerRequest();
	extTrigger->acquireItemUsage();
	//timerLaser->start();
    //connect(timerLaser,SIGNAL(timeout()),this,SLOT(laserPWM()));
    connect(&resyncTimer,SIGNAL(timeout()),this,SLOT(resyncProsilica()));
	connect(&resetTimer,SIGNAL(timeout()),this,SLOT(processCrumbReset()));
	
	connect(&resyncFunctionTimer,SIGNAL(timeout()),this,SLOT(setTriggerOn()));
	
	

	crumbResendTimer.setInterval(CRUMB_RESEND_INTERVAL);
	crumbResendTimer.setSingleShot(false);
	crumbResendTimer.start();
    connect(&crumbResendTimer,SIGNAL(timeout()),this,SLOT(resendCrumb()));


	for(int i=0;i<TemperatureReading::TEMP_MAX;i++){
		oldtemprature[i]=0;
		temprature[i] = 1;
	}
	debug = new DebugMessage();
	debug->acquireItemUsage();
	std::cout << Timer::getCurrentTime()/1000 << " Oeffne nun Device\n";
	servoPosition=DEFAULT_SERVO;
	serialPortW = open(CRUMBPORT,O_WRONLY | O_SYNC | O_NOCTTY);
	//serialPortW = 2;//open("test.out",O_WRONLY | O_SYNC);

	serialPortR = open(CRUMBPORT,O_RDONLY | O_SYNC | O_NOCTTY);
	struct termios term;
	tcgetattr(serialPortR, &term);
	cfsetispeed(&term, B19200);
	cfsetospeed(&term, B19200);
	
	//cfsetispeed(&term, B115200);
	//cfsetospeed(&term, B115200);
	term.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	term.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
	term.c_iflag = IGNPAR;
	term.c_oflag = 0;
	tcsetattr(serialPortR, TCSANOW, &term);
	tcsetattr(serialPortW, TCSANOW, &term);
	
	#if 0
	cpuTempFile = fopen("/proc/acpi/thermal_zone/THRM/temperature","r");
	if(cpuTempFile == 0){
		std::cerr << "Konnte CPU Port nicht öffnen\n";
	}
	#endif
	if(serialPortW < 0 || serialPortR < 0){
		std::cerr << "Konnte Seriellen Port nicht oeffnen";
		submitDebug(QString("Unable to open Crumb SerialPort"));
		exit(-2);
	}
	init_avalon_protocol_functions(serialPortW, serialPortR);
	init_callbacks(this);
	avalonInit(avalon_protocol_callbacks, sizeof(avalon_protocol_callbacks)/sizeof(ReceiveCallback*));
	std::cout << Timer::getCurrentTime()/1000 << " Crumb device offen\n";
	submitDebug(QString("LowLevel Elctronic Interface Started"));

	//Initialize SerialReding for LowLevel Sensors and Motcon Response also all needed DataItems
//	serialReader = new SerialThread(serialPortR,this);
	//connect(serialReader,SIGNAL(newData(char*)),this,SLOT(gotNewData(char*)),Qt::DirectConnection);

	shortExposure = DEFAULT_SHORT_EXPOSURE;
	longExposure = DEFAULT_LONG_EXPOSURE;
	sendExposure();
	serialreader = new CrumbSerialThread(this);
	serialreader->start();
	
	uint8_t	sendValue=1;
	setTrigger(true);

#ifdef DFKI_IMU
	initIMU();

	std::cout << Timer::getCurrentTime()/1000 << " DFKI-IMU initialized\n";
#else
	imuReading = new IMUReading();
	imuReading->acquireItemUsage();
	xsensThread = new XsensThread(this);
	xsensThread->start();
	std::cout << Timer::getCurrentTime()/1000 << "XSens-IMU initialized\n";
#endif

	getCrumbDepth();
}

void LowLevelProcessor::newDepthReading(int32_t depth) {
	depthValue = 0.8 * depthValue +
		0.2 * (static_cast<float>(static_cast<int16_t>(depth&0xFFFFL)) * DEPTHFACTOR);

	//calibration->setDepth(depthCache);
	//std::cout << "got Depth " << depthValue << std::endl;
}


void LowLevelProcessor::startResetTimer(){
	newDepthOffsetMarker=false;
	resetTimer.start();
}


void LowLevelProcessor::initIMU(){
    config = new Configuration(staticConfiguration);
	calibration = new Calibration(staticConfiguration);
	int16_t prevalue=400;
#ifdef NSTDLIB
	stateEstimator = new StateEstimator(*config,*calibration);
#else
	stateEstimator = new StateEstimator(*config,*calibration,std::cout);
#endif
	calibration->setDepth(prevalue);


    stateEstimatorInitialised=false;
	imuReading = new IMUReading();
	imuReading->acquireItemUsage();

	debug =  new DebugMessage();
	debug->acquireItemUsage();

#ifdef DEBUG
    imuSerialPort = fopen("IMU_STILL.RAW","r");
#else
	int serialFD = open(IMUPORT, O_RDONLY | O_NOCTTY);
	//int serialFD = fileno(serialPort);
	struct termios term;
	tcgetattr(serialFD, &term);

	cfsetispeed(&term, B115200);
	cfsetospeed(&term, B115200);

	term.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	term.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
	term.c_iflag = IGNPAR;
	term.c_oflag = 0;


	tcsetattr(serialFD, TCSANOW, &term);
	imuSerialPort = fdopen(serialFD, "r");

#endif //Debug
#ifndef IGNORE_IMU
	if(imuSerialPort == 0){
		perror("SerialPort(IMU)");
		std::cerr << "Konnte Seriellen Port nicht oeffnen";
		submitDebug(QString("Unable to open SerialPort for IMU"));
		exit(-2);
	}
	submitDebug(QString("IMU Interface Handler started"));

	//Initialize SerialReding for IMU SerialInterface
	imuSerialReader = new IMUSerialThread(imuSerialPort, this);
	imuSerialReader->start();
#endif

}


int LowLevelProcessor::send(void *buf, size_t count){
	int fd = fileno(imuSerialPort);
	return write(fd,buf,count);
}

int LowLevelProcessor::recv(void *buf, size_t count){
	int fd = fileno(imuSerialPort);
	size_t ret = read(fd,buf,count);
	return ret;
}

void LowLevelProcessor::laserPWM(){
	char buffer[1];
	buffer[0]='L';
	if(laserPulsed){
		if(!laserOn){
			std::cerr << "Laser an\n";
			laserOn=true;
		}else{
			std::cerr << "Laser aus\n";
			buffer[0]='K';
			laserOn=false;
		}
	}
	//mutexr.lock();
	//mutexw.lock();
//	write(serialPortW,&buffer,1);
//	write(serialPortW,"\n",1);
	//mutexw.unlock();
	//mutexr.unlock();
}

void LowLevelProcessor::laser(bool value){
	//mutexw.lock();
	
	char buffer[1];
	if(value){
		std::cerr << "Laser ein\n";
		buffer[0]='L';
		//write(serialPortW,&buffer,1);
		timerLaser->start();
	}else{
		std::cerr << "Laser aus\n";
		timerLaser->stop();
		buffer[0]='K';
	}
	//mutexw.unlock();
//	write(serialPortW,&buffer,1);
}

void LowLevelProcessor::sendServo(){
	mutexw.lock();
	uint16_t value = servoPosition + SERVO_OFFSET;
	avalonSendMessageSafe(0, 2, (uint8_t*)(&value));
	mutexw.unlock();
}

LowLevelProcessor::~LowLevelProcessor() {
	delete temp;
	delete timerLaser;
	close(serialPortW);
	close(serialPortR);
	delete protokoll;
	if(xsensThread)
	{
		xsensThread->exit();
		while(xsensThread->isRunning()) ;
		delete xsensThread;
	}
}

void LowLevelProcessor::resyncProsilica(){
	if(doResync){
		doResync=false;
		//uint8_t sendValue=0;
		//std::cout << "Starting resyncronisation" << std::endl;
		setTrigger(false);	
		//avalonSendMessageSafe(4, 1, &sendValue); //Stop triggering
		uint32_t oldID = syncFrameID;
		//Wird nebenläufig vom ADRF geändert darum warten ma hier
		extTrigger->setResync(true);
		//usleep(500000UL);
		dataSpace->addDataItem(extTrigger);
		//std::cout << "Resync: Waiting for new SyncFrame ID" << std::endl;
		int precounter=50;
		while(oldID == syncFrameID && precounter--){
			usleep(500000);
			precounter--;
		//	std::cout << "ICh warte..." << std::endl;
		}
		if(precounter<0)
			std::cerr << Timer::getCurrentTime()/1000 << "Konnte nicht resyncen, probleme mit frmaeid" << std::endl;
		//sendValue=1;
		//std::cout << "schalte laser an" << std::endl;
		setTrigger(true);	
		//avalonSendMessageSafe(4, 1, &sendValue); //Start Triggering
		std::cout << Timer::getCurrentTime()/1000 << "resync finished"<< std::endl;
		//resyncTimer.start();
	}else{
		//std::cout << "Do Resync = flase" << std::endl;
	}
}

/*
void LowLevelProcessor::sendSyncFrameItem(){
}
*/

void LowLevelProcessor::startSearching() {
	//dataSpace->searchForDataType(this, ditThrusterControlCommand, rtCallback);
	dataSpace->searchForDataType(this, ditLaserControl  ,rtCallback);
	dataSpace->searchForDataType(this, ditCalibration   ,rtCallback);
	dataSpace->searchForDataType(this, ditExtTriggerRequest   ,rtCallback);
	dataSpace->searchForDataType(this, ditProsilicaInfo   ,rtCallback);
}

void LowLevelProcessor::sendExposure(){
	uint16_t value = shortExposure; 
	avalonSendMessageSafe(2, 2, (uint8_t*)(&value));
	avalonSendMessageSafe(2, 2, (uint8_t*)(&value));
	uint16_t longValue = longExposure;
	avalonSendMessageSafe(1, 2, (uint8_t*)(&longValue));
	avalonSendMessageSafe(1, 2, (uint8_t*)(&longValue));
//	std::cout << "Exposure values are: " << value << "|"<< longValue << std::endl; 
}

void LowLevelProcessor::searchResponse(SearchItem *item, QList<DataItem*> *itemList) {
	switch(item->getRequestItemTypeID()) {
	case ditLaserControl:
		{
			LaserControl* lc;
			lc = (LaserControl*)(itemList->at(0));
			processLaserControl(lc);
			break;
		}
    case ditCalibration:
		{
			DataItems::Calibration * c;
			c = (DataItems::Calibration*)(itemList->at(0));
            updateIMUCalibration(c);
			break;
		}
	case ditExtTriggerRequest:
		{
			ExtTriggerRequest *t;
			t = (ExtTriggerRequest*)(itemList->at(0));
			if(t->isShortTimeSet())		shortExposure = t->getShortTime();
			if(t->isLongTimeSet()){
				if((t->getLongTime()+10) > shortExposure)
					longExposure = t->getLongTime();
				else
					std::cerr << "Don't setting longExposure, because it's not higer than lowExposure +10" << std::endl;
			}
			if(t->isLongTimeSet() || t->isShortTimeSet()) 	
				sendExposure();
			if(t->isResyncRequest()){
				if((t->getTime() + 1000000UL) > askTime.getTime()){  
					//std::cout << "Kam ein Resync request an!---------" << std::endl;
					if(!doResync){
						doResync=true;
					//	std::cout << "starte resync timer" << std::endl;
					}else{
					//	std::cout << "Timer ist schon aktiv" << std::endl;
					}
				}else{
					std::cerr << "Got an too old dataitem so i don't do an resyncronistation!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << t->getTime() << "<" << askTime.getTime() << "diff: " << (askTime.getTime())- (t->getTime()) << std::endl;
				}
			}
			break;
		}
	case ditProsilicaInfo:
		{
			ProsilicaInfo *p;
			p = (ProsilicaInfo*)itemList->at(0);
			syncFrameID=p->getSyncFrameID();
			break;
		}
	default:
		{
			submitDebug(QString("---- Unbekanntes Item im Thruster? What?\n"));
			std::cerr << "---- Unbekanntes Item im Thruster? What?\n";
		}
	}
}

void LowLevelProcessor::processLaserControl(LaserControl* control) {
	std::cerr << "Laser-Winkel: " << control->getAngle() << std::endl;
	servoPosition= control->getAngle();
	sendServo();
	//laser(control->isEnabled());
	//laserPulsed = control->isPulsed();
	//control->releaseItemUsage();
}

/*
 * This Method is called if an new DataRecord Packed is comletly percept from the SerialPort Thread
 * It has to set new SensorValues for ADRF
 */
#if 0
void LowLevelProcessor::gotNewCrumbData(unsigned char* data) {

	int sc=0; //sensorCounter
#if 0
	double cpuTemp;
	if(cpuTempFile){
		char cpuBuff[100];
		int readCount = fread(cpuBuff,1,100,cpuTempFile);
		rewind(cpuTempFile);
		QString erg(cpuBuff);
	        erg = erg.left(readCount-3);
	        erg = erg.right(5);
		erg = erg.trimmed();
		cpuTemp = erg.toDouble();
	}else{
		submitDebug("CPU Datei nimmer da?!");
		std::cerr << "CPU Reader File missing"<< std::endl;
		cpuTemp =-5;
	}
#endif
	int16_t ads1112[3],adcValue[8];

	int16_t ads1110[5];


	for(int i=0;i<3;i++){
		ads1112[i]= (data[sc] | (data[sc+1]<<8));
		sc+=2;
	}
	for(int i=0;i<5;i++){
			ads1110[i]= data[sc] | (data[sc+1]<<8);
			sc+=2;
	}
	for(int i=0;i<8;i++){
		int names[] = { TemperatureReading::TEMP_BACK_1, TemperatureReading::TEMP_BACK_2, TemperatureReading::TEMP_BATTERY, TemperatureReading::TEMP_FRONT_1, TemperatureReading::TEMP_FRONT_2, TemperatureReading::TEMP_MOTCON, TemperatureReading::TEMP_WATER };

		//temprature[names[i]] = data[sc] | (data[sc+1]<<8);
		sc+=2;
	}
	for(int i=0;i<8;i++){
			adcValue[i] = (data[sc] | (data[sc+1]<<8));
			sc+=2;
	}


    if(water->getSensor(WaterReading::WaterFront) != adcValue[0]/1.0){
        water->setSensor(WaterReading::WaterFront,adcValue[0]/1.0);
        dataSpace->addDataItem(water);
    }
	//printf("Water Sensor: %f\n", adcValue[0]/1.0);

    
	if(depth->getDepth() != ads1112[0] / 1.0){
        depth->setDepth( ads1112[0] / 1.0);
        dataSpace->addDataItem(depth);
		calibration->setDepth(ads1112[0]);
//		std::cerr << "Depth: " << ads1112[0] << std::endl;
    }


//	std::cout << voltage /1200.0 << std::endl;
//	std::cout << voltage2 << std::endl;
//	std::cout << vdepth << std::endl<< std::endl;

    bool batChanged = false;

    if(batt->getSensor(BatteryCharge::Voltage) != (ads1112[1] / 1197.0) ){
        batt->setSensor(BatteryCharge::Voltage, ads1112[1] / 1197.0);
        batChanged = true;
    }
	//printf("Voltage: %f\n", ads1112[1]/1197.0);

	//batt->setSensor(BatteryCharge::VoltageBack, -1);


	double current = floor((ads1110[4]-16390) / 4.29);
	if(abs(current) > 10 && batt->getSensor(BatteryCharge::Current) != current){
	     batt->setSensor(BatteryCharge::Current,current);
	     batChanged = true;
	}

	if(batt->getSensor(BatteryCharge::CurrentBack) != ads1110[1] / 1.0){
        batt->setSensor(BatteryCharge::CurrentBack, 	ads1110[1] / 1.0);
        batChanged = true;
	}

    if(batt->getSensor(BatteryCharge::CurrentFront) != ads1110[2] / 1.0){
        batt->setSensor(BatteryCharge::CurrentFront, 	ads1110[2] / 1.0);
        batChanged = true;
    }

    if(batt->getSensor(BatteryCharge::CurrentMotCon) != ads1110[3] / 1.0){
        batt->setSensor(BatteryCharge::CurrentMotCon, 	ads1110[3] / 1.0);
        batChanged = true;
    }

	current =  floor((ads1110[3]-3290) / 8.54);

	if(abs(current) > 10 && batt->getSensor(BatteryCharge::CurrentPC) != current){
        batt->setSensor(BatteryCharge::CurrentPC,current);
        batChanged = true;
	}

	if(batChanged){
	    dataSpace->addDataItem(batt);
	}

	read_sensors();

    bool tempChanged=false;
    for(int i=0;i<TemperatureReading::TEMP_MAX;i++){
        if(oldtemprature[i] != temprature[i]){
             tempChanged = true;
             oldtemprature[i] = temprature[i];
        }
    }

    if(tempChanged){
        temp->setTemperature(TemperatureReading::TEMP_BACK_1,temprature[1]/1.0);
        temp->setTemperature(TemperatureReading::TEMP_BACK_2,temprature[2]/1.0);
        temp->setTemperature(TemperatureReading::TEMP_BATTERY,temprature[3]/1.0);
        temp->setTemperature(TemperatureReading::TEMP_FRONT_1,temprature[4]/1.0);
        temp->setTemperature(TemperatureReading::TEMP_FRONT_2,temprature[5]/1.0);
        temp->setTemperature(TemperatureReading::TEMP_MOTCON,temprature[6]/1.0);
        temp->setTemperature(TemperatureReading::TEMP_WATER,temprature[7]/1.0);
        dataSpace->addDataItem(temp);
    }


	//printf("Value: %f\n",temp->getTemperature(TemperatureReading::TEMP_BATTERY));
	//char buffer[200];
	//sprintf(buffer, "V1: %u, V2: %u, V3: %u\n",vdepth,voltage,voltage2);
	//std::cout << buffer;
	//submitDebug(buffer);

	//char motConResponse[500];
	//for(int i=sc,j=0; data[i-1] != 0; i++){
	//	motConResponse[j] = data[i];
	//}
	//QString s("Debug-> MotConResponse:");
	//s += motConResponse;
	//submitDebug(s);
}
#endif

/*
void LowLevelProcessor::sendToCalib(double v,bool newline){
	char buff[1000];
	int len = sprintf(buff,"%f ",v);
	fwrite(buff,len,1,calibProc);
}
*/

void LowLevelProcessor::gotNewIMUDataXsens(double acc[3], double gyro[3], double mag[3], double quat[4], double euler[3])
{
	// compute the XSens reading: change the frame of reference to match the AUV's
	// meaning X to the right, y facing down and z facing front. 
	
	double newQuat[4];
	double position[3];

	// do angle/radian converstion and quaternion calculation at the same time
	double rAngle = M_PI / 180.0 * euler[2];
	newQuat[0] = cos(rAngle * 0.5);
	newQuat[1] = 0.0;
	newQuat[2] = sin(rAngle * 0.5);
	newQuat[3] = 0.0;

	position[0]=0.0;
	position[1]=depthValue;
	position[2]=0.0;

	imuReading->setPosition(position);
    imuReading->setCalculatedQuaternion(newQuat);
    imuReading->setCalculatedGyro(gyro);
    imuReading->setCalculatedAccerleration(acc);
    imuReading->setCalculatedMagn(mag); 
	dataSpace->addDataItem(imuReading);
	logfile << std::setw(15) << Timer::getCurrentTime() / 1000 << " " 
			<< std::setw(10) << position[1] << " " 
			<< std::setw(10) << euler[2] << " " 
			<< std::setw(10) << newQuat[0] << " " 
			<< std::setw(10) << newQuat[1] << " " 
			<< std::setw(10) << newQuat[2] << " " 
			<< std::setw(10) << newQuat[3] << " " 
			<< std::setw(10) << gyro[0] << " " 
			<< std::setw(10) << gyro[1] << " " 
			<< std::setw(10) << gyro[2] << " " 
			<< std::setw(10) << acc[0] << " " 
			<< std::setw(10) << acc[1] << " " 
			<< std::setw(10) << acc[2] << " " 
			<< std::setw(10) << mag[0] << " " 
			<< std::setw(10) << mag[1] << " " 
			<< std::setw(10) << mag[2] << "\n"; 
}

void LowLevelProcessor::gotNewIMUData(int16_t *magn, int16_t *accel, int16_t *rotation){
	//int ac=0;  //arrayCounter
	int32_t quaternion[4];
	//int16_t rotation[3];
	int16_t temp[3];
	//int16_t accel[3];
	//int16_t magn[3];
	double speed[3];
	double position[3];
   	float calcQuat[4];

	
	for(int i=0;i<4;i++){
		quaternion[i] =0;
		//quaternion[i] = ntohl((data[ac]) + (data[ac+1] << 8)  + (data[ac+2] << 16) + (data[ac+3] << 24));
		//ac+=4;
	}

	//VALGRIND_CHECK_VALUE_IS_DEFINED(magn);
	//VALGRIND_CHECK_VALUE_IS_DEFINED(accel);
	//VALGRIND_CHECK_VALUE_IS_DEFINED(rotation);

	imuReading->setQuaternion(quaternion);

	/*
	for(int i=0;i<3;i++){
		rotation[i] = (data[ac] << 8) | (data[ac+1]); //ntohs((data[ac]) | (data[ac+1] << 8));
		ac+=2;
	}
	*/
	imuReading->setRotation(rotation);

	/*
	for(int i=0;i<3;i++){
		temp[i] = (data[ac] << 8) | (data[ac+1] );
		ac+=2;
	}
	*/
	imuReading->setTemp(temp);

	/*
	for(int i=0;i<3;i++){
		accel[i] = (data[ac])<<8 | (data[ac+1] );
		ac+=2;
	}
	*/
	imuReading->setAcceleration(accel);

	/*
	for(int i=0;i<3;i++){
		magn[i] = (data[ac] <<8) | (data[ac+1] );
		ac+=2;
	}
	*/
	imuReading->setMagnetField(magn);

	char meldung[800];
	sprintf(meldung,"Quaternion: %i,%i,%i,%i",
			quaternion[0],quaternion[1],quaternion[2],quaternion[3]);

	sprintf(meldung,"Rot: %i,%i,%i\t\tAcc: %i,%i,%i\t\tMagn: %i,%i,%i",
			rotation[0],rotation[1],rotation[2],
			accel[0],accel[1],accel[2],
			magn[0],magn[1],magn[2]
			                     );
	//std::cout << meldung<<std::endl;
	//submitDebug(QString(meldung));
    	//Begin off-IMU calculation

    for(int i=0;i<3;i++) {
		omega_rmeas(i) = rotation[i];
		a_rmeas(i) = accel[i];
		m_rmeas(i) = magn[i];
    }

    calibration->update_a_meas(a_rmeas);
    calibration->update_omega_meas(omega_rmeas);
    calibration->update_m_meas(m_rmeas);

   //if(precounter != 0){
   // 	precounter--;
   //}else{ 
    if( !stateEstimatorInitialised ) {
        	stateEstimator->initState();
	        stateEstimatorInitialised = true;
    } else {
		stateEstimator->step(18); // 18 ms steps
    }
	//}

    body b;
    b.pos[0]= 0.0;
    b.pos[1]= 1.2;
    b.pos[2]= 0.0;
    b.amr[0] = 0.0;
    b.amr[1] = 0.0;
    b.amr[2] = 0.0;
    b.amr[3] = 0.0;

    const StateVector x_k = stateEstimator->getState();

    b.qrot[0] = (float)x_k.q.w();
    b.qrot[1] = (float)x_k.q.x();
    b.qrot[2] = -(float)x_k.q.y();
    b.qrot[3] = (float)x_k.q.z();

    double gyroCalced[3],accCalced[3],magCalced[3];

    if(stateEstimatorInitialised){
		for(int i=0; i<3; i++){
		     gyroCalced[i] = stateEstimator->omega_meas[i];
		     accCalced[i] = stateEstimator->a_meas[i];
		     magCalced[i] = stateEstimator->z_m[i];
		}
    }

    for(int i=0;i<4;i++)
        calcQuat[i] = b.qrot[i];


    for(int i=0;i<3;i++)
        position[i] = x_k.xi[i];

    for(int i=0;i<3;i++)
        speed[i] = x_k.v[i];



	//sprintf(meldung,"Quaternion: %f,%f,%f,%f",
	//	calcQuat[0],calcQuat[1],calcQuat[2],calcQuat[3]);
	//std::cout << meldung << "acc: " << magCalced[0] << std::endl;

    imuReading->setSpeed(speed);
    imuReading->setPosition(position);

/*
	static xymreal corr_data[] = {1,0,0,0,0,1,0,-1,0};
	static Eigen::Matrix<xymreal, 3, 3> corr_mat(corr_data);
	static Eigen::Quaternion<xymreal> correction_quaternion(corr_mat);
	Eigen::Quaternion<xymreal> calc_quat(calcQuat[0],calcQuat[1],calcQuat[2],calcQuat[3]);
	Eigen::Quaternion<xymreal> corrected_quaternion = calc_quat * correction_quaternion;

	calcQuat[0] = corrected_quaternion.w();
	calcQuat[1] = corrected_quaternion.x();
	calcQuat[2] = corrected_quaternion.y();
	calcQuat[3] = corrected_quaternion.z();
*/
    imuReading->setCalculatedQuaternion(calcQuat);
    imuReading->setCalculatedGyro(gyroCalced);
    imuReading->setCalculatedAccerleration(accCalced);
    imuReading->setCalculatedMagn(magCalced);
	//std::cout << "Adde IMU item\n";
	dataSpace->addDataItem(imuReading);

	char calib[1000];
	if(false){
       for(int i=0;i<3;i++){
               double value = rotation[i];
			   std::cout << value << " ";
               //std::cerr << value << "|";
       }
       for(int i=0;i<3;i++){
               double value = accel[i];
               std::cout << value << " ";
       }
       for(int i=0;i<3;i++){
               double value = magn[i];
               std::cout << value << " ";
       }
       std::cout << std::endl;
       //std::cerr << std::endl;
	}
}

/*
int16_t* LowLevelProcessor::convertData(unsigned char *value){
	int16_t erg[3];
	for(int i=0;i<3;i++){
		erg[i] = (value[i]<<8) | value[i+1]; 
	}
	retunr erg;
}
*/

void LowLevelProcessor::reader(){
	avalonRunProtocol();
#if 0
	timeval starttime,endtime;
	gettimeofday(&starttime,NULL);
	if(serialPortR < 0 || serialPortW < 0) return;

	mutexr.lock();


	int pos=0;
	ioctl(serialPortR,TCFLSH);
	int bytes;
	ioctl(serialPortR,FIONREAD,&bytes);
	unsigned char	buffer[bytes],
					result[bytes];
	for(int i=0;i<bytes;i++){
		result[i]=0;
		buffer[i]=0;
	}

	read(serialPortR,buffer,bytes); //Buffer leeren
	ioctl(serialPortR,FIONREAD,&bytes);
	bool firstRun=true;
	while(bytes < 100){
		usleep(1000);
		ioctl(serialPortR,FIONREAD,&bytes);
	}

	if(bytes > 3000)
		std::cerr << "err";

	while(1) {
		int len = read(serialPortR,&buffer[pos++],1);\
		if(pos>=bytes-1){
			pos=0;
			std::cerr << "Achtung Puffer Überlauf abgefangen"<<std::endl;
			usleep(1000000);
		}
		if(pos > bytes){
			pos=0;
			continue;
		}
		if(pos > 0) if(buffer[pos-1] == '\n' && buffer[pos-2] == '\r'){
			for(int i=0;i<pos;i++)
				result[i]=buffer[i];
			pos=0;
			if(firstRun)
				firstRun=false;
			else{
				gotNewCrumbData(result);
				break;
			}
		}
	}
	mutexr.unlock();
	gettimeofday(&endtime,NULL);
	//int sec = endtime.tv_sec-starttime.tv_sec;
	//int usec = endtime.tv_usec-starttime.tv_usec;
	//std::cout << "Empfangs und verschickdauer" << sec << "." << usec << "\n";
#endif
}


void LowLevelProcessor::submitDebug(QString s) {
	debug->setMessage(s);
	dataSpace->addDataItem(debug);
}

void LowLevelProcessor::nsleep(int nanoSecounds){
	struct timespec time;
	time.tv_sec = 0;
	time.tv_nsec = nanoSecounds;
	while(nanosleep(&time,&time) ==-1){
		continue;
	}
}


CrumbSerialThread::CrumbSerialThread(LowLevelProcessor *parent){
	this->parent=parent;
}

void CrumbSerialThread::run(){
	while(1) {
		parent->reader();
		//usleep(10); //we use blokcin sys-calls so we
		//don't need to wait
	}
}

IMUSerialThread::IMUSerialThread(FILE* serialPort, LowLevelProcessor* p){
	this->serialPort = serialPort;
	parent = p;
}

void IMUSerialThread::run(){
	//unsigned char buffer[50];
	int16_t recBuff[9];
	int precounter=50;
	while(1){
		int id;
		size_t size;
		unsigned char buff[9];
		//std::cout << "IMU thread vorm lesen" << std::endl;
		int ret = parent->protokoll->recv(id,(void*)buff,size);
		//std::cout << "IMU thread nachm lesen" << std::endl;
		if(ret != 0){
		//printf("Return: %i\tID: 0x%X\tSize: %i\tBuff: %i\n",ret,id,size,buff[1]);
			if(id == 0x10){
				int offset=0;
				for(int i=0;i<3;i++){
					recBuff[i] = (buff[(i*2)+offset]<<8) | buff[(i*2)+1+offset];
				}
			}else if(id == 0x11){
				 int offset=4;
				 for(int i=0;i<2;i++){
                                        recBuff[i+3] = (buff[(i*2)+offset]<<8) | buff[(i*2)+1+offset];
                 }
			}else if(id == 0x12){
				int offset=0;
				for(int i=0;i<4;i++){
					recBuff[i+5] = (buff[(i*2)+offset]<<8) | buff[(i*2)+1+offset];
                }
				//printf("%i|%i|%i\n",recBuff[0],recBuff[1],recBuff[2]);

				//for(int i=0;i<3;i++)
				//	recBuff[i+6]=0;i
				/*
				for(int i=0;i<9;i++){
					int16_t value = recBuff[i];
					VALGRIND_CHECK_VALUE_IS_DEFINED(value);
				}
				*/
				int16_t leer[3] = {0,0,0};
//void LowLevelProcessor::gotNewIMUData(int16_t *magn, int16_t *accel, int16_t *rotation){
				if(precounter==0){
					parent->gotNewIMUData(&recBuff[6],&recBuff[3],&recBuff[0]);	
				}else{
					printf("Precounter: %i\n",precounter--);
				}
			}
		}
		usleep(10);
    }
}


void LowLevelProcessor::updateIMUCalibration(DataItems::Calibration *c){
	Eigen::Matrix<xymreal,3,3> newCalib;
	for(int i=0;i<9;i++){
        newCalib[i] = c->get_C_imu()[i];
	}
	calibration->set_C_imu(newCalib);
	//c->releaseItemUsage();
}


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

void LowLevelProcessor::resendCrumb(){
	sendServo();
	//getCrumbDepth();
	sendExposure();

	uint8_t value = laserOverride?1:0;
	value |= 2;
	value &= ~(laserOff?2:0);
	avalonSendMessageSafe(6,1,&value);
//	std::cout << "sending to crumb" << std::endl;
}

void LowLevelProcessor::setCrumbTriggerState(uint8_t v){
	/*
	if(v)
		std::cout << "got trigger information: an" << std::endl;
	else
		std::cout << "got trigger information: aus" << std::endl;
	*/
	if (crumbTriggerOn)
		std::cout << Timer::getCurrentTime()/1000 << "trigger is on" << std::endl;
	else
		std::cout << Timer::getCurrentTime()/1000 << "trigger is off" << std::endl;

	crumbTriggerOn = (bool)v;
}


void LowLevelProcessor::setTriggerOn()
{
	setTrigger(true);
}

void LowLevelProcessor::setTriggerOff()
{
	setTrigger(false);
}

void LowLevelProcessor::setTrigger(bool enabled){
	crumbTriggerOn = !enabled;

	uint8_t sendValue = 0;
	if(enabled)
		sendValue = 1;
	else
		sendValue = 0;

	int	maxTry = 500;
	while(!(crumbTriggerOn ^ enabled) && maxTry--){
		avalonSendMessageSafe(4, 1, &sendValue);
		avalonSendMessageSafe(8, 0, 0);
		usleep(50000);
	}
	if(maxTry <=0){
		std::cout << Timer::getCurrentTime()/1000 << "MaxTry is smaller than 0, so i can'T set trigger mode sucsessful" << std::endl;
	}
}

void LowLevelProcessor::processCrumbReset(){
	avalonSendMessageSafe(10, 4, (uint8_t*)&depthOffset);
	std::cout << Timer::getCurrentTime()/1000 << "Crumb wurde resettet, setzte depth-calib neu: " << depthOffset << std::endl;
	if(newDepthOffsetMarker){
		resetTimer.stop();
	}
}

void LowLevelProcessor::processDepthOffset(int32_t v){
	depthOffset = v;
	std::cout << Timer::getCurrentTime()/1000 << "got Depth-Offset: " << v << std::endl;
	newDepthOffsetMarker=true;
}



void LowLevelProcessor::getCrumbDepth(){
	
	int maxTry=15;
	//uint8_t value = 1;
	while(depthOffset == 0 && maxTry--){
		avalonSendMessageSafe(9, 0, 0);
		usleep(500000);
		//std::cout << "triing again" << std::endl;
	}
	if(maxTry<=0){
		std::cerr << Timer::getCurrentTime()/1000 << "Konnte tiefenoffset nicht abfragen" << std::endl;
	}
	
}
