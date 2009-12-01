#include <string.h>
#include <fstream>
#include <iostream>
#include "avalon_protocol_callbacks.h"
#include "lowlevel_processor.h"
#include <timer.h>

static LowLevelProcessor* lowLevelProcessor;

void init_callbacks(LowLevelProcessor* processor) {
	lowLevelProcessor = processor;
}


void trigger_state_callback(uint8_t type, uint16_t length, const uint8_t *payload) {
	if(length==sizeof(uint8_t)) {
		lowLevelProcessor->setCrumbTriggerState(*(uint8_t*)payload);
	}
}

void depth_reading_callback(uint8_t type, uint16_t length, const uint8_t *payload) {
	if(length==sizeof(int32_t)) {
		static std::ofstream output("depth_log.txt");
		static Timer timer;
		//std::cout << "Tiefe: " << (*(int32_t*)payload) << std::endl;
		output << timer.getTime() << " " << (*(int32_t*)payload) << std::endl;
		lowLevelProcessor->newDepthReading(*(int32_t*)payload);
	}
}

void debug_message_callback(uint8_t type, uint16_t length, const uint8_t *payload) {
	char *buf;
	buf = new char[length+1];
	memcpy(buf, payload, length);
	buf[length] = 0;
	std::cout << Timer::getCurrentTime() << " Debug-Message: " << buf << std::endl;

	delete [] buf;
}

void depth_offset_callback(uint8_t type, uint16_t length, const uint8_t *payload){
	if(length==sizeof(int32_t)) {
		lowLevelProcessor->processDepthOffset(*(int32_t*)payload);
	}else{
		std::cerr << "Got wrong packed size vor depth" << std::endl;
	}

}


void crumb_reset_callback(uint8_t type, uint16_t length, const uint8_t *payload){
	lowLevelProcessor->startResetTimer();
}

