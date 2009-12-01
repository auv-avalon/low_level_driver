#ifndef AVALON_PROTOCOL_CALLBACKS
#define AVALON_PROTOCOL_CALLBACKS

#include "../../../firmware/crumb-front/avalon_protocol.h"

void depth_reading_callback(uint8_t type, uint16_t length, const uint8_t *payload);
void debug_message_callback(uint8_t type, uint16_t length, const uint8_t *payload);

void trigger_state_callback(uint8_t type, uint16_t length, const uint8_t *payload);
void depth_offset_callback(uint8_t type, uint16_t length, const uint8_t *payload);
void crumb_reset_callback(uint8_t type, uint16_t length, const uint8_t *payload);

class LowLevelProcessor;
void init_callbacks(LowLevelProcessor* processor);

const ReceiveCallback avalon_protocol_callbacks[] = {
	depth_reading_callback,
	debug_message_callback,
	trigger_state_callback,
	depth_offset_callback,
	crumb_reset_callback
};

#endif
