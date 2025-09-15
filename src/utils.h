#pragma once
#include "sx1281.h"

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define print_radio_status(void) \
    if (true) { \
        uint8_t status = sx1281_get_status(); \
        Serial.print(" Status: 0x"); \
        Serial.print(RADIO_STATE(status), HEX); \
        Serial.print(" 0x"); \
        Serial.println(CMD_STATUS(status), HEX); \
    }
