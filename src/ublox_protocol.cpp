#include "ublox_protocol.h"
#include "circular_buffer.h"

CircularBuffer<uint8_t> ubx_buffer(UBX_BUFFER_SIZE);

void ublox_init() {
    UBX_SERIAL.begin(UBX_SERIAL_SPEED, SERIAL_8N1);
}

bool ublox_parse_msg(UBXNavPVT_t *nav_pvt){
    while (ubx_buffer.get_count() > sizeof(UBXHeader_t) + sizeof(UBXNavPVT_t)) {
        int ret = ublox_get_next_msg((uint8_t *)nav_pvt, sizeof(UBXNavPVT_t));
        if (ret == -1) {
            return false;
        }
        if(ret == sizeof(UBXNavPVT_t)){
            return true;
        }
    }
    return false;
}

void ublox_get_new_data() {
    int new_data = UBX_SERIAL.available();
    if (new_data > 0) {
        for (int i = 0; i < new_data; i++) {
            int ret = ubx_buffer.push(UBX_SERIAL.read());
            if (ret == -1) {
                break;
            }
        }
    }
}

int ublox_get_next_msg(uint8_t *buffer, int buf_len) {
    if (ubx_buffer.get_count() < sizeof(UBXHeader_t)) {
        return 0;
    }
    UBXHeader_t header;
    ubx_buffer.peek(&header.sync1, 0);
    ubx_buffer.peek(&header.sync2, 1);
    if (header.sync1 != UBX_SYNC1 || header.sync2 != UBX_SYNC2) {
        ubx_buffer.pop();
        return 0;
    }
    for (uint16_t i = 2; i < sizeof(UBXHeader_t); i++) {
        ubx_buffer.peek((uint8_t*)&header+i, i);
    } 
    if (header.len + sizeof(UBXHeader_t) > UBX_BUFFER_SIZE){
        ubx_buffer.pop();
        return -1;
    }
    if (header.len + sizeof(UBXHeader_t) > ubx_buffer.get_count()) {
        // Serial.print("Not enough data: ");
        // Serial.print(ubx_buffer.get_count());
        return -1;
    }
    for (unsigned int i = 0; i < sizeof(UBXHeader_t); i++) {
        uint8_t h = 0;
        ubx_buffer.pop(&h);
        // ubx_buffer.pop();
    }
    if (header.cls == UBX_CLASS_NAV && header.id == UBX_ID_NAV_PVT) {
        for (int i = 0; i < header.len; i++) {
            if (i == buf_len) {
                return buf_len;
            }
            ubx_buffer.pop(buffer+i);
        }
        return header.len;
    } else {
        // Serial.println("Unknown message");
    }
    return 0;
}


String ublox_get_fix_type(uint8_t fix_type) {
    switch (fix_type) {
        case NO_FIX:
            return "No fix";
        case DEAD_RECKONING_ONLY:
            return "DRECK";
        case FIX_2D:
            return "2D";
        case FIX_3D:
            return "3D";
        case GPS_DEAD_RECKONING_COMBINED:
            return "GPS + DRC";
        case TIME_ONLY_FIX:
            return "Time";
        default:
            return "Unknown";
    }
}
