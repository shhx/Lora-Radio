#include "ublox_protocol.h"
#include "circular_buffer.h"

CircularBuffer<uint8_t> ubx_buffer(UBX_BUFFER_SIZE);

void ublox_init() {
    UBX_SERIAL.begin(UBX_SERIAL_SPEED, SERIAL_8N1);
    ublox_set_cfg_pm_operatemode(PM_MODE_FULL);
}

bool ublox_parse_msg(ubx_nav_pvt_t *nav_pvt){
    while (ubx_buffer.get_count() > sizeof(ubx_header_t) + sizeof(ubx_nav_pvt_t)) {
        int ret = ublox_get_next_msg((uint8_t *)nav_pvt, sizeof(ubx_nav_pvt_t));
        if (ret == -1) {
            return false;
        }
        if(ret == sizeof(ubx_nav_pvt_t)){
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
    if (ubx_buffer.get_count() < sizeof(ubx_header_t)) {
        return 0;
    }
    ubx_header_t header;
    ubx_buffer.peek(&header.sync1, 0);
    ubx_buffer.peek(&header.sync2, 1);
    if (header.sync1 != UBX_SYNC1 || header.sync2 != UBX_SYNC2) {
        ubx_buffer.pop();
        return 0;
    }
    for (uint16_t i = 2; i < sizeof(ubx_header_t); i++) {
        ubx_buffer.peek((uint8_t*)&header+i, i);
    } 
    if (header.length + sizeof(ubx_header_t) > UBX_BUFFER_SIZE){
        ubx_buffer.pop();
        return -1;
    }
    if (header.length + sizeof(ubx_header_t) > ubx_buffer.get_count()) {
        // Serial.print("Not enough data: ");
        // Serial.print(ubx_buffer.get_count());
        return -1;
    }
    for (unsigned int i = 0; i < sizeof(ubx_header_t); i++) {
        uint8_t h = 0;
        ubx_buffer.pop(&h);
        // ubx_buffer.pop();
    }
    if (header.cls == UBX_CLASS_NAV) {
        if (header.id == UBX_ID_NAV_PVT) {
            for (int i = 0; i < header.length; i++) {
                if (i == buf_len) {
                    return buf_len;
                }
                ubx_buffer.pop(buffer+i);
            }
        }
        return header.length;
    } else if (header.cls == UBX_CLASS_ACK) {
        if (header.id == UBX_ID_ACK_ACK) {
            Serial.println("GPS ACK received");
        } else if (header.id == UBX_ID_ACK_NAK) {
            Serial.println("GPS NAK received");
        } else {
            Serial.println("Unknown ACK message");
        }

    } else {
        Serial.println("Unknown message");
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

static uint16_t ublox_checksum(const uint8_t *msg, uint16_t length) {
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    for (uint16_t i = 0; i < length; i++) {
        ck_a += msg[i];
        ck_b += ck_a;
    }
    return ck_a | (ck_b << 8);
}

int ublox_send_message(uint8_t cls, uint8_t id, uint8_t* payload, uint16_t payload_length) {
    uint16_t length = payload_length + 4; // 2 bytes for header, 2 bytes for length
    ubx_header_t header = {
        .sync1 = UBX_SYNC1,
        .sync2 = UBX_SYNC2,
        .cls = cls,
        .id = id,
        .length = payload_length
    };
    static uint8_t uart_buffer[UBX_BUFFER_SIZE];
    memcpy(uart_buffer, &header, sizeof(ubx_header_t));
    memcpy(uart_buffer + sizeof(ubx_header_t), payload, payload_length);
    // Calculate checksum over class, id, length, and payload (skip sync bytes)
    uint16_t checksum = ublox_checksum(uart_buffer + 2, length);

    Serial.print("\nSending UBX message: ");
    for (uint16_t i = 0; i < sizeof(ubx_header_t) + payload_length; i++) {
        Serial.printf("%02X ", uart_buffer[i]);
    }
    Serial.printf("CK_A: %02X CK_B: %02X\n", (checksum & 0xFF), (checksum >> 8) & 0xFF);
    UBX_SERIAL.write(uart_buffer, sizeof(ubx_header_t) + payload_length);
    UBX_SERIAL.write((uint8_t*)&checksum, sizeof(uint16_t));
    return 0;
}

int ublox_send_cfg_valset(uint32_t key, uint8_t* value, uint8_t size) {
    // 4 bytes for key, 3 config_op and up to 4 bytes for value
    uint8_t payload[sizeof(uint32_t) + sizeof(ubx_cfg_op_t) + 4];
    uint8_t total_size = sizeof(uint32_t) + sizeof(ubx_cfg_op_t) + size;
    ubx_cfg_op_t cfg_op = {
        .version = 0x1,
        .ram = 1,
        .bbr = 1,
        .flash = 0,
    };
    if (size > 4) size = 4; // Limit size to 4 bytes
    memcpy(payload, &cfg_op, sizeof(ubx_cfg_op_t));
    memcpy(payload + sizeof(ubx_cfg_op_t), &key, 4);
    memcpy(payload + sizeof(ubx_cfg_op_t) + 4, value, size);
    // Send message
    ublox_send_message(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, payload, total_size);
    return 0;
}

int ublox_set_cfg_pm_operatemode(ConfigPMMode mode) {
    ublox_send_cfg_valset(CFG_PM_OPERATEMODE, (uint8_t*)&mode, 1); // Key for PM_OPERATEMODE
    return 0;
    // Wait for ACK or NAK
    // return ublox_wait_for_ack(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, 1000); // 1 second timeout
}
