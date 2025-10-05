#include <Arduino.h>
#include "cassert.h"

enum PacketType {
    PACKET_TYPE_GPS = 0,
    PACKET_TYPE_ACK  = 1,
};

typedef struct __attribute__((packed)) {
    uint8_t reserved[7];
    int8_t rssi;
    uint8_t seq_num;
} PacketAck_t;

typedef struct __attribute__((packed)) {
    int32_t lon;
    int32_t lat;
    uint8_t fix_ok : 1;
    uint8_t fix_type: 3;
    uint8_t sv_num: 4;
} PacketGPS_t;

typedef struct __attribute__((packed)) {
    union {
        PacketAck_t ack;
        PacketGPS_t gps;
        uint8_t raw[9];
    };
    uint8_t seq_num;
    uint8_t packet_type;
} Packet_t;

const int PACKET_SIZE = sizeof(Packet_t);
// CASSERT(sizeof(PacketAck_t) == PACKET_SIZE);
// CASSERT(sizeof(PacketGPS_t) == PACKET_SIZE);
