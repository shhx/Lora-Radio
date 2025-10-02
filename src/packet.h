#include <Arduino.h>

enum PacketType {
    PACKET_TYPE_GPS = 0,
    PACKET_TYPE_ACK  = 1,
};

typedef struct __attribute__((packed)) {
    uint8_t reserved[7];
    uint8_t rssi;
    uint8_t seq_num;
} PacketAck_t;

typedef struct __attribute__((packed)) {
    int32_t lon;
    int32_t lat;
    uint8_t sv_num;
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
// static_assert(sizeof(PacketAck_t) == 10);
// static_assert(sizeof(PacketGPS_t) == 10);
