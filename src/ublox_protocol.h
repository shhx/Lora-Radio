#ifndef UBLOX_PROTOCOL_H
#define UBLOX_PROTOCOL_H

#include <Arduino.h>

#define UBX_SERIAL Serial
#define UBX_SERIAL_SPEED 115200
#define UBX_BUFFER_SIZE 512
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62
#define UBX_CLASS_NAV 0x01
#define UBX_ID_NAV_PVT 0x07

enum FixType {
    NO_FIX = 0,
    DEAD_RECKONING_ONLY = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    GPS_DEAD_RECKONING_COMBINED = 4,
    TIME_ONLY_FIX = 5
};

typedef struct {
    uint8_t sync1;
    uint8_t sync2;
    uint8_t cls;
    uint8_t id;
    uint16_t len;
} UBXHeader_t; 

typedef struct {
    uint8_t gnssFixOK : 1;
    uint8_t diffSoln : 1;
    uint8_t psmState : 3;
    uint8_t headVehValid : 1;
    uint8_t carrSoln : 2;
} UBXNavPVT_Flags_t;

typedef struct {
    uint32_t iTOW;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    UBXNavPVT_Flags_t flags;
    uint8_t flags2;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint8_t flags3;
    uint8_t reserved1[5];
    int32_t headVeh;
    int16_t magDec;
    uint16_t magAcc;
} UBXNavPVT_t;


void ublox_init();
String ublox_get_fix_type(uint8_t fix_type);
void ublox_get_new_data();
int ublox_get_next_msg(uint8_t *buffer, int length);
bool ublox_parse_msg(UBXNavPVT_t *nav_pvt);


#endif // UBLOX_PROTOCOL_H
