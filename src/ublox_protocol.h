#ifndef UBLOX_PROTOCOL_H
#define UBLOX_PROTOCOL_H

#include <Arduino.h>
#include "cassert.h"

#define UBX_SERIAL Serial
#define UBX_SERIAL_SPEED 115200
#define UBX_BUFFER_SIZE 512
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62
#define UBX_CHECKSUM_SIZE 2

#define UBX_CLASS_ACK 0x05
#define UBX_ID_ACK_ACK 0x01
#define UBX_ID_ACK_NAK 0x00

#define UBX_CLASS_NAV 0x01
#define UBX_ID_NAV_PVT 0x07
#define UBX_CFG_NAV5 0x2406

#define UBX_CLASS_CFG 0x06
#define UBX_ID_CFG_VALSET 0x8A

#define CFG_PM_OPERATEMODE 0x20d00001

enum ParseOut {NEED_DATA, PARSE_OK, WRONG_HEADER, WRONG_CHECKSUM};

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
    uint16_t length;
} ubx_header_t; 


typedef struct {
    uint8_t version = 0x1;
    uint8_t ram: 1;
    uint8_t bbr: 1;
    uint8_t flash: 1;
    uint8_t reserved1[2];
} ubx_cfg_op_t;


enum ConfigStorageSize {
    SIZE_1BIT  = 1,
    SIZE_8BIT  = 2,
    SIZE_16BIT = 3,
    SIZE_32BIT = 4,
    SIZE_64BIT = 5,
};

typedef struct {
    uint32_t reserved1: 1;
    uint32_t storage_size: 3;
    uint32_t reserved2: 4;
    uint32_t group_id: 8;
    uint32_t reserved3: 4;
    uint32_t item_id: 12;
} ubx_cfg_valset_key_t;
CASSERT(sizeof(ubx_cfg_valset_key_t) == 4, ubx_cfg_valset_key_t_size_mismatch);

typedef struct {
    uint8_t gnssFixOK : 1;
    uint8_t diffSoln : 1;
    uint8_t psmState : 3;
    uint8_t headVehValid : 1;
    uint8_t carrSoln : 2;
} ubx_nav_pvt_flags_t;

typedef struct {
    uint16_t msg_class_id;
    uint8_t rates[6];
} ubx_cfg_msg_t;

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
    ubx_nav_pvt_flags_t flags;
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
} ubx_nav_pvt_t;

enum ConfigPMMode {
    PM_MODE_FULL = 0,
    PM_MODE_PSMOO = 1,
    PM_MODE_PSMCT = 2
};

void ublox_init();
String ublox_get_fix_type(uint8_t fix_type);
void ublox_get_new_data();
int ublox_get_next_msg(uint8_t *buffer, int length);
bool ublox_parse_msg(ubx_nav_pvt_t *nav_pvt);

void ublox_receive();
int ublox_parse();
int ublox_send_message(uint8_t cls, uint8_t id, uint8_t* payload, uint16_t payload_length);
int ublox_set_cfg_pm_operatemode(ConfigPMMode mode);

#endif // UBLOX_PROTOCOL_H
