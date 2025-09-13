#include <Arduino.h>
#include <SPI.h>
#include "sx1281.h"
#include "pins.h"
#include "SX1280_Regs.h"

static SX1280_RadioOperatingModes_t current_mode;
static uint32_t current_freq = (uint32_t)((double)2400000000 / (double)FREQ_STEP);

/*
 * Period Base from table 11-24, page 79 datasheet rev 3.2
 * SX1280_RADIO_TICK_SIZE_0015_US = 15625 nanos
 * SX1280_RADIO_TICK_SIZE_0062_US = 62500 nanos
 * SX1280_RADIO_TICK_SIZE_1000_US = 1000000 nanos
 * SX1280_RADIO_TICK_SIZE_4000_US = 4000000 nanos
 */
#define RX_TIMEOUT_PERIOD_BASE SX1280_RADIO_TICK_SIZE_0015_US
#define RX_TIMEOUT_PERIOD_BASE_NANOS 15625

void sx1281_check_busy() {
    uint8_t busy_timeout_cnt;
    busy_timeout_cnt = 0;

    while (digitalRead(RADIO_BUSY)) {
        delay(1);
        busy_timeout_cnt++;

        if (busy_timeout_cnt > 10) {
            // wait 10ms for busy to complete
            busy_timeout_cnt = 0;
            Serial.println(F("ERROR - Busy Timeout!"));
            break;
        }
    }
}

void sx1281_read_registers(uint16_t address, uint8_t* buffer, uint16_t size) {
    uint16_t halSize = 4 + size;
    uint8_t halTxBuffer[halSize];
    uint8_t halRxBuffer[halSize];

    halTxBuffer[0] = SX1280_RADIO_READ_REGISTER;
    halTxBuffer[1] = (address & 0xFF00) >> 8;
    halTxBuffer[2] = address & 0x00FF;
    halTxBuffer[3] = 0x00;                // Dummy byte
    memset(halTxBuffer + 4, 0x00, size);  // Fill the rest with dummy bytes

    sx1281_check_busy();
    digitalWrite(RADIO_NSS, LOW);
    SPI.transferBytes(halTxBuffer, halRxBuffer, halSize);
    digitalWrite(RADIO_NSS, HIGH);

    memcpy(buffer, halRxBuffer + 4, size);
}

void sx1281_write_registers(uint16_t address, uint8_t* buffer, uint16_t size) {
    uint16_t halSize = size + 3;
    uint8_t halTxBuffer[halSize];

    halTxBuffer[0] = SX1280_RADIO_WRITE_REGISTER;
    halTxBuffer[1] = (address & 0xFF00) >> 8;
    halTxBuffer[2] = address & 0x00FF;
    memcpy(halTxBuffer + 3, buffer, size);

    sx1281_check_busy();
    digitalWrite(RADIO_NSS, LOW);
    SPI.writeBytes(halTxBuffer, halSize);
    digitalWrite(RADIO_NSS, HIGH);
}

void sx1281_write_register(uint16_t address, uint8_t val) {
    sx1281_write_registers(address, &val, 1);
}

void sx1281_write_commands(uint8_t command, uint8_t* buffer, uint16_t size) {
    uint16_t halSize = size + 1;
    uint8_t halTxBuffer[halSize];
    uint8_t halRxBuffer[halSize];
    halTxBuffer[0] = command;
    if (size > 0 && buffer != nullptr) {
        memcpy(halTxBuffer + 1, buffer, size);
    }
    sx1281_check_busy();
    digitalWrite(RADIO_NSS, LOW);
    SPI.transferBytes(halTxBuffer, halRxBuffer, halSize);
    digitalWrite(RADIO_NSS, HIGH);
    if (CMD_STATUS(halRxBuffer[0]) == 0x3 ||
        CMD_STATUS(halRxBuffer[0]) == 0x4 ||
        CMD_STATUS(halRxBuffer[0]) == 0x5) {
        Serial.println(F("ERROR - Write Command Error!"));
    }
    // Serial.print(" Status: 0x");
    // Serial.print(RADIO_STATE(halRxBuffer[0]), HEX);
    // Serial.print(" 0x");
    // Serial.println(CMD_STATUS(halRxBuffer[0]), HEX);
}

void sx1281_write_command(uint8_t command, uint8_t val) {
    sx1281_write_commands(command, nullptr, 0);
}

void sx1281_read_commands(uint8_t command, uint8_t* buffer, uint16_t size) {
    uint16_t halSize = size + 2;
    uint8_t halTxBuffer[halSize] = {0};
    halTxBuffer[0] = command;

    uint8_t halRxBuffer[halSize];
    sx1281_check_busy();
    if (command == SX1280_RADIO_GET_STATUS) {
        digitalWrite(RADIO_NSS, LOW);
        SPI.transferBytes(halTxBuffer, halRxBuffer, 1);
        digitalWrite(RADIO_NSS, HIGH);
        buffer[0] = halRxBuffer[0];
        return;
    }
    digitalWrite(RADIO_NSS, LOW);
    SPI.transferBytes(halTxBuffer, halRxBuffer, halSize);
    digitalWrite(RADIO_NSS, HIGH);
    memcpy(buffer, halRxBuffer + 2, size);
}

void sx1281_write_buffer(uint8_t offset, uint8_t* buffer, uint8_t size) {
    uint16_t halSize = size + 2;
    uint8_t halTxBuffer[halSize];
    halTxBuffer[0] = SX1280_RADIO_WRITE_BUFFER;
    halTxBuffer[1] = offset;
    memcpy(halTxBuffer + 2, buffer, size);

    sx1281_check_busy();
    digitalWrite(RADIO_NSS, LOW);
    SPI.writeBytes(halTxBuffer, halSize);
    digitalWrite(RADIO_NSS, HIGH);
}

void sx1281_read_buffer(uint8_t offset, uint8_t* buffer, uint8_t size) {
    uint16_t halSize = size + 3;
    uint8_t halRxBuffer[halSize];
    uint8_t halTxBuffer[halSize] = {0};
    halTxBuffer[0] = SX1280_RADIO_READ_BUFFER;
    halTxBuffer[1] = offset;

    sx1281_check_busy();
    digitalWrite(RADIO_NSS, LOW);
    SPI.transferBytes(halTxBuffer, halRxBuffer, halSize);
    digitalWrite(RADIO_NSS, HIGH);
    memcpy(buffer, halRxBuffer + 3, size);
}

void sx1281_reset() {
    digitalWrite(RADIO_RST, LOW);
    delay(50);
    digitalWrite(RADIO_RST, HIGH);
    delay(20);
}

void sx1281_set_mode(SX1280_RadioOperatingModes_t mode, uint16_t incomingTimeout) {
    uint8_t buf[3];
    uint16_t tempTimeout;
    switch (mode) {
    case SX1280_MODE_STDBY_RC:
        sx1281_write_command(SX1280_RADIO_SET_STANDBY, SX1280_STDBY_RC);
        break;

    case SX1280_MODE_RX_CONT:
        buf[0] = RX_TIMEOUT_PERIOD_BASE;
        buf[1] = 0xFFFF >> 8;
        buf[2] = 0xFFFF & 0xFF;
        sx1281_write_commands(SX1280_RADIO_SET_RX, buf, sizeof(buf));
        break;

    case SX1280_MODE_RX:
        tempTimeout = (incomingTimeout * 1000 / RX_TIMEOUT_PERIOD_BASE_NANOS);
        buf[0] = RX_TIMEOUT_PERIOD_BASE;
        buf[1] = tempTimeout >> 8;
        buf[2] = tempTimeout & 0xFF;
        sx1281_write_commands(SX1280_RADIO_SET_RX, buf, sizeof(buf));
        break;

    case SX1280_MODE_TX:
        //uses timeout Time-out duration = periodBase * periodBaseCount
        buf[0] = RX_TIMEOUT_PERIOD_BASE;
        buf[1] = 0xFF; // no timeout set for now
        buf[2] = 0xFF; // TODO dynamic timeout based on expected onairtime
        sx1281_write_commands(SX1280_RADIO_SET_TX, buf, sizeof(buf));
        break;
    default:
        break;
    }
    current_mode = mode;
}

void sx1281_set_packet_type(SX1280_RadioPacketTypes_t type) {
    sx1281_write_command(SX1280_RADIO_SET_PACKETTYPE, type);
}

void sx1281_set_freq_hz(uint32_t freq) {
    uint32_t regfreq = (uint32_t)((double)freq / (double)FREQ_STEP);

    uint8_t buf[3] = {0};

    buf[0] = (uint8_t)((regfreq >> 16) & 0xFF);
    buf[1] = (uint8_t)((regfreq >> 8) & 0xFF);
    buf[2] = (uint8_t)(regfreq & 0xFF);

    sx1281_write_commands(SX1280_RADIO_SET_RFFREQUENCY, buf, sizeof(buf));

    current_freq = regfreq;
}

void sx1281_cfg_mod_params_lora(uint8_t sf, uint8_t bw, uint8_t cr) {
    // Care must therefore be taken to ensure that modulation parameters are set using the command
    // SetModulationParam() only after defining the packet type SetPacketType() to be used
    uint8_t buf[3] = {sf, bw, cr};
    sx1281_write_commands(SX1280_RADIO_SET_MODULATIONPARAMS, buf, sizeof(buf));
    switch (sf) {
        case SX1280_LORA_SF5:
        case SX1280_LORA_SF6:
            sx1281_write_register(SX1280_REG_SF_ADDITIONAL_CONFIG, 0x1E); // for SF5 or SF6
            break;
        case SX1280_LORA_SF7:
        case SX1280_LORA_SF8:
            sx1281_write_register(SX1280_REG_SF_ADDITIONAL_CONFIG, 0x37); // for SF7 or SF8
            break;
        default:
            sx1281_write_register(SX1280_REG_SF_ADDITIONAL_CONFIG, 0x32); // for SF9, SF10, SF11, SF12
    }
    // Datasheet in LoRa Operation says "After SetModulationParams command:
    // In all cases 0x1 must be written to the Frequency Error Compensation mode register 0x093C"
    // However, this causes CRC errors for SF9 when using a high deviation TX (145kHz) and not using Explicit Header mode.
    // The default register value (0x1b) seems most compatible, so don't mess with it
    // InvertIQ=0 0x00=No reception 0x01=Poor reception w/o Explicit Header 0x02=OK 0x03=OK
    // InvertIQ=1 0x00, 0x01, 0x02, and 0x03=Poor reception w/o Explicit Header
    // hal.WriteRegister(SX1280_REG_FREQ_ERR_CORRECTION, 0x03, SX12XX_Radio_All);
}


void sx1281_set_packet_params_lora(uint8_t preamble_len, SX1280_RadioLoRaPacketLengthsModes_t header_type,
                                   uint8_t payload_len, SX1280_RadioLoRaCrcModes_t crcType, uint8_t invert_iq) {
    uint8_t buf[7] = {0};
    buf[0] = preamble_len; // recomended value is 12 symbols (pag 132)
    buf[1] = header_type;
    buf[2] = payload_len;
    buf[3] = crcType;
    buf[4] = invert_iq ? SX1280_LORA_IQ_INVERTED : SX1280_LORA_IQ_NORMAL;;
    buf[5] = 0x00;
    buf[6] = 0x00; // Not used

    sx1281_write_commands(SX1280_RADIO_SET_PACKETPARAMS, buf, sizeof(buf));
}

void sx1281_set_tx_params(uint8_t power, SX1280_RadioRampTimes_t ramp_time) {
    uint8_t buf[2] = {power, ramp_time};
    sx1281_write_commands(SX1280_RADIO_SET_TXPARAMS, buf, sizeof(buf));
}

void sx1281_set_regulator_mode(SX1280_RadioRegulatorModes_t mode) {
    sx1281_write_command(SX1280_RADIO_SET_REGULATORMODE, mode);
}

void sx1281_get_packet_status_lora(SX1280_PacketStatusLoRa_t* status) {
    uint8_t buf[2] = {0};
    sx1281_write_commands(SX1280_RADIO_GET_PACKETSTATUS, buf, sizeof(buf));
    status->rssi = -(int8_t)buf[0] / 2.0;
    status->snr = (int8_t)buf[1] / RADIO_SNR_SCALE;

    // Datasheet pag93 Table 11-66: RSSI and SNR Packet Status
    // need to subtract SNR from RSSI when SNR <= 0;
    int8_t negOffset = (status->snr  < 0) ? status->snr : 0;
    status->rssi += negOffset;
}

uint8_t sx1281_get_status(void) {
    uint8_t status = 0;
    sx1281_read_commands(SX1280_RADIO_GET_STATUS, &status, 1);
    return status;
}

uint8_t sx1281_get_rx_buf_addr() {
    uint8_t buf[2] = {0};
    sx1281_read_commands(SX1280_RADIO_GET_RXBUFFERSTATUS, buf, sizeof(buf));
    return buf[1];
}

void sx1281_set_rx_gain_regime(SX1280_RxGainRegime_t regime) {
    sx1281_write_register(SX1280_REG_RX_GAIN, regime);
}

void sx1281_set_dio_irq_params(uint16_t irq_mask, uint16_t dio1_mask, uint16_t dio2_mask, uint16_t dio3_mask) {
    uint8_t buf[8] = {0};
    buf[0] = (irq_mask >> 8) & 0xFF;
    buf[1] = irq_mask & 0xFF;
    buf[2] = (dio1_mask >> 8) & 0xFF;
    buf[3] = dio1_mask & 0xFF;
    buf[4] = (dio2_mask >> 8) & 0xFF;
    buf[5] = dio2_mask & 0xFF;
    buf[6] = (dio3_mask >> 8) & 0xFF;
    buf[7] = dio3_mask & 0xFF;
    sx1281_write_commands(SX1280_RADIO_SET_DIOIRQPARAMS, buf, sizeof(buf));
}

uint16_t sx1281_get_irq_status(void) {
    uint8_t buf[2] = {0};
    sx1281_read_commands(SX1280_RADIO_GET_IRQSTATUS, buf, sizeof(buf));
    return buf[0] << 8 | buf[1];
}

void sx1281_clear_irq_status(uint16_t irq_mask) {
    uint8_t buf[2] = {0};
    buf[0] = (uint8_t)(((uint16_t)irq_mask >> 8) & 0x00FF);
    buf[1] = (uint8_t)((uint16_t)irq_mask & 0x00FF);
    sx1281_write_commands(SX1280_RADIO_CLR_IRQSTATUS, buf, sizeof(buf));
}
