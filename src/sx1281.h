#ifndef __SX1281_H__
#define __SX1281_H__
#include "SX1280_Regs.h"

#define RADIO_SNR_SCALE 4.0f
#define RADIO_STATE(status) ((status >> 5) & 0b111)
#define CMD_STATUS(status) (status >> 2 & 0b111)

/* Steps for startup

1. If not in STDBY_RC mode, then go to this mode by sending the command:
SetStandby(STDBY_RC)

2. Define the LoRa® packet type by sending the command:
SetPacketType(PACKET_TYPE_LORA)

3. Define the RF frequency by sending the command:
SetRfFrequency(rfFrequency)
The LSB of rfFrequency is equal to the PLL step i.e. 52e6/2^18 Hz. SetRfFrequency() defines the Tx frequency.

4. Indicate the addresses where the packet handler will read (txBaseAddress in Tx) or write (rxBaseAddress in Rx) the first
byte of the data payload by sending the command:
SetBufferBaseAddress(txBaseAddress, rxBaseAddress)
Note:
txBaseAddress and rxBaseAddress are offset relative to the beginning of the data memory map.

5. Define the modulation parameter signal BW SF CR
*/

typedef struct {
    float rssi;
    float snr;
} SX1280_PacketStatusLoRa_t;

void sx1281_check_busy();
void sx1281_reset( void );
void sx1281_write_registers(uint16_t address, uint8_t* buffer, uint16_t size);
void sx1281_write_register(uint16_t address, uint8_t val);
void sx1281_read_registers(uint16_t address, uint8_t* buffer, uint16_t size);
void sx1281_write_commands(uint8_t command, uint8_t* buffer, uint16_t size);
void sx1281_write_command(uint8_t command, uint8_t val);
void sx1281_read_commands(uint8_t command, uint8_t* buffer, uint16_t size);
void sx1281_write_buffer(uint8_t offset, uint8_t* buffer, uint8_t size);
void sx1281_read_buffer(uint8_t offset, uint8_t* buffer, uint8_t size);

void sx1281_set_mode(SX1280_RadioOperatingModes_t mode, uint16_t incomingTimeout);
void sx1281_set_packet_type(SX1280_RadioPacketTypes_t type);
void sx1281_set_freq_hz(uint32_t freq);
void sx1281_cfg_mod_params_lora(uint8_t sf, uint8_t bw, uint8_t cr);
void sx1281_set_packet_params_lora(uint8_t preambleLen, SX1280_RadioLoRaPacketLengthsModes_t headerType,
                                   uint8_t payloadLen, SX1280_RadioLoRaCrcModes_t crcType, uint8_t invertIQ);
void sx1281_set_tx_params(uint8_t power, SX1280_RadioRampTimes_t ramp_time);
void sx1281_set_regulator_mode(SX1280_RadioRegulatorModes_t mode);
void sx1281_get_packet_status_lora(SX1280_PacketStatusLoRa_t* status);
int8_t sx1281_get_rssi_inst(void);
void sx1281_set_rx_gain_regime(SX1280_RxGainRegime_t regime);

uint8_t sx1281_get_status(void);
uint8_t sx1281_get_rx_buf_addr();
void sx1281_set_rx_buffer_baseaddr(uint8_t tx_addr, uint8_t rx_addr);
void sx1281_get_rx_buffer_status(uint8_t* payload_len, uint8_t* buffer_ptr);
void sx1281_set_dio_irq_params(uint16_t irq_mask, uint16_t dio1_mask, uint16_t dio2_mask, uint16_t dio3_mask);
uint16_t sx1281_get_irq_status(void);
void sx1281_clear_irq_status(uint16_t irq_mask);
void sx1281_set_cont_preamble();
void sx1281_set_sleep(bool keep_data_buffer, bool keep_ram);

#endif // __SX1281_H__
