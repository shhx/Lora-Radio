#include <Arduino.h>
#include <SPI.h>
#include "pins.h"
#include "sx1281.h"
#include "rf_amp.h"

#define PACKET_SIZE 8

void setup() {
    Serial.begin(115200);
    SPI.begin();
    if (SPI.pins(RADIO_SCK, RADIO_MISO, RADIO_MOSI, RADIO_NSS) == false) {
        Serial.println(F("ERROR - SPI Pins not set correctly!"));
        ESP.reset();
    }
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

    pinMode(LED_PIN, OUTPUT);
    pinMode(RADIO_NSS, OUTPUT);
    pinMode(RADIO_RST, OUTPUT);
    pinMode(POWER_RXEN, OUTPUT);
    pinMode(POWER_TXEN, OUTPUT);
    pinMode(RADIO_BUSY, INPUT);
    pinMode(RADIO_DIO1, INPUT);
    delay(2000);

    sx1281_reset();
    uint8_t fw_version[2] = {0};
    sx1281_read_registers(REG_LR_FIRMWARE_VERSION_MSB, fw_version, 2);
    uint16_t firmwareRev = (fw_version[1] << 8) | fw_version[0];
    if ((firmwareRev == 0) || (firmwareRev == 65535)) {
        Serial.println(F("ERROR - Failed to read radio version!"));
        while(1);
    }
    Serial.print("Firmware Version: 0x"); Serial.println(firmwareRev, HEX);

    sx1281_set_mode(SX1280_MODE_STDBY_RC, 0);
    sx1281_set_packet_type(SX1280_PACKET_TYPE_LORA);
    sx1281_set_freq_hz(2440000000);

    sx1281_cfg_mod_params_lora(SX1280_LORA_SF11, SX1280_LORA_BW_0200, SX1280_LORA_CR_LI_4_8);
    sx1281_set_packet_params_lora(PREAMBLE_SIZE,
                                  SX1280_LORA_PACKET_FIXED_LENGTH,
                                  PACKET_SIZE,
                                  SX1280_LORA_CRC_ON,
                                  SX1280_LORA_IQ_NORMAL);

    sx1281_set_rx_buffer_baseaddr(0, 0);
    sx1281_set_regulator_mode(SX1280_USE_DCDC);
    sx1281_set_tx_params(SX1280_POWER_MAX, SX1280_RADIO_RAMP_04_US);

    uint16_t dio1_mask = SX1280_IRQ_TX_DONE;
    uint16_t irq_mask  = SX1280_IRQ_TX_DONE | SX1280_IRQ_CRC_ERROR;
    sx1281_set_dio_irq_params(irq_mask, dio1_mask, 0, 0);

    radio_rfamp_tx_enable(); // ensure TX path enabled
    sx1281_clear_irq_status(0xFFFF);
}

void loop() {
    const char msg[PACKET_SIZE] = "ABCDEFG"; // 8 bytes
    //sx1281_set_rx_buffer_baseaddr(0, 0);
    sx1281_write_buffer(0, (uint8_t*)msg, sizeof(msg));
    sx1281_clear_irq_status(0xFFFF);

    sx1281_set_mode(SX1280_MODE_TX, 0);
    // wait for TX_DONE via IRQ
    unsigned long t0 = millis();
    while (millis() - t0 < 2000) { // 2s timeout
        uint16_t irq = sx1281_get_irq_status();
        if (irq) {
            Serial.print("IRQ: 0x"); Serial.println(irq, HEX);
            if (irq & SX1280_IRQ_TX_DONE) {
                Serial.println("TX_DONE");
                sx1281_clear_irq_status(irq);
                break;
            }
            if (irq & SX1280_IRQ_CRC_ERROR) {
                Serial.println("TX CRC_ERROR?"); // unlikely on TX but print for debug
                sx1281_clear_irq_status(irq);
            }
        }
        delay(10);
    }

    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
}
