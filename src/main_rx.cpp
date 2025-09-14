#include <Arduino.h>
#include <SPI.h>
#include "pins.h"
#include "sx1281.h"
#include "rf_amp.h"
#include "utils.h"

// RX_minimal.ino
#define PACKET_SIZE 8
#define PREAMBLE_SIZE 40

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
    Serial.print("Firmware Version: 0x");
    Serial.println(firmwareRev, HEX);

    sx1281_set_mode(SX1280_MODE_STDBY_RC, 0);
    sx1281_set_packet_type(SX1280_PACKET_TYPE_LORA);
    sx1281_set_freq_hz(2440000000);  // 2.4 GHz, match TX

    sx1281_cfg_mod_params_lora(SX1280_LORA_SF10, SX1280_LORA_BW_0800, SX1280_LORA_CR_4_5);
    sx1281_set_packet_params_lora(PREAMBLE_SIZE,
                                  SX1280_LORA_PACKET_IMPLICIT,
                                  PACKET_SIZE,
                                  SX1280_LORA_CRC_ON,
                                  SX1280_LORA_IQ_INVERTED);


    sx1281_set_regulator_mode(SX1280_USE_DCDC);
    sx1281_set_tx_params(0, SX1280_RADIO_RAMP_04_US); // TX params okay even on RX side

    // IRQs: request SYNCWORD_VALID, RX_DONE, CRC_ERROR
    uint16_t dio1_mask = SX1280_IRQ_RX_DONE | SX1280_IRQ_SYNCWORD_VALID | SX1280_IRQ_CRC_ERROR;
    uint16_t irq_mask  = SX1280_IRQ_RX_DONE | SX1280_IRQ_SYNCWORD_VALID | SX1280_IRQ_SYNCWORD_ERROR | SX1280_IRQ_CRC_ERROR;
    irq_mask |= SX1280_IRQ_RADIO_ALL;
    sx1281_set_dio_irq_params(irq_mask, dio1_mask, 0, 0);

    radio_rfamp_rx_enable();        // enable LNA / RF switch to RX
    sx1281_clear_irq_status(0xFFFF); // clear anything stale
    sx1281_set_mode(SX1280_MODE_RX_CONT, 50000);
    delay(100);
}

void loop() {
    // Poll IRQ register every ~100 ms
    uint16_t irq = sx1281_get_irq_status();
    if (irq != 0) {
        Serial.print("IRQ raw: 0x"); Serial.println(irq, HEX);

        if (irq & SX1280_IRQ_RX_TX_TIMEOUT) {
            Serial.println("  RX_TX_TIMEOUT - no packet");
            sx1281_set_mode(SX1280_MODE_RX_CONT, 10000); // restart RX
        }
        // Check common flags
        if (irq & SX1280_IRQ_SYNCWORD_VALID) {
            Serial.println("  SYNCWORD_VALID");
        }
        if (irq & SX1280_IRQ_SYNCWORD_ERROR) {
            Serial.println("  SYNCWORD_ERROR");
        }
        if (irq & SX1280_IRQ_CRC_ERROR) {
            Serial.println("  CRC_ERROR");
        }
        if (irq & SX1280_IRQ_RX_DONE) {
            Serial.println("  RX_DONE");

            // Get packet status (SNR, RSSI)
            SX1280_PacketStatusLoRa_t pkt;
            sx1281_get_packet_status_lora(&pkt);
            Serial.print("  pkt RSSI: "); Serial.print(pkt.rssi); Serial.print(" dBm");
            Serial.print("  SNR: "); Serial.println(pkt.snr);

            // Get RX buffer status (payload length and buffer pointer)
            uint8_t payload_len = 0;
            uint8_t buffer_ptr = 0;
            // Use your wrapper that implements "GetRxBufferStatus"
            sx1281_get_rx_buffer_status(&payload_len, &buffer_ptr);
            Serial.print("  rx buffer ptr: "); Serial.print(buffer_ptr);
            Serial.print("  payload_len: "); Serial.println(payload_len);

            if (payload_len > 0) {
                uint8_t payload[payload_len + 1];
                memset(payload, 0, payload_len + 1);
                sx1281_read_buffer(buffer_ptr, payload, payload_len);
                Serial.print("  payload: ");
                for (int i = 0; i < payload_len; ++i) Serial.print((char)payload[i]);
                Serial.println();
            } else {
                Serial.println("  payload_len == 0");
            }
        }

        // Clear the IRQs we just handled
        sx1281_clear_irq_status(irq);
    }

    // Optional: print RSSI floor every 1s so you know radio sees energy
    static unsigned long last = 0;
    if (millis() - last > 1000) {
        last = millis();
        int8_t r = sx1281_get_rssi_inst(); // get instant RSSI (signed dBm)
        Serial.print("RSSI floor: "); Serial.print(r); Serial.println(" dBm");
    }

    delay(100);
}
