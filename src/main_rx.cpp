#include <Arduino.h>
#include <SPI.h>
#include "pins.h"
#include "sx1281.h"
#include "rf_amp.h"
#include "utils.h"

// RX_minimal.ino
#define PACKET_SIZE 8
#define PREAMBLE_SIZE 12

void setup() {
    Serial.begin(115200);
    SPI.begin();
    if (SPI.pins(RADIO_SCK, RADIO_MISO, RADIO_MOSI, RADIO_NSS) == false) {
        Serial.println(F("ERROR - SPI Pins not set correctly!"));
        ESP.reset();
    }
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

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

    // Mod / packet params: SF7, BW 800k, CR 4/5, explicit header, CRC ON
    sx1281_cfg_mod_params_lora(SX1280_LORA_SF7, SX1280_LORA_BW_0800, SX1280_LORA_CR_LI_4_5);
    sx1281_set_packet_params_lora(PREAMBLE_SIZE,
                                  SX1280_LORA_PACKET_EXPLICIT,
                                  0,                       // length ignored in explicit
                                  SX1280_LORA_CRC_ON,
                                  SX1280_LORA_IQ_NORMAL);


    sx1281_set_regulator_mode(SX1280_USE_LDO);
    sx1281_set_tx_params(0, SX1280_RADIO_RAMP_04_US); // TX params okay even on RX side

    // IRQs: request SYNCWORD_VALID, RX_DONE, CRC_ERROR
    uint16_t dio1_mask = SX1280_IRQ_RX_DONE | SX1280_IRQ_SYNCWORD_VALID | SX1280_IRQ_CRC_ERROR;
    uint16_t irq_mask  = SX1280_IRQ_RX_DONE | SX1280_IRQ_SYNCWORD_VALID | SX1280_IRQ_SYNCWORD_ERROR | SX1280_IRQ_CRC_ERROR;
    irq_mask |= SX1280_IRQ_RADIO_ALL;
    sx1281_set_dio_irq_params(irq_mask, dio1_mask, 0, 0);

    radio_rfamp_rx_enable();        // enable LNA / RF switch to RX
    sx1281_clear_irq_status(0xFFFF); // clear anything stale
    sx1281_set_mode(SX1280_MODE_RX_CONT, 0);
    delay(100);

    Serial.println(F("RX ready (SF7 BW800 CR4/5 preamble12 explicit CRC ON)"));
}

void loop() {
    // blink for activity
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);

    int8_t rssi = sx1281_get_rssi_inst();
    Serial.print("RSSI: "); Serial.print(rssi); Serial.println(" dBm");

    // Poll DIO1 (IRQ pin) or optionally poll IRQ register periodically
    if (digitalRead(RADIO_DIO1) == HIGH) {
        uint16_t irq = sx1281_get_irq_status();
        Serial.print("IRQ: 0x"); Serial.println(irq, HEX);

        // If sync valid seen, show a message
        if (irq & SX1280_IRQ_SYNCWORD_VALID) {
            Serial.println("SYNCWORD_VALID");
        }
        if (irq & SX1280_IRQ_CRC_ERROR) {
            Serial.println("CRC_ERROR");
            sx1281_clear_irq_status(irq);
            return;
        }

        SX1280_PacketStatusLoRa_t pkt;
        if (irq & SX1280_IRQ_RX_DONE) {
            sx1281_get_packet_status_lora(&pkt); // fills .rssi and .snr
            // get length (explicit header): read buffer start and payload length
            uint8_t payload_len = sx1281_get_rx_buf_addr(); // your wrapper returns length or buffer offset depending on lib
            if (payload_len == 0) {
                // fallback: read the PacketLen register or assume PACKET_SIZE
                payload_len = PACKET_SIZE;
            }

            uint8_t payload[payload_len + 1];
            memset(payload, 0, payload_len + 1);
            sx1281_read_buffer(0, payload, payload_len);

            Serial.print("RX: '");
            for (uint8_t i = 0; i < payload_len; ++i) Serial.print((char)payload[i]);
            Serial.print("' | RSSI: ");
            Serial.print(pkt.rssi);
            Serial.print(" dBm | SNR: ");
            Serial.print(pkt.snr);
            Serial.println(" dB");

            sx1281_clear_irq_status(irq);
        } else {
            // If DIO1 high but no RX_DONE, just clear and continue
            sx1281_clear_irq_status(irq);
        }
    } else {
        print_radio_status();
        uint16_t irq = sx1281_get_irq_status();
        Serial.print("IRQ: 0x"); Serial.println(irq, HEX);
    }
}
