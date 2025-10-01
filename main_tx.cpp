#include <Arduino.h>
#include <SPI.h>

#include "ArduinoOTA.h"
#include "OTA.h"
#include "packet.h"
#include "pins.h"
#include "rf_amp.h"
#include "sx1281.h"
#include "ublox_protocol.h"

enum RadioState {
    RADIO_STATE_IDLE = 0,
    RADIO_STATE_RX,
    RADIO_STATE_TX,
};

const uint64_t SLEEP_DURATION_US = 30 * 1000 * 1000;
static uint16_t radio_tx_period_ms = 1000;
static uint32_t last_tx_time = 0;
static RadioState radio_state = RADIO_STATE_IDLE;
static bool ota_enabled = false;
static uint64_t button_press_time = 0;

UBXNavPVT_t nav_pvt;

void start_ota_updater(void) {
    setupWiFi();
    setupOTA("radio");
    ota_enabled = true;
}

void setup() {
    // Serial.begin(115200);
    ublox_init();
    Serial.println(ESP.getCpuFreqMHz());
    SPI.begin();
    if (SPI.pins(RADIO_SCK, RADIO_MISO, RADIO_MOSI, RADIO_NSS) == false) {
        Serial.println(F("ERROR - SPI Pins not set correctly!"));
        ESP.reset();
    }
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

    // pinMode(LED_PIN, OUTPUT);
    pinMode(RADIO_NSS, OUTPUT);
    pinMode(RADIO_RST, OUTPUT);
    pinMode(POWER_RXEN, OUTPUT);
    pinMode(POWER_TXEN, OUTPUT);
    pinMode(RADIO_BUSY, INPUT);
    pinMode(RADIO_DIO1, INPUT);

    sx1281_reset();
    uint8_t fw_version[2] = {0};
    sx1281_read_registers(REG_LR_FIRMWARE_VERSION_MSB, fw_version, 2);
    uint16_t firmwareRev = (fw_version[1] << 8) | fw_version[0];
    if ((firmwareRev == 0) || (firmwareRev == 65535)) {
        Serial.println(F("ERROR - Failed to read radio version!"));
        while (1);
    }
    Serial.print("Firmware Version: 0x");
    Serial.println(firmwareRev, HEX);
    sx1281_set_mode(SX1280_MODE_STDBY_RC, 0);
    sx1281_set_packet_type(SX1280_PACKET_TYPE_LORA);
    sx1281_set_freq_hz(2440000000);

    sx1281_cfg_mod_params_lora(SX1280_LORA_SF9, SX1280_LORA_BW_0200,
                               SX1280_LORA_CR_LI_4_8);
    sx1281_set_packet_params_lora(PREAMBLE_SIZE,
                                  SX1280_LORA_PACKET_FIXED_LENGTH, PACKET_SIZE,
                                  SX1280_LORA_CRC_ON, SX1280_LORA_IQ_NORMAL);

    sx1281_set_rx_buffer_baseaddr(0, 0);
    sx1281_set_regulator_mode(SX1280_USE_DCDC);
    sx1281_set_tx_params(SX1280_POWER_MAX, SX1280_RADIO_RAMP_04_US);

    uint16_t dio1_mask = SX1280_IRQ_TX_DONE;
    uint16_t irq_mask = SX1280_IRQ_TX_DONE | SX1280_IRQ_CRC_ERROR;
    sx1281_set_dio_irq_params(irq_mask, dio1_mask, 0, 0);

    sx1281_clear_irq_status(0xFFFF);
}

void transmit_packet(Packet_t *msg) {
    sx1281_set_rx_buffer_baseaddr(0, 0);
    sx1281_write_buffer(0, (uint8_t *)msg, PACKET_SIZE);
    sx1281_clear_irq_status(0xFFFF);
    radio_rfamp_tx_enable();  // ensure TX path enabled
    sx1281_set_mode(SX1280_MODE_TX, 0);
    radio_state = RADIO_STATE_TX;
}

void loop() {
    Packet_t packet = {
        .lon = nav_pvt.lon,
        .lat = nav_pvt.lat,
        .sv_num = nav_pvt.numSV,
        .rssi = 0,
    };
    switch (radio_state) {
        case RADIO_STATE_IDLE:
            if (last_tx_time != 0 &&
                millis() - last_tx_time < radio_tx_period_ms)
                break;
            Serial.println("Starting TX");
            transmit_packet(&packet);
            last_tx_time = millis();
            break;
        case RADIO_STATE_TX: {
            uint16_t irq = sx1281_get_irq_status();
            if (irq & SX1280_IRQ_TX_DONE) {
                Serial.println("TX_DONE");
                uint32_t tx_time = millis() - last_tx_time;
                Serial.print("TX time (ms): ");
                Serial.println(tx_time);
                sx1281_clear_irq_status(irq);
                radio_rfamp_rx_enable();
                sx1281_set_mode(SX1280_MODE_STDBY_RC, 0);
                sx1281_set_sleep(false, true);  // keep RAM, discard buffer
                radio_state = RADIO_STATE_IDLE;
                if (!ota_enabled) {
                    Serial.println("Going to sleep");
                    ESP.deepSleep(SLEEP_DURATION_US, WAKE_RF_DISABLED);
                }
            }
            break;
        }
        case RADIO_STATE_RX:
            break;
        default:
            break;
    }
    if (digitalRead(BUTTON_PIN) == LOW && button_press_time == 0) {
        button_press_time = millis();
    } else if (digitalRead(BUTTON_PIN) == HIGH) {
        button_press_time = 0;
    }
    if (millis() - button_press_time > 2000) {
        Serial.println("Button held for 2 seconds, going to sleep");
        ESP.deepSleep(SLEEP_DURATION_US, WAKE_RF_DISABLED);
    }
    if (digitalRead(BUTTON_PIN) == LOW && !ota_enabled) {
        Serial.println("Button pressed, starting OTA");
        start_ota_updater();
    }
    if (button_press_time != 0 && (millis() - button_press_time) > 5000) {
        Serial.println("Button held for 5 seconds, restarting...");
        ESP.restart();
    }
    if (ota_enabled) ArduinoOTA.handle();

    ublox_get_new_data();
    bool ret = ublox_parse_msg(&nav_pvt);
}
