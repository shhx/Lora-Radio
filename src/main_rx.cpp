#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <SPI.h>

#include "ArduinoOTA.h"
#include "OTA.h"
#include "credentials.h"
#include "packet.h"
#include "pins.h"
#include "rf_amp.h"
#include "sx1281.h"

enum RadioState {
    RADIO_STATE_IDLE = 0,
    RADIO_STATE_RX,
    RADIO_STATE_TX_ACK,
};

const char *ap_ssid = "Radio-RX";
const char *ap_password = "12345678";

static RadioState radio_state = RADIO_STATE_RX;
static bool ota_enabled = false;
static uint32_t rx_pkt_counter = 0;
static uint32_t ack_sent_counter = 0;
static int16_t last_rssi = 0;
static int8_t last_snr = 0;
static wl_status_t last_wifi_status = WL_DISCONNECTED;

void setupWiFi() {
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
    WiFi.setOutputPower(5.0);  // dBm
    WiFi.softAP(ap_ssid, ap_password);
    WiFi.begin(SSID, PASS);

    Serial.print("Connecting to WiFi");

    if (!MDNS.begin("radio")) {
        Serial.println("Error initiating mDNS");
    }

    Serial.println("\nWiFi AP started: " + String(ap_ssid));
}

void start_ota_updater(void) {
    setupWiFi();
    setupOTA("radio-rx");
    ota_enabled = true;
}

void handle_wifi() {
    wl_status_t wifi_status = WiFi.status();
    if (wifi_status == WL_CONNECTED && last_wifi_status != WL_CONNECTED) {
        Serial.print("WiFi connected! IP address: ");
        Serial.println(WiFi.localIP());
    } else if (last_wifi_status == WL_CONNECTED &&
               wifi_status != WL_CONNECTED) {
        Serial.println("WiFi disconnected!");
    }
    last_wifi_status = wifi_status;
    MDNS.update();
}

void transmit_ack(uint8_t seq_num, int8_t rssi) {
    PacketAck_t packet_ack = {
        .reserved = {0},
        .rssi = rssi,
        .seq_num = seq_num,
    };

    Packet_t packet = {
        .ack = packet_ack,
        .seq_num = seq_num,
        .packet_type = PACKET_TYPE_ACK,
    };

    sx1281_set_rx_buffer_baseaddr(0, 0);
    sx1281_write_buffer(0, (uint8_t *)&packet, PACKET_SIZE);
    sx1281_clear_irq_status(0xFFFF);
    radio_rfamp_tx_enable();
    sx1281_set_mode(SX1280_MODE_TX, 0);
    radio_state = RADIO_STATE_TX_ACK;
    ack_sent_counter++;
}

void print_packet_info(Packet_t *pkt, int16_t rssi, int8_t snr) {
    uint32_t timestamp_ms = millis();

    if (pkt->packet_type == PACKET_TYPE_GPS) {
        // CSV-like format for easy parsing
        Serial.print("GPS,");
        Serial.print(timestamp_ms);
        Serial.print(",");
        Serial.print(pkt->seq_num);
        Serial.print(",");
        Serial.print(pkt->gps.lat * 1e-7, 7);  // Decimal degrees
        Serial.print(",");
        Serial.print(pkt->gps.lon * 1e-7, 7);  // Decimal degrees
        Serial.print(",");
        Serial.print(pkt->gps.sv_num);
        Serial.print(",");
        Serial.print(pkt->gps.fix_type);
        Serial.print(",");
        Serial.print(pkt->gps.fix_ok);
        Serial.print(",");
        Serial.print(rssi);
        Serial.print(",");
        Serial.print(snr);
        Serial.println();
    } else if (pkt->packet_type == PACKET_TYPE_ACK) {
        Serial.print("ACK,");
        Serial.print(timestamp_ms);
        Serial.print(",");
        Serial.print(pkt->seq_num);
        Serial.print(",");
        Serial.print(pkt->ack.rssi);
        Serial.print(",");
        Serial.print(rssi);
        Serial.print(",");
        Serial.print(snr);
        Serial.print(",");
        Serial.print(ack_sent_counter);
        Serial.println();
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n\n=== LoRa RX Starting ===");

    // Initialize SPI
    SPI.begin();
    if (SPI.pins(RADIO_SCK, RADIO_MISO, RADIO_MOSI, RADIO_NSS) == false) {
        Serial.println("ERROR - SPI pin configuration failed!");
        ESP.reset();
    }

    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    pinMode(RADIO_NSS, OUTPUT);
    pinMode(RADIO_RST, OUTPUT);
    pinMode(POWER_RXEN, OUTPUT);
    pinMode(POWER_TXEN, OUTPUT);
    pinMode(RADIO_BUSY, INPUT);
    pinMode(RADIO_DIO1, INPUT);

    sx1281_reset();

    // Verify radio communication
    uint8_t fw_version[2] = {0};
    sx1281_read_registers(REG_LR_FIRMWARE_VERSION_MSB, fw_version, 2);
    uint16_t firmwareRev = (fw_version[1] << 8) | fw_version[0];
    if ((firmwareRev == 0) || (firmwareRev == 65535)) {
        Serial.println(F("ERROR - Failed to read radio version!"));
        while (1);
    }
    Serial.printf("Radio firmware version: 0x%04X\n", firmwareRev);

    // Configure radio (must match TX settings)
    sx1281_set_mode(SX1280_MODE_STDBY_RC, 0);
    sx1281_set_packet_type(SX1280_PACKET_TYPE_LORA);
    sx1281_set_freq_hz(2440000000);
    sx1281_cfg_mod_params_lora(SX1280_LORA_SF9, SX1280_LORA_BW_0200,
                               SX1280_LORA_CR_LI_4_8);

#define PREAMBLE_SIZE 12
    sx1281_set_packet_params_lora(PREAMBLE_SIZE,
                                  SX1280_LORA_PACKET_FIXED_LENGTH, PACKET_SIZE,
                                  SX1280_LORA_CRC_ON, SX1280_LORA_IQ_NORMAL);

    sx1281_set_rx_buffer_baseaddr(0, 0);
    sx1281_set_regulator_mode(SX1280_USE_DCDC);
    sx1281_set_tx_params(SX1280_POWER_MAX, SX1280_RADIO_RAMP_04_US);

    uint16_t dio1_mask = SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE;
    uint16_t irq_mask =
        SX1280_IRQ_TX_DONE | SX1280_IRQ_CRC_ERROR | SX1280_IRQ_RX_DONE;
    sx1281_set_dio_irq_params(irq_mask, dio1_mask, 0, 0);
    sx1281_clear_irq_status(0xFFFF);

    // Start in RX mode
    radio_rfamp_rx_enable();
    sx1281_set_mode(SX1280_MODE_RX_CONT, 0);
    radio_state = RADIO_STATE_RX;

    Serial.println("RX initialized, listening for packets...");
    Serial.println("Send 'OTA' command via serial to enable OTA updates\n");
}

void loop() {

    // If OTA is enabled, prioritize OTA handling
    if (ota_enabled) {
        // Put radio in standby to avoid conflicts during OTA
        // Handle WiFi status
        handle_wifi();
        sx1281_set_mode(SX1280_MODE_STDBY_RC, 0);
        radio_rfamp_rx_enable();
        ArduinoOTA.handle();
        // If OTA is enabled, we return early to avoid processing other tasks
        return;
    }

    // Check for serial commands
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toUpperCase();

        if (cmd == "OTA") {
            Serial.println("Starting OTA updater...");
            start_ota_updater();
        }
    }

    uint16_t irq = sx1281_get_irq_status();

    switch (radio_state) {
        case RADIO_STATE_RX: {
            if (irq & SX1280_IRQ_RX_DONE) {
                // Read received packet
                uint8_t buf[PACKET_SIZE] = {0};
                uint8_t payload_len = 0;
                uint8_t buffer_ptr = 0;

                sx1281_get_rx_buffer_status(&payload_len, &buffer_ptr);
                sx1281_read_buffer(buffer_ptr, buf, PACKET_SIZE);

                // Get packet status (RSSI/SNR)
                SX1280_PacketStatusLoRa_t pkt_status;
                sx1281_get_packet_status_lora(&pkt_status);
                last_rssi = pkt_status.rssi;
                last_snr = pkt_status.snr;

                Packet_t *rx_pkt = (Packet_t *)buf;
                rx_pkt_counter++;

                // Send ACK if it's a GPS packet
                if (rx_pkt->packet_type == PACKET_TYPE_GPS) {
                    transmit_ack(rx_pkt->seq_num, last_rssi);
                } else {
                    // Clear IRQ and continue receiving
                    sx1281_clear_irq_status(irq);
                    radio_rfamp_rx_enable();
                    sx1281_set_mode(SX1280_MODE_RX_CONT, 0);
                }

                // Print packet info to serial
                print_packet_info(rx_pkt, last_rssi, last_snr);
            } else if (irq & SX1280_IRQ_CRC_ERROR) {
                Serial.println("CRC Error detected!");
                sx1281_clear_irq_status(irq);
                radio_rfamp_rx_enable();
                sx1281_set_mode(SX1280_MODE_RX_CONT, 0);
            }
            break;
        }

        case RADIO_STATE_TX_ACK: {
            if (irq & SX1280_IRQ_TX_DONE) {
                sx1281_clear_irq_status(irq);
                radio_rfamp_rx_enable();
                sx1281_set_mode(SX1280_MODE_RX_CONT, 0);
                radio_state = RADIO_STATE_RX;
            }
            break;
        }

        default:
            break;
    }
}
