#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <SPI.h>
#include "ArduinoOTA.h"

#include "OTA.h"
#include "packet.h"
#include "pins.h"
#include "rf_amp.h"
#include "sx1281.h"
#include "ublox_protocol.h"
#include "credentials.h"

enum RadioState {
    RADIO_STATE_IDLE = 0,
    RADIO_STATE_RX,
    RADIO_STATE_TX,
    RADIO_STATE_WAIT_ACK,
};

const char* ap_ssid = "Radio-TX";
const char* ap_password = "12345678";
static uint16_t radio_tx_period_ms = 2500;
static uint32_t last_tx_time = 0;
static RadioState radio_state = RADIO_STATE_IDLE;
static bool ota_enabled = false;
// static uint64_t button_press_time = 0;
// static bool tx_waiting_for_ack = false;
static uint32_t tx_pkt_counter = 0;
static uint32_t ack_pkt_counter = 0;
static int16_t last_rssi = 0;

static uint32_t last_ack_received_time = 0;
static uint32_t last_pkt_sent_time = 0;
static double last_distance_m = 0;

// History buffers for plotting (adjust size to balance memory and history depth)
#define STATS_HISTORY_LEN 120 // Save last 120 samples (~4 minutes at 2s updates)
static int16_t rssi_history[STATS_HISTORY_LEN] = {0};
static double distance_history[STATS_HISTORY_LEN] = {0};
static int stats_head = 0;
static bool stats_buffer_full = false;

wl_status_t last_wifi_status = WL_DISCONNECTED;

ubx_nav_pvt_t nav_pvt;

AsyncWebServer server(80);

void start_ota_updater(void) {
    setupOTA("radiotx");
    ota_enabled = true;
}

void setupFS() {
    if (!LittleFS.begin()) {
        Serial.println("LittleFS mount failed");
        ESP.restart();
    }
}

void setupWiFi() {
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
    WiFi.setOutputPower(5.0); // dBm
    WiFi.softAP(ap_ssid, ap_password);  // Start the access point
    WiFi.begin(SSID, PASS);
    Serial.print("Connecting to WiFi");
    if (!MDNS.begin("radio")) {
        Serial.println("Error initiating mDNS");
    }
}

void serveStaticFiles() {
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    server.on("/stats.json", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = "{";
        json += "\"packets_sent\":" + String(tx_pkt_counter) + ",";
        json += "\"packets_acked\":" + String(ack_pkt_counter) + ",";
        json += "\"last_rssi\":" + String(last_rssi) + ",";
        json += "\"distance\":" + String(last_distance_m, 2) + ",";
        json += "\"last_ack\":" + String(last_ack_received_time) + ",";
        json += "\"last_sent\":" + String(last_pkt_sent_time) + ",";
        // Loss percentage
        float loss_pct = 0.0;
        if (tx_pkt_counter > 0) loss_pct = 100.0 * (tx_pkt_counter - ack_pkt_counter) / tx_pkt_counter;
        json += "\"loss_pct\":" + String(loss_pct, 2) + ",";
        // GPS info as before
        json += "\"gps_lat\":" + String(nav_pvt.lat * 1e-7, 7) + ",";
        json += "\"gps_lon\":" + String(nav_pvt.lon * 1e-7, 7) + ",";
        json += "\"gps_numSV\":" + String(nav_pvt.numSV) + ",";
        json += "\"gps_fix_type\":" + String(nav_pvt.fixType) + ",";
        json += "\"gps_fix_ok\":" + String((nav_pvt.flags.gnssFixOK) ? "true" : "false") + ",";
        char utc_time[9];
        uint32_t sec = nav_pvt.iTOW / 1000;
        uint8_t h = (sec / 3600) % 24;
        uint8_t m = (sec % 3600) / 60;
        uint8_t s = sec % 60;
        snprintf(utc_time, sizeof(utc_time), "%02d:%02d:%02d", h, m, s);
        json += "\"gps_utc_time\":\"" + String(utc_time) + "\",";
        // History stats for plot
        json += "\"rssi_hist\":[";
        int samples = stats_buffer_full ? STATS_HISTORY_LEN : stats_head;
        for (int i = 0; i < samples; ++i) {
            int idx = (stats_buffer_full ? (stats_head + i) % STATS_HISTORY_LEN : i);
            json += String(rssi_history[idx]);
            if (i < samples - 1) json += ",";
        }
        json += "],";
        json += "\"distance_hist\":[";
        for (int i = 0; i < samples; ++i) {
            int idx = (stats_buffer_full ? (stats_head + i) % STATS_HISTORY_LEN : i);
            json += String(distance_history[idx], 2);
            if (i < samples - 1) json += ",";
        }
        json += "]";
        json += "}";

        request->send(200, "application/json", json);
    });
    server.on("/reset_stats", HTTP_POST, [](AsyncWebServerRequest *request) {
        tx_pkt_counter = 0;
        ack_pkt_counter = 0;
        last_rssi = 0;
        last_distance_m = 0.0;
        last_ack_received_time = 0;
        last_pkt_sent_time = 0;
        stats_head = 0;
        stats_buffer_full = false;
        for (int i = 0; i < STATS_HISTORY_LEN; ++i) {
            rssi_history[i] = 0;
            distance_history[i] = 0;
        }
        request->send(200, "text/plain", "OK");
    });
    server.on("/current_pos", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = "{";
        json += "\"gps_lat\":" + String(nav_pvt.lat * 1e-7, 7) + ",";
        json += "\"gps_lon\":" + String(nav_pvt.lon * 1e-7, 7) + ",";
        json += "\"gps_numSV\":" + String(nav_pvt.numSV) + ",";
        json += "\"gps_fix_type\":" + String(nav_pvt.fixType) + ",";
        json += "\"gps_fix_ok\":" + String((nav_pvt.flags.gnssFixOK) ? "true" : "false");
        json += "}";
        request->send(200, "application/json", json);
    });
    server.on("/ota", HTTP_POST, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "OK");
        start_ota_updater();
    });
    server.on("/bootloader", HTTP_POST, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "OK");
        delay(100);
        ESP.rebootIntoUartDownloadMode();
    });

    server.begin();
}

double haversine(float lat1, float lon1, float lat2, float lon2) {
    // all in degrees, returns meters
    constexpr double R_EARTH = 6371000.0;
    double phi1 = radians(lat1), phi2 = radians(lat2);
    double dphi = radians(lat2-lat1);
    double dlambda = radians(lon2-lon1);
    double a = sin(dphi/2.0)*sin(dphi/2.0) +
               cos(phi1)*cos(phi2)*sin(dlambda/2.0)*sin(dlambda/2.0);
    double c = 2*atan2(sqrt(a), sqrt(1-a));
    return R_EARTH * c;
}

void transmit_packet(Packet_t *msg) {
    sx1281_set_rx_buffer_baseaddr(0, 0);
    sx1281_write_buffer(0, (uint8_t *)msg, PACKET_SIZE);
    sx1281_clear_irq_status(0xFFFF);
    radio_rfamp_tx_enable();  // ensure TX path enabled
    sx1281_set_mode(SX1280_MODE_TX, 0);
    radio_state = RADIO_STATE_TX;
}

void setup() {
    Serial.begin(115200);
    setupFS();
    setupWiFi();
    serveStaticFiles();

    ublox_init();
    SPI.begin();
    if (SPI.pins(RADIO_SCK, RADIO_MISO, RADIO_MOSI, RADIO_NSS) == false) {
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
    uint8_t fw_version[2] = {0};
    sx1281_read_registers(REG_LR_FIRMWARE_VERSION_MSB, fw_version, 2);
    uint16_t firmwareRev = (fw_version[1] << 8) | fw_version[0];
    if ((firmwareRev == 0) || (firmwareRev == 65535)) {
        Serial.println(F("ERROR - Failed to read radio version!"));
        while (1);
    }
    sx1281_set_mode(SX1280_MODE_STDBY_RC, 0);
    sx1281_set_packet_type(SX1280_PACKET_TYPE_LORA);
    sx1281_set_freq_hz(2440000000);

    sx1281_cfg_mod_params_lora(SX1280_LORA_SF12, SX1280_LORA_BW_0400,
                               SX1280_LORA_CR_LI_4_8);
    sx1281_set_packet_params_lora(PREAMBLE_SIZE,
                                  SX1280_LORA_PACKET_FIXED_LENGTH, PACKET_SIZE,
                                  SX1280_LORA_CRC_ON, SX1280_LORA_IQ_NORMAL);

    sx1281_set_rx_buffer_baseaddr(0, 0);
    sx1281_set_regulator_mode(SX1280_USE_DCDC);
    sx1281_set_tx_params(SX1280_POWER_MAX, SX1280_RADIO_RAMP_04_US);

    uint16_t dio1_mask = SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE;
    uint16_t irq_mask  = SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE | SX1280_IRQ_SYNCWORD_VALID | SX1280_IRQ_SYNCWORD_ERROR | SX1280_IRQ_CRC_ERROR;
    sx1281_set_dio_irq_params(irq_mask, dio1_mask, 0, 0);

    sx1281_clear_irq_status(0xFFFF);

    last_tx_time = millis();
}

void handle_wifi() {
    wl_status_t wifi_status = WiFi.status();
    if (wifi_status == WL_CONNECTED && last_wifi_status != WL_CONNECTED) {
        Serial.print("Wifi connected! IP address: ");
        Serial.println(WiFi.localIP());
    } else if (last_wifi_status == WL_CONNECTED && wifi_status != WL_CONNECTED) {
        Serial.println("Wifi disconnected!!!");
    }
    last_wifi_status = wifi_status;
    MDNS.update();
}

void update_history_buffers(int16_t rssi, double distance) {
    rssi_history[stats_head] = rssi;
    distance_history[stats_head] = distance;
    stats_head++;
    if (stats_head >= STATS_HISTORY_LEN) {
        stats_head = 0;
        stats_buffer_full = true;
    }
}

void loop() {
    handle_wifi();
    if (ota_enabled) {
        // Stop web server to avoid conflicts during OTA
        server.end();
        sx1281_set_mode(SX1280_MODE_STDBY_RC, 0); // ensure in standby for OTA
        radio_rfamp_rx_enable();
        ArduinoOTA.handle();
        // If OTA is enabled, we return early to avoid processing other tasks
        return;
    }

    PacketGPS_t packet_gps = {
        .lon = nav_pvt.lon,
        .lat = nav_pvt.lat,
        .fix_ok = nav_pvt.flags.gnssFixOK,
        .fix_type = nav_pvt.fixType,
        .sv_num = (uint8_t)(nav_pvt.numSV > 15 ? 15 : nav_pvt.numSV), // limit to 15 for 4 bits
    };
    switch (radio_state) {
        case RADIO_STATE_IDLE: {
            if ((millis() - last_tx_time) < radio_tx_period_ms) break;

            // Serial.println("Starting TX");
            tx_pkt_counter++;
            last_pkt_sent_time = millis() / 1000;
            Packet_t packet = {
                .gps = packet_gps,
                .seq_num = (uint8_t)(tx_pkt_counter % 256),
                .packet_type = PACKET_TYPE_GPS,
            };
            transmit_packet(&packet);
            radio_state = RADIO_STATE_TX;
            break;
        }

        case RADIO_STATE_WAIT_ACK: {
            uint16_t irq = sx1281_get_irq_status();
            if (irq & SX1280_IRQ_RX_DONE) {
                uint8_t buf[PACKET_SIZE] = {0};
                sx1281_read_buffer(0, buf, PACKET_SIZE);

                Packet_t *rx_pkt = (Packet_t *)buf;
                bool is_ack = (rx_pkt->packet_type == PACKET_TYPE_ACK);
                if (is_ack) {
                    ack_pkt_counter++;
                    last_ack_received_time = millis() / 1000;
                    // Serial.printf("ACK received, RSSI: %d\n", rx_pkt->ack.rssi);
                    last_rssi = rx_pkt->ack.rssi;
                    update_history_buffers(last_rssi, last_distance_m);
                } else {
                    Serial.println("Received non-ACK packet while waiting for ACK");
                }
                sx1281_clear_irq_status(irq);
                radio_rfamp_rx_enable();
                radio_state = RADIO_STATE_IDLE;
            } else if (millis() - last_tx_time > 1800) {
                Serial.println("ACK timeout");
                sx1281_clear_irq_status(0xFFFF);
                radio_rfamp_rx_enable();
                radio_state = RADIO_STATE_IDLE;
                update_history_buffers(-127, last_distance_m); // Mark lost ACK with low RSSI
            }
            break;
        }

        case RADIO_STATE_TX: {
            uint16_t irq = sx1281_get_irq_status();
            if (irq & SX1280_IRQ_TX_DONE) {
                sx1281_clear_irq_status(irq);
                radio_rfamp_rx_enable();
                sx1281_set_mode(SX1280_MODE_RX_CONT, 0);
                last_tx_time = millis();
                radio_state = RADIO_STATE_WAIT_ACK;
            }
            break;
        }

        case RADIO_STATE_RX:
        default:
            break;
    }
    if (digitalRead(BUTTON_PIN) == LOW) {
        Serial.println("Button pressed, starting OTA updater");
        start_ota_updater();
    }
    ublox_get_new_data();
    if (ublox_parse_msg(&nav_pvt)) {
        if(nav_pvt.flags.gnssFixOK) {
            last_distance_m = haversine(RECEIVER_LAT, RECEIVER_LON,
                                        nav_pvt.lat * 1e-7, nav_pvt.lon * 1e-7);
        }
    }
}
