#include <Arduino.h>
#include <ESP8266WiFi.h>
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

const char* ap_ssid = "LoraRadioAP";
const char* ap_password = "12345678";
const float RECEIVER_LAT = 40.00000;      // replace with real receiver lat
const float RECEIVER_LON = -3.00000;      // replace with real receiver lon
static uint16_t radio_tx_period_ms = 2000;
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

wl_status_t last_wifi_status = WL_DISCONNECTED;

UBXNavPVT_t nav_pvt;

AsyncWebServer server(80);

void start_ota_updater(void) {
    setupOTA("radio");
    ota_enabled = true;
}

void setupFS() {
    if (!LittleFS.begin()) {
        Serial.println("LittleFS mount failed");
        ESP.restart();
    }
}

void setupWiFiSTA() {
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleepMode(WIFI_LIGHT_SLEEP);
    WiFi.setOutputPower(5.0); // dBm
    WiFi.softAP(ap_ssid, ap_password);  // Start the access point
    WiFi.begin(SSID, PASS);
    Serial.print("Connecting to WiFi");
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

        // Added GPS info - convert lat/lon from 1e-7 int to float degrees:
        json += "\"gps_lat\":" + String(nav_pvt.lat * 1e-7, 7) + ",";
        json += "\"gps_lon\":" + String(nav_pvt.lon * 1e-7, 7) + ",";
        json += "\"gps_numSV\":" + String(nav_pvt.numSV) + ",";
        json += "\"gps_fix_type\":" + String(nav_pvt.fixType) + ",";
        json += "\"gps_fix_ok\":" + String((nav_pvt.flags.gnssFixOK) ? "true" : "false") + ",";

        // Format UTC time HHMMSS from nav_pvt:
        char utc_time[9]; // hh:mm:ss
        uint32_t time_int = nav_pvt.iTOW / 1000; // milliseconds since week start → rough UTC seconds
        uint8_t hours = (time_int / 3600) % 24;
        uint8_t minutes = (time_int % 3600) / 60;
        uint8_t seconds = time_int % 60;
        snprintf(utc_time, sizeof(utc_time), "%02d:%02d:%02d", hours, minutes, seconds);
        json += "\"gps_utc_time\":\"" + String(utc_time) + "\"";

        json += "}";
        request->send(200, "application/json", json);
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
    setupWiFiSTA();
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

    uint16_t dio1_mask = SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE;
    uint16_t irq_mask = SX1280_IRQ_TX_DONE | SX1280_IRQ_CRC_ERROR | SX1280_IRQ_RX_DONE;
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
        Serial.println("Wifi disconected!!!");
    }
    last_wifi_status = wifi_status;
}

void loop() {
    handle_wifi();
    if (ota_enabled) {
        sx1281_set_mode(SX1280_MODE_STDBY_RC, 0); // ensure in standby for OTA
        radio_rfamp_rx_enable();
        ArduinoOTA.handle();
        // If OTA is enabled, we return early to avoid processing other tasks
        return;
    }
    
    PacketGPS_t packet_gps = {
        .lon = nav_pvt.lon,
        .lat = nav_pvt.lat,
        .sv_num = nav_pvt.numSV,
    };

    switch (radio_state) {
        case RADIO_STATE_IDLE: {
            if ((millis() - last_tx_time) < radio_tx_period_ms) break;

            // Prepare to send packet
            Serial.println("Starting TX");
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
                    Serial.printf("ACK received, RSSI: %d\n", rx_pkt->ack.rssi);
                }
                sx1281_clear_irq_status(irq);
                radio_rfamp_rx_enable();
                radio_state = RADIO_STATE_IDLE;
                last_tx_time = millis();
            } else if (millis() - last_tx_time > 200) {
                Serial.println("ACK timeout");
                sx1281_clear_irq_status(0xFFFF);
                radio_rfamp_rx_enable();
                radio_state = RADIO_STATE_IDLE;
                last_tx_time = millis();
            }
            break;
        }

        case RADIO_STATE_TX: {
            uint16_t irq = sx1281_get_irq_status();
            if (irq & SX1280_IRQ_TX_DONE) {
                sx1281_clear_irq_status(irq);
                radio_rfamp_rx_enable();
                sx1281_set_mode(SX1280_MODE_RX_CONT, 0);
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
    ublox_parse_msg(&nav_pvt);
}
