#include "OTA.h"
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include "credentials.h"

static bool lastLEDState = LOW;

void setupWiFiOTA() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASS);
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASS);
    WiFi.setOutputPower(0);  // max power
    WiFi.setSleep(false);    // disable sleep mode
    Serial.print("Connecting to WiFi");
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
    }
    Serial.print("Connected, IP address: ");
    Serial.println(WiFi.localIP());
}

void setupOTA(const char* nameprefix) {
    const int maxlen = 40;
    char      fullhostname[maxlen];
    uint8_t   mac[6];
    WiFi.macAddress(mac);
    snprintf(fullhostname, maxlen, "%s-%02x%02x%02x", nameprefix, mac[3], mac[4], mac[5]);
    ArduinoOTA.setHostname(fullhostname);

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else  // U_SPIFFS
            type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        digitalWrite(LED_BUILTIN, lastLEDState); // blink LED during OTA
        lastLEDState = !lastLEDState;
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
            Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
            Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
            Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
            Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
            Serial.println("End Failed");
    });

    ArduinoOTA.begin();

    Serial.println("OTA Initialized");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.println();
}
