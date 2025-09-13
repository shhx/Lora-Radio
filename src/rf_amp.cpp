#include <Arduino.h>
#include "pins.h"
#include "rf_amp.h"

static uint8_t tx_enabled = false;
static uint8_t rx_enabled = false;

void radio_rfamp_tx_enable() {
    if (tx_enabled) {
        return;
    }
    digitalWrite(POWER_TXEN, HIGH);
    digitalWrite(POWER_RXEN, LOW);
    rx_enabled = false;
    tx_enabled = true;
}

void radio_rfamp_rx_enable() {
    if (rx_enabled) {
        return;
    }
    digitalWrite(POWER_TXEN, LOW);
    digitalWrite(POWER_RXEN, HIGH);
    tx_enabled = false;
    rx_enabled = true;
}
