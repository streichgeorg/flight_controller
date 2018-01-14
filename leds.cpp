#include "leds.hpp"

#include <Arduino.h>

const int NUM_LEDS = 1;

struct Led_State {
    int pin;
    bool state = false;
    int last_change = 0;
    float freq;
    bool enabled = true;
} leds[NUM_LEDS];

int led_count = 0;
Led_State* init_led(int pin, float freq) {
    leds[led_count].pin = pin;
    leds[led_count].freq = freq;
    pinMode(pin, OUTPUT);

    return &leds[led_count++];
}

void init_leds() {
    init_led(13, 3);
}

void update_leds() {
    for (int i = 0; i < led_count; i++) {
        if (millis() - leds[i].last_change > 1000.0 / leds[i].freq) {
            leds[i].last_change = millis();
            leds[i].state = !leds[i].state;
            digitalWrite(leds[i].pin, leds[i].state && leds[i].enabled);
        }
    }
}