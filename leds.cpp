#include "leds.hpp"

#include "state.hpp"

#include <Arduino.h>

const int NUM_LEDS = 1;

enum Led_Mode {
    DISABLED,
    ENABLED,
    BLINKING,
};

struct Led {
    Led_Mode mode;

    int pin;

    bool state = false;
    int last_change = 0;
    float freq;
} leds[NUM_LEDS];

int led_count = 0;
Led* init_led(int pin, Led_Mode mode, float freq) {
    leds[led_count].pin = pin;
    leds[led_count].freq = freq;
    leds[led_count].mode = mode;
    pinMode(pin, OUTPUT);

    return &leds[led_count++];
}

Led *status_led;
void init_leds() {
    status_led = init_led(13, BLINKING, 3.5);
}

void update_leds() {
    status_led->mode = (finished_initialization && succesful_initialization) ? BLINKING : DISABLED;

    for (int i = 0; i < led_count; i++) {
        Led &led = leds[i];

        if (led.mode == BLINKING) {
            if (millis() - led.last_change > 1000.0 / led.freq) {
                led.last_change = millis();
                led.state = !led.state;
                digitalWrite(led.pin, led.state);
            }
        } else {
            digitalWrite(led.pin, led.mode == ENABLED);
        }

    }
}