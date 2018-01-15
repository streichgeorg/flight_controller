#include "leds.hpp"

#include "config.hpp"
#include "state.hpp"

#include <Arduino.h>


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
Led *arm_led;
void init_leds() {
    status_led = init_led(STATUS_LED, ENABLED, 3.5);
    arm_led = init_led(ARM_LED, DISABLED, 3.5);
}

void update_leds() {
    if (finished_initialization && succesful_initialization) {
        if (started_up) {
            status_led->mode = ENABLED;
        } else {
            status_led->mode = BLINKING;
        }
    } else {
        status_led->mode = DISABLED;
    }

    if (arming) {
        arm_led->mode = BLINKING;
    } else if (armed) {
        arm_led->mode = ENABLED;
    } else {
        arm_led->mode = DISABLED;
    }


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