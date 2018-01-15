#include "config.hpp"
#include "state.hpp"
#include "leds.hpp"
#include "sensor.hpp"
#include "rc_receiver.hpp"
#include "xbee_link.hpp"
#include "motor.hpp"
#include "kinematics.hpp"
#include "math.hpp"

void setup() {
    #ifdef DEBUG
        Serial.begin(9600);
    #endif

    succesful_initialization = true;

    init_leds();

    if (!init_sensors()) {
        #ifdef DEBUG
            Serial.println("Failed to initialize sensor");
        #endif

        succesful_initialization = false;
    }

    init_rc_receiver();

    if (!init_xbee_link()) {
        #ifdef DEBUG
            Serial.println("Failed to initialize xbee link");
        #endif

        succesful_initialization = false;
    }

    init_motors();

    #ifdef DEBUG
        if (succesful_initialization) {
            Serial.println("Initialization was succesful");
        } else {
            Serial.println("Initialization was not succesful");
        }
    #endif

    finished_initialization = true;

    start_up_begin_ms = millis();
}

void begin_arm() {
    arming = true;
    arm_begin_ms = millis();
}

void loop() {
    update_leds();
    update_kinematics();

    if (!succesful_initialization) {
        return;
    }

    update_xbee_link();

    if (!started_up && millis() - start_up_begin_ms > STARTUP_TIME_S * 1000) {
        started_up = true;
    }

    if (!started_up) {
        return;
    }

    if (arming && millis() - arm_begin_ms > ARM_DELAY_S * 1000) {
        arming = false;
    }

    float throttle = fmap(throttle_channel->value, 1000, 2000, 0.0, 1.0);
}