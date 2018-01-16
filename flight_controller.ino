#include "config.hpp"
#include "common.hpp"
#include "state.hpp"
#include "leds.hpp"
#include "sensor.hpp"
#include "rc_receiver.hpp"
#include "xbee_link.hpp"
#include "motor.hpp"
#include "kinematics.hpp"
#include "math.hpp"

void setup() {
    succesful_initialization = true;

    init_leds();
    init_motors();
    init_rc_receiver();

    has_xbee_link = init_xbee_link();

    if (!init_sensors()) {
        debug("Failed to initialize sensor");
        succesful_initialization = false;
    }

    if (succesful_initialization) {
        debug("Initialization was succesful");
    } else {
        debug("Initialization was not succesful");
    }

    send_flight_info();

    finished_initialization = true;

    start_up_begin_ms = millis();
}

void begin_arm() {
    arming = true;
    arm_begin_ms = millis();
}

void loop() {
    update_leds();

    if (!succesful_initialization) {
        return;
    }

    #ifdef TIME
        int start_imu_update_us = micros();
    #endif
    update_imu_values();
    #ifdef TIME
        {
            char debug_str[30];
            sprintf(debug_str, "IMU update took %d us", micros() - start_imu_update_us);
            debug(debug_str);
        }
    #endif

    #ifdef TIME
        int start_kinematics_update_us = micros();
    #endif
    update_kinematics();
    #ifdef TIME
        {
            char debug_str[30];
            sprintf(debug_str, "Kinematics update took %d us", micros() - start_kinematics_update_us);
            debug(debug_str);
        }
    #endif


    if (has_xbee_link) {
        update_xbee_link();
    }

    if (!started_up && millis() - start_up_begin_ms > STARTUP_TIME_S * 1000) {
        started_up = true;
    }

    if (!started_up) {
        return;
    }

    if (arming && millis() - arm_begin_ms > ARM_DELAY_S * 1000) {
        arming = false;
    }
}