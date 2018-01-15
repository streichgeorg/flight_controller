#include "state.hpp"
#include "leds.hpp"
#include "motor.hpp"
#include "kinematics.hpp"
#include "sensor.hpp"

#define DEBUG

void setup() {
    #ifdef DEBUG
        Serial.begin(9600);
    #endif

    bool failed_initialization = false;

    init_leds();

    if (!init_sensors()) {
        #ifdef DEBUG
            Serial.println("Failed to initialize sensor");
        #endif
        failed_initialization = true;
    }

    init_motors();

    succesful_initialization = !failed_initialization;
    finished_initialization = true;
}

int16_t ax, ay, az, gx, gy, gz;

void loop() {
    update_leds();
}