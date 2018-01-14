#include "leds.hpp"
#include "motor.hpp"

void setup() {
    init_leds();
    init_motors();
}

void loop() {
    update_leds();

    if (millis() > 5000) {
        write_motors(0.5, 0.5, 0.5, 0.5);
    }
}