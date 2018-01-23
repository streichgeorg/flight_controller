#include "motor.hpp"

#include "config.hpp"
#include "math.hpp"

#include <Arduino.h>

void init_motor(int pin) {
    pinMode(pin, OUTPUT);
}

void init_motors() {
    analogWriteResolution(PWM_RESOLUTION);

    init_motor(MOTOR_0_PIN);
    init_motor(MOTOR_1_PIN);
    init_motor(MOTOR_2_PIN);
    init_motor(MOTOR_3_PIN);

    write_motors(0.0, 0.0, 0.0, 0.0);
}

void write_motors(float motor_0, float motor_1, float motor_2, float motor_3) {
    analogWrite(MOTOR_0_PIN, (int) fmap(max(min(motor_0, 1.0), 0.0), 0.0, 1.0, PWM_MIN, PWM_MAX));
    analogWrite(MOTOR_1_PIN, (int) fmap(max(min(motor_1, 1.0), 0.0), 0.0, 1.0, PWM_MIN, PWM_MAX));
    analogWrite(MOTOR_2_PIN, (int) fmap(max(min(motor_2, 1.0), 0.0), 0.0, 1.0, PWM_MIN, PWM_MAX));
    analogWrite(MOTOR_3_PIN, (int) fmap(max(min(motor_3, 1.0), 0.0), 0.0, 1.0, PWM_MIN, PWM_MAX));
}

void write_motors_raw(int motor_0, int motor_1, int motor_2, int motor_3) {
    analogWrite(MOTOR_0_PIN, motor_0);
    analogWrite(MOTOR_1_PIN, motor_1);
    analogWrite(MOTOR_2_PIN, motor_2);
    analogWrite(MOTOR_3_PIN, motor_3);
}