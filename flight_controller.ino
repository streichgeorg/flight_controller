#include "config.hpp"
#include "common.hpp"
#include "state.hpp"
#include "leds.hpp"
#include "sensor.hpp"
#include "rc_receiver.hpp"
#include "xbee_link.hpp"
#include "motor.hpp"
#include "kinematics.hpp"
#include "pid.hpp"
#include "math.hpp"

void setup() {
    succesful_initialization = true;

    init_leds();
    init_motors();
    init_rc_receiver();

    init_xbee_link();

    // Take some time to find a connection, if not found
    // we can still try later.
    int xbee_init_time_ms = millis();
    while (millis() - xbee_init_time_ms < 1e3 * 10 && !xbee_link_is_healthy()) {
        TASK(update_connection(), XBEE_UPDATE_RATE_HZ);
    }

    if (!init_sensors()) {
        debug("Failed to initialize sensor");
        succesful_initialization = false;
    }

    if (succesful_initialization) {
        debug("Initialization was succesful");
    } else {
        debug("Initialization was not succesful");
    }

    finished_initialization = true;

    // Entering startup
    start_up_begin_ms = millis();
}

int32_t last_motor_update = 0;
void update_motors() {
    float dt = (micros() - last_motor_update) / 1e6;

    float pitch_target_rad_s, roll_target_rad_s;
    float yaw_target_rad_s = fmap(yaw_channel->get_value(), 0.0, 1.0, -1.0, 1.0);

    if (flight_mode == Flight_Mode::AUTO_LEVEL) {
        float pitch_target_rad = fmap(pitch_channel->get_value(), 0.0, 1.0, -0.3, 0.3);
        float roll_target_rad = fmap(roll_channel->get_value(), 0.0, 1.0, -0.3, 0.3);

        pitch_angle_pid.update(dt, pitch_target_rad, est_pitch_rad);
        roll_angle_pid.update(dt, roll_target_rad, est_roll_rad);

        pitch_target_rad_s = pitch_angle_pid.value;
        roll_target_rad_s = roll_angle_pid.value;
    } else if (flight_mode == Flight_Mode::ACRO) {
        pitch_target_rad_s = fmap(pitch_channel->get_value(), 0.0, 1.0, -1.0, 1.0);
        roll_target_rad_s = fmap(roll_channel->get_value(), 0.0, 1.0, -1.0, 1.0);
    }

    pitch_velocity_pid.update(dt, pitch_target_rad_s, raw_gyro_rad_s.x);
    roll_velocity_pid.update(dt, roll_target_rad_s, raw_gyro_rad_s.y);
    yaw_velocity_pid.update(dt, yaw_target_rad_s, raw_gyro_rad_s.z);

    float throttle = fmap(throttle_channel->get_value(), 0.0, 1.0, MIN_THROTTLE, MAX_THROTTLE);

    float pitch = pitch_velocity_pid.value;
    float roll = roll_velocity_pid.value;
    float yaw = yaw_velocity_pid.value;

    write_motors(
        throttle - pitch - roll + yaw,
        throttle + pitch - roll - yaw,
        throttle - pitch + roll - yaw,
        throttle + pitch + roll + yaw
    );
}

void loop() {
    TASK(update_leds(), LED_UPDATE_RATE_HZ);

    if (!succesful_initialization) {
        return;
    }


    update_xbee_link();

    TASK(TIME_EXPR("imu", update_imu_values()), IMU_RATE_HZ);
    TASK(TIME_EXPR("kinematics", update_kinematics()), KINEMATICS_RATE_HZ);

    if (!started_up && aux0_channel->get_value() > 0.75) {
        TASK(debug("Set the copter to unarmed on the controller"), 1.0);
    }

    if (!started_up && millis() - start_up_begin_ms > STARTUP_TIME_S * 1000 && aux0_channel->get_value() < 0.25) {
        started_up = true;
        init_kinematics();
    }

    if (!started_up) {
        return;
    }

    if (!armed && !arming && aux0_channel->get_value() > 0.75) {
        arming = true;
        arm_begin_ms = millis();

        debug("Begin arming copter");
    }

    if (armed && aux0_channel->get_value() < 0.75) {
        write_motors(0.0, 0.0, 0.0, 0.0);

        armed = false;

        pitch_angle_pid.reset();
        roll_angle_pid.reset();

        pitch_velocity_pid.reset();
        roll_velocity_pid.reset();
        yaw_velocity_pid.reset();

        debug("Copter is Unarmed");
    }

    if (arming && millis() - arm_begin_ms > ARM_DELAY_S * 1000) {
        armed = true;
        arming = false;

        debug("Copter is Armed");
    }

    if (armed) {
        TASK(update_motors(), MOTOR_UPDATE_RATE_HZ);
    }
}