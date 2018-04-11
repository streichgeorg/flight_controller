#pragma once

// #define DEBUG
//#define TIME

#define VERSION 1

#define NUM_LEDS 2
#define STATUS_LED 13
#define ARM_LED -1

#define MOTOR_1_PIN 6
#define MOTOR_3_PIN 4
#define MOTOR_0_PIN 23
#define MOTOR_2_PIN 22

#define NUM_RC_CHANNELS 6

#define PWM_SIGNAL
// #define PPM_SIGNAL

#ifdef PWM_SIGNAL
    #define RC_CHANNEL_0_PIN 21 
    #define RC_CHANNEL_1_PIN 20
    #define RC_CHANNEL_2_PIN 17
    #define RC_CHANNEL_3_PIN 16
    #define RC_CHANNEL_4_PIN 15
    #define RC_CHANNEL_5_PIN 14

    const int RC_CHANNEL_PINS[NUM_RC_CHANNELS] = {
        RC_CHANNEL_0_PIN,
        RC_CHANNEL_1_PIN,
        RC_CHANNEL_2_PIN,
        RC_CHANNEL_3_PIN,
        RC_CHANNEL_4_PIN,
        RC_CHANNEL_5_PIN,
    };
#elif PPM_SIGNAL
    #define PPM_INPUT_PIN -1
#endif

#define THROTTLE_CHANNEL 5
#define PITCH_CHANNEL 3
#define ROLL_CHANNEL 2
#define YAW_CHANNEL 4
#define AUX0_CHANNEL 1
#define AUX1_CHANNEL 0 // Broken :(

#define STARTUP_TIME_S 2
#define ARM_DELAY_S 3

#define LED_UPDATE_RATE_HZ 100
#define IMU_RATE_HZ 1000
#define KINEMATICS_RATE_HZ 1000
#define MOTOR_UPDATE_RATE_HZ 1000
#define TELEMETRY_RATE_HZ 100 // TODO: Set this to a higher value, when the connection problems are solved
#define XBEE_UPDATE_RATE_HZ 20

#define XBEE_HEALTH_CHECK_DELAY_S 2.0
#define XBEE_MAX_RESPONSE_TIME_S 1.0
#define XBEE_RESET_DURATION_S 3.0

#define PWM_RESOLUTION 11
// Theoretically from 1024 to 2048
#define PWM_MAX 1900 
#define PWM_MIN 1100

#define ACCEL_OFFSET_X 1415
#define ACCEL_OFFSET_Y 2192
#define ACCEL_OFFSET_Z 1994

// #define MANUAL_GYRO_CALIBRATION

#ifdef MANUAL_GYRO_CALIBRATION
    #define GYRO_OFFSET_X -39
    #define GYRO_OFFSET_Y 23
    #define GYRO_OFFSET_Z 39
#endif

#define DRIFT_CORRECTION_P 0.1
#define DRIFT_CORRECTION_I 0.00001
#define MAX_ACCEL_ACCELERATION 0.1

#define ACCEL_1G 16384
#define GYRO_SENS_DEG (250.0 / 32767.0)

#define PITCH_ROLL_ANGLE_P 0.9
#define PITCH_ROLL_ANGLE_I 0.0
#define PITCH_ROLL_ANGLE_D 0.0

#define PITCH_ROLL_VELOCITY_P 0.35
#define PITCH_ROLL_VELOCITY_I 0.0
#define PITCH_ROLL_VELOCITY_D 0.0

#define YAW_VELOCITY_P 0.1
#define YAW_VELOCITY_I 0.0
#define YAW_VELOCITY_D 0.0

#define RC_MIN 1100.0
#define RC_MAX 1900.0

#define MIN_THROTTLE 0.0
#define MAX_THROTTLE 0.6

#define MAX_MESSAGE_LENGTH 50