#include "state.hpp"

#include "config.hpp"
#include "math.hpp"
#include "pid.hpp"

bool succesful_initialization = false;
bool finished_initialization = false;

int start_up_begin_ms;
bool started_up = false;

float arm_begin_ms;
bool arming = false;
bool armed = false;

Flight_Mode flight_mode;

PID pitch_angle_pid(PITCH_ROLL_ANGLE_P, PITCH_ROLL_ANGLE_I, PITCH_ROLL_ANGLE_D);
PID roll_angle_pid(PITCH_ROLL_ANGLE_P, PITCH_ROLL_ANGLE_I, PITCH_ROLL_ANGLE_D);

PID pitch_velocity_pid(PITCH_ROLL_VELOCITY_P, PITCH_ROLL_VELOCITY_I, PITCH_ROLL_VELOCITY_D);
PID roll_velocity_pid(PITCH_ROLL_VELOCITY_P, PITCH_ROLL_VELOCITY_I, PITCH_ROLL_VELOCITY_D);
PID yaw_velocity_pid(YAW_VELOCITY_P, YAW_VELOCITY_I, YAW_VELOCITY_D);