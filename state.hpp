#pragma once

extern bool succesful_initialization;
extern bool finished_initialization;

extern int start_up_begin_ms;
extern bool started_up;

extern bool arming;
extern float arm_begin_ms;
extern bool armed;

enum class Flight_Mode {
    ACRO,
    AUTO_LEVEL
};
extern Flight_Mode flight_mode;

struct PID;
extern PID pitch_angle_pid;
extern PID roll_angle_pid;

extern PID pitch_velocity_pid;
extern PID roll_velocity_pid;
extern PID yaw_velocity_pid;