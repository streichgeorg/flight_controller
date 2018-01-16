#pragma once

extern bool succesful_initialization;
extern bool finished_initialization;

extern bool has_xbee_link;

extern int start_up_begin_ms;
extern bool started_up;

extern bool arming;
extern float arm_begin_ms;
extern bool armed;

struct vec3;
extern vec3 raw_gyro_rad_s;
extern vec3 raw_accel;

struct AHRS;
extern AHRS ahrs;

struct RC_Channel;
extern RC_Channel *throttle_channel;
extern RC_Channel *pitch_channel;
extern RC_Channel *roll_channel;
extern RC_Channel *yaw_channel;
extern RC_Channel *aux0_channel;
extern RC_Channel *aux1_channel;