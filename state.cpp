#include "state.hpp"

#include "math.hpp"
#include "ahrs.hpp"

bool succesful_initialization = false;
bool finished_initialization = false;

bool has_xbee_link;

int start_up_begin_ms;
bool started_up = false;

bool arming = false;
float arm_begin_ms;
bool armed = false;

int gyro_offset_pitch, gyro_offset_roll, gyro_offset_yaw;
vec3 raw_gyro_rad_s;
vec3 raw_accel;

AHRS ahrs;

RC_Channel *throttle_channel;
RC_Channel *pitch_channel;
RC_Channel *roll_channel;
RC_Channel *yaw_channel;
RC_Channel *aux0_channel;
RC_Channel *aux1_channel;