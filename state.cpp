#include "state.hpp"

#include "ahrs.hpp"

bool succesful_initialization = false;
bool finished_initialization = false;

int start_up_begin_ms;
bool started_up = false;

bool arming = false;
float arm_begin_ms;
bool armed = false;

AHRS ahrs;

RC_Channel *throttle_channel;
RC_Channel *pitch_channel;
RC_Channel *roll_channel;
RC_Channel *yaw_channel;
RC_Channel *aux0_channel;
RC_Channel *aux1_channel;