#include "rc_receiver.hpp"

#include "state.hpp"
#include "math.hpp"
#include "common.hpp"

RC_Channel RC_Channel::channels[];

RC_Channel *throttle_channel;
RC_Channel *pitch_channel;
RC_Channel *roll_channel;
RC_Channel *yaw_channel;
RC_Channel *aux0_channel;
RC_Channel *aux1_channel;

void init_rc_receiver() {
    init_channel<THROTTLE_CHANNEL>(throttle_channel);
    init_channel<PITCH_CHANNEL>(pitch_channel);
    init_channel<ROLL_CHANNEL>(roll_channel);
    init_channel<YAW_CHANNEL>(yaw_channel);
    init_channel<AUX0_CHANNEL>(aux0_channel);
    init_channel<AUX1_CHANNEL>(aux1_channel);
}

float RC_Channel::get_value() {
    if (value > RC_IGNORE_MIN && value < RC_IGNORE_MAX)  {
        return 0.5;
    } else if (value < RC_IGNORE_MIN) {
        return fmap(value, RC_MIN, RC_IGNORE_MIN, 0.0, 0.5);
    } else {
        return fmap(value, RC_IGNORE_MAX, RC_MAX, 0.5, 1.0);
    }
}