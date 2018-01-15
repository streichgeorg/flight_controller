#include "rc_receiver.hpp"

#include "state.hpp"

RC_Channel RC_Channel::channels[];

void init_rc_receiver() {
    init_channel<THROTTLE_CHANNEL>(throttle_channel);
    init_channel<PITCH_CHANNEL>(pitch_channel);
    init_channel<ROLL_CHANNEL>(roll_channel);
    init_channel<YAW_CHANNEL>(yaw_channel);
    init_channel<AUX0_CHANNEL>(aux0_channel);
    init_channel<AUX1_CHANNEL>(aux1_channel);
}