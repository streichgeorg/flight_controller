#include "rc_receiver.hpp"

#include "state.hpp"
#include "math.hpp"
#include "common.hpp"

RC_Channel RC_Channel::channels[];

RC_Channel *throttle_channel = &channels[0];
RC_Channel *pitch_channel = &channels[1];
RC_Channel *roll_channel = &channels[2];
RC_Channel *yaw_channel = &channels[3];
RC_Channel *aux0_channel = &channels[4];
RC_Channel *aux1_channel = &channels[5];

void init_rc_receiver() {
    #ifdef PWM_SIGNAL
        attachInterrupt(RC_CHANNEL_PINS[THROTTLE_CHANNEL], RC_Receiver::isr<THROTTLE_CHANNEL>, CHANGE);
        attachInterrupt(RC_CHANNEL_PINS[PITCH_CHANNEL], RC_Receiver::isr<PITCH_CHANNEL>, CHANGE);
        attachInterrupt(RC_CHANNEL_PINS[ROLL_CHANNEL], RC_Receiver::isr<ROLL_CHANNEL>, CHANGE);
        attachInterrupt(RC_CHANNEL_PINS[YAW_CHANNEL], RC_Receiver::isr<YAW_CHANNEL>, CHANGE);
        attachInterrupt(RC_CHANNEL_PINS[AUX0_CHANNEL], RC_Receiver::isr<AUX0_CHANNEL>, CHANGE);
        attachInterrupt(RC_CHANNEL_PINS[AUX1_CHANNEL], RC_Receiver::isr<AUX1_CHANNEL>, CHANGE);
    #else PPM_SIGNAL
        attachInterrupt(PPM_INPUT_PIN, RC_Receiver::isr, RISING);
    #endif
}

float RC_Channel::get_value() {
    return fmap(value, RC_MIN, RC_MAX, 0.0, 1.0);
}