#pragma once

#include "config.hpp"

#include <Arduino.h>

struct RC_Channel {
    static int num_channels;
    static RC_Channel channels[NUM_RC_CHANNELS];

    volatile bool high;
    volatile int time_rising_edge_us;
    volatile float value = 0;

    void isr() {
        if (high) {
            value = micros() - time_rising_edge_us;
        } else {
            time_rising_edge_us = micros();
        }

        high = !high;
    }

    float get_value();
};

extern RC_Channel *throttle_channel;
extern RC_Channel *pitch_channel;
extern RC_Channel *roll_channel;
extern RC_Channel *yaw_channel;
extern RC_Channel *aux0_channel;
extern RC_Channel *aux1_channel;

template<int CHANNEL> 
void isr() {
    RC_Channel::channels[CHANNEL].isr();
}

template<int CHANNEL>
void init_channel(RC_Channel *&out) {
    attachInterrupt(RC_CHANNEL_PINS[CHANNEL], isr<CHANNEL>, CHANGE);

    out = &RC_Channel::channels[CHANNEL];
}

void init_rc_receiver();