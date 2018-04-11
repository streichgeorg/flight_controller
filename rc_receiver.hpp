#pragma once

#include "config.hpp"

#include <Arduino.h>

struct RC_Channel {
    static int num_channels;
    static RC_Channel channels[NUM_RC_CHANNELS];

    #ifdef PWM_SIGNAL
        volatile bool high;
        volatile int time_rising_edge_us;

        template<int CHANNEL> 
        static void isr() {
            RC_Channel &channel = RC_Channel::channels[CHANNEL];

            if (channel.high) {
                channel.value = micros() - channel.time_rising_edge_us;
            } else {
                channel.time_rising_edge_us = micros();
            }

            channel.high = !channel.high;
        }
    #elif PPM_SIGNAL
        volatile static int current_channel;
        volatile static bool synced = false;
        volatile static int time_rising_edge_us = 0;

        static void isr() {
            int value = micros() - time_rising_edge_us;
            time_rising_edge_us = micros();

            // Sync pulse is apprx. 12ms long.
            if (!synced && abs(value - 12e3) < 100) {
                synced = true;
                current_channel = 0;
            } else if (current_channel < NUM_RC_CHANNELS) {
                channels[current_channel] = value;
            }

            current_channel = (current_channel + 1) % (NUM_RC_CHANNELS + 1);
        }
    #endif

    volatile float value = 0;

    float get_value();
};

extern RC_Channel *throttle_channel;
extern RC_Channel *pitch_channel;
extern RC_Channel *roll_channel;
extern RC_Channel *yaw_channel;
extern RC_Channel *aux0_channel;
extern RC_Channel *aux1_channel;

void init_rc_receiver();