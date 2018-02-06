#pragma once

#include <Arduino.h>

void init_xbee_link();
void update_xbee_link();
void update_connection();

void send_flight_info();
void send_debug_message(char *message);

bool xbee_link_is_healthy();
bool xbee_link_is_resetting();
bool xbee_link_is_connecting();

namespace Error {
    enum uint8_t {
        TIMEOUT,
        INVALID_MESSAGE_TYPE,
        INVALID_MESSAGE_LENGTH,
        INVALID_RESPONSE_CODE,
        RESPONSE_WHILE_NOT_PENDING,
    };
}

namespace TX_Message_Type {
    enum uint8_t {
        FLIGHT_INFO,
        TELEMETRY,
        DEBUG_MSG,
        HEALTH_CHECK,
        ERROR,
    };
}

namespace RX_Message_Type {
    enum uint8_t {
        HEALTH_CHECK_RESPONSE = 5,
        LAST_VALUE = 6,
    };
}

template<typename T> 
void send(T value) {
    char buffer[sizeof(T)];
    memcpy(buffer, &value, sizeof(T));
    Serial1.write(buffer, sizeof(T));
}

template<typename T>
void send_message(uint8_t type, T value) {
    char buffer[2 + sizeof(T)];
    buffer[0] = type;
    buffer[1] = (uint8_t) sizeof(T);
    memcpy(buffer + 2, &value, sizeof(T));
    Serial1.write(buffer, 2 + sizeof(T));
}

template<typename T>
void read(T *out) {
    char buffer[sizeof(T)];
    Serial1.readBytes(buffer, sizeof(T));
    memcpy(out, buffer, sizeof(T));
}