#pragma once

#include <Arduino.h>

bool init_xbee_link();
void update_xbee_link();

void send_bytes();

void xbee_debug(char *message);

void send_flight_info();

enum Message_Types: uint8_t {
    FLIGHT_INFO,
    TELEMETRY,
    DEBUG_MSG,
};

template<typename T>
struct Message {
    Message(Message_Types type, T *contents) :
        type(type), contents(contents){};

    Message_Types type;
    T *contents;
};

template<typename T> 
bool xbee_available() {
    return Serial1.available() >= sizeof(T);
}

template<typename T> 
void xbee_read(T *out) {
    char buffer[sizeof(T)];
    Serial1.readBytes(buffer, sizeof(T));

    memcpy(out, buffer, sizeof(T));
}

template<typename T> 
void xbee_send(T &value) {
    char buffer[sizeof(T)];
    memcpy(buffer, &value, sizeof(T));
    Serial1.write(buffer, sizeof(T));
}

template<typename T>
void xbee_send_message(Message<T> message) {
    char buffer[2 + sizeof(T)];
    buffer[0] = message.type;
    buffer[1] = (uint8_t) sizeof(T);
    memcpy(buffer + 2, message.contents, sizeof(T));
    Serial1.write(buffer, 2 + sizeof(T));
}