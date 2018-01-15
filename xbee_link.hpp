#pragma once

#include <Arduino.h>

bool init_xbee_link();
void update_xbee_link();

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
void xbee_write(T &value) {
    char buffer[sizeof(T)];
    memcpy(buffer, &value, sizeof(T));
    Serial1.write(buffer, sizeof(T));
}