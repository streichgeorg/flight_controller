#include "common.hpp"

#include "config.hpp"
#include "state.hpp"
#include "xbee_link.hpp"
#include "math.hpp"

void debug(char *message) {
    #ifdef DEBUG
        if (has_xbee_link) {
            xbee_debug(message);
        }
    #endif
}

void debug_num(char *message, float num) {
    #ifdef DEBUG
        char s[30];
        sprintf(s, message, num);
        debug(s);
    #endif
}

void debug_vec(char *message, vec3 v) {
    #ifdef DEBUG
        char s[50];
        sprintf(s, message, v.x, v.y, v.z);
        debug(s);
    #endif
}