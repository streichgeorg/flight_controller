#include "common.hpp"

#include "config.hpp"
#include "xbee_link.hpp"
#include "math.hpp"

void debug(char *message) {
    if (xbee_link_is_healthy()) {
        send_debug_message(message);
    }
}

void debug_num(char *message, float num) {
    char s[30];
    sprintf(s, message, num);
    debug(s);
}

void debug_vec(char *message, vec3 v) {
    char s[50];
    sprintf(s, message, v.x, v.y, v.z);
    debug(s);
}