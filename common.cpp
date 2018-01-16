#include "common.hpp"

#include "state.hpp"
#include "xbee_link.hpp"

void debug(char *message) {
    #ifdef DEBUG
        if (has_xbee_link) {
            xbee_debug(message);
        }
    #endif
}

void debug(String message) {
    #ifdef DEBUG
        if (has_xbee_link) {
            xbee_debug(message);
        }
    #endif
}