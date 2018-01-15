#include "xbee_link.hpp"

bool init_xbee_link() {
    Serial1.begin(19200);

    // TODO: Handshake

    return true;
}