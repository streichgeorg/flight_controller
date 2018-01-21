#include "state.hpp"

#include "math.hpp"

bool succesful_initialization = false;
bool finished_initialization = false;

bool has_xbee_link;

int start_up_begin_ms;
bool started_up = false;

bool arming = false;
float arm_begin_ms;
bool armed = false;