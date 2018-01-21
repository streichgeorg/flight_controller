#pragma once

#include <Arduino.h>

#ifdef TIME
#define TIME_EXPR(name, expr)\
do {\
    static int last_measured_ms = 0;\
\
    if (millis() - last_measured_ms < 1e3) {\
        expr;\
        return;\
    }\
\
    last_measured_ms = millis();\
\
    int start_us = micros();\
    expr;\
    char debug_str[30];\
    sprintf(debug_str, name" took %d us", micros() - start_us);\
    debug(debug_str);\
} while (false)
#else
#define TIME_EXPR(name, expr) expr;
#endif

#define TASK(expr, rate)\
do {\
    static int last_call_us = 0;\
    if (micros() - last_call_us > 1e6 / rate) {\
        last_call_us = micros();\
        expr;\
    }\
} while(false)

struct vec3;

void debug(char *message);
void debug_num(char *message, float num);
void debug_vec(char *message, vec3 v);