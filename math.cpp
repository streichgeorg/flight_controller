#include "math.hpp"

float fmap(float value, float a, float b, float c, float d) {
    float p = (value - a) / (b - a);
    return c + p * (d - c);
}