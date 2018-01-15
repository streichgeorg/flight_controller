#include "ahrs.hpp"

#include "math.hpp"

void AHRS::update_euler_angles() {
    pitch_rad = rotation.get_pitch_rad();
    roll_rad = rotation.get_roll_rad();
}