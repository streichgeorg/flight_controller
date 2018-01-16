#include "ahrs.hpp"

#include "math.hpp"

void AHRS::update_euler_angles() {
    est_pitch_rad = rotation.get_pitch_rad();
    est_roll_rad = rotation.get_roll_rad();
}