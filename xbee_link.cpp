#include "xbee_link.hpp"

#include "config.hpp"
#include "state.hpp"
#include "sensor.hpp"
#include "kinematics.hpp"
#include "math.hpp"

#define HANDSHAKE_TIMEOUT_S 2 

bool init_xbee_link() {
    Serial1.begin(115200);

    int32_t msg = 0x01020304;
    xbee_send<int32_t>(msg);

    int started_waiting = millis();
    while (!xbee_available<int32_t>()) {
        if (millis() - started_waiting > HANDSHAKE_TIMEOUT_S * 1000) {
            return false;
        }
    }

    int32_t value;
    xbee_read(&value);

    return value == 0x01020304;
}

void xbee_send_bytes(char *bytes, int count) {
    Serial1.write(bytes, count);
}

void xbee_debug(char* message) {
    uint8_t length = strlen(message);
    uint8_t type = DEBUG_MSG; 

    xbee_send<uint8_t>(type);
    xbee_send<uint8_t>(length);

    xbee_send_bytes(message, length);
}

struct __attribute__ ((packed)) Flight_Info {
    uint8_t version = VERSION;
};

void send_flight_info() {
    Flight_Info info;
    xbee_send_message(Message<Flight_Info>(FLIGHT_INFO, &info));
}

struct __attribute__ ((packed)) Telemetry_Frame {
    uint32_t time_stamp;

    int16_t raw_gyro_pitch_crad_s;
    int16_t raw_gyro_roll_crad_s;
    int16_t raw_gyro_yaw_crad_s;

    int16_t raw_accel_x;
    int16_t raw_accel_y;
    int16_t raw_accel_z;

    int16_t pitch_mrad;
    int16_t roll_mrad;
};

void send_telemetry() {
    Telemetry_Frame frame;

    frame.time_stamp = millis();

    frame.raw_gyro_pitch_crad_s = raw_gyro_rad_s.x * 100;
    frame.raw_gyro_roll_crad_s = raw_gyro_rad_s.y * 100;
    frame.raw_gyro_yaw_crad_s = raw_gyro_rad_s.z * 100;

    frame.raw_accel_x = raw_accel.x;
    frame.raw_accel_y = raw_accel.y;
    frame.raw_accel_z = raw_accel.z;

    frame.pitch_mrad = est_pitch_rad * 1000;
    frame.roll_mrad = est_roll_rad * 1000;

    xbee_send_message(Message<Telemetry_Frame>(TELEMETRY, &frame));
}

void update_xbee_link() {
    send_telemetry();
}