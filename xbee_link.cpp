#include "xbee_link.hpp"

#include "config.hpp"
#include "state.hpp"
#include "math.hpp"

#define HANDSHAKE_TIMEOUT_S 2 

bool init_xbee_link() {
    Serial1.begin(19200);

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

void debug(char* message) {
    uint8_t length = strlen(message);
    uint8_t type = DEBUG_MSG; 

    xbee_send<uint8_t>(type);
    xbee_send<uint8_t>(length);

    xbee_send_bytes(message, length);
}

void debug(String message) {
    uint8_t length = message.length();
    uint8_t type = DEBUG_MSG; 

    xbee_send<uint8_t>(type);
    xbee_send<uint8_t>(length);

    Serial1.print(message);
}

struct __attribute__ ((packed)) Flight_Info {
    uint8_t version = VERSION;

    int32_t gyro_offset_pitch;
    int32_t gyro_offset_roll;
    int32_t gyro_offset_yaw;
};

void send_flight_info() {
    Flight_Info info;

    char debug_str[30];
    sprintf(debug_str, "sizeof(Flight_Info): %d", sizeof(Flight_Info));
    debug(debug_str);

    info.gyro_offset_pitch = gyro_offset_pitch;
    info.gyro_offset_roll = gyro_offset_roll;
    info.gyro_offset_yaw = gyro_offset_yaw;

    xbee_send_message(Message<Flight_Info>(FLIGHT_INFO, &info));
}

struct Telemetry_Frame {
    int16_t raw_gyro_pitch_crad_s;
    int16_t raw_gyro_roll_crad_s;
    int16_t raw_gyro_yaw_crad_s;

    int16_t raw_accel_x;
    int16_t raw_accel_y;
    int16_t raw_accel_z;

    // int16_t pitch_vel_deg_s;
    // int16_t roll_vel_deg_s;
    // int16_t yaw_vel_deg_s;

    // int16_t pitch_deg;
    // int16_t roll_deg;
};

void send_telemetry() {
    Telemetry_Frame frame;
    frame.raw_gyro_pitch_crad_s = raw_gyro_rad_s.x * 100;
    frame.raw_gyro_roll_crad_s = raw_gyro_rad_s.y * 100;
    frame.raw_gyro_yaw_crad_s = raw_gyro_rad_s.z * 100;

    frame.raw_accel_x = raw_accel.x;
    frame.raw_accel_y = raw_accel.y;
    frame.raw_accel_z = raw_accel.z;

    xbee_send_message(Message<Telemetry_Frame>(TELEMETRY, &frame));
}

int last_telemetry_ms = 0;
void update_xbee_link() {
    if (millis() - last_telemetry_ms > 1000.0 / TELEMETRY_FREQ_HZ)  {
        send_telemetry();
        last_telemetry_ms = millis();
    }
}