#include "xbee_link.hpp"

#include "config.hpp"
#include "state.hpp"
#include "sensor.hpp"
#include "kinematics.hpp"
#include "pid.hpp"
#include "math.hpp"

#define HEALTH_CHECK_CODE "begin" 
#define HEALTH_CHECK_CODE_LENGTH 5

enum class Connection_Status {
    HEALTHY,
    RESETTING,
    CONNECTING,
};

Connection_Status connection_status;

bool xbee_link_is_healthy() {
    return connection_status == Connection_Status::HEALTHY;
}

bool xbee_link_is_resetting() {
    return connection_status == Connection_Status::RESETTING;
}

bool xbee_link_is_connecting() {
    return connection_status == Connection_Status::CONNECTING;
}

void send_bytes(char *bytes, int count) {
    Serial1.write(bytes, count);
}

void read_bytes(char *bytes, int count) {
    Serial1.readBytes(bytes, count);
}

int time_code_sent_ms;
void init_xbee_link() {
    Serial1.begin(115200);

    connection_status = Connection_Status::CONNECTING;
    send_bytes(HEALTH_CHECK_CODE, HEALTH_CHECK_CODE_LENGTH);
    time_code_sent_ms = millis();
}

int available() {
    return Serial1.available();
}

void handle_health_check_response(char *response);
void reset_xbee_link();

bool read_type = false;
uint8_t message_type;
bool read_length = false;
uint8_t message_length;
// Try to read a message. If something is wrong reset the connection.
void read_message() {
    if (available() < 0) {
        return;
    }

    if (!read_type) {
        read(&message_type);

        if (message_type >= RX_Message_Type::LAST_VALUE) {
            reset_xbee_link();
            return;
        }

        read_type = true;

        if (available() == 0) {
            return;
        }
    }

    if (!read_length) {
        read(&message_length);
        if (message_length > MAX_MESSAGE_LENGTH) {
            reset_xbee_link();
            return;
        }

        read_length = true;

        if (available() == 0) {
            return;
        }
    }

    if (available() < message_length) {
        return;
    }

    switch(message_type) {
        case RX_Message_Type::HEALTH_CHECK_RESPONSE: {
            char response[HEALTH_CHECK_CODE_LENGTH];
            read_bytes(response, HEALTH_CHECK_CODE_LENGTH);
            send<uint8_t>(TX_Message_Type::DEBUG_MSG);
            send<uint8_t>(HEALTH_CHECK_CODE_LENGTH);
            send_bytes(response, HEALTH_CHECK_CODE_LENGTH);
            handle_health_check_response(response);
            break;
        }
        default: {
        }
    }
}

void send_debug_message(char* message) {
    uint8_t length = strlen(message);

    send<uint8_t>(TX_Message_Type::DEBUG_MSG);
    send<uint8_t>(length);

    send_bytes(message, length);
}

struct __attribute__ ((packed)) Flight_Info {
    uint8_t version = VERSION;
};

void send_flight_info() {
    Flight_Info info;
    send_message(TX_Message_Type::FLIGHT_INFO, info);
}

struct __attribute__ ((packed)) Telemetry_Frame {
    uint32_t time_stamp_ms;

    int16_t raw_gyro_pitch_crad_s;
    int16_t raw_gyro_roll_crad_s;
    int16_t raw_gyro_yaw_crad_s;

    int16_t raw_accel_x;
    int16_t raw_accel_y;
    int16_t raw_accel_z;

    int16_t pitch_mrad;
    int16_t roll_mrad;

    int16_t pitch_angle_pid;
    int16_t roll_angle_pid;

    int16_t pitch_velocity_pid;
    int16_t roll_velocity_pid;
    int16_t yaw_velocity_pid;
};

void send_telemetry() {
    Telemetry_Frame frame;

    frame.time_stamp_ms = millis();

    frame.raw_gyro_pitch_crad_s = raw_gyro_rad_s.x * 100;
    frame.raw_gyro_roll_crad_s = raw_gyro_rad_s.y * 100;
    frame.raw_gyro_yaw_crad_s = raw_gyro_rad_s.z * 100;

    frame.raw_accel_x = raw_accel.x;
    frame.raw_accel_y = raw_accel.y;
    frame.raw_accel_z = raw_accel.z;

    frame.pitch_mrad = est_pitch_rad * 1000;
    frame.roll_mrad = est_roll_rad * 1000;

    frame.pitch_angle_pid = pitch_angle_pid.value * 1000;
    frame.roll_angle_pid = roll_angle_pid.value * 1000;

    frame.pitch_velocity_pid = pitch_velocity_pid.value * 1000;
    frame.roll_velocity_pid = roll_velocity_pid.value * 1000;
    frame.yaw_velocity_pid = yaw_velocity_pid.value * 1000;

    send_message(TX_Message_Type::TELEMETRY, frame);
}

int reset_start_ms;
void reset_xbee_link() {
    connection_status = Connection_Status::RESETTING;
    reset_start_ms = millis();

    // Clear input buffer
    // TODO: Protect from infinite loop
    while (Serial1.available()) {
        Serial1.read();
    }
}


bool health_check_pending = false;
int health_check_sent_ms;
int last_health_check_ms = 0;
void update_health_check() {
    if (health_check_pending && millis() - health_check_sent_ms > XBEE_MAX_RESPONSE_TIME_S * 1e3) {
        health_check_pending = false;
        last_health_check_ms = 0;

        reset_xbee_link();
    } else if (!health_check_pending && millis() - last_health_check_ms > XBEE_HEALTH_CHECK_DELAY_S * 1e3) {
        send<uint8_t>(TX_Message_Type::HEALTH_CHECK);
        send<uint8_t>(HEALTH_CHECK_CODE_LENGTH);
        send_bytes(HEALTH_CHECK_CODE, HEALTH_CHECK_CODE_LENGTH);

        health_check_pending = true;
        health_check_sent_ms = millis();
    }
}

void handle_health_check_response(char* response) {
    if (!health_check_pending) {
        send_debug_message("not pending");
        reset_xbee_link();
    } else if (memcmp(response, HEALTH_CHECK_CODE, HEALTH_CHECK_CODE_LENGTH) == 0) {
        send_debug_message("succ");
        last_health_check_ms = millis();
    } else {
        send_debug_message("fail");
        reset_xbee_link();
    }
}

void update_xbee_link() {
    switch (connection_status) {
        case Connection_Status::HEALTHY: {
            send_telemetry();
            update_health_check();
            read_message();
            break;
        }

        case Connection_Status::RESETTING: {
            if (millis() - reset_start_ms > XBEE_RESET_DURATION_S * 1e3) {
                connection_status = Connection_Status::CONNECTING;

                send_bytes(HEALTH_CHECK_CODE, HEALTH_CHECK_CODE_LENGTH);
                time_code_sent_ms = millis();
            }

            break;
        }

        case Connection_Status::CONNECTING: {
            if (available() >= HEALTH_CHECK_CODE_LENGTH) {
                char response[HEALTH_CHECK_CODE_LENGTH];
                read_bytes(response, HEALTH_CHECK_CODE_LENGTH);

                if (memcmp(response, HEALTH_CHECK_CODE, HEALTH_CHECK_CODE_LENGTH) == 0) {
                    last_health_check_ms = millis();
                    connection_status = Connection_Status::HEALTHY;
                } else {
                    reset_xbee_link();
                }
            } else if (millis() - time_code_sent_ms > XBEE_MAX_RESPONSE_TIME_S * 1e3) {
                reset_xbee_link();
            }

            break;
        }
    }
}