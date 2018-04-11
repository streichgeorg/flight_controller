#include "math.hpp"

#include <Arduino.h>

float fmap(float value, float a, float b, float c, float d) {
    float p = (value - a) / (b - a);
    return c + p * (d - c);
}

void vec3::add(vec3 other, vec3 &out) {
    out.x = this->x + other.x;
    out.y = this->y + other.y;
    out.z = this->z + other.z;
}

vec3& vec3::operator+=(const vec3 &rhs) {
    this->add(rhs, *this);
    return *this;
}

vec3 vec3::operator+(const vec3 &rhs) {
    vec3 out;
    this->add(rhs, out);
    return out;
}

void vec3::sub(vec3 other, vec3 &out) {
    out.x = this->x - other.x;
    out.y = this->y - other.y;
    out.z = this->z - other.z;
}

vec3& vec3::operator-=(const vec3 &rhs) {
    this->sub(rhs, *this);
    return *this;
}

vec3 vec3::operator-(const vec3 &rhs) {
    vec3 out;
    this->sub(rhs, out);
    return out;
}

void vec3::mul(float p, vec3 &out) {
    out.x = this->x * p;
    out.y = this->y * p;
    out.z = this->z * p;
}

vec3& vec3::operator*=(float rhs) {
    this->mul(rhs, *this);
    return *this;
}

vec3 vec3::operator*(float rhs) {
    vec3 out;
    this->mul(rhs, out);
    return out;
}

void vec3::div(float p, vec3 &out) {
    this->mul(1 / p, out);
}

vec3& vec3::operator/=(float rhs) {
    this->div(rhs, *this);
    return *this;
}

vec3 vec3::operator/(float rhs) {
    vec3 out;
    this->div(rhs, out);
    return out;
}

void vec3::reversed(vec3 &out) {
    out.x = -x;
    out.y = -y;
    out.z = -z;
}

vec3 vec3::operator-() {
    vec3 out;
    this->reversed(out);
    return out;
}

float vec3::length() {
    return sqrt(x * x + y * y + z * z);
}

void vec3::cross(vec3 other, vec3 &out) {
    out.x = y * other.z - z * other.y;
    out.y = z * other.x - x * other.z;
    out.z = x * other.y - y * other.x;
}

vec3 vec3::cross(vec3 other) {
    vec3 out;
    cross(other, out);
    return out;
}

float vec3::dot(vec3 other) {
    return x * other.x + y * other.y + z * other.z;
}

void vec3::normalized(vec3 &out) {
    float l = length();
    out = vec3(x / l, y / l, z / l);
}

vec3 vec3::normalized() {
    vec3 out;
    normalized(out);
    return out;
}

void vec3::normalize() {
    float l = length();

    x /= l;
    y /= l;
    z /= l;
}

quat quat::from_axis_angle(float angle, vec3 axis) {
    return quat(
        cos(angle / 2),
        axis.x * sin(angle / 2),
        axis.y * sin(angle / 2),
        axis.z * sin(angle / 2)
    );
}

void quat::normalized(quat &out) {
    float p = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    out.q0 /= p;
    out.q1 /= p;
    out.q2 /= p;
    out.q3 /= p;
}

void quat::normalize() {
    normalized(*this);
}

float quat::get_pitch_rad() {
    return atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
}

float quat::get_roll_rad() {
    return asin(2 * (q0 * q2 - q1 * q3));
}

float quat::get_yaw_rad() {
    return atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
}