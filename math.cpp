#include "math.hpp"

#include <Arduino.h>

float fmap(float value, float a, float b, float c, float d) {
    float p = (value - a) / (b - a);
    return c + p * (d - c);
}

void vec3::add(vec3 other, vec3 &out) {
    out = vec3(x + other.x, y + other.y, z + other.z);
}

void vec3::add(vec3 other) {
    x += other.x;
    y += other.y;
    z += other.z;
}

vec3 vec3::radd(vec3 other) {
    vec3 out; 
    add(other, out);
    return out;
}

void vec3::sub(vec3 other, vec3 &out) {
    out = vec3(x - other.x, y - other.y, z - other.z);
}

void vec3::sub(vec3 other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
}

vec3 vec3::rsub(vec3 other) {
    vec3 out;
    sub(other, out);
    return out;
}

void vec3::mul(float p, vec3 &out) {
    out = vec3(x * p, y * p, z * p);
}

void vec3::mul(float p) {
    x *= p;
    y *= p;
    z *= p;
}

vec3 vec3::rmul(float p) {
    vec3 out;
    mul(p, out);
    return out;
}

void vec3::reversed(vec3 &out) {
    out = vec3(-x, -y, -z);
}

void vec3::reverse() {
    x = -x;
    y = -y;
    z = -z;
}

vec3 vec3::rreversed() {
    vec3 out;
    reversed(out);
    return out;
}

float vec3::length() {
    return sqrt(x * x + y * y + z * z);
}

void vec3::cross(vec3 other, vec3 &out) {
    out = vec3(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    );
}

vec3 vec3::rcross(vec3 other) {
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

void vec3::normalize() {
    float l = length();

    x /= l;
    y /= l;
    z /= l;
}

vec3 vec3::rnormalized() {
    vec3 out;
    normalized(out);
    return out;
}

quat quat::from_axis_angle(float angle, vec3 axis) {
    return quat(
        cos(angle / 2),
        axis.x * sin(angle / 2),
        axis.y * sin(angle / 2),
        axis.z * sin(angle / 2)
    );
}

void quat::mul(quat other, quat &out) {
    out.q0 = q0 * other.q0 - q1 * other.q1 - q2 * other.q2 - q3 * other.q3;
    out.q1 = q0 * other.q1 + q1 * other.q0 - q2 * other.q3 + q3 * other.q2;
    out.q2 = q0 * other.q2 + q1 * other.q3 + q2 * other.q0 - q3 * other.q1;
    out.q3 = q0 * other.q3 - q1 * other.q2 + q2 * other.q1 + q3 * other.q0;
}

void quat::mul(quat other) {
    mul(other, *this);
}

quat quat::rmul(quat other) {
    quat out;
    mul(other, out);
    return out;
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