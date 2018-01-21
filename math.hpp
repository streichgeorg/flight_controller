#pragma once

float fmap(float value, float a, float b, float c, float d);

struct vec3 {
    float x, y, z;

    vec3() : x(0.0), y(0.0), z(0.0){};
    vec3(float x, float y, float z) : x(x), y(y), z(z){};

    void add(vec3 other, vec3 &out);
    void add(vec3 other);
    vec3 radd(vec3 other);

    void sub(vec3 other, vec3 &out);
    void sub(vec3 other);
    vec3 rsub(vec3 other);

    void mul(float p, vec3 &out);
    void mul(float p);
    vec3 rmul(float p);

    void reversed(vec3 &out);
    void reverse();
    vec3 rreversed();

    float length();

    void cross(vec3 other, vec3 &out);
    vec3 rcross(vec3 other);

    float dot(vec3 other);

    void normalized(vec3 &out);
    vec3 rnormalized();
    void normalize();
};

struct quat {
    float q0, q1, q2, q3;

    quat() : q0(0.0), q1(0.0), q2(0.0), q3(0.0){};
    quat(float q0, float q1, float q2, float q3) : q0(q0), q1(q1), q2(q2), q3(q3){};

    static quat from_axis_angle(float angle, vec3 axis);

    void conjugate();
    quat rconjugate();

    void mul(quat other, quat &out);
    void mul(quat other);
    quat rmul(quat other);

    void normalized(quat &out);
    void normalize();

    float get_pitch_rad();
    float get_roll_rad();
    float get_yaw_rad();
};