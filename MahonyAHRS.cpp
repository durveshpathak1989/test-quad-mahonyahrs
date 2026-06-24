/**
 * ============================================================
 *  MahonyAHRS.cpp  —  Application  (v4.0)
 *  Adafruit HUZZAH32 Feather (ESP32-WROOM-32E)
 *  SPI (VSPI) — FreeRTOS compatible
 *  Created By - Durvesh Pathak
 * ============================================================
 **/

 #include "MahonyAHRS.h"

MahonyAHRS::MahonyAHRS() {
    reset();
}

void MahonyAHRS::reset() {
    _q0 = 1.0f;
    _q1 = 0.0f;
    _q2 = 0.0f;
    _q3 = 0.0f;

    _ix = 0.0f;
    _iy = 0.0f;
    _iz = 0.0f;
}

void MahonyAHRS::setGains(float kp, float ki) {
    _kp = kp;
    _ki = ki;
}

float MahonyAHRS::_invSqrt(float x) {
    union {
        float f;
        uint32_t i;
    } conv;

    conv.f = x;
    conv.i = 0x5F3759DFul - (conv.i >> 1);
    conv.f *= 1.5f - (x * 0.5f * conv.f * conv.f);

    return conv.f;
}

// Runs Mahony AHRS.
// If s.magOk == true, uses 9-DOF accel + gyro + mag.
// If s.magOk == false, falls back to 6-DOF accel + gyro.

bool MahonyAHRS::update(const MPU_SensorData& s, float dt, AttitudeEstimate& att) {
    // Protect against bad dt on startup or task delay
    if (dt <= 0.0f || dt > 0.05f) {
        dt = 0.001f;
    }

    // Convert gyro from deg/sec to rad/sec
    float gx = s.gx_dps * DEG2RAD;
    float gy = s.gy_dps * DEG2RAD;
    float gz = s.gz_dps * DEG2RAD;

    // Copy accel
    float ax = s.ax_g;
    float ay = s.ay_g;
    float az = s.az_g;

    // Copy mag
    float mx = s.mx_uT;
    float my = s.my_uT;
    float mz = s.mz_uT;

    float ex = 0.0f;
    float ey = 0.0f;
    float ez = 0.0f;

    // ── Accelerometer correction: valid for 6-DOF and 9-DOF ──
    float a2 = ax*ax + ay*ay + az*az;

    if (a2 > 0.0001f) {
        float invA = _invSqrt(a2);
        ax *= invA;
        ay *= invA;
        az *= invA;

        // Estimated gravity direction from quaternion
        float vx = 2.0f * (_q1*_q3 - _q0*_q2);
        float vy = 2.0f * (_q0*_q1 + _q2*_q3);
        float vz = _q0*_q0 - _q1*_q1 - _q2*_q2 + _q3*_q3;

        // Accel error: measured gravity × estimated gravity
        ex += ay*vz - az*vy;
        ey += az*vx - ax*vz;
        ez += ax*vy - ay*vx;

        // ── Magnetometer correction: only if mag vector is valid ──
        float m2 = mx*mx + my*my + mz*mz;

        if (s.magOk && m2 > 0.01f) {
            float invM = _invSqrt(m2);
            mx *= invM;
            my *= invM;
            mz *= invM;

            // Reference direction of Earth's magnetic field
            float hx = 2.0f * (
                mx * (0.5f - _q2*_q2 - _q3*_q3) +
                my * (_q1*_q2 - _q0*_q3) +
                mz * (_q1*_q3 + _q0*_q2)
            );

            float hy = 2.0f * (
                mx * (_q1*_q2 + _q0*_q3) +
                my * (0.5f - _q1*_q1 - _q3*_q3) +
                mz * (_q2*_q3 - _q0*_q1)
            );

            float bx = sqrtf(hx*hx + hy*hy);

            float bz = 2.0f * (
                mx * (_q1*_q3 - _q0*_q2) +
                my * (_q2*_q3 + _q0*_q1) +
                mz * (0.5f - _q1*_q1 - _q2*_q2)
            );

            // Estimated magnetic field direction
            float wx = 2.0f * (
                bx * (0.5f - _q2*_q2 - _q3*_q3) +
                bz * (_q1*_q3 - _q0*_q2)
            );

            float wy = 2.0f * (
                bx * (_q1*_q2 - _q0*_q3) +
                bz * (_q0*_q1 + _q2*_q3)
            );

            float wz = 2.0f * (
                bx * (_q0*_q2 + _q1*_q3) +
                bz * (0.5f - _q1*_q1 - _q2*_q2)
            );

            // Mag error: measured mag × estimated mag
            ex += my*wz - mz*wy;
            ey += mz*wx - mx*wz;
            ez += mx*wy - my*wx;
        }

        // ── Integral feedback ───────────────────────────────
        if (_ki > 0.0f) {
            _ix += _ki * ex * dt;
            _iy += _ki * ey * dt;
            _iz += _ki * ez * dt;
        } else {
            _ix = 0.0f;
            _iy = 0.0f;
            _iz = 0.0f;
        }

        // ── Apply correction to gyro ─────────────────────────
        gx += _kp * ex + _ix;
        gy += _kp * ey + _iy;
        gz += _kp * ez + _iz;
    }

    // ── Integrate quaternion rate ───────────────────────────
    float halfDt = 0.5f * dt;

    float dq0 = (-_q1*gx - _q2*gy - _q3*gz) * halfDt;
    float dq1 = ( _q0*gx + _q2*gz - _q3*gy) * halfDt;
    float dq2 = ( _q0*gy - _q1*gz + _q3*gx) * halfDt;
    float dq3 = ( _q0*gz + _q1*gy - _q2*gx) * halfDt;

    _q0 += dq0;
    _q1 += dq1;
    _q2 += dq2;
    _q3 += dq3;

    // ── Normalize quaternion ────────────────────────────────
    float invQ = _invSqrt(_q0*_q0 + _q1*_q1 + _q2*_q2 + _q3*_q3);

    _q0 *= invQ;
    _q1 *= invQ;
    _q2 *= invQ;
    _q3 *= invQ;

    // ── Quaternion → Euler output ───────────────────────────
    att.q0 = _q0;
    att.q1 = _q1;
    att.q2 = _q2;
    att.q3 = _q3;

    att.roll_deg = RAD2DEG * atan2f(
        2.0f * (_q0*_q1 + _q2*_q3),
        1.0f - 2.0f * (_q1*_q1 + _q2*_q2)
    );

    float sinP = 2.0f * (_q0*_q2 - _q3*_q1);
    sinP = constrain(sinP, -1.0f, 1.0f);

    att.pitch_deg = RAD2DEG * asinf(sinP);

    att.yaw_deg = RAD2DEG * atan2f(
        2.0f * (_q0*_q3 + _q1*_q2),
        1.0f - 2.0f * (_q2*_q2 + _q3*_q3)
    );

    if (att.yaw_deg < 0.0f) {
        att.yaw_deg += 360.0f;
    }

    return true;
}

void MahonyAHRS::getQuaternion(float& q0, float& q1, float& q2, float& q3) const {
    q0 = _q0;
    q1 = _q1;
    q2 = _q2;
    q3 = _q3;
}