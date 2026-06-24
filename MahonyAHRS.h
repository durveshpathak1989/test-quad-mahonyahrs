/*
 * Name: MahonyAHRS.h
 * Use: Declaration for the Mahony AHRS attitude filter.
 * Version: 4.0.0
 * Created by: Durvesh Pathak dp676@cornell.edu
 */

/**
 * ============================================================
 *  MahonyAHRS.h  —  Mahony Attitude Estimator  (v4.0)
 *  Adafruit HUZZAH32 Feather (ESP32-WROOM-32E)
 *  Created By - Durvesh Pathak
 * ============================================================
 **/

#pragma once
#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H

#include <Arduino.h>
#include <math.h>
#include "../IMU/MPU9250.h"
#include "../AHRS/AHRSCommon.h"


class MahonyAHRS {
public:
    MahonyAHRS();

    void reset();

    void setGains(float kp, float ki);

    // Uses 9-DOF when s.magOk == true.
    // Falls back to 6-DOF when s.magOk == false.
    bool update(const MPU_SensorData& s, float dt, AttitudeEstimate& att);
    bool update(const AHRSInput& in, float dt, AttitudeEstimate& att);

    // Read current internal quaternion.
    // const means this function will not modify the Mahony object.
    void getQuaternion(float& q0, float& q1, float& q2, float& q3) const;

    float kp() const { return _kp; }
    float ki() const { return _ki; }

private:
    float _q0 = 1.0f;
    float _q1 = 0.0f;
    float _q2 = 0.0f;
    float _q3 = 0.0f;

    float _ix = 0.0f;
    float _iy = 0.0f;
    float _iz = 0.0f;

    float _kp = 1.0f;
    float _ki = 0.005f;

    static float _invSqrt(float x);
};

#endif // MAHONY_AHRS_H
