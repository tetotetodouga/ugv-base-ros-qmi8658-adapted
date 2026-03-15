#pragma once

#include <Wire.h>
#include <QMI8658.h>
#include <math.h>

QMI8658 myIMU;

float imu_bias_gx = 0.0f;
float imu_bias_gy = 0.0f;
float imu_bias_gz = 0.0f;

float imu_bias_ax = 0.0f;
float imu_bias_ay = 0.0f;
float imu_bias_az = 0.0f;

float imu_bias_cx = 0.0f;
float imu_bias_cy = 0.0f;
float imu_bias_cz = 0.0f;

bool imu_ok = false;

static void imu_compute_rp_from_accel(float axv, float ayv, float azv) {
    icm_roll  = atan2(ayv, azv);
    icm_pitch = atan2(-axv, sqrt(ayv * ayv + azv * azv));
    icm_yaw   = 0.0f;

    q0 = 1.0;
    q1 = 0.0;
    q2 = 0.0;
    q3 = 0.0;
}

void imu_init() {
    Serial.println("IMU INIT START");

    if (!myIMU.begin(32, 33, QMI8658_ADDRESS_HIGH)) {
        Serial.println("IMU FAIL: QMI8658 begin");
        imu_ok = false;
        return;
    }

    imu_ok = true;
    Serial.println("IMU OK: QMI8658");
}

void updateIMUData() {
    if (!imu_ok) return;

    float raw_ax = 0.0f, raw_ay = 0.0f, raw_az = 0.0f;
    float raw_gx = 0.0f, raw_gy = 0.0f, raw_gz = 0.0f;

    bool okA = myIMU.readAccelMPS2(raw_ax, raw_ay, raw_az);
    bool okG = myIMU.readGyroRADS(raw_gx, raw_gy, raw_gz);

    if (!okA && !okG) {
        return;
    }

    if (okA) {
        ax = raw_ax;
        ay = raw_ay;
        az = raw_az;

        icm_roll  = atan2(ay, az);
        icm_pitch = atan2(-ax, sqrt(ay * ay + az * az));
        icm_yaw   = 0.0f;

        q0 = 1.0;
        q1 = 0.0;
        q2 = 0.0;
        q3 = 0.0;
    }

    if (okG) {
        gx = raw_gx - imu_bias_gx;
        gy = raw_gy - imu_bias_gy;
        gz = raw_gz - imu_bias_gz;
    }

    mx = 0.0f;
    my = 0.0f;
    mz = 0.0f;
}
void imuCalibration() {
    if (!imu_ok) return;

    float raw_ax = 0.0f, raw_ay = 0.0f, raw_az = 0.0f;
    float raw_gx = 0.0f, raw_gy = 0.0f, raw_gz = 0.0f;

    bool okA = myIMU.readAccelMPS2(raw_ax, raw_ay, raw_az);
    bool okG = myIMU.readGyroRADS(raw_gx, raw_gy, raw_gz);

    if (okG) {
        imu_bias_gx = raw_gx;
        imu_bias_gy = raw_gy;
        imu_bias_gz = raw_gz;
    }

    imu_bias_ax = 0.0f;
    imu_bias_ay = 0.0f;
    imu_bias_az = 0.0f;

    imu_bias_cx = 0.0f;
    imu_bias_cy = 0.0f;
    imu_bias_cz = 0.0f;

    store.biasGyroX  = imu_bias_gx;
    store.biasGyroY  = imu_bias_gy;
    store.biasGyroZ  = imu_bias_gz;

    store.biasAccelX = imu_bias_ax;
    store.biasAccelY = imu_bias_ay;
    store.biasAccelZ = imu_bias_az;

    store.biasCPassX = 0.0f;
    store.biasCPassY = 0.0f;
    store.biasCPassZ = 0.0f;

    jsonInfoHttp.clear();
    jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;
    jsonInfoHttp["gx"] = store.biasGyroX;
    jsonInfoHttp["gy"] = store.biasGyroY;
    jsonInfoHttp["gz"] = store.biasGyroZ;
    jsonInfoHttp["ax"] = store.biasAccelX;
    jsonInfoHttp["ay"] = store.biasAccelY;
    jsonInfoHttp["az"] = store.biasAccelZ;
    jsonInfoHttp["cx"] = 0.0;
    jsonInfoHttp["cy"] = 0.0;
    jsonInfoHttp["cz"] = 0.0;

    String getInfoJsonString;
    serializeJson(jsonInfoHttp, getInfoJsonString);
    Serial.println(getInfoJsonString);

    qc0 = 1.0;
    qc1 = 0.0;
    qc2 = 0.0;
    qc3 = 0.0;
}

void imuCalibration_bk() {
    if (!imu_ok) {
        jsonInfoHttp.clear();
        jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;
        jsonInfoHttp["status"] = 0;

        String getInfoJsonString;
        serializeJson(jsonInfoHttp, getInfoJsonString);
        Serial.println(getInfoJsonString);
        return;
    }

    imuCalibration();

    jsonInfoHttp.clear();
    jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;
    jsonInfoHttp["status"] = 1;

    jsonInfoHttp["gx"] = store.biasGyroX;
    jsonInfoHttp["gy"] = store.biasGyroY;
    jsonInfoHttp["gz"] = store.biasGyroZ;

    jsonInfoHttp["ax"] = store.biasAccelX;
    jsonInfoHttp["ay"] = store.biasAccelY;
    jsonInfoHttp["az"] = store.biasAccelZ;

    jsonInfoHttp["cx"] = 0.0;
    jsonInfoHttp["cy"] = 0.0;
    jsonInfoHttp["cz"] = 0.0;

    String getInfoJsonString;
    serializeJson(jsonInfoHttp, getInfoJsonString);
    Serial.println(getInfoJsonString);
}

// {"T":126}
void getIMUData() {
    jsonInfoHttp.clear();
    jsonInfoHttp["T"] = FEEDBACK_IMU_DATA;

    jsonInfoHttp["r"] = icm_roll;
    jsonInfoHttp["p"] = icm_pitch;
    jsonInfoHttp["y"] = icm_yaw;

    jsonInfoHttp["q0"] = q0;
    jsonInfoHttp["q1"] = q1;
    jsonInfoHttp["q2"] = q2;
    jsonInfoHttp["q3"] = q3;

    String getInfoJsonString;
    serializeJson(jsonInfoHttp, getInfoJsonString);
    Serial.println(getInfoJsonString);
}

// {"T":128}
void getIMUOffset() {
    double halfRoll = -icm_roll / 2.0;
    double qr0 = cos(halfRoll);
    double qr1 = sin(halfRoll);
    double qr2 = 0.0;
    double qr3 = 0.0;

    double halfPitch = -icm_pitch / 2.0;
    double qp0 = cos(halfPitch);
    double qp1 = 0.0;
    double qp2 = sin(halfPitch);
    double qp3 = 0.0;

    qc0 = qr0 * qp0 - qr1 * qp1 - qr2 * qp2 - qr3 * qp3;
    qc1 = qr0 * qp1 + qr1 * qp0 + qr2 * qp3 - qr3 * qp2;
    qc2 = qr0 * qp2 - qr1 * qp3 + qr2 * qp0 + qr3 * qp1;
    qc3 = qr0 * qp3 + qr1 * qp2 - qr2 * qp1 + qr3 * qp0;

    jsonInfoHttp.clear();
    jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;
    jsonInfoHttp["qc0"] = qc0;
    jsonInfoHttp["qc1"] = qc1;
    jsonInfoHttp["qc2"] = qc2;
    jsonInfoHttp["qc3"] = qc3;

    String getInfoJsonString;
    serializeJson(jsonInfoHttp, getInfoJsonString);
    Serial.println(getInfoJsonString);
}

void getIMUOffset_bk() {
    jsonInfoHttp.clear();
    jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;

    jsonInfoHttp["gx"] = imu_bias_gx;
    jsonInfoHttp["gy"] = imu_bias_gy;
    jsonInfoHttp["gz"] = imu_bias_gz;

    jsonInfoHttp["ax"] = imu_bias_ax;
    jsonInfoHttp["ay"] = imu_bias_ay;
    jsonInfoHttp["az"] = imu_bias_az;

    jsonInfoHttp["cx"] = 0.0;
    jsonInfoHttp["cy"] = 0.0;
    jsonInfoHttp["cz"] = 0.0;

    String getInfoJsonString;
    serializeJson(jsonInfoHttp, getInfoJsonString);
    Serial.println(getInfoJsonString);
}

// {"T":129,"gx":0,"gy":0,"gz":0,"ax":0,"ay":0,"az":0,"cx":0,"cy":0,"cz":0}
void setIMUOffset(int32_t inGX, int32_t inGY, int32_t inGZ,
                  int32_t inAX, int32_t inAY, int32_t inAZ,
                  int32_t inCX, int32_t inCY, int32_t inCZ) {
    imu_bias_gx = (float)inGX;
    imu_bias_gy = (float)inGY;
    imu_bias_gz = (float)inGZ;

    imu_bias_ax = (float)inAX;
    imu_bias_ay = (float)inAY;
    imu_bias_az = (float)inAZ;

    imu_bias_cx = (float)inCX;
    imu_bias_cy = (float)inCY;
    imu_bias_cz = (float)inCZ;

    store.biasGyroX  = imu_bias_gx;
    store.biasGyroY  = imu_bias_gy;
    store.biasGyroZ  = imu_bias_gz;

    store.biasAccelX = imu_bias_ax;
    store.biasAccelY = imu_bias_ay;
    store.biasAccelZ = imu_bias_az;

    store.biasCPassX = imu_bias_cx;
    store.biasCPassY = imu_bias_cy;
    store.biasCPassZ = imu_bias_cz;

    jsonInfoHttp.clear();
    jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;
    jsonInfoHttp["status"] = 1;

    jsonInfoHttp["gx"] = store.biasGyroX;
    jsonInfoHttp["gy"] = store.biasGyroY;
    jsonInfoHttp["gz"] = store.biasGyroZ;

    jsonInfoHttp["ax"] = store.biasAccelX;
    jsonInfoHttp["ay"] = store.biasAccelY;
    jsonInfoHttp["az"] = store.biasAccelZ;

    jsonInfoHttp["cx"] = store.biasCPassX;
    jsonInfoHttp["cy"] = store.biasCPassY;
    jsonInfoHttp["cz"] = store.biasCPassZ;

    String getInfoJsonString;
    serializeJson(jsonInfoHttp, getInfoJsonString);
    Serial.println(getInfoJsonString);
}
