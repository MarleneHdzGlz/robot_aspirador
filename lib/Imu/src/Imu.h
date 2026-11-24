#pragma once
// IMU.h — Roomba cuadrícula — BNO055
// - BNO055 en modo NDOF.
// - ESP32, I2C 100 kHz.
// - Dirección 0x29.
// - Acceso I2C protegido con mutex FreeRTOS.
// - Rumbo antihorario positivo.
// - Yaw continuo sin saltos 0/360.


#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>



class Imu {


public:


    struct Config {
        int sda;
        int scl;
        int rst;


        uint32_t i2cFreq;
        uint16_t i2cTimeoutMs;


        uint8_t address;


        bool extCrystal;


        uint8_t retries;
        uint16_t bootWaitMs;


        float yawSign;
    };



    void diagnostic() {
        if (!ready_ || bno_ == nullptr) {
            Serial.println("[imu-test] Sensor no preparado");
            return;
        }


        if (xSemaphoreTake(i2cMutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
            return;
        }


        imu::Vector<3> euler =
            bno_->getVector(Adafruit_BNO055::VECTOR_EULER);


        imu::Vector<3> gyro =
            bno_->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);


        imu::Vector<3> accel =
            bno_->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);


        xSemaphoreGive(i2cMutex_);


        Serial.printf(
            "[imu-test] H=%.2f R=%.2f P=%.2f | "
            "GX=%.2f GY=%.2f GZ=%.2f | "
            "AX=%.2f AY=%.2f AZ=%.2f\n",
            euler.x(), euler.y(), euler.z(),
            gyro.x(), gyro.y(), gyro.z(),
            accel.x(), accel.y(), accel.z()
        );
    }



    bool begin(const Config &c) {
        cfg_ = c;


        if (i2cMutex_ == nullptr) {
            i2cMutex_ = xSemaphoreCreateMutex();
        }


        if (cfg_.rst >= 0) {
            pinMode(cfg_.rst, OUTPUT);
            digitalWrite(cfg_.rst, HIGH);
        }


        Wire.begin(cfg_.sda, cfg_.scl, cfg_.i2cFreq);
        Wire.setTimeOut(cfg_.i2cTimeoutMs);


        if (bno_ == nullptr) {
            bno_ = new Adafruit_BNO055(55, cfg_.address, &Wire);
        }


        bootT0_ = millis();


        return start();
    }



    bool reset() {
        resets_++;


        ready_ = false;
        ok_ = false;
        haveYaw_ = false;


        if (cfg_.rst >= 0) {
            digitalWrite(cfg_.rst, LOW);
            delay(10);
            digitalWrite(cfg_.rst, HIGH);
        }


        bootT0_ = millis();


        return start();
    }



    bool update() {
        if (!ready_ || bno_ == nullptr) {
            return false;
        }


        if (!ok_) {
            if (millis() - lastRetryMs_ >= 2000) {
                lastRetryMs_ = millis();
                reset();
            }
            return false;
        }


        if (xSemaphoreTake(i2cMutex_, pdMS_TO_TICKS(50)) != pdTRUE) {
            return false;
        }


        imu::Vector<3> euler =
            bno_->getVector(Adafruit_BNO055::VECTOR_EULER);


        imu::Vector<3> gyro =
            bno_->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);


        // Calibracion sin ping I2C
        if (++pingCount_ >= 50) {
            pingCount_ = 0;


            if (calibrating_) {
                uint8_t sys = 0, gyroCal = 0, accel = 0, mag = 0;
                bno_->getCalibration(&sys, &gyroCal, &accel, &mag);


                calSys_ = sys;
                calGyro_ = gyroCal;
                calAccel_ = accel;
                calMag_ = mag;


                if (calGyro_ == 3 && calAccel_ == 3) {
                    calibrating_ = false;
                    calSaved_ = saveOffsets();


                    Serial.printf(
                        "[imu] Calibracion finalizada. Offsets guardados: %d\n",
                        (int)calSaved_
                    );
                }
            }
        }


        float heading = euler.x();
        float gyroX = gyro.x();
        float gyroY = gyro.y();
        float gyroZ = gyro.z();


        bool valid =
            isfinite(heading) &&
            isfinite(gyroX) &&
            isfinite(gyroY) &&
            isfinite(gyroZ) &&
            heading >= 0.0f &&
            heading < 360.0f;


        xSemaphoreGive(i2cMutex_);


        if (!valid) {
            failures_++;


            if (failures_ >= 10) {
                Serial.println("[imu] ERROR: demasiadas lecturas invalidas");


                failures_ = 0;
                ok_ = false;
                lastRetryMs_ = millis();
            }


            return false;
        }


        failures_ = 0;


        lastRawHeadingDeg_ = heading;


        float yawDeg = cfg_.yawSign * heading;


        while (yawDeg > 180.0f) {
            yawDeg -= 360.0f;
        }


        while (yawDeg < -180.0f) {
            yawDeg += 360.0f;
        }


        if (!haveYaw_) {
            lastRawDeg_ = yawDeg;
            unwrappedDeg_ = 0.0f;
            haveYaw_ = true;
        } else {
            float delta = yawDeg - lastRawDeg_;


            if (delta > 180.0f) {
                delta -= 360.0f;
            }


            if (delta < -180.0f) {
                delta += 360.0f;
            }


            unwrappedDeg_ += delta;
            lastRawDeg_ = yawDeg;
        }


        yawRateDegS_ = cfg_.yawSign * gyroZ;


        lastGoodMs_ = millis();


        static uint32_t lastDebug = 0;


        if (millis() - lastDebug >= 250) {
            lastDebug = millis();


            Serial.printf(
                "[imu] heading=%.2f | yaw=%.2f | "
                "acumulado=%.2f | gyroZ=%.2f | "
                "cal=%u,%u,%u,%u | ok=%d\n",
                heading, yawDeg, unwrappedDeg_, yawRateDegS_,
                calSys_, calGyro_, calAccel_, calMag_,
                (int)ok_
            );
        }


        return true;
    }



    float yaw() const {
        return unwrappedDeg_ * DEG_TO_RAD + offsetRad_;
    }



    float yawRate() const {
        return yawRateDegS_ * DEG_TO_RAD;
    }



    float rawHeadingDeg() const {
        return lastRawHeadingDeg_;
    }



    float gyroZDegS() const {
        return yawRateDegS_;
    }



    float yawAccumulatedDeg() const {
        return unwrappedDeg_;
    }



    void setYaw(float yawRad) {
        offsetRad_ = yawRad - unwrappedDeg_ * DEG_TO_RAD;
    }



    void zeroYaw() {
        setYaw(0.0f);
    }



    bool ok() const {
        return ok_;
    }



    bool fresh(uint32_t maxAgeMs = 300) const {
        return ok_ && ready_ && (millis() - lastGoodMs_) < maxAgeMs;
    }



    uint32_t resets() const {
        return resets_;
    }



    void calibration(uint8_t &sys, uint8_t &gyro, uint8_t &accel) const {
        sys = calSys_;
        gyro = calGyro_;
        accel = calAccel_;
    }



    void startCalibration() {
        calSaved_ = false;
        calibrating_ = ok_;
    }



    bool calibrating() const {
        return calibrating_;
    }



    bool calibrationSaved() const {
        return calSaved_;
    }



    bool offsetsRestored() const {
        return restored_;
    }



    bool saveOffsets() {
        if (!ok_ || bno_ == nullptr) {
            return false;
        }


        if (xSemaphoreTake(i2cMutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
            return false;
        }


        adafruit_bno055_offsets_t offsets;


        bool readOk = bno_->getSensorOffsets(offsets);


        bool saved = false;


        if (readOk) {
            Preferences prefs;


            if (prefs.begin("imu", false)) {
                size_t written = prefs.putBytes("off", &offsets, sizeof(offsets));
                prefs.end();


                saved = (written == sizeof(offsets));
            }
        }


        xSemaphoreGive(i2cMutex_);


        return saved;
    }



private:


    bool start() {
        ready_ = false;
        ok_ = false;


        while (millis() - bootT0_ < cfg_.bootWaitMs) {
            delay(10);
        }


        if (xSemaphoreTake(i2cMutex_, pdMS_TO_TICKS(500)) != pdTRUE) {
            Serial.println("[imu] ERROR: no se pudo tomar el mutex I2C");
            return false;
        }


        Wire.beginTransmission(cfg_.address);
        Wire.write(0x00);
        uint8_t txError = Wire.endTransmission(false);


        uint8_t chipId = 0xFF;


        if (txError == 0) {
            uint8_t received =
                Wire.requestFrom((uint8_t)cfg_.address, (uint8_t)1);


            if (received == 1 && Wire.available()) {
                chipId = Wire.read();
            }
        }


        Serial.printf("[imu] CHIP_ID=0x%02X\n", chipId);


        if (chipId != 0xA0) {
            Serial.println("[imu] ERROR: BNO055 no detectado");


            xSemaphoreGive(i2cMutex_);
            return false;
        }


        for (uint8_t i = 0; i < cfg_.retries; i++) {
            Serial.printf("[imu] Inicializacion %u/%u\n", i + 1, cfg_.retries);


            if (bno_->begin(OPERATION_MODE_NDOF)) {
                ok_ = true;
                break;
            }


            delay(250);
        }


        if (!ok_) {
            Serial.println("[imu] ERROR: no se pudo iniciar el BNO055");


            xSemaphoreGive(i2cMutex_);
            return false;
        }


        delay(50);


        bno_->setExtCrystalUse(cfg_.extCrystal);


        restored_ = restoreOffsetsLocked();


        uint8_t sysStatus = 0, selfTest = 0, sysError = 0;
        bno_->getSystemStatus(&sysStatus, &selfTest, &sysError);


        Serial.printf(
            "[imu] sysStatus=%u selfTest=0x%02X error=%u\n",
            sysStatus, selfTest, sysError
        );


        Serial.printf("[imu] Offsets restaurados=%d\n", (int)restored_);


        xSemaphoreGive(i2cMutex_);


        failures_ = 0;
        pingFails_ = 0;
        pingCount_ = 0;
        haveYaw_ = false;


        lastGoodMs_ = millis();
        lastRetryMs_ = millis();


        ready_ = true;


        Serial.println("[imu] BNO055 preparado correctamente");


        return true;
    }



    bool restoreOffsetsLocked() {
        Preferences prefs;


        if (!prefs.begin("imu", true)) {
            return false;
        }


        adafruit_bno055_offsets_t offsets;


        size_t n = prefs.getBytes("off", &offsets, sizeof(offsets));
        prefs.end();


        if (n != sizeof(offsets)) {
            return false;
        }


        bno_->setSensorOffsets(offsets);


        return true;
    }



    Config cfg_{};
    Adafruit_BNO055 *bno_ = nullptr;


    SemaphoreHandle_t i2cMutex_ = nullptr;


    bool ok_ = false;
    bool ready_ = false;
    bool restored_ = false;


    volatile bool calibrating_ = false;
    volatile bool calSaved_ = false;


    volatile uint8_t calSys_ = 0;
    volatile uint8_t calGyro_ = 0;
    volatile uint8_t calAccel_ = 0;
    volatile uint8_t calMag_ = 0;


    bool haveYaw_ = false;


    uint32_t bootT0_ = 0;
    uint32_t lastRetryMs_ = 0;
    uint32_t lastGoodMs_ = 0;


    uint32_t resets_ = 0;


    uint16_t failures_ = 0;


    uint8_t pingCount_ = 0;
    uint8_t pingFails_ = 0;


    float lastRawDeg_ = 0.0f;
    float unwrappedDeg_ = 0.0f;
    float offsetRad_ = 0.0f;
    float yawRateDegS_ = 0.0f;
    float lastRawHeadingDeg_ = 0.0f;
};