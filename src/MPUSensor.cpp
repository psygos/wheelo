#include "MPUSensor.h"
#include "config.h"
#include <math.h>

MPUSensor::MPUSensor(uint8_t addr, int sda, int scl)
    : _addr(addr), _sda(sda), _scl(scl) {}

static float lowPass(float prev, float x, float cutoffHz, float dt) {
    const float rc = 1.0f / (2.0f * 3.1415926535f * cutoffHz);
    const float alpha = constrain(dt / (rc + dt), 0.0f, 1.0f);
    return prev + alpha * (x - prev);
}

bool MPUSensor::writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool MPUSensor::readReg(uint8_t reg, uint8_t &value) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(_addr, (uint8_t)1, (uint8_t)true) != 1) return false;
    int v = Wire.read();
    if (v < 0) return false;
    value = (uint8_t)v;
    return true;
}

bool MPUSensor::rawRead(int16_t out[7]) {
    Wire.beginTransmission(_addr);
    Wire.write(REG_ACCEL_XOUT);
    if (Wire.endTransmission(false) != 0) return false;

    uint8_t n = Wire.requestFrom(_addr, (uint8_t)14, (uint8_t)true);
    if (n != 14) return false;

    for (int i = 0; i < 7; i++) {
        int hi = Wire.read();
        int lo = Wire.read();
        if (hi < 0 || lo < 0) return false;
        out[i] = (int16_t)((hi << 8) | lo);
    }
    return true;
}

float MPUSensor::angleForAxis(float ax, float ay, float az, uint8_t axis) const {
    if (axis == ROLL_AXIS_Y) {
        return atan2f(-ax, az) * 57.2957795f;
    }
    return atan2f(ay, az) * 57.2957795f;
}

void MPUSensor::begin(Preferences& prefs) {
    Wire.begin(_sda, _scl);
    Wire.setClock(100000);
    delay(250);

    Serial.printf("I2C SDA=GPIO%d SCL=GPIO%d\n", _sda, _scl);
    int found = 0;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) { Serial.printf("  device @ 0x%02X\n", a); found++; }
    }
    if (!found) { Serial.println("  no I2C devices"); ok = false; return; }

    Wire.beginTransmission(_addr);
    if (Wire.endTransmission() != 0) { Serial.println("MPU6050: no ACK"); ok = false; return; }

    uint8_t who = 0;
    if (readReg(REG_WHO_AM_I, who)) {
        Serial.printf("MPU6050 WHO_AM_I=0x%02X\n", who);
    } else {
        Serial.println("MPU6050 WHO_AM_I read failed");
    }

    if (!writeReg(REG_PWR_MGMT_1, 0x01)) { ok = false; return; }
    delay(10);
    if (!writeReg(REG_CONFIG, MPU_DLPF_CFG)) { ok = false; return; }
    if (!writeReg(REG_SMPLRT_DIV, MPU_SMPLRT_DIV)) { ok = false; return; }
    if (!writeReg(REG_ACCEL_CFG, 0x00)) { ok = false; return; } // +/-2g
    if (!writeReg(REG_GYRO_CFG, 0x00)) { ok = false; return; }  // +/-250 dps
    ok = true;

    prefs.begin(NVS_NS, true);
    _biasAx     = prefs.getFloat("b_ax",    0);
    _biasAy     = prefs.getFloat("b_ay",    0);
    _biasAz     = prefs.getFloat("b_az",    0);
    _biasGx     = prefs.getFloat("b_gx",    0);
    _biasGy     = prefs.getFloat("b_gy",    0);
    _biasGz     = prefs.getFloat("b_gz",    0);
    angleOffset = prefs.getFloat("ang_off", 0);
    rollAxis    = (uint8_t)prefs.getUInt("roll_axis", ROLL_AXIS_X);
    rollSign    = prefs.getFloat("roll_sign", 1.0f);
    bool hasBias = prefs.getBool("bias_ok", false);
    prefs.end();

    if (rollAxis != ROLL_AXIS_X && rollAxis != ROLL_AXIS_Y) {
        rollAxis = ROLL_AXIS_X;
    }
    rollSign = rollSign < 0.0f ? -1.0f : 1.0f;

    if (!hasBias) {
        Serial.println("No bias — auto-cal (keep still ~2 s)…");
        calibrate(prefs, 200);
    } else {
        Serial.println("MPU6050 ready");
    }
}

void MPUSensor::resetFilters() {
    _eAx = _eAy = _eAz = _eGx = _eGy = 0.0f;
    _cfAngle = 0.0f;
    _lastUs = 0;
    _cfInited = false;
    rateDps = 0.0f;
    accelAngle = 0.0f;
    accelNormG = 1.0f;
}

void MPUSensor::startRollCalibration() {
    rollCalibrating = true;
    rollCalSamples = 0;
    rollCalSpanDeg = 0.0f;
    rollCalQuality = 0.0f;
    _rollCalAbsGx = 0.0;
    _rollCalAbsGy = 0.0;
    _rollCalMinX = _rollCalMaxX = 0.0f;
    _rollCalMinY = _rollCalMaxY = 0.0f;
}

void MPUSensor::updateRollCalibration(float ax, float ay, float az, float gx, float gy) {
    if (!rollCalibrating) return;

    float xAngle = angleForAxis(ax, ay, az, ROLL_AXIS_X);
    float yAngle = angleForAxis(ax, ay, az, ROLL_AXIS_Y);

    if (rollCalSamples == 0) {
        _rollCalMinX = _rollCalMaxX = xAngle;
        _rollCalMinY = _rollCalMaxY = yAngle;
    } else {
        _rollCalMinX = min(_rollCalMinX, xAngle);
        _rollCalMaxX = max(_rollCalMaxX, xAngle);
        _rollCalMinY = min(_rollCalMinY, yAngle);
        _rollCalMaxY = max(_rollCalMaxY, yAngle);
    }

    _rollCalAbsGx += fabsf(gx);
    _rollCalAbsGy += fabsf(gy);
    rollCalSamples++;

    float xSpan = _rollCalMaxX - _rollCalMinX;
    float ySpan = _rollCalMaxY - _rollCalMinY;
    if (_rollCalAbsGy > _rollCalAbsGx) {
        rollCalSpanDeg = ySpan;
        rollCalQuality = (float)(_rollCalAbsGy / max(1.0, _rollCalAbsGx));
    } else {
        rollCalSpanDeg = xSpan;
        rollCalQuality = (float)(_rollCalAbsGx / max(1.0, _rollCalAbsGy));
    }
}

bool MPUSensor::finishRollCalibration(Preferences& prefs) {
    if (!rollCalibrating) return false;

    rollCalibrating = false;
    if (rollCalSamples < 20 || rollCalSpanDeg < 8.0f) {
        return false;
    }

    rollAxis = _rollCalAbsGy > _rollCalAbsGx ? ROLL_AXIS_Y : ROLL_AXIS_X;
    saveRollAxis(prefs);
    resetFilters();
    return true;
}

void MPUSensor::cancelRollCalibration() {
    rollCalibrating = false;
}

void MPUSensor::setRollAxis(uint8_t axis, float sign, Preferences& prefs) {
    rollAxis = axis == ROLL_AXIS_Y ? ROLL_AXIS_Y : ROLL_AXIS_X;
    rollSign = sign < 0.0f ? -1.0f : 1.0f;
    saveRollAxis(prefs);
    resetFilters();
}

bool MPUSensor::read() {
    if (!ok || calibrating) return false;

    int16_t raw[7];
    if (!rawRead(raw)) {
        droppedReads++;
        return false;
    }

    float rax = raw[0] / 16384.0f - _biasAx;
    float ray = raw[1] / 16384.0f - _biasAy;
    float raz = raw[2] / 16384.0f - _biasAz;
    float rgx = raw[4] / 131.0f   - _biasGx;
    float rgy = raw[5] / 131.0f   - _biasGy;

    updateRollCalibration(rax, ray, raz, rgx, rgy);

    uint32_t now = micros();
    float dt = _cfInited ? constrain((now - _lastUs) / 1e6f, 0.005f, 0.025f) : 0.01f;
    _lastUs = now;

    accelNormG = sqrtf(rax * rax + ray * ray + raz * raz);

    if (!_cfInited) {
        _eAx = rax;
        _eAy = ray;
        _eAz = raz;
        _eGx = rgx;
        _eGy = rgy;
        accelAngle = rollSign * angleForAxis(_eAx, _eAy, _eAz, rollAxis);
        _cfAngle = accelAngle;
        _cfInited = true;
    } else {
        _eAx = lowPass(_eAx, rax, 12.0f, dt);
        _eAy = lowPass(_eAy, ray, 12.0f, dt);
        _eAz = lowPass(_eAz, raz, 12.0f, dt);
        _eGx = lowPass(_eGx, rgx, 20.0f, dt);
        _eGy = lowPass(_eGy, rgy, 20.0f, dt);
        accelAngle = rollSign * angleForAxis(_eAx, _eAy, _eAz, rollAxis);
    }

    rateDps = rollSign * (rollAxis == ROLL_AXIS_Y ? _eGy : _eGx);

    float accelCutoffHz = 0.35f;
    if (fabsf(accelNormG - 1.0f) > 0.18f) {
        accelCutoffHz = 0.05f;
    }
    const float accelTrust = constrain(
        dt / ((1.0f / (2.0f * 3.1415926535f * accelCutoffHz)) + dt),
        0.0f, 1.0f);

    _cfAngle = (1.0f - accelTrust) * (_cfAngle + rateDps * dt)
             + accelTrust * accelAngle;

    angle = _cfAngle + angleOffset;
    return true;
}

void MPUSensor::calibrate(Preferences& prefs, int samples) {
    if (!ok) return;
    calibrating = true;
    Serial.printf("Bias cal (%d samples)…\n", samples);
    double ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
    int collected = 0;
    int attempts = 0;
    const int maxAttempts = samples * 4;
    while (collected < samples && attempts < maxAttempts) {
        attempts++;
        int16_t raw[7];
        if (!rawRead(raw)) {
            droppedReads++;
            delay(2);
            continue;
        }
        ax+=raw[0]/16384.0; ay+=raw[1]/16384.0; az+=raw[2]/16384.0;
        gx+=raw[4]/131.0;   gy+=raw[5]/131.0;   gz+=raw[6]/131.0;
        collected++;
        delay(2);
    }

    if (collected == 0) {
        Serial.println("Bias cal failed: no valid MPU samples");
        calibrating = false;
        return;
    }
    if (collected < samples) {
        Serial.printf("Bias cal used %d/%d valid samples\n", collected, samples);
    }

    _biasAx=ax/collected;   _biasAy=ay/collected;
    _biasAz=az/collected-1.0f;
    _biasGx=gx/collected;   _biasGy=gy/collected;   _biasGz=gz/collected;
    resetFilters();
    Serial.printf("Bias: ax=%.3f ay=%.3f az=%.3f gx=%.3f\n",
                  _biasAx, _biasAy, _biasAz, _biasGx);
    saveBias(prefs);
    calibrating = false;
}

void MPUSensor::resetAngle(Preferences& prefs) {
    angleOffset = -_cfAngle;
    saveOffset(prefs);
}

void MPUSensor::setAngle(float target, Preferences& prefs) {
    angleOffset = target - _cfAngle;
    saveOffset(prefs);
}

void MPUSensor::saveBias(Preferences& prefs) {
    prefs.begin(NVS_NS, false);
    prefs.putFloat("b_ax",_biasAx); prefs.putFloat("b_ay",_biasAy);
    prefs.putFloat("b_az",_biasAz); prefs.putFloat("b_gx",_biasGx);
    prefs.putFloat("b_gy",_biasGy); prefs.putFloat("b_gz",_biasGz);
    prefs.putBool("bias_ok", true);
    prefs.end();
}

void MPUSensor::saveOffset(Preferences& prefs) {
    prefs.begin(NVS_NS, false);
    prefs.putFloat("ang_off", angleOffset);
    prefs.end();
}

void MPUSensor::saveRollAxis(Preferences& prefs) {
    prefs.begin(NVS_NS, false);
    prefs.putUInt("roll_axis", rollAxis);
    prefs.putFloat("roll_sign", rollSign);
    prefs.end();
}
