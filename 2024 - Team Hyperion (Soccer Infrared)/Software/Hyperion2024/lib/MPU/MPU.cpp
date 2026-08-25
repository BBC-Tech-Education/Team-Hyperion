#include "MPU.h"

void MPU::init() {
    Wire.begin();
    I2CwriteByte(MPU9250_ADDRESS, 29, 0x06);
    I2CwriteByte(MPU9250_ADDRESS, 26, 0x06);
    I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS);
    I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_2_G);
    I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);
    I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16);
    previousTimeGyro = micros();
    calibrate();
}

void MPU::readGyroscope(float gyroData[3]) {
    uint8_t buffer[14];
    I2Cread(MPU9250_ADDRESS, 0x3B, 14, buffer);
    int16_t gx = -(buffer[8] << 8 | buffer[1]);
    int16_t gy = -(buffer[10] << 8 | buffer[11]);
    int16_t gz = buffer[12] << 8 | buffer[13];
    gyroData[0] = convertRawGyro(gx);
    gyroData[1] = convertRawGyro(gy);
    gyroData[2] = convertRawGyro(gz);
}

void MPU::update() {
    float gyroData[3];
    readGyroscope(gyroData);
    float reading = gyroData[2];
    long currentTime = micros();
    heading += -(((float)(currentTime - previousTimeGyro) / 1000000.0) * (reading - calibrationGyro));
    if (millis() - previousDrift > 100) {
        heading += drift;
        previousDrift = millis();
    }
    heading = floatMod(heading, 360);
    previousTimeGyro = currentTime;
}

void MPU::calibrate() {
    for (uint8_t i = 0; i < MPU_CALIBRATION_COUNT; i++) {
        float gyroData[3];
        readGyroscope(gyroData);
        calibrationGyro += gyroData[2];
        delay(MPU_CALIBRATION_TIME);
    }
    calibrationGyro /= MPU_CALIBRATION_COUNT;
    update();
    float firstVal = heading;
    delay(100);
    update();
    float secondVal = heading;
    drift = secondVal - firstVal;
    previousDrift = millis();
}

float MPU::convertRawGyro(int raw) {
    float g = (raw * 1000.0) / 32768.0;
    return g;
}

float MPU::floatMod(float x, float y) {
    if (y == 0) return x;
    float mod = fmod(x, y);
    if (mod < 0) mod += y;
    return mod;
}

void MPU::findTargetVal() {
    if(!flag1) {
        update();
        targetVal = heading;
        flag1 = 1;
    }
}