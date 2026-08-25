#include "drive_system.h"
#include <pins.h>
#include <math.h>
#include <common.h>

void Drive_system::init() {
    for(uint8_t i = 0; i < MOTOR_NUM; i++){
        pinMode(inA[i], OUTPUT);
        pinMode(inB[i], OUTPUT);
        pinMode(pwm[i], OUTPUT);
    }
}

void Drive_system::update(float speed, float angle, float heading, float correction){
    float maxSpeed = 0;
    float minSpeed = 5000;
    for(uint8_t i = 0; i < MOTOR_NUM; i++){
        values[i] = sinf(DEG_TO_RAD * (angles[i] - angle)) * speed + correction;
        // Serial.print(values[i]); Serial.print(' ');
        if(abs(values[i]) > maxSpeed){
            maxSpeed = abs(values[i]);
        }
        if(abs(values[i]) < minSpeed){
            minSpeed = abs(values[i]);
        }
    }
    // Serial.println();
    for(uint8_t i = 0; i < MOTOR_NUM; i++){
        float scaledSpeed = 94 + ((255 - 94) / 100.0) * abs(values[i]);
        // Serial.print(scaledSpeed);
        // Serial.print(" ");
        analogWrite(pwm[i], (uint8_t)constrain(scaledSpeed, 94, 255));
        digitalWrite(inA[i], (values[i]) > 0);
        digitalWrite(inB[i], (values[i]) < 0);
    }
    // Serial.println();
//tssp, mpu, light system
}