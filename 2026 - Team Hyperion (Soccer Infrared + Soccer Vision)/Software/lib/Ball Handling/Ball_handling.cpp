#include "Ball_handling.h"
#include <math.h>
 
BallHandling::BallHandling()
    : kickerVd(KICKER_VD_PIN, KICKER_VOLTAGE_STABALISER, KICKER_VOLTAGE_OFFSET),
      pulseTimer(KICK_PULSE_US),
      rechargeTimer(KICK_RECHARGE_US),
      cooldownTimer(KICK_COOLDOWN_US),
      kicks(MAX_KICKS),
      isKicking(false),
      cooldownActive(false) {}

bool BallHandling::photogate_triggered() {
    return analogRead(PHOTOGATE_PIN) < (photogateThresh - 75);
}

void BallHandling::init() {
    pinMode(KICKER_PIN, OUTPUT);
    digitalWrite(KICKER_PIN, HIGH);
 
    pinMode(PHOTOGATE_PIN, INPUT);
    kickerVd.init();

    pinMode(DRINA, OUTPUT);
    pinMode(DRINB, OUTPUT);
    pinMode(DRPWM, OUTPUT);
    digitalWrite(DRINA, HIGH);
    digitalWrite(DRINB, HIGH);
    delayMicroseconds(100);
    analogWriteFrequency(DRPWM, MOTOR_ANALOG_FRQ);
    run_dribbler(0.0f);

    photogateThresh = analogRead(PHOTOGATE_PIN);
 
    kicks = MAX_KICKS;
    isKicking = false;
    cooldownActive = false;
 
    pulseTimer.update();
    rechargeTimer.update();
    cooldownTimer.update();
}
 
bool BallHandling::can_kick() {
    if (kicks <= 0) {
        return false;
    }
    if (kickerVd.get_lvl() < KICKER_REQUIRED_VOLT) {
        return false;
    }
    if (!photogate_triggered()) {
        return false;
    }
    if (isKicking) {
        return false;
    }
    if (cooldownActive && !cooldownTimer.time_has_passed_no_update()) {
        return false;
    }
    return true;
}
 
void BallHandling::kick() {
    if (!can_kick()) return;

    run_dribbler(0.0f);
    digitalWrite(KICKER_PIN, LOW);
    isKicking = true;
    kicks--;
    pulseTimer.update();
}
 
void BallHandling::update(float ballStr) {
    update_caps_led();
    if (isKicking && pulseTimer.time_has_passed_no_update()) {
        digitalWrite(KICKER_PIN, HIGH);
        isKicking = false;
        cooldownActive = true;
        cooldownTimer.update();
    }
 
    if (kicks < MAX_KICKS) {
        if (rechargeTimer.time_has_passed()) {
            kicks++;
        }
    } else {
        rechargeTimer.update();
    }

    if (!isKicking && ((ballStr > DRIBBLER_STR_THRESH) || photogate_triggered())) {
        run_dribbler(DRIBBLER_SPEED);
    } else {
        run_dribbler(0.0f);
    }
}

void BallHandling::run_dribbler(float spd) {
    if (spd > 255.0f) spd = 255.0f;
    else if (spd < -255.0f) spd = -255.0f;

    uint8_t finalSpd = round(fabs(spd));
    analogWrite(DRPWM, finalSpd);
    digitalWrite(DRINA, (spd > 0.0f));
    digitalWrite(DRINB, (spd < 0.0f));
}
 
void BallHandling::update_caps_led() {
    if(kickerVd.get_lvl() < 8) {
        digitalWrite(CAPS_LED, LOW);
    } else {
        digitalWrite(CAPS_LED, HIGH);
    }
}