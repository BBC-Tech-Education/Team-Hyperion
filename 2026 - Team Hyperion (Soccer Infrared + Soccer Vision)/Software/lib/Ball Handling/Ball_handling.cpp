#include "Ball_handling.h"

// voltage divider
// photogate
// consult raj about the length of the timers

BallHandling::BallHandling()
    : kickerVd(KICKER_VD_PIN, KICKER_VOLTAGE_STABALISER, KICKER_VOLTAGE_OFFSET),
      pulseTimer(KICK_PULSE_US),
      rechargeTimer(KICK_RECHARGE_US),
      cooldownTimer(KICK_COOLDOWN_US),
      kicks(MAX_KICKS),
      isKicking(false),
      cooldownActive(false) {}

bool BallHandling::photogate_triggered() {
    return analogRead(PHOTOGATE_PIN) < PHOTOGATE_THRESH;
}

void BallHandling::init() {
    pinMode(KICKER_PIN, OUTPUT);
    digitalWrite(KICKER_PIN, HIGH);

    pinMode(PHOTOGATE_PIN, INPUT);
    kickerVd.init();

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

    digitalWrite(KICKER_PIN, LOW);
    isKicking = true;
    kicks--;
    pulseTimer.update();
}

void BallHandling::update() {
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
}

void BallHandling::update_caps_led() {
    if(kickerVd.get_lvl() < 8) {
        digitalWrite(CAPS_LED, LOW);
    } else {
        digitalWrite(CAPS_LED, HIGH);
    }
}