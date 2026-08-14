#include "PID_Autotune.h"
#include "Common.h"

#ifndef PI
#define PI 3.14159265f
#endif

void HeadingPIDAutotune::computeGains(float amplitudeDeg, float puSec,
                                      float& outKp, float& outKd,
                                      float& outKu, bool& kpCapped) {
    outKu = (4.0f * TUNE_RELAY_COR) / (PI * amplitudeDeg);
    float kpRaw = 0.4f * outKu;
    float kdRaw = 0.15f * outKu * puSec;
    kpCapped = (kpRaw > TUNE_KP_MAX);
    outKp = kpCapped ? TUNE_KP_MAX : kpRaw;
    outKd = TUNE_KD_FRAC * kdRaw;
}

void HeadingPIDAutotune::begin() {
    _phase = PHASE_IDLE;
    _prevEnable = false;
    _finished = false;
    _hasResult = false;
    _kp = 0.0f;
    _kd = 0.0f;
    _ku = 0.0f;
    _pu = 0.0f;
    _amp = 0.0f;
    _kpCapped = false;
    resetRun();
}

void HeadingPIDAutotune::resetRun() {
    _relayCor = TUNE_RELAY_COR;
    _prevError = 0.0f;
    _havePrevError = false;
    _discardNextPeriod = true;
    _runStartMs = 0;
    _lastPrintMs = 0;
    _lastPeakMs = 0;
    _periodCount = 0;
    _peakPos = 0.0f;
    _peakNeg = 0.0f;
    _seekingPosPeak = true;
    _periodSumSec = 0.0f;
    _ampSumDeg = 0.0f;
}

void HeadingPIDAutotune::finishSuccess() {
    _amp = _ampSumDeg / (float)_periodCount;
    _pu = _periodSumSec / (float)_periodCount;
    if (_amp < TUNE_MIN_AMPLITUDE_DEG || _pu <= 0.0f) {
        finishFail("Amplitude too small or invalid Pu");
        return;
    }
    computeGains(_amp, _pu, _kp, _kd, _ku, _kpCapped);
    _hasResult = true;
    _finished = true;
    _phase = PHASE_DONE_OK;

    Serial.println(F("=== IMU heading PD autotune result ==="));
    Serial.printf("Kp=%.4f  Ki=0.00  Kd=%.4f\n", _kp, _kd);
    Serial.println(F("Copy to Config.h: KP_IMU / KD_IMU"));
    Serial.printf("(Kp capped: %s | Ku=%.4f Pu=%.4f A=%.4f)\n",
                  _kpCapped ? "yes" : "no", _ku, _pu, _amp);
    Serial.println(F("==="));
}

void HeadingPIDAutotune::finishFail(const char* reason) {
    _hasResult = false;
    _finished = false;
    _phase = PHASE_DONE_FAIL;
    Serial.println(F("=== IMU heading PD autotune incomplete ==="));
    Serial.println(reason);
    Serial.printf("Need more oscillation cycles (got %u, need %u). Flip enable ON and retry.\n",
                  (unsigned)_periodCount, (unsigned)TUNE_MIN_CYCLES);
    Serial.println(F("==="));
}

void HeadingPIDAutotune::printProgress(uint32_t nowMs) {
    if (nowMs - _lastPrintMs < TUNE_PRINT_INTERVAL_MS) {
        return;
    }
    _lastPrintMs = nowMs;
    float puEst = (_periodCount > 0) ? (_periodSumSec / (float)_periodCount) : 0.0f;
    float aEst = (_periodCount > 0) ? (_ampSumDeg / (float)_periodCount) : 0.0f;
    Serial.printf("TUNE running cycles=%u/%u Pu_est=%.3f A=%.2f\n",
                  (unsigned)_periodCount, (unsigned)TUNE_MIN_CYCLES, puEst, aEst);
}

void HeadingPIDAutotune::sampleRelay(float errorDeg, uint32_t nowMs) {
    // Sign matches game: moveCor = -correction.update(error, 0)
    if (errorDeg > TUNE_HYSTERESIS_DEG) {
        _relayCor = -TUNE_RELAY_COR;
    } else if (errorDeg < -TUNE_HYSTERESIS_DEG) {
        _relayCor = TUNE_RELAY_COR;
    }

    if (!_havePrevError) {
        _prevError = errorDeg;
        _havePrevError = true;
        _peakPos = errorDeg;
        _peakNeg = errorDeg;
        return;
    }

    if (_seekingPosPeak) {
        if (errorDeg > _peakPos) {
            _peakPos = errorDeg;
        }
        if (_prevError > 0.0f && errorDeg <= 0.0f && _peakPos > TUNE_HYSTERESIS_DEG) {
            _seekingPosPeak = false;
            _peakNeg = errorDeg;
        }
    } else {
        if (errorDeg < _peakNeg) {
            _peakNeg = errorDeg;
        }
        if (_prevError < 0.0f && errorDeg >= 0.0f && _peakNeg < -TUNE_HYSTERESIS_DEG) {
            float periodSec = (nowMs - _lastPeakMs) / 1000.0f;
            float amp = 0.5f * (_peakPos - _peakNeg);
            _lastPeakMs = nowMs;
            _peakPos = errorDeg;
            _seekingPosPeak = true;

            if (periodSec > 0.05f && amp >= TUNE_MIN_AMPLITUDE_DEG) {
                if (_discardNextPeriod) {
                    _discardNextPeriod = false;
                } else {
                    _periodSumSec += periodSec;
                    _ampSumDeg += amp;
                    _periodCount++;
                }
            }
        }
    }

    _prevError = errorDeg;
}

void HeadingPIDAutotune::update(bool enableOn, float bearingDeg, DriveSystem& motors) {
    uint32_t nowMs = millis();
    float errorDeg = normaliseAngle180(bearingDeg);

    bool rising = enableOn && !_prevEnable;
    bool falling = !enableOn && _prevEnable;
    _prevEnable = enableOn;

    if (rising) {
        resetRun();
        _runStartMs = nowMs;
        _lastPrintMs = nowMs;
        _lastPeakMs = nowMs;
        _phase = PHASE_RUNNING;
        _finished = false;
        _hasResult = false;
        Serial.println(F("TUNE: enable ON - relay start"));
    }

    if (_phase == PHASE_RUNNING && enableOn) {
        if (nowMs - _runStartMs >= TUNE_TIMEOUT_MS) {
            motors.run(0.0f, 0.0f, 0.0f);
            if (_periodCount >= TUNE_MIN_CYCLES) {
                finishSuccess();
            } else {
                finishFail("Timeout while waiting for enough cycles");
            }
            return;
        }

        sampleRelay(errorDeg, nowMs);
        motors.run(0.0f, 0.0f, _relayCor);
        printProgress(nowMs);
        return;
    }

    if (falling && (_phase == PHASE_RUNNING)) {
        motors.run(0.0f, 0.0f, 0.0f);
        if (_periodCount >= TUNE_MIN_CYCLES) {
            finishSuccess();
        } else {
            finishFail("Enable turned OFF early");
        }
        return;
    }

    if (!enableOn) {
        motors.run(0.0f, 0.0f, 0.0f);
    }
}
