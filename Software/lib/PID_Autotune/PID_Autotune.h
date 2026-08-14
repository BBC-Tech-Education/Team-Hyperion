#ifndef PID_AUTOTUNE_H
#define PID_AUTOTUNE_H

#include <Arduino.h>
#include "Config.h"
#include "Drive_system.h"

class HeadingPIDAutotune {
public:
    void begin();
    void update(bool enableOn, float bearingDeg, DriveSystem& motors);

    bool finished() const { return _finished; }
    bool hasResult() const { return _hasResult; }
    float kp() const { return _kp; }
    float kd() const { return _kd; }

    static void computeGains(float amplitudeDeg, float puSec,
                             float& outKp, float& outKd,
                             float& outKu, bool& kpCapped);

private:
    enum Phase {
        PHASE_IDLE,
        PHASE_RUNNING,
        PHASE_DONE_OK,
        PHASE_DONE_FAIL
    };

    void resetRun();
    void sampleRelay(float errorDeg, uint32_t nowMs);
    void finishSuccess();
    void finishFail(const char* reason);
    void printProgress(uint32_t nowMs);

    Phase _phase;
    bool _prevEnable;
    bool _finished;
    bool _hasResult;
    bool _discardNextPeriod;

    float _relayCor;
    float _prevError;
    bool _havePrevError;

    uint32_t _runStartMs;
    uint32_t _lastPrintMs;
    uint32_t _lastPeakMs;
    uint8_t _periodCount;

    float _peakPos;
    float _peakNeg;
    bool _seekingPosPeak;

    float _periodSumSec;
    float _ampSumDeg;

    float _kp;
    float _kd;
    float _ku;
    float _pu;
    float _amp;
    bool _kpCapped;
};

#endif
