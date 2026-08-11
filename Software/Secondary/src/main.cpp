/**
 * @file main.cpp
 * @brief Secondary Teensy 4.1 — TSSP IR ball localisation co-processor.
 *
 * @details
 * Samples a ring of TSSP580xx (or equivalent) IR detectors around the robot,
 * estimates ball direction and strength, and streams a framed UART packet to
 * the Primary Teensy on Serial1.
 *
 * Timing model (Teensy IntervalTimer ISRs):
 *   - readTimer     @ READ_PERIOD     — accumulate digital hits into rawValues[]
 *   - processTimer  @ PROCESS_PERIOD  — normalise, filter, vector-sum, TX packet
 *
 * During process_values(), the read ISR is briefly stopped so sampleCount and
 * rawValues[] are a consistent snapshot, then restarted after zeroing.
 *
 * Packet TX (5 bytes):
 *   [0xFF][0xFF][dir_hi][dir_lo][str]
 *   dir = ballDir * 100  (0.01° fixed-point, big-endian)
 *   str = ballStr        (uint8 magnitude after scaling)
 *
 * Sensor geometry: index i sits at angle i * (360 / TSSP_NUM) degrees.
 * Unit vectors (cos, sin) are precomputed in init_tssp().
 *
 * @note loop() is intentionally empty — all work runs in hardware timers.
 */

#include <Arduino.h>
#include "Config.h"
#include "Pins.h"

/* -------------------------------------------------------------------------- */
/* Sensor buffers                                                             */
/* -------------------------------------------------------------------------- */

/** GPIO pin map for each TSSP channel (see Pins.h). */
uint8_t tsspPins[TSSP_NUM] =
    {T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11, T12, T13,
     T14, T15, T16, T17, T18, T19, T20, T21, T22, T23, T24};

uint8_t rawValues[TSSP_NUM] = {0};              /**< Hit counts since last process. */
float processedValues[TSSP_NUM] = {0.0f};       /**< Duty-normalised % activity. */
float weightedValues[TSSP_NUM] = {0.0f};        /**< EMA-smoothed processed values. */
float previousWeightedValues[TSSP_NUM] = {0.0f};/**< EMA state z^{-1}. */
float sortedValues[TSSP_NUM] = {0.0f};          /**< Reserved / unused in current path. */
uint8_t indexValues[TSSP_NUM] = {0};            /**< Reserved / unused in current path. */
float tsspXValues[TSSP_NUM] = {0.0f};           /**< Unit-circle X for channel i. */
float tsspYValues[TSSP_NUM] = {0.0f};           /**< Unit-circle Y for channel i. */
float highest[TSSP_NUM] = {0.0f};               /**< Debug peak hold (optional). */
uint16_t sampleCount = 0;                       /**< Number of read() ISR ticks in window. */
float ballDir = 0.0f;                           /**< Estimated ball angle (deg, [0, 360)). */
float ballStr = 0.0f;                           /**< Estimated ball strength / proximity. */

/**
 * Per-channel additive calibration (counts) compensating uneven IR response /
 * mechanical occlusion. CONTROL vs non-CONTROL robots use different tables.
 */
#if CONTROL
float tsspAdd[TSSP_NUM] = {6.23f ,5.26f ,6.60f ,5.49f ,6.47f ,6.42f ,6.39f ,6.10f ,6.18f ,6.03f ,6.38f ,6.28f ,6.87f ,6.44f ,6.72f ,5.91f ,6.42f ,6.70f ,6.21f ,5.97f ,6.03f ,7.03f ,6.20f ,5.80f};
#else
float tsspAdd[TSSP_NUM] = {16.77f, 11.30f, 6.38f, 7.08f, 6.10f, 6.83f, 7.23f, 5.99f, 6.21f, 6.0f, 6.60f, 6.40f, 5.77f, 6.73f, 6.40f, 7.13f, 6.90f, 6.87f, 6.19f, 5.89f, 6.70f, 6.83f, 6.49f, 15.21f};
#endif

IntervalTimer readTimer;    /**< High-rate digital sampling ISR. */
IntervalTimer processTimer; /**< Lower-rate fusion + UART TX ISR. */

/**
 * @brief Configure TSSP GPIOs and precompute unit vectors for vector summing.
 *
 * INPUT_PULLUP: IR modules pull low when illuminated; inactive reads high.
 * Unit vectors assume equal angular spacing around 360°.
 */
void init_tssp() {
    for (uint8_t i = 0; i < TSSP_NUM; i++) {
        pinMode(tsspPins[i], INPUT_PULLUP);
        tsspXValues[i] = cos((float)i * (360.0f / (float)TSSP_NUM) * DEG_TO_RAD);
        tsspYValues[i] = sin((float)i * (360.0f / (float)TSSP_NUM) * DEG_TO_RAD);
    }
}

/**
 * @brief ISR: sample all detectors once and accumulate active-low hits.
 *
 * digitalReadFast is used for minimum ISR latency. Interrupts are briefly
 * masked around the full ring read so sampleCount stays consistent with
 * the buffer update (nested ISR safety vs processTimer).
 *
 * Active-low encoding: rawValues[i] += (1 - pin) → +1 when IR asserts LOW.
 */
void read() {
    noInterrupts();
    for (uint8_t i = 0; i < TSSP_NUM; i++) {
        rawValues[i] += 1 - digitalReadFast(tsspPins[i]);
    }
    sampleCount++;
    interrupts();
}

/**
 * @brief ISR: convert accumulated hits → ball vector → UART packet to Primary.
 *
 * Steps:
 *   1. Stop readTimer; bail (and restart) if no samples this window.
 *   2. Apply per-channel offset; optionally interpolate broken channels.
 *   3. Normalise by sampleCount and BALL_DUTY_CYCLE → percent activity.
 *   4. First-order EMA (ALPHA) for temporal smoothing.
 *   5. Weighted vector sum over the ring → (xSum, ySum).
 *   6. Strength from vector magnitude with floor/scale; direction from atan2.
 *   7. Emit sync-framed packet on Serial1; clear accumulators; restart readTimer.
 *
 * Direction mapping: 450° - atan2(y,x) in degrees, then wrap to [0, 360)
 * aligns 0° with the robot's mechanical forward reference.
 */
void process_values() {
    readTimer.end();

    if (sampleCount == 0) {
        readTimer.begin(read, READ_PERIOD);
        return;
    }

    /* --- Duty-cycle normalisation (+ calibration offsets) --- */
    for (uint8_t i = 0; i < TSSP_NUM; i++) {
        if (rawValues[i] != 0) {
            rawValues[i] += tsspAdd[i];
            if (rawValues[i] < 0.0f) {
                rawValues[i] = 0.0f;
            }
        }
        #if CONTROL
        /* CONTROL robot: no dead-channel interpolation in current build. */
        #else
        /* Replace known-bad channels with neighbour average. */
        if (i == 17 || i == 10 || i == 8) {
            rawValues[i] = (rawValues[i - 1] + rawValues[i + 1]) / 2.0f;
        }
        #endif

        processedValues[i] =
            ((float)rawValues[i] / (float)sampleCount / BALL_DUTY_CYCLE) * 100.0f;
    }

    /* --- Exponential moving average --- */
    for (uint8_t i = 0; i < TSSP_NUM; i++) {
        weightedValues[i] =
            previousWeightedValues[i] + (ALPHA * (processedValues[i] - previousWeightedValues[i]));
        previousWeightedValues[i] = weightedValues[i];
    }

    /* --- Soft-argmax via vector sum on the unit circle --- */
    float xSum = 0.0f;
    float ySum = 0.0f;

    for (uint8_t i = 0; i < TSSP_NUM; i++) {
        xSum += tsspXValues[i] * weightedValues[i];
        ySum += tsspYValues[i] * weightedValues[i];
    }

    xSum /= (float)TSSP_NUM;
    ySum /= (float)TSSP_NUM;

    ballStr = sqrtf(xSum * xSum + ySum * ySum) * 4.0f;

    /* Reject noise floor and rescale remaining headroom to ~0..100. */
    ballStr -= TSSP_STR_FLOOR;
    ballStr *= 100.0f / (100.0f - TSSP_STR_FLOOR);
    if (ballStr <= 0.0f) {
        ballStr = 0.0f;
        ballDir = 0.0f;
    } else {
        ballDir = 450.0f - RAD_TO_DEG * atan2f(ySum, xSum);
        if (ballDir > 360.0f) {
            ballDir -= 360.0f;
        }
    }

    /* --- UART TX to Primary --- */
    Serial1.write(255);
    Serial1.write(255);

    uint16_t dir = ballDir * 100; /* 0.01° LSB */
    uint8_t str  = ballStr;
    Serial1.write((dir >> 8) & 0xFF);
    Serial1.write(dir & 0xFF);
    Serial1.write(str);

    #if DEBUG_TSSP_RAW
    for (uint8_t i = 0; i < TSSP_NUM - 1; i++) {
        Serial.print(rawValues[i]);
        Serial.print("\t");
    }
    Serial.println(rawValues[TSSP_NUM - 1]);
    #endif

    #if DEBUG_TSSP_PROCESS
    for (uint8_t i = 0; i < TSSP_NUM - 1; i++) {
        Serial.print(processedValues[i]);
        Serial.print("\t");
    }
    Serial.println(processedValues[TSSP_NUM - 1]);
    #endif

    #if DEBUG_TSSP_WEIGHTED
    for (uint8_t i = 0; i < TSSP_NUM - 1; i++) {
        Serial.print(weightedValues[i]);
        Serial.print("\t");
    }
    Serial.println(weightedValues[TSSP_NUM - 1]);
    #endif

    #if DEBUG_TSSP_BALL
    Serial.printf("Dir: %.2f\tStr: %.2f\n", ballDir, ballStr);
    #endif

    #if DEBUG_TSSP
    Serial.println();
    #endif

    /* Clear window and resume high-rate sampling. */
    for (int i = 0; i < TSSP_NUM; i++) {
        rawValues[i] = 0;
    }
    sampleCount = 0;

    readTimer.begin(read, READ_PERIOD);
}

/**
 * @brief Bring up debug serial (optional), Secondary↔Primary UART, and timers.
 */
void setup()
{
    #if DEBUG_TSSP
    Serial.begin(9600);
    #endif

    Serial1.begin(115200);
    init_tssp();
    readTimer.begin(read, READ_PERIOD);
    processTimer.begin(process_values, PROCESS_PERIOD);
}

/** @brief Unused — processing is entirely timer-driven. */
void loop() {}
