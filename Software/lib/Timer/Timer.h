/**
 * @file Timer.h
 * @brief Micros()-based one-shot / interval helper for soft timing.
 *
 * Durations are in microseconds. Useful for LED blink latch, BT link timeout,
 * and other non-blocking delays without blocking the control loop.
 *
 * @date 17/06/25
 * @author J.Huang (Brisbane Boys' College)
 */

#ifndef TIMER_H
#define TIMER_H

class Timer {
public:
    /**
     * @param duration Interval length in microseconds.
     */
    Timer(unsigned long duration);

    /** Stamp lastUpdate = micros() (restart the interval). */
    void update();

    /**
     * @brief True if duration elapsed; auto-calls update() on success.
     * Suitable for periodic actions (blink, heartbeat).
     */
    bool time_has_passed();

    /**
     * @brief True if duration elapsed; does not restart the timer.
     * Suitable for sticky timeout / latch checks.
     */
    bool time_has_passed_no_update();

private:
    unsigned long timerDuration; /**< Configured period (µs). */
    unsigned long lastUpdate;    /**< micros() at last update(). */
};

#endif
