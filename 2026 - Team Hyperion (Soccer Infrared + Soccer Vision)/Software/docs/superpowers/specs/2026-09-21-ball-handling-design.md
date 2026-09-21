# BallHandling Library Design

**Date:** 2026-09-21  
**Scope:** Kicker safety library (`Ball_handling.h` / `Ball_handling.cpp`) — no dribbler API.

## Goal

`BallHandling` owns all kicker safety. Strategy code in `main.cpp` may call `kick()` at any time; the library decides whether a kick actually fires based on charge, voltage, photogate possession, pulse state, and cooldown.

## Approach

Self-contained class (same pattern as `DriveSystem`): owns its `VoltageDivider`, timers, kick count, and pin state. Main only constructs one instance and calls `init` / `update` / `kick` / `can_kick`.

## Public API

```cpp
class BallHandling {
public:
    BallHandling();
    void init();
    void update();
    void kick();
    bool can_kick();
    int get_current_kicks();
};
```

- **`init()`** — Configure `KICKER_PIN` (OUTPUT, idle HIGH), `PHOTOGATE_PIN` (INPUT), init kicker voltage divider. Set `kicks = MAX_KICKS`. Cooldown clear so first kick is allowed if other checks pass.
- **`update()`** — Call every loop. Ends pulse, starts cooldown when pulse ends, regenerates kicks on recharge interval.
- **`kick()`** — If `can_kick()`, start pulse (pin LOW), decrement kicks. Otherwise no-op.
- **`can_kick()`** — `true` only when all safety conditions pass (see below).
- **`get_current_kicks()`** — Current charge count.

No dribbler methods.

## Safety conditions (`can_kick`)

All must be true:

1. `kicks > 0`
2. Kicker capacitor voltage ≥ `KICKER_REQUIRED_VOLT` (via existing `VoltageDivider` on `KICKER_VD_PIN`)
3. Photogate shows ball held: `analogRead(PHOTOGATE_PIN) > PHOTOGATE_THRESH` (placeholder polarity; invert in Config if hardware is opposite)
4. Not currently pulsing (`!isKicking`)
5. Kick cooldown finished (`cooldownTimer.time_has_passed_no_update()`, or treated as clear after `init` before any kick)

## Timers (three)

| Timer | Role |
|-------|------|
| `pulseTimer` | Duration pin stays LOW while kicking |
| `rechargeTimer` | Every interval, `kicks++` if below max |
| `cooldownTimer` | After pulse ends, block new kicks until elapsed |

### `update()` sequence

1. If `isKicking` and pulse duration elapsed → set pin HIGH, `isKicking = false`, **reset/start cooldown timer**.
2. If `kicks < MAX_KICKS` and recharge interval elapsed → `kicks++` (recharge timer self-resets via `time_has_passed()`).

### `kick()` sequence

1. If `!can_kick()` return.
2. `digitalWrite(KICKER_PIN, LOW)`.
3. `isKicking = true`.
4. `kicks--`.
5. Reset `pulseTimer`.

Cooldown starts only when the pulse **ends** (in `update`), not when the kick begins.

## Pin polarity

- Idle / not kicking: `KICKER_PIN` **HIGH**
- During kick pulse: `KICKER_PIN` **LOW**

## Config placeholders (`Config.h`)

All tunables live in `Config.h` as placeholders (same style as existing robot voltage defines). Suggested names:

```cpp
// --- Ball handling / kicker ---
#define KICK_PULSE_US              10000UL    // pin LOW duration
#define KICK_RECHARGE_US           2000000UL  // +1 kick interval
#define KICK_COOLDOWN_US           500000UL   // after pulse ends
#define MAX_KICKS                  3
#define KICKER_REQUIRED_VOLT       20.0f
#define KICKER_VOLTAGE_STABALISER  71.58083   // calibrate later
#define KICKER_VOLTAGE_OFFSET      0.0146378  // calibrate later
#define PHOTOGATE_THRESH           512        // ball held if reading > this
```

Pins already exist in `Pins.h`: `KICKER_PIN`, `PHOTOGATE_PIN`, `KICKER_VD_PIN`.

## Ownership / dependencies

- Includes: `Arduino.h`, `Config.h`, `Pins.h`, `Timer.h`, `Voltage_divider.h`
- Private: `VoltageDivider kickerVd`, three `Timer`s, `int kicks`, `bool isKicking`

## Main usage (integration sketch)

```cpp
BallHandling ballHandle;

void setup() {
    // ...
    ballHandle.init();
}

void loop() {
    ballHandle.update();
    // strategy:
    if (/* want to shoot */) {
        ballHandle.kick();  // safe no-op if not ready
    }
}
```

Wiring `main.cpp` can be a follow-up; this spec’s implementation focus is the library files plus Config defines.

## Out of scope

- Dribbler control / API
- Strategy when to kick (orbit, goal tracking, etc.)
- Calibrated real voltage/photogate numbers (placeholders only)

## Error / edge behaviour

- Calling `kick()` while unsafe: silent no-op (by design).
- Calling `kick()` while already pulsing: blocked by `!isKicking`.
- Recharge never exceeds `MAX_KICKS`.
- After boot, kicks start at max; cooldown does not block until a pulse has completed once.
