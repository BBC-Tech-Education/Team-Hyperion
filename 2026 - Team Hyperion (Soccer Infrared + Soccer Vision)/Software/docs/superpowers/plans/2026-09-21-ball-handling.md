# BallHandling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement a self-contained `BallHandling` kicker safety library with charge bank, voltage gate, photogate possession, pulse, and cooldown.

**Architecture:** One class owns `VoltageDivider` + three `Timer`s + kick state. `main` may call `kick()` anytime; `update()` must run every loop. Matches existing `DriveSystem` / `Timer` / `VoltageDivider` patterns. No unit-test harness in this PlatformIO firmware repo — verify by code review against the spec and optional compile of `Primary`.

**Tech Stack:** Arduino / PlatformIO C++, existing `Timer`, `VoltageDivider`, `Config.h`, `Pins.h`

---

## File map

| File | Responsibility |
|------|----------------|
| `lib/Config/Config.h` | Placeholder tunables for pulse / recharge / cooldown / max kicks / VD / photogate |
| `lib/Ball Handling/Ball_handling.h` | `BallHandling` class declaration |
| `lib/Ball Handling/Ball_handling.cpp` | Implementation of init / update / kick / can_kick / getters |

Out of scope for this plan: wiring into `Primary/src/main.cpp` (optional follow-up).

**Cooldown edge case:** After `init`, cooldown must not block. Use private `bool cooldownActive` (false until a pulse ends). `can_kick()` requires `!cooldownActive || cooldownTimer.time_has_passed_no_update()`.

**Commits:** Only if the user explicitly asks — do not auto-commit.

---

### Task 1: Add Config placeholders

**Files:**
- Modify: `lib/Config/Config.h` (before `#endif`, after voltage divider section)

- [ ] **Step 1: Insert ball-handling defines**

Add a new section (keep robot battery defines as they are; add kicker-specific ones):

```cpp
/////////////////////////////// BALL HANDLING /////////////////////////////////

#define KICK_PULSE_US              10000UL    // pin LOW duration (us)
#define KICK_RECHARGE_US           2000000UL  // +1 kick every this many us
#define KICK_COOLDOWN_US           500000UL   // after pulse ends (us)
#define MAX_KICKS                  3
#define KICKER_REQUIRED_VOLT       20.0f
#define KICKER_VOLTAGE_STABALISER  71.58083   // calibrate later
#define KICKER_VOLTAGE_OFFSET      0.0146378  // calibrate later
#define PHOTOGATE_THRESH           512        // ball held if analogRead > this
```

- [ ] **Step 2: Sanity-check names match Pins**

Confirm `Pins.h` still has `KICKER_PIN`, `PHOTOGATE_PIN`, `KICKER_VD_PIN` — no pin changes needed.

---

### Task 2: Write `Ball_handling.h`

**Files:**
- Create/overwrite: `lib/Ball Handling/Ball_handling.h`

- [ ] **Step 1: Write the full header**

```cpp
#ifndef BALL_HANDLING_H
#define BALL_HANDLING_H

#include <Arduino.h>
#include "Config.h"
#include "Pins.h"
#include "Timer.h"
#include "Voltage_divider.h"

class BallHandling {
public:
    BallHandling();
    void init();
    void update();
    void kick();
    bool can_kick();
    int get_current_kicks();

private:
    VoltageDivider kickerVd;
    Timer pulseTimer;
    Timer rechargeTimer;
    Timer cooldownTimer;

    int kicks;
    bool isKicking;
    bool cooldownActive;

    bool ball_held();
};

#endif
```

---

### Task 3: Write `Ball_handling.cpp`

**Files:**
- Create/overwrite: `lib/Ball Handling/Ball_handling.cpp`

- [ ] **Step 1: Constructor + helpers**

```cpp
#include "Ball_handling.h"

BallHandling::BallHandling()
    : kickerVd(KICKER_VD_PIN, KICKER_VOLTAGE_STABALISER, KICKER_VOLTAGE_OFFSET),
      pulseTimer(KICK_PULSE_US),
      rechargeTimer(KICK_RECHARGE_US),
      cooldownTimer(KICK_COOLDOWN_US),
      kicks(MAX_KICKS),
      isKicking(false),
      cooldownActive(false) {}

bool BallHandling::ball_held() {
    return analogRead(PHOTOGATE_PIN) > PHOTOGATE_THRESH;
}

int BallHandling::get_current_kicks() {
    return kicks;
}
```

- [ ] **Step 2: `init()`**

```cpp
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
```

- [ ] **Step 3: `can_kick()` and `kick()`**

```cpp
bool BallHandling::can_kick() {
    if (kicks <= 0) return false;
    if (kickerVd.get_lvl() < KICKER_REQUIRED_VOLT) return false;
    if (!ball_held()) return false;
    if (isKicking) return false;
    if (cooldownActive && !cooldownTimer.time_has_passed_no_update()) return false;
    return true;
}

void BallHandling::kick() {
    if (!can_kick()) return;

    digitalWrite(KICKER_PIN, LOW);
    isKicking = true;
    kicks--;
    pulseTimer.update();
}
```

- [ ] **Step 4: `update()`**

```cpp
void BallHandling::update() {
    if (isKicking && pulseTimer.time_has_passed_no_update()) {
        digitalWrite(KICKER_PIN, HIGH);
        isKicking = false;
        cooldownActive = true;
        cooldownTimer.update();
    }

    if (kicks < MAX_KICKS && rechargeTimer.time_has_passed()) {
        kicks++;
    }
}
```

---

### Task 4: Spec checklist (manual verify)

**Files:** review only

- [ ] **Step 1: Walk the spec against the code**

| Spec requirement | Where |
|------------------|--------|
| Idle HIGH / kick LOW | `init` / `kick` / `update` |
| Pulse duration define | `KICK_PULSE_US` + `pulseTimer` |
| +1 kick every interval, capped | `update` recharge |
| −1 on kick | `kick` |
| Start at max | ctor + `init` |
| Voltage gate | `can_kick` + `kickerVd` |
| Photogate + threshold | `ball_held` |
| Cooldown after pulse ends | `update` then `can_kick` |
| `void kick()` + `bool can_kick()` | public API |
| `get_current_kicks()` | public API |
| No dribbler | confirmed absent |
| `update()` every loop (caller) | documented in header comment optional |

- [ ] **Step 2: Optional compile**

If PlatformIO env available for Primary:

```bash
pio run -d "Primary"
```

Expected: build succeeds (or only pre-existing unrelated errors).

---

## Spec coverage self-review

- All public API methods: Tasks 2–3  
- Config defines: Task 1  
- Three timers + cooldown-after-pulse + boot cooldown clear: Task 3  
- Voltage divider ownership: Task 2–3  
- No dribbler / no main wiring: explicit out of scope  

No TBD placeholders remain in the plan.
