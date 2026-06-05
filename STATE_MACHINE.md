# State Machine Architecture

## Overview

The Stepper Controller uses a non-blocking, event-driven state machine on the RP2040.
All timing uses `millis()` comparisons — no `delay()` calls block the main loop.
Sensor inputs are polled every loop iteration; motor steps and display updates are
rate-limited by independent interval timers.

---

## State Diagram

```
                    ┌──────────────────────────────────┐
                    │            STATE_IDLE             │
                    │  Motor off, all outputs off       │
                    │  Waiting for spray valve          │
                    └─────────────┬────────────────────┘
                                  │ Spray valve ON
                                  ▼
                    ┌──────────────────────────────────┐
                    │           STATE_HOMING            │
                    │  Motor steps toward limit (−dir)  │
                    │  Yellow LED ON                    │
                    └─────────────┬────────────────────┘
                                  │ Limit switch pressed
                                  │ (motorPosition reset to 0)
                                  ▼
                    ┌──────────────────────────────────┐
                    │           STATE_PARKED            │
                    │  Motor steps forward to PARK_STEPS│
                    │  Motor disabled when at park      │
                    │  Green LED ON                     │
                    └──────┬──────────────┬────────────┘
                           │              │
              Spray ON,    │              │ Spray ON,
              no flow      │              │ flow ON
                           ▼              ▼
          ┌─────────────────────┐   (→ STATE_SPRAY_ACTIVE)
          │  STATE_WAITING_SPRAY│
          │  Fan 50 %           │
          │  Yellow LED ON      │
          └──────────┬──────────┘
                     │ Flow ON
                     ▼
                    ┌──────────────────────────────────┐
                    │         STATE_SPRAY_ACTIVE        │
                    │  Fan 100 %, Ultrasonic ON         │
                    │  Motor moves to CENTER_STEPS      │
                    │  2 s stabilisation wait           │
                    │  Green LED ON                     │
                    └─────────────┬────────────────────┘
                                  │ Motor at centre AND 2 s elapsed
                                  ▼
                    ┌──────────────────────────────────┐
                    │         STATE_OSCILLATING         │
                    │  Motor sweeps ±OSCILLATION_STEPS  │
                    │  One step per OSCILLATION_DELAY   │
                    │  Fan 100 %, Ultrasonic ON         │
                    │  Green LED ON                     │
                    └─────────────┬────────────────────┘
                                  │ 20 sweeps complete
                                  │ OR spray/flow lost
                                  ▼
                    ┌──────────────────────────────────┐
                    │            STATE_IDLE             │
                    └──────────────────────────────────┘

At any point, spray or flow loss during SPRAY_ACTIVE or OSCILLATING
immediately transitions back to STATE_IDLE.

                    ┌──────────────────────────────────┐
                    │           STATE_ERROR             │
                    │  Motor off, all outputs off       │
                    │  Yellow LED blinks at 1 Hz        │
                    │  Requires power-cycle to reset    │
                    └──────────────────────────────────┘
```

---

## State Definitions

### STATE_IDLE
**Entry**: Power-on or any cleaning cycle end  
**Actions**: Motor disabled, all LEDs off, fan off, ultrasonic off  
**Exit**: Spray valve goes HIGH → STATE_HOMING

---

### STATE_HOMING
**Entry**: Spray valve detected  
**Actions**:
- Motor enabled
- Steps toward limit switch at `MOTOR_UPDATE_INTERVAL` (50 ms/step)
- Yellow LED ON

**Exit**: Limit switch pressed → `motorPosition` reset to 0 → STATE_PARKED

---

### STATE_PARKED
**Entry**: After homing completes  
**Actions**:
- Motor steps forward (1 step per 50 ms) until `motorPosition >= PARK_STEPS`
- Motor disabled once at park position
- Green LED ON

**Exit conditions**:
- At park AND spray ON AND no flow → STATE_WAITING_SPRAY
- At park AND spray ON AND flow ON → STATE_SPRAY_ACTIVE

---

### STATE_WAITING_SPRAY
**Entry**: Spray detected but flow not yet present  
**Actions**:
- Motor disabled
- Fan at 50 % (PWM 128) to prime circulation
- Yellow LED ON

**Exit conditions**:
- Flow detected → STATE_SPRAY_ACTIVE
- Spray lost → STATE_PARKED

---

### STATE_SPRAY_ACTIVE
**Entry**: Both spray and flow confirmed  
**Actions**:
- Fan at 100 % (PWM 255)
- Ultrasonic relay ON (GPIO 4 LOW)
- Motor steps toward `CENTER_STEPS` (1 step per 50 ms)
- Green LED ON
- Waits `SPRAY_ACTIVE_WAIT` (2 s) for the system to stabilise

**Exit conditions**:
- Motor at centre AND ≥ 2 s elapsed → STATE_OSCILLATING
- Spray or flow lost → STATE_IDLE

---

### STATE_OSCILLATING
**Entry**: From SPRAY_ACTIVE once motor is centred and system is stable  
**Actions**:
- Motor alternates direction every `OSCILLATION_STEPS` steps
- One step fired every `OSCILLATION_DELAY` ms
- `oscillationCount` increments after each directional sweep
- Fan 100 %, Ultrasonic ON, Green LED ON

**Oscillation pattern** (default parameters):
```
Sweep 1: pos 26 → 10  (16 steps × 10 s = 160 s, toward limit)
Sweep 2: pos 10 → 26  (16 steps × 10 s = 160 s, away from limit)
Sweep 3: pos 26 → 10  ...
...
Sweep 20: pos 10 → 26
```

Motor oscillates between `CENTER_STEPS − OSCILLATION_STEPS` and `CENTER_STEPS`.

**Timing**:
- 1 sweep = 16 steps × 10 s = 160 s ≈ 2.7 min
- 20 sweeps = 3 200 s ≈ **53 min** total cleaning time

**Exit conditions**:
- `oscillationCount >= OSCILLATION_CYCLES` → STATE_IDLE (cycle complete)
- Spray or flow lost → STATE_IDLE (emergency stop)

---

### STATE_ERROR *(future)*
**Entry**: Fault condition (not yet wired to automatic detection)  
**Actions**: Motor disabled, all outputs off, yellow LED blinks 1 Hz  
**Recovery**: Power-cycle required (no automatic recovery)

---

## State Transition Table

| Current State | Condition | Next State | Key Actions |
|---|---|---|---|
| IDLE | Spray ON | HOMING | Enable motor, yellow LED |
| HOMING | Limit switch pressed | PARKED | Reset position=0, step to park |
| PARKED | At park + Spray ON + no flow | WAITING_SPRAY | Fan 50 %, yellow LED |
| PARKED | At park + Spray ON + Flow ON | SPRAY_ACTIVE | Fan 100 %, ultrasonic ON |
| WAITING_SPRAY | Flow ON | SPRAY_ACTIVE | Fan 100 %, ultrasonic ON |
| WAITING_SPRAY | Spray OFF | PARKED | Fan off |
| SPRAY_ACTIVE | At centre + 2 s elapsed | OSCILLATING | Begin alternating sweeps |
| SPRAY_ACTIVE | Spray or flow lost | IDLE | All outputs off |
| OSCILLATING | 20 sweeps complete | IDLE | All outputs off |
| OSCILLATING | Spray or flow lost | IDLE | Emergency stop |

---

## Timing Parameters

```cpp
const int STEPS_PER_MM       = 1;      // Calibrate to your hardware
const int PARK_MM            = 7;      // mm from limit switch
const int CENTER_MM          = 26;     // mm from limit switch
const int OSCILLATION_STEPS  = 16;     // Steps per directional sweep
const unsigned long OSCILLATION_DELAY  = 10000; // ms per step
const unsigned long OSCILLATION_CYCLES = 20;    // Sweeps before stopping
const unsigned long SPRAY_ACTIVE_WAIT  = 2000;  // ms stabilisation delay
const unsigned long MOTOR_UPDATE_INTERVAL = 50; // ms per step during homing/parking
```

---

## Control Flow (Main Loop)

```cpp
void loop() {
    readEncoder();           // Every 20 ms
    readSensors();           // Every iteration (debounced internally)
    updateStateMachine();    // Check transition conditions
    handleState();           // Execute current-state actions
    updateDisplay();         // Redraw LCD only on state/position change
}
```

---

## Output Behaviour by State

### LED Indicators

| State | Green | Yellow |
|-------|-------|--------|
| IDLE | OFF | OFF |
| HOMING | OFF | ON |
| PARKED | ON | OFF |
| WAITING_SPRAY | OFF | ON |
| SPRAY_ACTIVE | ON | OFF |
| OSCILLATING | ON | OFF |
| ERROR | OFF | Blink 1 Hz |

### Fan Speed

| State | PWM | % |
|-------|-----|---|
| IDLE / HOMING / PARKED | 0 | 0 % |
| WAITING_SPRAY | 128 | 50 % |
| SPRAY_ACTIVE / OSCILLATING | 255 | 100 % |

### Ultrasonic Relay (GPIO 4, active-low)

| State | GPIO 4 | Relay |
|-------|--------|-------|
| IDLE → WAITING_SPRAY | HIGH | OFF |
| SPRAY_ACTIVE / OSCILLATING | LOW | ON |
| Return to IDLE | HIGH | OFF |

---

## Safety Features

1. **Motor disabled at startup** — EN HIGH until HOMING begins
2. **Limit switch debounced** — 20 ms settle time prevents bounce triggering
3. **Two-condition spray start** — Both spray AND flow required to begin cleaning
4. **Emergency stop** — Spray or flow loss in SPRAY_ACTIVE/OSCILLATING → immediate IDLE
5. **Ultrasonic interlock** — Ultrasonic only ON during active cleaning states
6. **StealthChop current limiting** — TMC2130 limits motor current via `rms_current`

---

## Tuning Parameters

Edit constants at the top of `src/main.cpp`:

```cpp
// Longer cleaning: increase sweeps
const unsigned long OSCILLATION_CYCLES = 30;   // Was 20 (~80 min)

// Faster steps: decrease delay
const unsigned long OSCILLATION_DELAY = 5000;  // Was 10000 (5 s/step → 26 min)

// Wider oscillation: increase steps (ensure CENTER_STEPS − OSCILLATION_STEPS > 0)
const int OSCILLATION_STEPS = 24;              // Was 16

// Park further from limit:
const int PARK_MM = 10;                        // Was 7

// Motor current (reduce if hot, increase if stalling):
driver.rms_current(800);                       // Was 600 mA
```

---

## References

- [UML State Machine](https://en.wikipedia.org/wiki/UML_state_machine)
- [TMCStepper Library](https://github.com/teemuatlut/TMCStepper)
- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
