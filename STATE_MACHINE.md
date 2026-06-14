# State Machine Architecture

## Overview

MEGASONIC uses a non-blocking, event-driven state machine on Core 0 of the RP2040.
All timing uses `millis()` / `micros()` comparisons — no `delay()` blocks the loop.
Sensors are polled every iteration; the encoder uses GPIO interrupts and is consumed by
the main loop. Motor steps are rate-limited by interval timers. Core 1 owns the LCD.

Two things shape the behaviour throughout:

- **Operating mode** — the **Debug** setting. When Debug is `OFF`
  (`SENSOR_INPUTS_ENABLED = true`), the spray valve and flow sensor gate the cycle.
  When Debug is `ON` (the default), those inputs are ignored and the cycle is driven
  from the menu (`sensorBypassCycleArmed`).
- **Position knowledge** — `needsHoming`. Whenever the motor is de-energised the
  position is considered unknown, so the controller **re-homes before every run**.
  Homing clears the flag once the limit switch is reached.

Motion is expressed in **arm angle** (tenths of a degree, `…DegX10`). `PARK_DEG_X10`
is a small angle near the limit switch; `CENTER_DEG_X10` is the sweep centre over the
wafer. The sweep half-width is derived from the wafer diameter and arm length.

---

## State Diagram

```
        ┌───────────────────────────────────────────────┐
        │                  STATE_IDLE                    │
        │  Motor disabled, all outputs off               │
        │  START (menu)  ── or, in sensor mode, spray ON │
        └───────────────┬───────────────────────────────┘
                        │  capture homingStartPos
                        ▼
        ┌───────────────────────────────────────────────┐
        │                 STATE_HOMING                   │
        │  Step toward limit (dir −1), yellow LED         │
        │  Abort → ERROR if no endstop within 70° travel  │
        └───────────────┬───────────────────────────────┘
                        │  limit pressed → position = 0, needsHoming = false
                        ▼
        ┌───────────────────────────────────────────────┐
        │                 STATE_PARKED                   │
        │  Move to PARK_DEG_X10, then park-hold current   │
        │  Green LED                                      │
        └──┬─────────────┬──────────────┬────────────────┘
           │ stop / none │ DEBUG armed   │ sensor mode
           ▼             ▼  or spray+flow ▼  spray, no flow
      STATE_IDLE   STATE_SPRAY_ACTIVE   STATE_WAITING_SPRAY
      (disable)          ▲                     │ flow ON
                         └─────────────────────┘
                         ▼
        ┌───────────────────────────────────────────────┐
        │              STATE_SPRAY_ACTIVE                │
        │  Fan 100 %, generator ON over wafer             │
        │  Move to sweep start; wait SPRAY_ACTIVE_WAIT    │
        └───────────────┬───────────────────────────────┘
                        │  at start AND ≥ 2 s
                        ▼
        ┌───────────────────────────────────────────────┐
        │               STATE_OSCILLATING                │
        │  Sweep across the wafer per SWEEP_TIME/profile  │
        │  Fan 100 %, generator ON only over the wafer    │
        └───────────────┬───────────────────────────────┘
                        │  OSCILLATION_CYCLES done (0 = forever)
                        ▼
                  STATE_PARKED ── nothing pending → STATE_IDLE (disable)

  STOP (menu) at any active state → PARKED → disable → IDLE.
  In sensor mode, loss of spray/flow during SPRAY_ACTIVE/OSCILLATING → PARKED.

        ┌───────────────────────────────────────────────┐
        │                 STATE_ERROR                    │
        │  Motor disabled, outputs off, yellow LED 1 Hz   │
        │  Entered on driver fault or home-search timeout │
        │  Recover with START (re-homes, clears fault)    │
        └───────────────────────────────────────────────┘
```

---

## Menu START / STOP

`START/STOP` is menu item 0; the icon shows ▶ when idle and ■ when running.

- **START from `IDLE` or `ERROR`** — clears `faultLatched`/`stopRequested`, arms the
  bypass cycle in DEBUG mode, captures `homingStartPos`, and enters `HOMING`. This
  always re-establishes position first.
- **START from `PARKED` with known position (DEBUG mode)** — begins the cleaning cycle
  directly (`SPRAY_ACTIVE`).
- **STOP (any other active state)** — sets `stopRequested`, routes through `PARKED`,
  then disables the motor and returns to `IDLE`.

---

## State Definitions

### STATE_IDLE
**Entry**: power-on, end of cycle, or after a stop.
**Actions**: motor disabled, LEDs off, fan off, generator off.
**Exit**: driver fault → `ERROR`; START → `HOMING`; (sensor mode) spray ON → `HOMING`.

### STATE_HOMING
**Entry**: START or spray detected.
**Actions**: motor enabled, steps toward the limit at `MOTOR_UPDATE_INTERVAL_US`,
yellow LED on.
**Safety**: if the limit is not reached within `HOMING_MAX_DEG_X10` (70°) of travel,
the motor is disabled and the state latches `ERROR`.
**Exit**: limit pressed → `motorPosition = 0`, `needsHoming = false` →
`PARKED` (or `ERROR`/`IDLE` if a fault or stop/abort is pending).

### STATE_PARKED
**Entry**: after homing, after a stop, or at the end of a cleaning cycle.
**Actions**: steps to `PARK_DEG_X10`; once there, switches to park-hold current
(reduced) — green LED on, fan off, generator off.
**Exit (once at the park angle)**:
- fault latched → `ERROR`
- `stopRequested` → `IDLE` (motor disabled)
- DEBUG bypass armed, or (sensor mode) spray + flow → `SPRAY_ACTIVE`
- (sensor mode) spray, no flow → `WAITING_SPRAY`
- nothing pending → `IDLE` (motor disabled)

### STATE_WAITING_SPRAY *(sensor mode only)*
**Entry**: spray detected but no flow yet.
**Actions**: motor disabled, fan at 50 % (`FAN_WAITING`), yellow LED on.
**Exit**: flow ON → `SPRAY_ACTIVE`; spray lost → `PARKED`.

### STATE_SPRAY_ACTIVE
**Entry**: cycle gated (bypass armed, or spray + flow confirmed).
**Actions**: fan 100 %, generator ON **while the arm is over the wafer**, green LED on;
moves to the sweep start (`sweepBackSteps()`); waits `SPRAY_ACTIVE_WAIT` (2 s).
**Exit**: at the sweep start AND ≥ 2 s elapsed → `OSCILLATING`;
(sensor mode) spray or flow lost → `PARKED`.

### STATE_OSCILLATING
**Entry**: from `SPRAY_ACTIVE` once at the sweep start and settled.
**Actions**: alternates between the sweep start and end; step interval comes from
`SWEEP_TIME_MS` shaped by the velocity profile; one `SWEEP_TIME_MS` is a full
back-forward-back cycle, and `oscillationCount` increments after each return to the back.
Fan 100 %, generator ON over the wafer, green LED on.
**Exit**: `oscillationCount >= OSCILLATION_CYCLES` (when > 0) → `PARKED` (cycle complete);
(sensor mode) spray or flow lost → `PARKED`.

### STATE_ERROR
**Entry**: a latched driver fault (overtemperature, short-to-ground, or charge-pump
undervoltage) detected by `pollDriverStatus()`, or a home-search timeout.
**Actions**: motor disabled, all outputs off, yellow LED blinks at 1 Hz.
**Recovery**: press START — it clears `faultLatched` and re-homes. (No power-cycle needed.)

---

## State Transition Table

| Current State | Condition | Next State | Key Actions |
|---|---|---|---|
| IDLE | Driver fault | ERROR | Disable, needsHoming |
| IDLE | START / (sensor mode) spray ON | HOMING | Capture homingStartPos, enable, yellow LED |
| HOMING | Endstop not found within 70° | ERROR | Disable |
| HOMING | Limit pressed | PARKED | position = 0, needsHoming = false |
| HOMING | Limit pressed + fault/abort | ERROR / IDLE | Disable |
| PARKED | At park + stop / nothing pending | IDLE | Disable |
| PARKED | At park + bypass armed / spray+flow | SPRAY_ACTIVE | Fan 100 %, generator over wafer |
| PARKED | At park + spray, no flow (sensor) | WAITING_SPRAY | Fan 50 %, yellow LED |
| WAITING_SPRAY | Flow ON / bypass armed | SPRAY_ACTIVE | Fan 100 % |
| WAITING_SPRAY | Spray lost | PARKED | Fan off |
| SPRAY_ACTIVE | At sweep start + 2 s | OSCILLATING | Begin alternating sweeps |
| SPRAY_ACTIVE | (sensor) spray/flow lost | PARKED | — |
| OSCILLATING | Cycles complete (> 0) | PARKED | Disarm bypass |
| OSCILLATING | (sensor) spray/flow lost | PARKED | — |
| any active | Driver fault | PARKED → ERROR | Park, then disable + latch |

---

## Timing & Motion Parameters

```cpp
int  PARK_DEG_X10   = 70;     // 7.0° park angle (near the limit)
int  CENTER_DEG_X10 = 260;    // 26.0° sweep centre (over the wafer)
int  ARM_LENGTH_MM  = 250;    // arm length (transducer radius)
unsigned long SWEEP_TIME_MS      = 4000;  // ms per full back-forward-back cycle
unsigned long OSCILLATION_CYCLES = 4;     // full cycles to run (0 = run forever)
const unsigned long SPRAY_ACTIVE_WAIT      = 2000; // ms settle before oscillation
const unsigned long MOTOR_UPDATE_INTERVAL_US = 500; // µs per step (homing/parking)
const int           HOMING_MAX_DEG_X10     = 700;  // 70° home-search limit
```

Steps are derived from angle: `steps = degX10 × FULL_STEPS_PER_REV × microsteps / 3600`.
The sweep half-width = `asin((waferØ/2) / armLength)`, so the sweep extremes reach the
wafer edges; `Back-Centre` travels half of this, `Back-Front` the full width.

---

## Control Flow (Dual-Core)

**Core 0** — state machine, I/O, driver:
```cpp
void loop() {
    handleSerialDebug();
    readEncoder();            // consumes interrupt encoder/button events + acceleration
    if (menuButtonPressed) handleMenuSelect();
    readSensors();            // limit always; spray/flow only when Debug = OFF
    updateStateMachine();     // evaluate transitions
    handleState();            // drive motor, LEDs, fan, generator
    pollDriverStatus();       // every 500 ms; latches faults
    // debounced auto-save of settings to flash
}
```

**Core 1** — LCD rendering:
```cpp
void setup1() { while (!core0_ready) tight_loop_contents(); }
void loop1()  { updateDisplay(); delay(50); }   // ~20 fps, redraws on change only
```

Variables shared between cores (`currentState`, `motorPosition`, `menuIndex`,
`ultrasonicActive`, …) are `volatile`. A single mutex serialises all SPI access across
both cores (LCD on SPI0, TMC2130 on SPI1).

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

### Ultrasonic Generator (GPIO 4, active-low)

| State | Relay |
|-------|-------|
| IDLE → WAITING_SPRAY | OFF |
| SPRAY_ACTIVE / OSCILLATING | ON **only while the arm is over the wafer** |
| PARKED / ERROR | OFF |

---

## Safety Features

1. **Re-home before every run** — position is treated as unknown whenever the motor is
   disabled (`needsHoming`); START always homes first.
2. **Home-search timeout** — homing aborts to `ERROR` if the endstop is not found within
   70° of travel.
3. **Driver fault latch** — overtemperature, short-to-ground, or charge-pump
   undervoltage parks the arm, disables the motor, and latches `ERROR`.
4. **Ultrasonic interlock** — the generator is energised only while the arm tip is over
   the wafer, and only in active cleaning states.
5. **Park-then-disable stop** — STOP always parks the arm before cutting current.
6. **Limit-switch debounce** — 20 ms settle prevents bounce from false-triggering home.
7. **Recoverable ERROR** — START clears the fault and re-homes; no power-cycle required.

---

## Tuning

All sweep and hardware parameters are editable on-device (Settings / Setup) and persist
to flash. To change defaults, edit the constants near the top of `src/main.cpp`
(`PARK_DEG_X10`, `CENTER_DEG_X10`, `ARM_LENGTH_MM`, `SWEEP_TIME_MS`,
`OSCILLATION_CYCLES`, `driverCurrent`, `driverMicrosteps`).

---

## References

- [UML State Machine](https://en.wikipedia.org/wiki/UML_state_machine)
- [TMCStepper Library](https://github.com/teemuatlut/TMCStepper)
- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
</content>
