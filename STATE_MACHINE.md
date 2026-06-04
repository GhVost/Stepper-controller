# State Machine Architecture

## Overview

The Stepper Controller uses a non-blocking, event-driven state machine running on the RP2040. Each state executes independently with millisecond-based timing, allowing responsive sensor input and smooth motor control.

## State Diagram

```
                    ┌─────────────────────────────────────┐
                    │          START / IDLE               │
                    │  • Motor disabled, all outputs OFF  │
                    │  • Waiting for spray valve trigger  │
                    └──────────────────┬──────────────────┘
                                      │
                    Spray Valve ON ────┴─────────┐
                                                  │
                                          ┌───────▼────────┐
                                          │    HOMING      │
                                          │ • Motor ON     │
                                          │ • Move to      │
                                          │   limit switch │
                                          └───────┬────────┘
                                                  │
                                    Limit switch ─┴─────────┐
                                                            │
                                              ┌─────────────▼────────┐
                                              │     PARKED           │
                                              │ • Motor at position  │
                                              │   zero (Park = 7mm)  │
                                              │ • Wait for flow      │
                                              └─────────────┬────────┘
                                                            │
                                    Spray ON + Flow ON ─────┴────────┐
                                                                      │
                              ┌─────────────────────────────────┐    │
                              │  WAITING_SPRAY                 │◄───┘
                              │ • Spray detected but no flow    │
                              │ • Wait for flow sensor          │
                              └─────────────┬───────────────────┘
                                            │
                                    Flow ON ┼────────────────────┐
                                            │                    │
                    ┌───────────────────────┴──────────────┐     │
                    │                                      │     │
        ┌───────────▼─────────────┐      ┌────────────────▼───────┐
        │  SPRAY_ACTIVE           │      │  SPRAY_ACTIVE (cont'd) │
        │ • Prep for cleaning     │      │ • Fan ON (100%)        │
        │ • Green LED ON          │      │ • Ultrasonic ON        │
        │ • Motor ready           │      │ • Park position (7mm)  │
        │ • Fan starts (50%)      │      │ • Check timeout        │
        └─────────┬──────────────┘      └────────────┬───────────┘
                  │                                  │
       After 2s ──┴───────────────────────────────────┘
                  │
        ┌─────────▼──────────────┐
        │   OSCILLATING          │
        │ • Motor oscillates     │
        │   back/forth           │
        │ • Fan full speed (255%)│
        │ • Ultrasonic ON        │
        │ • Repeat cycles        │
        └─────────┬──────────────┘
                  │
    After 20 cycles ────► Return to IDLE ──────┐
    or Spray OFF ──────► Error/Reset ────────┐ │
    or Flow lost ──────► Error/Reset ────────┼─┘
                                             │
                        ┌────────────────────┘
                        │
                    ┌───▼────────────────┐
                    │  ERROR (Future)    │
                    │ • Motor disabled   │
                    │ • Red LED ON       │
                    │ • Manual reset     │
                    └────────────────────┘
```

## State Definitions

### STATE_IDLE
**Entry**: System startup or cycle complete  
**Behavior**:
- Motor disabled (EN pin HIGH)
- All outputs OFF (LEDs, fan, ultrasonic)
- Monitoring spray valve (GPIO 2)

**Exit Condition**: Spray valve goes HIGH (spray activated)  
**Next State**: STATE_HOMING

**Purpose**: Safe wait state, no motion

---

### STATE_HOMING
**Entry**: From IDLE when spray valve detected  
**Behavior**:
- Motor enabled (EN pin LOW)
- Direction set toward limit switch (DIR pin LOW by convention)
- Pulse STEP pin slowly (10ms per step)
- Poll limit switch (GPIO 28)
- Green LED ON

**Exit Condition**: Limit switch pressed (GPIO 28 LOW)  
**Next State**: STATE_PARKED

**Exit Actions**:
- Set position counter to 0
- Move forward PARK_POSITION steps (7mm)

**Purpose**: Establish zero reference position

---

### STATE_PARKED
**Entry**: After homing completes and motor moves to park position  
**Behavior**:
- Motor disabled (EN pin HIGH)
- Position = PARK_POSITION (7mm from limit)
- Monitor: Spray valve (GPIO 2) + Flow sensor (GPIO 3)
- Green LED ON

**Exit Condition**: Both spray HIGH AND flow HIGH  
**Next State**: STATE_WAITING_SPRAY or STATE_SPRAY_ACTIVE

**Purpose**: Safe hold position waiting for cleaning requirements

---

### STATE_WAITING_SPRAY
**Entry**: Spray detected but no flow yet  
**Behavior**:
- Motor disabled
- Yellow LED ON
- Fan starts (50% PWM = 128)
- Monitor flow sensor

**Exit Condition**: Flow detected (GPIO 3 HIGH)  
**Next State**: STATE_SPRAY_ACTIVE

**Timeout**: 30 seconds → STATE_ERROR (future)

**Purpose**: Intermediate state while system primes

---

### STATE_SPRAY_ACTIVE (Prep Phase)
**Entry**: Flow detected  
**Behavior**:
- Motor disabled
- Green LED ON
- Fan speed 100% (PWM 255)
- Ultrasonic relay ON (GPIO 4 LOW)
- Wait 2 seconds

**Exit Condition**: After 2 second delay  
**Next State**: STATE_OSCILLATING

**Purpose**: Allow ultrasonic and circulation to stabilize

---

### STATE_OSCILLATING
**Entry**: From SPRAY_ACTIVE after 2 second delay  
**Behavior**:
- Motor ENABLED (EN pin LOW)
- Move from CENTER_POSITION toward limit in OSCILLATION_STEPS increments
- One step every OSCILLATION_DELAY ms (10 seconds)
- Green LED ON
- Fan PWM 255 (full speed)
- Ultrasonic ON (GPIO 4 LOW)
- Increment cycle counter each 16 steps

**Pattern**:
```
Oscillation Step:  0    1    2   ...   15   (repeat pattern)
Motor Position:   26   25   24  ...    10  (moving toward limit)
Direction:        ◄────────────────────────
Wait:             10s  10s  10s        10s
```

**Exit Conditions**:
- Spray valve OFF → STATE_IDLE
- Flow sensor OFF → STATE_IDLE
- Cycle count ≥ 20 → STATE_IDLE
- Timeout > 60 min → STATE_ERROR

**Purpose**: Main cleaning operation

---

### STATE_ERROR (Future Implementation)
**Entry**: Fault condition detected  
**Behavior**:
- Motor disabled
- Red LED ON (GPIO 7 + 8 both at fault pattern)
- All outputs OFF
- Wait for manual reset

**Recovery**: Manual intervention required

---

## Timing Parameters

```cpp
// Motion
const int STEPS_PER_REVOLUTION = 200;     // NEMA17
const int PARK_POSITION = 7;              // mm from limit
const int CENTER_POSITION = 26;           // 34mm from limit
const int OSCILLATION_STEPS = 16;         // steps per cycle

// Delays
const unsigned long OSCILLATION_DELAY = 10000;    // ms between steps
const unsigned long OSCILLATION_CYCLES = 20;      // total repetitions
const unsigned long SPRAY_ACTIVE_WAIT = 2000;     // ms before oscillation
const unsigned long STATE_TIMEOUT = 3600000;      // 1 hour max

// Fan PWM
const int FAN_IDLE = 0;
const int FAN_WAITING_SPRAY = 128;  // 50%
const int FAN_SPRAY = 255;          // 100%
```

## State Transition Table

| Current State | Condition | Next State | Actions |
|---|---|---|---|
| IDLE | Spray ON | HOMING | Start motor, green LED |
| HOMING | Limit pressed | PARKED | Stop motor, move forward 7mm |
| PARKED | Spray + Flow | WAITING_SPRAY | Yellow LED, fan 50% |
| WAITING_SPRAY | Flow detected | SPRAY_ACTIVE | Wait 2s, prep system |
| SPRAY_ACTIVE | After 2s | OSCILLATING | Start oscillation, fan 100% |
| OSCILLATING | Cycles complete | IDLE | Motor off, all outputs off |
| OSCILLATING | Spray OFF | IDLE | Emergency stop, motor off |
| OSCILLATING | Flow lost | IDLE | Loss of circulation |
| Any | Timeout | ERROR | Fault condition (future) |

## Control Flow (Main Loop)

```cpp
void loop() {
    readSensors();              // Read GPIO inputs
    updateStateMachine();       // Check transition logic
    handleState();              // Execute state actions
    updateDisplay();            // Serial output (LCD future)
    delay(50);                  // Non-blocking, 50ms poll interval
}
```

### Sensor Polling
- **Spray valve**: Every 50ms
- **Flow sensor**: Every 50ms
- **Limit switch**: Every 50ms during HOMING
- **Motor position**: Tracked by step counter

### Timing
- No blocking delays (no `delay()` in motor movement)
- All delays use `millis()` comparison
- 50ms polling interval provides ~20Hz responsiveness
- Step pulses: 10ms pulse width for TMC2130

## Event Handling

### Spray Valve On/Off
- Monitored every loop
- ON → transitions toward HOMING
- OFF → forces return to IDLE (emergency stop)

### Flow Sensor On/Off
- Monitored every loop
- ON + Spray ON → enables spray active phase
- OFF during cleaning → stops oscillation

### Limit Switch Press
- Polled during HOMING state
- Activates on LOW (connected to GND)
- Sets position reference to 0

### Rotary Encoder (TODO)
- A, B on GPIO 26, 27
- Implement quadrature decoding
- Could adjust oscillation parameters

## Safety Features

1. **Motor disable on entry to IDLE**
2. **Ultrasonic only ON during active cleaning**
3. **Fan speed ramping** (not instant)
4. **Spray/Flow validation** (both required for cleaning)
5. **Timeout protection** (future: 60-minute max cleaning)
6. **Limit switch safeguard** (prevents motor runaway)

## Tuning Parameters

To modify cleaning behavior, adjust in `src/main.cpp`:

```cpp
// Clean longer: Increase OSCILLATION_CYCLES
const unsigned long OSCILLATION_CYCLES = 20;  // Try 30 for longer

// Clean faster: Decrease OSCILLATION_DELAY
const unsigned long OSCILLATION_DELAY = 10000;  // Try 5000 for 5sec steps

// Wider oscillation: Increase OSCILLATION_STEPS
const int OSCILLATION_STEPS = 16;  // Try 24 for wider motion

// Different park position: Adjust PARK_POSITION (in mm)
const int PARK_POSITION = 7;  // Try 5 or 10
```

## Implementation Notes

### Non-Blocking Design
```cpp
// BAD (blocking delay)
if (spray_on) {
    delay(10000);  // Blocks ALL input, including emergency stop
    start_oscillation();
}

// GOOD (non-blocking)
if (spray_on && state == SPRAY_ACTIVE) {
    if (millis() - spray_on_time > 10000) {
        start_oscillation();
    }
}
```

### Position Tracking
- Position increments with each STEP pulse
- Reference (0) set during HOMING
- Allows motor position without encoder
- Encoder (future) will add validation

## Debug/Monitor

Use serial monitor at 115200 baud to observe state transitions:
```
State: IDLE | Pos: 0 | Spray: OFF | Flow: NO
State: HOMING | Pos: 0 | Spray: ON | Flow: NO
State: PARKED | Pos: 7 | Spray: ON | Flow: NO
State: WAITING_SPRAY | Pos: 7 | Spray: ON | Flow: YES
State: SPRAY_ACTIVE | Pos: 7 | Spray: ON | Flow: YES
State: OSCILLATING | Pos: 15 | Spray: ON | Flow: YES
State: OSCILLATING | Pos: 16 | Spray: ON | Flow: YES
...
```

## References

- UML State Machine: https://en.wikipedia.org/wiki/UML_state_machine
- Event-Driven Design: https://en.wikipedia.org/wiki/Event-driven_programming
