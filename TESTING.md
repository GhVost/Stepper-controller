# Testing & Validation Guide

## Pre-Hardware Testing

### Step 1: Build Verification

```bash
cd Stepper-controller
pio run
```

**Expected output (last lines):**
```
Linking .pio/build/pico/firmware.elf
Checking size .pio/build/pico/firmware.elf
RAM:   [==        ]  15.4% (used ~41 KB of 264 KB)
Flash: [          ]   0.2% (used ~4.3 KB of 2 MB)
========================= [SUCCESS] ...
```

**Common build failures:**

| Error | Fix |
|-------|-----|
| `no such file: TMCStepper.h` | Run `pio pkg install` |
| `undeclared identifier` | Check you have the latest `src/main.cpp` |
| Pin conflict warning | Review constants at top of `main.cpp` |

---

### Step 2: Serial Monitor Startup

Flash the firmware (BOOTSEL mode or `pio run --target upload`), then:

```bash
pio device monitor --baud 115200
```

**Expected startup output:**
```
=== Stepper Controller Initializing ===
Hardware initialized
SPI initialized: SCK=18 MOSI=19 MISO=16
Encoder initialized: A=26 B=27
TMC2130 configured: 600 mA, 16x microsteps, StealthChop
Display initialized (ST7789V3 240x280)
Initialization complete!
```

After startup, serial output appears **only when state, position, or menu changes**
(dirty-flag display). Trigger changes by connecting sensor pins to 3.3 V or GND.

**No serial output after startup:** Normal — nothing has changed. Connect GPIO 2
(Spray) to 3.3 V momentarily to trigger the first state transition and confirm output.

---

## Hardware Integration Testing

Connect hardware incrementally. Always verify the previous phase before adding more.

---

### Phase 1: Power & GPIO

**Equipment:** Multimeter

| Test | GPIO | Expected |
|------|------|----------|
| Motor disabled at startup | 13 (EN) | 3.3 V (HIGH) |
| Motor enabled in HOMING | 13 (EN) | 0 V (LOW) |
| STEP pulses in HOMING | 14 | Pulses at ~20 Hz |
| DIR in HOMING (toward limit) | 15 | 0 V (LOW) |

Trigger HOMING by briefly connecting GPIO 2 (Spray) to 3.3 V.

---

### Phase 2: Sensors

**Limit Switch (GPIO 28)**

```
Switch open  → GPIO 28 reads 3.3 V  (INPUT_PULLUP, switch not pressed)
Switch closed → GPIO 28 reads 0 V   (connected to GND)
```

Serial output when limit triggers during HOMING:
```
→ PARKED (moving to park position)
State:PARKED | Pos:0 | Spray:ON | Flow:NO
State:PARKED | Pos:1 | Spray:ON | Flow:NO
...
State:PARKED | Pos:7 | Spray:ON | Flow:NO
```

**Spray Valve (GPIO 2)**

```
Spray OFF → GPIO 2 reads 0 V
Spray ON  → GPIO 2 reads 3.3 V
```

**Flow Sensor (GPIO 3)**

```
No flow   → GPIO 3 reads 0 V
Flowing   → GPIO 3 reads 3.3 V
```

---

### Phase 3: Outputs

| Output | GPIO | Idle State | Active State | How to Check |
|--------|------|------------|--------------|--------------|
| Green LED | 8 | 0 V | 3.3 V | Visual / multimeter |
| Yellow LED | 7 | 0 V | 3.3 V | Visual / multimeter |
| Fan PWM | 12 | 0 V | PWM (avg ~2.5 V at 50 %) | Oscilloscope or PWM meter |
| Ultrasonic relay | 4 | 3.3 V | 0 V | Multimeter |

**Fan state vs PWM:**

| State | GPIO 12 duty |
|-------|-------------|
| IDLE / HOMING / PARKED | 0 % |
| WAITING_SPRAY | 50 % (PWM 128) |
| SPRAY_ACTIVE / OSCILLATING | 100 % (3.3 V) |

---

### Phase 4: TMC2130 SPI Verification

With TMC2130 powered (3.3 V logic, VM motor supply):

1. Flash firmware, open serial monitor.
2. Check startup prints `TMC2130 configured: 600 mA, 16x microsteps, StealthChop`.
3. Add a temporary GSTAT read to `setup()` to verify SPI communication:

```cpp
uint32_t gstat = driver.GSTAT();
Serial.print("GSTAT: 0x");
Serial.println(gstat, HEX);
// On first power-up: 0x00000001 (reset flag set — expected)
```

If GSTAT returns `0xFFFFFFFF`: SPI not communicating — check GPIO 16/17/18/19 wiring and R_SENSE value.

---

### Phase 5: Full State Machine Walkthrough

Use two terminals or a serial monitor while manually bridging GPIO pins:

**Simulated sequence:**

| Step | Action | Expected serial output |
|------|--------|----------------------|
| 1 | Connect GPIO 2 to 3.3 V | `→ HOMING` |
| 2 | Connect GPIO 28 to GND briefly | `→ PARKED (moving to park position)` then position increments to 7 |
| 3 | Connect GPIO 3 to 3.3 V | `→ SPRAY_ACTIVE` |
| 4 | Wait ~2 s | `→ OSCILLATING` |
| 5 | Disconnect GPIO 2 | `→ IDLE (spray or flow lost)` |

**Full expected log:**
```
→ HOMING
State:HOMING | Pos:0 | Spray:ON | Flow:NO
State:HOMING | Pos:-1 | Spray:ON | Flow:NO
...
→ PARKED (moving to park position)
State:PARKED | Pos:0 | Spray:ON | Flow:NO
State:PARKED | Pos:1 | Spray:ON | Flow:NO
...
State:PARKED | Pos:7 | Spray:ON | Flow:NO
→ SPRAY_ACTIVE
State:SPRAY_ACTIVE | Pos:7 | Spray:ON | Flow:YES
...
State:SPRAY_ACTIVE | Pos:26 | Spray:ON | Flow:YES
→ OSCILLATING
State:OSCILLATING | Pos:25 | Spray:ON | Flow:YES
...
Sweep 1/20
State:OSCILLATING | Pos:10 | Spray:ON | Flow:YES
State:OSCILLATING | Pos:11 | Spray:ON | Flow:YES
...
Sweep 2/20
...
→ IDLE (spray or flow lost)
```

---

### Phase 6: Motor Mechanical Testing

**Before power:**
- Motor shaft should spin freely (no current, no torque).

**During HOMING:**
- Motor steps toward limit switch, 1 step per 50 ms.
- Should move smoothly — no grinding.
- Limit switch stops motion; motor then steps 7 steps forward to park.

**During OSCILLATING:**
- Motor sweeps 16 steps in one direction, then 16 steps back, once per 10 s.
- Motion is slow and quiet (StealthChop).
- Each direction change is logged: `Sweep N/20`.

**Speed adjustment:**
```cpp
// Faster oscillation steps (5 s instead of 10 s):
const unsigned long OSCILLATION_DELAY = 5000;

// Faster homing/parking:
const unsigned long MOTOR_UPDATE_INTERVAL = 20;  // 50 ms default
```

---

## Integration Checklist

| Item | Test | ☐ |
|------|------|---|
| Firmware builds without errors | `pio run` exits SUCCESS | ☐ |
| Startup serial output matches expected | See Phase 2 | ☐ |
| EN pin HIGH at startup | Multimeter on GPIO 13 | ☐ |
| EN pin LOW in HOMING | Multimeter on GPIO 13 | ☐ |
| STEP pulses during HOMING | Scope or click sounds | ☐ |
| Limit switch stops motor | GPIO 28 to GND | ☐ |
| Motor steps to park (pos = 7) | Serial monitor | ☐ |
| Spray valve triggers HOMING | GPIO 2 to 3.3 V | ☐ |
| Flow sensor enables SPRAY_ACTIVE | GPIO 3 to 3.3 V | ☐ |
| Fan 0 % in IDLE | Multimeter GPIO 12 | ☐ |
| Fan 50 % in WAITING_SPRAY | Multimeter GPIO 12 | ☐ |
| Fan 100 % in OSCILLATING | Multimeter GPIO 12 | ☐ |
| Ultrasonic LOW in OSCILLATING | Multimeter GPIO 4 | ☐ |
| Oscillation reverses direction | Serial `Sweep N/20` | ☐ |
| Motor returns to IDLE cleanly | Disconnect GPIO 2 | ☐ |
| TMC2130 GSTAT readable | Add debug print | ☐ |

---

## Troubleshooting by Symptom

### Motor won't move
1. EN pin must be LOW: `motorSetEnable(true)` called in HOMING state.
2. VM (motor supply) connected and powered.
3. Coil connections correct (A, A~, B, B~).
4. `driver.toff(5)` called in `initTMC2130()` — required to activate chopper.

### Stuck in HOMING
- Limit switch not wired to GPIO 28 and GND.
- Verify: GPIO 28 reads 0 V when switch is manually pressed.

### Stuck in PARKED
- Both spray AND flow must be present to exit.
- With only spray: transitions to WAITING_SPRAY (yellow LED on).
- Add flow signal (GPIO 3 → 3.3 V) to continue.

### Fan always off
- Check `analogWrite(FAN_PWM, speed)` — not `digitalWrite`.
- Verify GPIO 12 is connected to fan driver input.

### Ultrasonic always on or off
- `setUltrasonic(true)` drives GPIO 4 LOW (relay ON).
- `setUltrasonic(false)` drives GPIO 4 HIGH (relay OFF).
- Verify relay module polarity.

### TMC2130 not responding (GSTAT = 0xFFFFFFFF)
- Check GPIO 16/17/18/19 wiring.
- Verify 3.3 V on VCC_IO.
- Confirm `R_SENSE` matches your board (default 0.11 Ω).

### Display flickering
- Should only update on state/position changes.
- If flickering persists, verify `updateDisplay()` dirty-flag logic is in the current `main.cpp`.

### Oscillation goes only one direction
- Verify `oscillationDir` flips in `handleState() STATE_OSCILLATING`.
- Ensure you have the latest `main.cpp` (this was a bug in earlier versions).

---

## Performance Validation

```bash
pio run
# Check: Flash < 5 %, RAM < 20 % — well within headroom for future features
```

**Oscillation timing** (verify with stopwatch):
- Each step: 10 s ± 1 s
- Each 16-step sweep: ~160 s ≈ 2.7 min
- Full 20-sweep cycle: ~53 min

---

## Appendix: Quick GPIO Test Snippets

Add these temporarily to `setup()` and remove before production:

```cpp
// Blink both LEDs 5× to confirm GPIO outputs
void testLEDs() {
    for (int i = 0; i < 5; i++) {
        digitalWrite(LED_GREEN, HIGH); delay(300);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_YELLOW, HIGH); delay(300);
        digitalWrite(LED_YELLOW, LOW);
    }
}

// Ramp fan PWM 0→100 %
void testFan() {
    for (int d = 0; d <= 255; d += 25) {
        analogWrite(FAN_PWM, d);
        Serial.print("Fan PWM: "); Serial.println(d);
        delay(500);
    }
    analogWrite(FAN_PWM, 0);
}

// Read TMC2130 GSTAT to verify SPI
void testTMCSPI() {
    uint32_t gstat = driver.GSTAT();
    Serial.print("GSTAT: 0x"); Serial.println(gstat, HEX);
}
```
