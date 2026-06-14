# Testing & Validation Guide

## Pre-Hardware Testing

### Step 1: Build Verification

```bash
cd Megasonic
pio run
```

Expect a `[SUCCESS]` link step with the RAM/Flash usage summary.

**Common build failures:**

| Error | Fix |
|-------|-----|
| `no such file: TMCStepper.h` | Run `pio pkg install` |
| `undeclared identifier` | Check you have the latest `src/main.cpp` |
| Pin conflict warning | Review the pin constants at the top of `main.cpp` |

---

### Step 2: Serial Monitor Startup

Flash (BOOTSEL or `pio run --target upload`), then:

```bash
pio device monitor --baud 115200
```

**Expected startup output:**
```
=== Stepper Controller Initializing ===
Settings: loaded from flash          (or "using defaults" on first boot)
Hardware initialized
SPI0 LCD initialized: SCK=18 MOSI=19
SPI1 TMC initialized: SCK=10 MOSI=11 MISO=12
TMC2130 configured: 600 mA, run hold 25%, park hold 10%, 256x microsteps, interpolation, StealthChop
=== TMC2130 DEBUG ===
...register dump...
Encoder initialized: CLK=26 DT=27 SW=22
Display initialized (GMT147SPI 1.47" 172x320)
Initialization complete!
```

After startup, state/position lines are emitted by Core 1 **only on change**. With
**Safety = DEBUG** (default) the cycle is driven from the menu; with **Safety = ON** it
responds to the spray/flow inputs.

---

## Hardware Integration Testing

Connect hardware incrementally; verify each phase before adding more.

### Phase 1: Power & GPIO

**Equipment:** multimeter. Trigger HOMING via the menu (START) or, in sensor mode, by
briefly pulling GPIO 2 to 3.3 V.

| Test | GPIO | Expected |
|------|------|----------|
| Motor disabled at startup | 1 (EN) | 3.3 V (HIGH) |
| Motor enabled in HOMING | 1 (EN) | 0 V (LOW) |
| STEP pulses in HOMING | 14 | Fast pulses (~500 µs period) |
| DIR toward limit in HOMING | 15 | Steady level (polarity per `motorDirectionInverted`) |

### Phase 2: Sensors

> Set **Safety = ON** in Setup to read the spray/flow inputs; in DEBUG they are forced
> inactive in firmware.

**Limit Switch (GPIO 28)**
```
Switch open   → GPIO 28 reads 3.3 V  (INPUT_PULLUP, not pressed)
Switch closed → GPIO 28 reads 0 V    (to GND)
```
Serial when the limit triggers during HOMING:
```
→ PARKED (moving to park position)
State:PARKED | Pos:0 steps (0.0 deg) | Spray:... | Flow:...
...
State:PARKED | Pos:78 steps (7.0 deg) | Spray:... | Flow:...
```

**Spray Valve (GPIO 2)** — `0 V` off / `3.3 V` active.
**Flow Sensor (GPIO 3)** — `0 V` no flow / `3.3 V` flowing.

### Phase 3: Outputs

| Output | GPIO | Idle | Active | Check |
|--------|------|------|--------|-------|
| Green LED | 8 | 0 V | 3.3 V | Visual / multimeter |
| Yellow LED | 7 | 0 V | 3.3 V | Visual / multimeter |
| Fan PWM | 21 | 0 V | PWM (avg ~1.65 V at 50 %) | Scope / PWM meter |
| Ultrasonic relay | 4 | 3.3 V | 0 V over the wafer | Multimeter |

**Fan duty by state:**

| State | GPIO 21 duty |
|-------|-------------|
| IDLE / HOMING / PARKED | 0 % |
| WAITING_SPRAY | 50 % (PWM 128) |
| SPRAY_ACTIVE / OSCILLATING | 100 % |

> The ultrasonic relay is LOW (ON) only while the arm is over the wafer disk during
> SPRAY_ACTIVE/OSCILLATING — it pulses off near the sweep extremes.

### Phase 4: TMC2130 SPI Verification

`initTMC2130()` runs at startup and prints the register dump. With the driver powered
(3.3 V logic, VM motor supply):

1. Confirm startup prints `TMC2130 configured: …`.
2. Send `d` in the serial monitor to dump `GSTAT` / `DRV_STATUS` and the decoded bits.
3. The **About** screen shows the same status live (OT, OTPW, CS, SG, STST, ERR).

All-ones (`0xFFFFFFFF`) or all-zero reads indicate a wiring/power problem — check SPI1
on GPIO 10/11/12/13, VCC_IO, and `R_SENSE`.

### Phase 5: State-Machine Walkthrough

**DEBUG mode (no sensors)** — drive from the menu:

| Step | Action | Expected |
|------|--------|----------|
| 1 | Menu START (from IDLE) | `Menu: START → HOMING` |
| 2 | Limit reached | `→ PARKED (moving to park position)`, pos → park |
| 3 | (auto, bypass armed) | `→ SPRAY_ACTIVE (sensor bypass)` |
| 4 | After ~2 s at sweep start | `→ OSCILLATING`, then `Sweep 1/4` … |
| 5 | Cycles complete | `→ PARKED (cleaning cycle complete)` → `→ IDLE` |
| — | Menu STOP at any time | `Menu: STOP → PARK then disable` → IDLE |

**Sensor mode (Safety = ON):**

| Step | Action | Expected |
|------|--------|----------|
| 1 | GPIO 2 → 3.3 V | `→ HOMING` |
| 2 | GPIO 28 → GND briefly | `→ PARKED`, pos increments to the park angle |
| 3 | GPIO 3 → 3.3 V | `→ SPRAY_ACTIVE` |
| 4 | Wait ~2 s | `→ OSCILLATING` |
| 5 | Disconnect GPIO 2 | `→ PARKED (spray or flow lost)` |

### Phase 6: Motor Mechanical Testing

- **HOMING**: arm steps toward the limit; if the endstop is not found within 70° of
  travel the firmware aborts to `ERROR`.
- **PARKED**: arm steps to the park angle, then drops to park-hold current.
- **OSCILLATING**: arm sweeps between the sweep start and end; each return sweep logs
  `Sweep N/<cycles>`. Total time ≈ `cycles × SWEEP_TIME_MS` (plus profile shaping).
- **Direction**: if motion is reversed, toggle **Invert** in Setup
  (`motorDirectionInverted`).

---

## Integration Checklist

| Item | Test | ☐ |
|------|------|---|
| Firmware builds | `pio run` → SUCCESS | ☐ |
| Startup serial matches expected | See Step 2 | ☐ |
| EN HIGH at startup | Multimeter GPIO 1 | ☐ |
| EN LOW in HOMING | Multimeter GPIO 1 | ☐ |
| STEP pulses during HOMING | Scope / sound | ☐ |
| Limit stops motor & homes | GPIO 28 → GND | ☐ |
| Arm reaches park angle | Status column / serial | ☐ |
| START runs a cycle (DEBUG) | Menu START | ☐ |
| Spray/flow drive cycle (sensor mode) | GPIO 2/3 | ☐ |
| Fan duty per state | Multimeter GPIO 21 | ☐ |
| Ultrasonic LOW over wafer | Multimeter GPIO 4 | ☐ |
| Oscillation reverses & counts | Serial `Sweep N/…` | ☐ |
| STOP parks then idles | Menu STOP | ☐ |
| Home timeout → ERROR | Block the limit switch | ☐ |
| START recovers from ERROR | Menu START | ☐ |
| TMC2130 status readable | About screen / `d` | ☐ |
| Settings survive reboot | Edit, power-cycle | ☐ |

---

## Troubleshooting by Symptom

### Motor won't move
1. EN (GPIO 1) must be LOW in active states; check `motorSetEnable(true)`.
2. VM motor supply connected and powered.
3. Coil pairs correct (1A/1B, 2A/2B).
4. `driver.toff(5)` runs in `initTMC2130()` — required to enable the chopper.

### Won't start in DEBUG mode
- START always re-homes first; the limit switch must be reachable. If it never triggers,
  homing times out to ERROR after 70°.

### Stuck in HOMING / times out to ERROR
- Limit switch not wired to GPIO 28 / GND, or the endstop is beyond 70° of travel.
- Verify GPIO 28 reads 0 V when the switch is pressed.

### Stuck in PARKED (sensor mode)
- Needs spray (→ WAITING_SPRAY) and flow (→ SPRAY_ACTIVE). Add GPIO 3 → 3.3 V, or use
  DEBUG mode + START.

### Sensors ignored
- Safety is `DEBUG`. Set it to `ON` in Setup to read GPIO 2/3.

### Fan always off
- `setFan()` uses `analogWrite(FAN_PWM, …)` on GPIO 21 — verify the fan driver input.

### Ultrasonic never on / always on
- `setUltrasonic(true)` drives GPIO 4 LOW (relay ON). It is gated by `armOverWafer()`, so
  it is only ON while the arm is over the wafer during a cycle. Check relay polarity.

### TMC2130 not responding (status all 0x… or 0xFFFFFFFF)
- Check SPI1 wiring (GPIO 10/11/12/13), 3.3 V VCC_IO, and `R_SENSE` (default 0.11 Ω).

### Settings not persisting
- Edits auto-save after 2.5 s of quiet; power-cycling immediately after an edit may lose
  it. A `SETTINGS_VERSION` bump intentionally resets stored values to defaults once.

### Display flicker / two rows highlighted
- Rendering runs on Core 1 and snapshots volatile state per call; flicker usually means a
  stale build. Rebuild from the latest `main.cpp`.

---

## Appendix: Quick GPIO Test Snippets

Add temporarily to `setup()` and remove before production:

```cpp
// Blink both LEDs 5×
void testLEDs() {
    for (int i = 0; i < 5; i++) {
        digitalWrite(LED_GREEN, HIGH); delay(300); digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_YELLOW, HIGH); delay(300); digitalWrite(LED_YELLOW, LOW);
    }
}

// Ramp fan PWM 0→100 %
void testFan() {
    for (int d = 0; d <= 255; d += 25) { analogWrite(FAN_PWM, d); delay(500); }
    analogWrite(FAN_PWM, 0);
}

// Read TMC2130 GSTAT to verify SPI
void testTMCSPI() {
    Serial.print("GSTAT: 0x"); Serial.println(driver.GSTAT(), HEX);
}
```

> The firmware already exposes these at runtime via the serial debug keys (`d`, `f`/`b`,
> `r`) — prefer those over editing `setup()`.
</content>
