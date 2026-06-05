# Quick Reference Card

## File Map

| File | Purpose |
|------|---------|
| [README.md](README.md) | Project overview & features |
| [HARDWARE.md](HARDWARE.md) | Pin assignments, wiring, calibration |
| [STATE_MACHINE.md](STATE_MACHINE.md) | State diagram & behaviour |
| [SETUP.md](SETUP.md) | Development environment & API |
| [TESTING.md](TESTING.md) | Validation & troubleshooting |
| [src/main.cpp](src/main.cpp) | All firmware |
| [platformio.ini](platformio.ini) | Project config & library deps |

---

## Build Commands

```bash
pio pkg install                     # Install dependencies (first time)
pio init --ide vscode               # Generate IntelliSense config
pio run                             # Build only
pio run --target upload             # Build and flash (BOOTSEL mode)
pio device monitor --baud 115200    # Serial monitor
pio run --target clean              # Remove build artefacts
```

---

## Pin Quick Map

| Function | GPIO |
|----------|------|
| TMC_CS | 17 |
| TMC_SCK | 18 (shared with LCD) |
| TMC_MOSI | 19 (shared with LCD) |
| TMC_MISO | 16 |
| TMC_STEP | 14 |
| TMC_DIR | 15 |
| TMC_EN | 13 (LOW = active) |
| LCD_CS | 9 |
| LCD_DC | 10 |
| LCD_RST | 11 |
| ENC_A | 26 (INPUT_PULLUP) |
| ENC_B | 27 (INPUT_PULLUP) |
| LIMIT_SWITCH | 28 (INPUT_PULLUP, LOW = pressed) |
| SPRAY_VALVE | 2 (HIGH = active) |
| FLOW_SENSOR | 3 (HIGH = flowing) |
| LED_GREEN | 8 |
| LED_YELLOW | 7 |
| FAN_PWM | 12 |
| ULTRASONIC | 4 (LOW = ON, active-low relay) |

---

## State Machine

```
IDLE → HOMING → PARKED → WAITING_SPRAY → SPRAY_ACTIVE → OSCILLATING → IDLE
                    └──────────────────────────────────────┘
                         (spray + flow both present skips WAITING_SPRAY)
```

### State Duration

| State | Duration |
|-------|----------|
| IDLE | Indefinite |
| HOMING | Until limit switch triggers |
| PARKED | Until at park position + spray/flow |
| WAITING_SPRAY | Until flow detected or spray lost |
| SPRAY_ACTIVE | 2 s + time to reach centre |
| OSCILLATING | 20 sweeps × 16 steps × 10 s ≈ **53 min** |

### LED Indicators

| State | Green | Yellow |
|-------|-------|--------|
| IDLE | OFF | OFF |
| HOMING | OFF | ON |
| PARKED | ON | OFF |
| WAITING_SPRAY | OFF | ON |
| SPRAY_ACTIVE | ON | OFF |
| OSCILLATING | ON | OFF |
| ERROR | OFF | Blink |

### Fan Speed

| State | PWM | % |
|-------|-----|---|
| IDLE / HOMING / PARKED | 0 | 0 % |
| WAITING_SPRAY | 128 | 50 % |
| SPRAY_ACTIVE / OSCILLATING | 255 | 100 % |

---

## Key Functions

```cpp
// Motor
motorStep(int dir);           // ±1 direction; updates motorPosition
motorSetEnable(bool en);      // true = coils energised
motorMoveTo(int target);      // Call repeatedly — steps one at a time

// Sensors (called every loop)
readSensors();                // Updates limitSwitchPressed, sprayActive, flowDetected
readEncoder();                // Updates menuIndex

// Outputs
setLED(pin, state);           // true = on
setFan(speed);                // 0–255 PWM (clamped)
setUltrasonic(bool on);       // true = relay ON (GPIO 4 LOW)

// State machine
updateStateMachine();         // Evaluates transitions
handleState();                // Executes state actions

// Display (LCD + serial)
updateDisplay();              // Redraws only on change
```

---

## Motion Parameters

```cpp
STEPS_PER_MM      = 1      // Calibrate before use (see HARDWARE.md)
PARK_MM           = 7      // mm from limit switch to park
CENTER_MM         = 26     // mm from limit switch to oscillation centre
OSCILLATION_STEPS = 16     // Steps per directional sweep
OSCILLATION_DELAY = 10000  // ms per step (10 s)
OSCILLATION_CYCLES = 20    // Sweeps before returning to IDLE
SPRAY_ACTIVE_WAIT = 2000   // ms stabilisation before oscillation
```

**Timing:**
- 1 sweep = 16 × 10 s = 160 s ≈ 2.7 min
- 20 sweeps = 3 200 s ≈ 53 min total

---

## Serial Monitor Output

**Startup:**
```
=== Stepper Controller Initializing ===
Hardware initialized
SPI initialized: SCK=18 MOSI=19 MISO=16
Encoder initialized: CLK=26 DT=27 SW=22
Display initialized (GMT147SPI 1.47" 172x320)
Initialization complete!
```

**During operation** (only on change):
```
→ HOMING
State:HOMING | Pos:-1 | Spray:ON | Flow:NO
→ PARKED (moving to park position)
State:PARKED | Pos:7 | Spray:ON | Flow:NO
→ SPRAY_ACTIVE
→ OSCILLATING
Sweep 1/20
Sweep 2/20
...
→ IDLE (cleaning cycle complete)
```

---

## Common Adjustments

### Longer cleaning

```cpp
const unsigned long OSCILLATION_CYCLES = 30;  // Was 20 (~80 min total)
```

### Faster steps

```cpp
const unsigned long OSCILLATION_DELAY = 5000;  // Was 10000 (26 min total)
```

### Higher motor current (more torque)

```cpp
driver.rms_current(800);  // Was 600 mA — in initTMC2130()
```

### Different microstep resolution

```cpp
driver.microsteps(32);         // In initTMC2130()
const int STEPS_PER_MM = ...;  // Recalculate in main.cpp
```

---

## Known Issues & TODO

| Item | Status | Notes |
|------|--------|-------|
| TMC2130 SPI config | ⚠️ Pending wiring | `initTMC2130()` ready, commented out; add SPI mutex when connected |
| Encoder quadrature + debounce | ✅ Done | 4-transition accumulator + 50 ms guard |
| Oscillation direction | ✅ Done | Alternates correctly |
| LCD display | ✅ Done | GMT147SPI 1.47" 172×320, 20 MHz HW SPI, Core 1 ~20 fps |
| STEPS_PER_MM calibration | ⚠️ Required | Defaults to 1 (uncalibrated) |
| Error recovery | 🔄 TODO | STATE_ERROR requires power-cycle |
| Encoder button (SELECT) | 🔄 TODO | GPIO 22 wired; no menu action yet |
| StallGuard fault detection | 🔄 TODO | TMC2130 supports it; needs wiring |

---

## Hardware BOM (Approximate)

| Component | Cost |
|-----------|------|
| RP2040 Pico | ~$5 |
| TMC2130 stepper driver | ~$10 |
| GMT147SPI 1.47" SPI LCD (172×320 ST7789) | ~$6 |
| NEMA17 stepper motor | ~$15 |
| Rotary encoder | ~$3 |
| Supporting electronics | ~$15 |
| **Total** | **~$56** |

---

## Links

- [GitHub Repository](https://github.com/GhVost/Stepper-controller)
- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [TMC2130 Datasheet](https://www.trinamic.com/fileadmin/assets/Documents/TMC2130_Datasheet_Rev1.35.pdf)
- [PlatformIO Docs](https://docs.platformio.org/)
