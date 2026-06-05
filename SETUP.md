# Developer Setup & API Reference

## Development Environment

### Requirements

- **PlatformIO Core** or **PlatformIO IDE** (VS Code extension recommended)
- **Python 3.7+** (required by PlatformIO)
- **Git**
- **RP2040 Pico** board with USB cable

### Installation

#### 1. Install PlatformIO

```bash
# Via pip
pip install platformio

# Or install the "PlatformIO IDE" VS Code extension (platformio.platformio-ide)
```

#### 2. Clone Repository

```bash
git clone https://github.com/GhVost/Stepper-controller.git
cd Stepper-controller
```

#### 3. Install Dependencies & Toolchain

```bash
pio pkg install
```

This downloads the RP2040 Arduino framework, GCC ARM toolchain, and all libraries
(TMCStepper, Adafruit GFX, Adafruit ST7789).

#### 4. Generate VS Code IntelliSense Config

```bash
pio init --ide vscode
```

### Build & Flash

```bash
# Build only
pio run

# Build and upload (Pico must be in BOOTSEL/bootloader mode)
pio run --target upload

# Monitor serial output at 115200 baud
pio device monitor --baud 115200

# Clean build artefacts
pio run --target clean
```

**Flash via drag-and-drop:**
1. Hold BOOTSEL button on Pico, then plug in USB.
2. Pico appears as a USB drive.
3. Drag `.pio/build/pico/firmware.uf2` onto the drive.
4. Pico reboots automatically.

---

## Project Structure

```
Stepper-controller/
├── platformio.ini          # PlatformIO project config & library deps
├── src/
│   └── main.cpp            # All firmware (~350 lines)
├── include/                # Header files (reserved for future modularisation)
├── lib/                    # Local libraries (reserved)
├── .vscode/
│   ├── extensions.json     # Recommended extensions
│   └── c_cpp_properties.json  # IntelliSense paths (auto-generated)
├── README.md               # Project overview
├── HARDWARE.md             # Pin assignments, wiring, calibration
├── STATE_MACHINE.md        # State diagram & behaviour reference
├── SETUP.md                # This file
├── TESTING.md              # Validation & troubleshooting
└── QUICK_REFERENCE.md      # One-page cheat sheet
```

---

## Architecture

The firmware uses both RP2040 cores:

| Core | Entry points | Responsibility |
|------|-------------|----------------|
| Core 0 | `setup()` / `loop()` | State machine, motor steps, sensor reading, encoder polling |
| Core 1 | `setup1()` / `loop1()` | LCD rendering (~20 fps) |

Core 1 spins on `core0_ready` (set at the end of `setup()`) before touching the display, ensuring SPI is initialised before either core accesses it. All variables shared between cores are declared `volatile`.

When the TMC2130 is connected, a mutex will be needed around SPI bus access because Core 0 (TMC2130) and Core 1 (display) share SPI0.

---

## Code Organisation (`src/main.cpp`)

### Pin Constants (top of file)

```cpp
const int TMC_CS = 17, TMC_SCK = 18, TMC_MOSI = 19, TMC_MISO = 16;
const int TMC_STEP = 14, TMC_DIR = 15, TMC_EN = 13;
const int LCD_CS = 9, LCD_DC = 10, LCD_RST = 11;
const int ENC_A = 26, ENC_B = 27;
const int LIMIT_SWITCH = 28, SPRAY_VALVE = 2, FLOW_SENSOR = 3;
const int LED_GREEN = 8, LED_YELLOW = 7, FAN_PWM = 12, ULTRASONIC = 4;
```

### Motion Parameters

```cpp
const int STEPS_PER_MM      = 1;     // Calibrate first — see HARDWARE.md
const int PARK_MM           = 7;     // Park position (mm from limit)
const int CENTER_MM         = 26;    // Oscillation centre (mm from limit)
const int OSCILLATION_STEPS = 16;    // Steps per directional sweep
const unsigned long OSCILLATION_DELAY  = 10000; // ms per step
const unsigned long OSCILLATION_CYCLES = 20;    // Sweeps before idle
const unsigned long SPRAY_ACTIVE_WAIT  = 2000;  // ms stabilisation delay
```

### State Machine

```cpp
enum SystemState {
    STATE_IDLE, STATE_HOMING, STATE_PARKED,
    STATE_WAITING_SPRAY, STATE_SPRAY_ACTIVE, STATE_OSCILLATING, STATE_ERROR
};
```

---

## Function API

### Motor Control

**`motorStep(int direction)`** — Pulse the STEP pin once; update position counter.
- `direction`: `1` = forward (away from limit), `−1` = toward limit.

**`motorSetEnable(bool enable)`** — Drive the TMC2130 EN pin.
- `true` = motor active (holds torque); `false` = motor disabled.

**`motorMoveTo(int target)`** — Call repeatedly; moves one step per call toward target.
- Stops when `motorPosition == target`.

### Sensor Reading

**`readSensors()`** — Polls all inputs; updates globals `limitSwitchPressed`, `sprayActive`, `flowDetected`.
- Limit switch is debounced (20 ms).

**`readEncoder()`** — Quadrature decoding; updates `menuIndex`.
- Encoder pins use internal pull-ups — no external resistors needed.
- Accumulates transitions; requires ±4 (one full KY-040 detent click) before registering a step.
- Enforces 50 ms minimum between accepted steps to suppress contact bounce.

### Output Control

**`setLED(int pin, bool state)`** — `true` = LED on.

**`setFan(int speed)`** — PWM 0–255 via `analogWrite`. Clamped with `constrain`.

**`setUltrasonic(bool on)`** — `true` = relay ON (GPIO 4 driven LOW). Active-low logic.

### TMC2130

The driver is accessed through the **TMCStepper** library object `driver`.
Raw SPI register access is available via `driver.read()` / `driver.write()` if needed.

Key calls used at startup in `initTMC2130()`:

```cpp
driver.begin();
driver.toff(5);              // Enable chopper (mandatory)
driver.rms_current(600);     // RMS current in mA
driver.microsteps(16);       // Microstep resolution
driver.en_pwm_mode(true);    // StealthChop
driver.pwm_autoscale(true);  // Auto-tune amplitude
```

### State Machine

**`updateStateMachine()`** — Evaluates transition conditions; updates `currentState`.  
**`handleState()`** — Executes actions for the current state (motor steps, LED, fan, etc.).  
Both are called every main loop iteration.

### Display

**`updateDisplay()`** — Called by `loop1()` on Core 1 at ~20 fps. Snapshots all volatile
shared variables once per call and redraws LCD + echoes to Serial only when something
has changed (dirty-flag check). Avoids screen flicker and cross-core tearing.

**`drawMenu()`** — On first call: paints title + all menu rows. On subsequent calls:
repaints only the two rows whose selection state changed (no `fillScreen`). Snapshots
`menuIndex` into a local variable at entry to prevent Core 0 from changing it
mid-draw and leaving two rows simultaneously highlighted.

---

## Adding Features

### 1. Calibrate Steps-per-mm

Update `const int STEPS_PER_MM` after measuring actual travel:

```cpp
// Example: 200 step/rev, 16× microsteps, 2 mm lead screw
const int STEPS_PER_MM = 1600;
```

### 2. Adjust Motor Current

In `initTMC2130()`:

```cpp
driver.rms_current(800);   // 800 mA for higher-torque motors
```

Reduce if driver or motor runs hot. Measure temperature after a few minutes.

### 3. Change Microstep Resolution

Update both the driver and the calibration constant:

```cpp
driver.microsteps(32);         // Change microstep setting
const int STEPS_PER_MM = ...;  // Recalculate accordingly
```

### 4. Add Error Detection

Wire a fault condition into `updateStateMachine()`:

```cpp
case STATE_OSCILLATING:
    if (driver.stallguard()) {  // TMC2130 stall detection
        currentState    = STATE_ERROR;
        lastStateChange = millis();
    }
    break;
```

### 5. Add Encoder Button Action (SELECT)

`ENC_SW` (GPIO 22) is already wired with `INPUT_PULLUP`. Add action handling in `readEncoder()`:

```cpp
// In readEncoder():
if (digitalRead(ENC_SW) == LOW) {
    // Confirm menu selection for menuIndex
}
```

---

## Debugging Tips

### Serial Monitor

```bash
pio device monitor --baud 115200
```

Output appears only on state or position changes (dirty flag). Force a debug
print by adding to any function:

```cpp
Serial.print("debug: motorPosition=");
Serial.println(motorPosition);
```

### GPIO Testing

Use a multimeter:
- `0 V` = LOW, `3.3 V` = HIGH
- EN pin should read `0 V` when motor is active, `3.3 V` when disabled
- ULTRASONIC pin should read `0 V` during SPRAY_ACTIVE and OSCILLATING

### TMC2130 Verification

Read a register to confirm SPI communication:

```cpp
uint32_t gstat = driver.GSTAT();
Serial.print("GSTAT: 0x");
Serial.println(gstat, HEX);
// Expected: 0x00000001 on first power-up (reset flag set)
```

---

## Performance

Current build (release mode, earlephilhower framework):

| Metric | Value |
|--------|-------|
| Flash | 75 KB (3.6 % of 2 MB) |
| RAM | 9.5 KB (3.6 % of 264 KB) |
| Motor/sensor update | Every 50 ms (Core 0) |
| Encoder poll | Every 20 ms (Core 0) |
| Display update | ~20 fps (Core 1, on change only) |
| SPI clock (LCD) | 20 MHz hardware SPI |

---

## References

- [PlatformIO Docs](https://docs.platformio.org/)
- [RP2040 Arduino Core (earlephilhower)](https://github.com/earlephilhower/arduino-pico)
- [TMCStepper Library](https://github.com/teemuatlut/TMCStepper)
- [Adafruit ST7789 Library](https://github.com/adafruit/Adafruit-ST7789-Library)
