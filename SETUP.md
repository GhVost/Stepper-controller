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

This downloads the earlephilhower RP2040 Arduino core, the GCC ARM toolchain, and the
libraries (TMCStepper, Adafruit GFX, Adafruit ST7789).

#### 4. Generate VS Code IntelliSense Config

```bash
pio init --ide vscode
```

### Build & Flash

```bash
pio run                          # Build only
pio run --target upload          # Build and upload (Pico in BOOTSEL mode)
pio device monitor --baud 115200 # Serial monitor
pio run --target clean           # Clean build artefacts
```

**Flash via drag-and-drop:**
1. Hold BOOTSEL on the Pico, then plug in USB.
2. The Pico appears as a USB drive.
3. Drag `.pio/build/<env>/firmware.uf2` onto the drive.
4. The Pico reboots automatically.

---

## Project Structure

```
Stepper-controller/
├── platformio.ini          # PlatformIO project config & library deps
├── src/
│   └── main.cpp            # All firmware (~2050 lines)
├── tools/
│   └── gui_preview.html    # Browser mock-up of the LCD UI
├── include/                # Header files (reserved for future modularisation)
├── lib/                    # Local libraries (reserved)
├── test/                   # Unit tests (reserved)
├── README.md               # Project overview
├── HARDWARE.md             # Pin assignments, wiring, driver config
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
| Core 0 | `setup()` / `loop()` | State machine, motor steps, sensors, encoder, TMC2130, settings |
| Core 1 | `setup1()` / `loop1()` | LCD rendering (~20 fps, on change only) |

Core 1 spins on `core0_ready` (set at the end of `setup()`) before touching the display.
All variables shared between cores are declared `volatile`. The TMC2130 (SPI1) and the
LCD (SPI0) are on **separate buses**; a single `mutex_t spi_mutex` serialises all SPI
access so the two cores never drive a bus concurrently.

`setup()` order: `loadSettings()` → `initHardware()` → `initSPI()` → `initTMC2130()`
→ `initEncoder()` → `initDisplay()`.

---

## Code Organisation (`src/main.cpp`)

### Pin Constants (top of file)

```cpp
// TMC2130 on SPI1
const int TMC_CS = 13, TMC_SCK = 10, TMC_MOSI = 11, TMC_MISO = 12;
const int TMC_STEP = 14, TMC_DIR = 15, TMC_EN = 1;   // EN LOW = active
// LCD on SPI0
const int LCD_CS = 9, LCD_DC = 5, LCD_RST = 6, LCD_MOSI = 19, LCD_SCK = 18, LCD_BL = 20;
// Encoder, sensors, outputs
const int ENC_A = 26, ENC_B = 27, ENC_SW = 22;
const int LIMIT_SWITCH = 28, SPRAY_VALVE = 2, FLOW_SENSOR = 3;
const int LED_GREEN = 8, LED_YELLOW = 7, FAN_PWM = 21, ULTRASONIC = 4;
```

### Motion Parameters (angle-based; editable on-device, persisted to flash)

```cpp
const int FULL_STEPS_PER_REV = 200;   // 1.8° NEMA17
int  PARK_DEG_X10   = 70;             // 7.0° park angle
int  CENTER_DEG_X10 = 260;            // 26.0° sweep centre
int  ARM_LENGTH_MM  = 250;            // arm length (transducer radius)
unsigned long SWEEP_TIME_MS      = 4000;  // ms per full back-forward-back cycle
unsigned long OSCILLATION_CYCLES = 4;     // full cycles to run (0 = forever)
const unsigned long SPRAY_ACTIVE_WAIT = 2000;
bool SENSOR_INPUTS_ENABLED = false;   // false = Debug ON (spray/flow bypassed)
```

### State Machine

```cpp
enum SystemState {
    STATE_IDLE, STATE_HOMING, STATE_PARKED,
    STATE_WAITING_SPRAY, STATE_SPRAY_ACTIVE, STATE_OSCILLATING, STATE_ERROR
};
```

Control flags: `needsHoming` (re-home before a run), `stopRequested` (park then
disable), `faultLatched` (driver fault), `homingToStop` (abort path), and
`sensorBypassCycleArmed` (DEBUG-mode run gate).

### Persistent Settings

`StoredSettings` is written to flash via the RP2040 `EEPROM` emulation, tagged with a
magic (`"MSGC"`), a version (`SETTINGS_VERSION`), a size, and an FNV-1a checksum.
Compatible older records are accepted, while empty/corrupt flash is initialized with
defaults. Edits call `markSettingsDirty()`; `loop()` commits them after
`SETTINGS_SAVE_DELAY` (2.5 s) of quiet so a burst of encoder steps becomes one flash
write.

---

## Function API

### Motor Control (angle-based)

**`motorStep(int direction)`** — pulse STEP once; update `motorPosition`.
`1` = away from limit, `−1` = toward limit. Honours `motorDirectionInverted`.

**`motorSetEnable(bool enable)`** — drive EN. `false` (HIGH) disables and implies the
position is now unknown (`needsHoming`).

**`motorMoveTo(int target)`** — call repeatedly; one step per call toward `target`.

**`motorMoveToBlocking(int target, us)`** — busy-loop to a target (used for the live
Centre jog in Setup).

**`degX10ToSteps()` / `stepsToDegX10()`** — convert between arm angle (0.1°) and motor
microsteps using the live microstep setting and the motor→arm gear ratio (default 15:108).

### Sensors / Input

**`readSensors()`** — debounces the limit switch always; reads spray/flow only when
`SENSOR_INPUTS_ENABLED`, otherwise forces them inactive for Debug mode.

**`encoderIsr()` / `encoderButtonIsr()` / `readEncoder()`** — GPIO interrupts capture
quadrature detents and button edges; `readEncoder()` consumes those events, applies
acceleration/debounce, routes deltas to the active screen, and handles the click /
short-then-long-press menu-unlock gesture.

### Output Control

**`setLED(pin, state)`** — `true` = on.
**`setFan(speed)`** — PWM 0–255 via `analogWrite` (clamped).
**`setUltrasonic(bool on)`** — `true` drives GPIO 4 LOW (relay ON) and sets
`ultrasonicActive`. Callers pass `armOverWafer()` so the generator is on only over the
wafer.
**`armOverWafer()`** — true when `|currentAngle − centre| ≤ half the sweep`.

### TMC2130

Accessed through the TMCStepper `driver` object on SPI1, guarded by `spi_mutex`.

- `initTMC2130()` — current, microsteps, StealthChop, interpolation, hold delays.
- `applyDriverSettings()` — re-apply current/microsteps after an edit.
- `setDriverParkHold(bool)` — switch between run-hold (25 %) and park-hold (10 %).
- `pollDriverStatus()` — every 500 ms; reads `DRV_STATUS`/`GSTAT` and latches a fault on
  overtemperature, short-to-ground, or charge-pump undervoltage.

### State Machine

**`updateStateMachine()`** — evaluate transition conditions; update `currentState`.
**`handleState()`** — execute the current state's outputs (motor, LEDs, fan, generator).
Both run every Core 0 loop iteration.

### Display (Core 1)

**`updateDisplay()`** — snapshots volatile state once per call and redraws the LCD +
echoes to serial only on change. Dispatches to `drawMenu()` / `drawSettings()` /
`drawSetup()` / `drawAbout()` and the status column.
**`drawArmAnim()`** — top-down sketch under the basic menu and the Sweep Settings screen:
wafer circle, park tick, live arm arrow, and a blinking lightning sign while the generator
is on. The side status column shows the live state/angle plus the sweep config summary
(angle, time, wafer, type, profile); the run/park hold indicator was removed.

---

## Extending the Firmware

### Change default sweep / hardware parameters
Edit the initialisers near the top of `main.cpp` (`PARK_DEG_X10`, `CENTER_DEG_X10`,
`ARM_LENGTH_MM`, `SWEEP_TIME_MS`, `OSCILLATION_CYCLES`, `driverCurrent`,
`driverMicrosteps`). All are also editable on-device and override defaults from flash.

### Add a persisted setting
1. Add a field to `StoredSettings` and bump `SETTINGS_VERSION`.
2. Populate it in `saveSettings()` and read it (with `constrain`) in `loadSettings()`.
3. Expose it in the relevant screen (`drawSettingsRow`/`drawSetupRow` +
   `adjustSettingsValue`/`adjustSetupValue`), then call `markSettingsDirty()`.

> Bumping the version invalidates old flash blobs, so the device falls back to defaults
> on the first boot after the change — expected.

### Add a velocity profile
Extend `sweepStepIntervalUs()` with a new branch and add the constant + UI label
(`SWEEP_PROFILE_*`, `drawSettingsRow` case 3).

### Microstep changes need no recalibration
Motion is angle-based, so changing `driverMicrosteps` keeps all angles correct — no
steps-per-mm to recompute.

---

## Debugging Tips

### Serial commands
Type a key in the monitor: `d` dump TMC, `e`/`x` enable/disable, `+`/`-` single step,
`f`/`b` 400-step burst, `r` one shaft revolution, `?` help. See `handleSerialDebug()`.

### GPIO testing (multimeter)
- `0 V` = LOW, `3.3 V` = HIGH.
- EN (GPIO 1) reads `0 V` when the motor is active, `3.3 V` when disabled.
- ULTRASONIC (GPIO 4) reads `0 V` only while the arm is over the wafer during a cycle.

### Driver status
The **About** screen shows live `DRV_STATUS`/`GSTAT` fields (OT, OTPW, CS, SG, STST,
ERR). `d` on serial dumps the full register decode.

---

## Performance

| Metric | Value |
|--------|-------|
| Motor step cadence (homing/parking) | every 500 µs |
| Encoder input | GPIO interrupts; main loop consumes pending events |
| Driver status poll | every 500 ms |
| Settings auto-save debounce | 2.5 s of quiet |
| Display update | ~20 fps (Core 1, on change only) |
| SPI clock (LCD) | 20 MHz |

---

## References

- [PlatformIO Docs](https://docs.platformio.org/)
- [RP2040 Arduino Core (earlephilhower)](https://github.com/earlephilhower/arduino-pico)
- [TMCStepper Library](https://github.com/teemuatlut/TMCStepper)
- [Adafruit ST7789 Library](https://github.com/adafruit/Adafruit-ST7789-Library)
</content>
