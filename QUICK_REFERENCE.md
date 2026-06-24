# Quick Reference Card

## File Map

| File | Purpose |
|------|---------|
| [README.md](README.md) | Project overview & features |
| [HARDWARE.md](HARDWARE.md) | Pin assignments, wiring, driver config |
| [STATE_MACHINE.md](STATE_MACHINE.md) | State diagram & behaviour |
| [SETUP.md](SETUP.md) | Development environment & API |
| [TESTING.md](TESTING.md) | Validation & troubleshooting |
| [src/main.cpp](src/main.cpp) | All firmware |
| [tools/gui_preview.html](tools/gui_preview.html) | Browser mock-up of the LCD UI |
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

Two separate SPI buses: **TMC2130 on SPI1**, **LCD on SPI0**.

| Function | GPIO |
|----------|------|
| TMC_CS | 13 |
| TMC_SCK | 10 (SPI1) |
| TMC_MOSI | 11 (SPI1) |
| TMC_MISO | 12 (SPI1) |
| TMC_STEP | 14 |
| TMC_DIR | 15 |
| TMC_EN | 1 (LOW = active) |
| LCD_CS | 9 |
| LCD_DC | 5 |
| LCD_RST | 6 |
| LCD_SCK | 18 (SPI0) |
| LCD_MOSI | 19 (SPI0) |
| LCD_BL | 20 (HIGH = on) |
| ENC_A / ENC_B | 26 / 27 (INPUT_PULLUP) |
| ENC_SW | 22 (INPUT_PULLUP, LOW = pressed) |
| LIMIT_SWITCH | 28 (INPUT_PULLUP, LOW = pressed) |
| SPRAY_VALVE | 2 (INPUT_PULLDOWN, HIGH = active) |
| FLOW_SENSOR | 3 (INPUT_PULLDOWN, HIGH = flowing) |
| LED_GREEN / LED_YELLOW | 8 / 7 |
| FAN_PWM | 21 |
| ULTRASONIC | 4 (LOW = ON, active-low relay) |

---

## On-Device UI

- **Rotate** = navigate / change value, **Click** = select / edit / confirm.
- **Long press** = back to the menu from Sweep Settings / Setup (no "< Back" row).
- **Basic menu**: START/STOP, Settings (+ arm-position animation).
- **Advanced menu** (Setup, About): short-click then long-press to toggle.
- **Sweep Settings**: Sweep time, Wafer diam., Sweep type, Speed prof. — each row shows
  `label:value` (large font, value highlighted) with the arm animation (~1/3 screen)
  underneath. Speed profiles are Sawtooth / Sine / Reciprocal; the sweep angle is in the status bar.
- **Setup** (hardware): 17 rows, **scrolls vertically** (6-row window, `row/total` counter,
  ▲/▼ markers). Park, Centre (live jog), Parkspd (deg/s positioning speed), Arm,
  Gear in / Gear out (teeth, default 15:108), Cycles, Accel (deg/s²), Jerk (deg/s³),
  Backlsh (µsteps on reversal), Current, RunHold %, PrkHold %, Chop (Stealth/Spread),
  Mstep, Invert, Debug (ON = ignore spray/flow, OFF = use safety inputs).
- Edits persist to RP2040 flash EEPROM emulation and are reloaded at boot.

---

## State Machine

```
IDLE → HOMING → PARKED → SPRAY_ACTIVE → OSCILLATING → PARKED → IDLE
                  └→ WAITING_SPRAY ─┘   (sensor mode: spray, then flow)
STOP → PARKED → IDLE         Fault/timeout → ERROR (START to recover)
```

START always re-homes first (position is unknown whenever the motor was disabled).

### State Duration

| State | Duration |
|-------|----------|
| IDLE | Indefinite |
| HOMING | Until limit switch (travel > 120° or homing timeout → ERROR; timeout scales with Parking speed / gear) |
| PARKED | Until at park angle, then branch |
| WAITING_SPRAY | Until flow detected or spray lost (sensor mode) |
| SPRAY_ACTIVE | ≥ 2 s + time to reach the sweep start |
| OSCILLATING | `OSCILLATION_CYCLES` full back-forward-back cycles × `SWEEP_TIME_MS` (0 cycles = forever) |

### LED Indicators

| State | Green | Yellow |
|-------|-------|--------|
| IDLE | OFF | OFF |
| HOMING | OFF | ON |
| PARKED | ON | OFF |
| WAITING_SPRAY | OFF | ON |
| SPRAY_ACTIVE / OSCILLATING | ON | OFF |
| ERROR | OFF | Blink 1 Hz |

### Fan Speed

| State | PWM | % |
|-------|-----|---|
| IDLE / HOMING / PARKED | 0 | 0 % |
| WAITING_SPRAY | 128 | 50 % |
| SPRAY_ACTIVE / OSCILLATING | 255 | 100 % |

---

## Key Functions

```cpp
// Motor (angle-based)
motorStep(int dir);           // ±1 step; updates motorPosition
motorSetEnable(bool en);      // LOW = energised
motorMoveTo(int target);      // call repeatedly — one step toward target
degX10ToSteps(int degX10);    // arm angle (0.1°) → motor microsteps (incl. gear ratio)

// Sensors / input
readSensors();                // limit always; spray/flow only when Debug = OFF
readEncoder();                // poll quadrature rotation + consume interrupt button events

// Outputs
setLED(pin, state);
setFan(speed);                // 0–255 PWM (clamped)
setUltrasonic(bool on);       // true = relay ON (GPIO 4 LOW)
armOverWafer();               // true when the tip is over the wafer disk

// State machine / settings
updateStateMachine();         // evaluate transitions
handleState();                // execute state actions
pollDriverStatus();           // poll + latch driver faults
saveSettings() / loadSettings();  // flash persistence (versioned + checksummed)
```

---

## Motion Parameters

```cpp
gearTeethMotor    = 15      // motor pinion teeth   } 15:108 = 7.2:1 reduction
gearTeethOutput   = 108     // arm output gear teeth }
PARK_DEG_X10      = 50      // 5.0° park angle near the limit
CENTER_DEG_X10    = 700     // 70.0° sweep centre over the wafer
parkingSpeedDegS  = 10      // arm deg/s for homing/park/staging (gear+µstep aware)
ARM_LENGTH_MM     = 250     // arm length (transducer radius)
backlashMicrosteps = 0      // extra µsteps injected on each direction reversal
SWEEP_TIME_MS     = 4000    // ms per full back-forward-back cycle
OSCILLATION_CYCLES = 4      // full cycles to run (0 = forever)
SPRAY_ACTIVE_WAIT = 2000    // ms settle before oscillation
```

Sweep half-width = `asin((waferØ/2)/armLength)`; `Edge↔(•)` = half (edge→centre),
`Edge↔Edge` = full (edge→edge).

---

## Serial Debug Commands

Type a key in the serial monitor (115200):

| Key | Action |
|-----|--------|
| `d` | Dump TMC2130 registers |
| `e` / `x` | Enable / disable motor |
| `+` / `-` | One step forward / back |
| `f` / `b` | 400 steps forward / back |
| `r` | One full shaft revolution |
| `k` | Toggle encoder diagnostics (per-transition + per-step serial log) |
| `[` / `]` | Decrease / increase the encoder poll interval (1-20 ms) |
| `a` / `A` | Decrease / increase endpoint accel (1-2000 deg/s², persisted) |
| `j` / `J` | Decrease / increase endpoint jerk (20-20000 deg/s³, persisted) |
| `?` | List commands |

**Startup:**
```
=== Stepper Controller Initializing ===
Settings: loaded from flash
Hardware initialized
SPI0 LCD initialized: SCK=18 MOSI=19
SPI1 TMC initialized: SCK=10 MOSI=11 MISO=12
TMC2130 configured: 800 mA, run hold 25%, park hold 10%, 256x microsteps, interpolation, StealthChop
Encoder initialized: CLK=26 DT=27 SW=22 (polled rotation, interrupt button)
Display initialized (GMT147SPI 1.47" 172x320)
Initialization complete!
```

**During operation** (on change):
```
Menu: START → HOMING
→ PARKED (moving to park position)
State:PARKED | Pos:78 steps (7.0 deg) | Spray:OFF | Flow:NO
→ SPRAY_ACTIVE (sensor bypass)
→ OSCILLATING
→ PARKED (cleaning cycle complete)
→ IDLE (parked, motor disabled)
```

---

## Common Adjustments

Most are editable on-device (Settings / Setup) and persist to flash. To change defaults,
edit the constants near the top of `src/main.cpp`:

```cpp
unsigned long OSCILLATION_CYCLES = 8;     // more/longer cleaning (0 = forever)
unsigned long SWEEP_TIME_MS      = 2000;  // faster sweeps
int           driverCurrent      = 800;   // more torque (was 600 mA)
int           driverMicrosteps   = 128;   // resolution (no recalibration needed)
```

---

## Known Issues & TODO

| Item | Status | Notes |
|------|--------|-------|
| TMC2130 SPI config | ✅ Done | SPI1; run/park hold modes; fault polling |
| Driver fault handling | ✅ Done | OT / S2G / charge-pump UV → park + ERROR |
| Encoder + menu UI | ✅ Done | Polled quadrature + interrupt button, navigation, edit, cyclic menus, basic/advanced unlock |
| Persistent settings | ✅ Done | Versioned, checksummed, compatible flash records |
| Error recovery | ✅ Done | START clears the latch and re-homes |
| LCD display | ✅ Done | GMT147SPI 172×320, 20 MHz HW SPI, ~20 fps, arm animation |
| StallGuard load detection | 🔄 Future | Hardware supports it |

---

## Links

- [GitHub Repository](https://github.com/GhVost/Stepper-controller)
- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [TMC2130 Datasheet](https://www.trinamic.com/fileadmin/assets/Documents/TMC2130_Datasheet_Rev1.35.pdf)
- [PlatformIO Docs](https://docs.platformio.org/)
</content>
