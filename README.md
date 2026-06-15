# MEGASONIC — Stepper Controller for an Ultrasonic Wafer Cleaner

RP2040-based controller for a megasonic/ultrasonic wafer cleaner. A NEMA17 arm
sweeps the ultrasonic transducer across a wafer while a TMC2130 driver, an SPI
LCD, and a rotary encoder provide a complete on-device UI. Motion is described in
**arm angle** (degrees), and the sweep width is derived automatically from the
selected wafer diameter and the arm length.

## Features

- **RP2040 microcontroller** – dual-core ARM, 133 MHz, 264 KB RAM
- **Dual-core architecture** – Core 0 runs the state machine and I/O; Core 1 renders the LCD at ~20 fps
- **TMC2130 stepper driver** – SPI-controlled, StealthChop, current limiting, run/park hold-current modes, live fault detection (overtemperature, short-to-ground, charge-pump UV)
- **Hardware SPI LCD** – GMT147SPI 1.47" 172×320 ST7789 at 20 MHz on its own SPI bus
- **Rotary encoder UI** – KY-040 quadrature + push-button; full menu, in-place value editing, and a basic/advanced menu unlock
- **Angle-based motion** – park and centre angles in degrees; sweep angle computed from wafer Ø and arm length
- **Configurable sweep** – sweep time, wafer diameter, path (back-centre / back-front), and velocity profile (linear / harmonic / inverse-distance)
- **Persistent settings** – all parameters stored in flash EEPROM emulation and reloaded at boot
- **Two operating modes** – sensor-driven (spray valve + flow sensor) or a menu-driven **Debug** bypass mode
- **Safety** – re-home before every run, home-search timeout, ultrasonic energised only while the arm is over the wafer, park-then-disable stop, and a recoverable ERROR state

## Quick Start

### 1. Hardware Setup
See [HARDWARE.md](HARDWARE.md) for:
- Pin assignments (TMC2130 on SPI1, LCD on SPI0, encoder, sensors)
- Wiring diagrams and power requirements
- TMC2130 driver configuration and `R_SENSE`

### 2. Build & Upload
```bash
pio run                    # Build
pio run --target upload    # Flash to Pico (BOOTSEL/bootloader mode)
```

### 3. Monitor Serial Output
```bash
pio device monitor --baud 115200
```

Expected startup:
```
=== Stepper Controller Initializing ===
Settings: loaded from flash
Hardware initialized
SPI0 LCD initialized: SCK=18 MOSI=19
SPI1 TMC initialized: SCK=10 MOSI=11 MISO=12
TMC2130 configured: 600 mA, run hold 25%, park hold 10%, 256x microsteps, interpolation, StealthChop
Encoder initialized: CLK=26 DT=27 SW=22 (interrupt mode)
Display initialized (GMT147SPI 1.47" 172x320)
Initialization complete!
```

State/position changes are echoed to serial by Core 1 (only on change):
```
State:IDLE | Pos:0 steps (0.0 deg) | Spray:OFF | Flow:NO
```

## On-Device UI

The encoder drives the whole interface: **rotate** to move the selection or change a
value, **click** to select / enter-edit / confirm, and **long-press** to go back to the
menu from the Sweep Settings / Setup screens (there is no "< Back" row).

- **Basic menu** (default): `START/STOP` and `Settings`. An arm-position animation under
  the rows shows the wafer (circle), the park tick, the live arm position (red arrow), and
  a blinking lightning sign while the ultrasonic generator is energised.
- **Advanced menu**: a short-click immediately followed by a long-press toggles the
  advanced items (`Setup` and `About`) on/off.
- **Sweep Settings**: sweep time, wafer diameter, **sweep type**, and **speed profile** —
  each row shows `label:value` in a large font with the value highlighted, and the arm
  animation (about a third of the screen height) sits underneath. The calculated sweep angle
  is shown in the side status bar.
- **Setup** (hardware): park angle, centre angle (live jog while editing), arm length,
  cycles, driver current, microsteps, direction invert, and the **Debug** toggle
  (`ON` = spray/flow ignored, `OFF` = spray/flow safety inputs active).
- **About**: firmware version and live TMC2130 driver status.
- **Status bar** (right side of every screen): live state, arm angle, the sweep summary
  (sweep angle, time, wafer, type, profile), and spray/flow.

Settings are stored in RP2040 flash EEPROM emulation and reloaded at boot. Empty or
corrupt flash is initialized with defaults; compatible older records are accepted and
rewritten with the current version when saved.

## System States

The controller is a non-blocking state machine with 7 states:

| State | Behavior |
|-------|----------|
| `IDLE` | Motor disabled; waiting for START (menu) or, in sensor mode, the spray valve |
| `HOMING` | Driving toward the limit switch to re-establish the zero reference |
| `PARKED` | Moving to / holding the park angle, then deciding the next step |
| `WAITING_SPRAY` | Spray on but no flow yet (sensor mode only); fan at 50 % |
| `SPRAY_ACTIVE` | Fan 100 %, generator on over the wafer; moving to the sweep start |
| `OSCILLATING` | Sweeping the arm across the wafer for the configured cycles |
| `ERROR` | Fault latched; motor disabled, yellow LED blinks. Recover with START |

See [STATE_MACHINE.md](STATE_MACHINE.md) for the full flow diagram and transition table.

## Pin Assignments

The TMC2130 and the LCD are on **separate** SPI buses (no shared bus).

### Motor Driver — TMC2130 on SPI1
| Pin | GPIO | Function |
|-----|------|----------|
| CS  | 13   | SPI chip select |
| SCK | 10   | SPI1 clock |
| MOSI (SDI) | 11 | Data out (controller → driver) |
| MISO (SDO) | 12 | Data in (driver → controller) |
| STEP | 14  | Step pulse |
| DIR | 15   | Direction |
| EN  | 1    | Enable (LOW = active) |

### LCD Display — GMT147SPI 1.47" 172×320 ST7789 on SPI0
| Board Label | GPIO | Function |
|-------------|------|----------|
| CS  | 9  | Chip select |
| DC  | 5  | Data/Command |
| RES | 6  | Reset |
| SCL | 18 | SPI0 clock |
| SDA | 19 | SPI0 data (MOSI) |
| BL  | 20 | Backlight (HIGH = on) |

### Rotary Encoder (KY-040)
| Board Label | GPIO | Function |
|-------------|------|----------|
| CLK | 26 | Quadrature A |
| DT  | 27 | Quadrature B |
| SW  | 22 | Push-button (LOW when pressed) |

### Sensors & Outputs
| Signal | GPIO | Function |
|--------|------|----------|
| Limit | 28 | Home limit switch (INPUT_PULLUP, LOW when pressed) |
| Spray | 2  | Spray-valve status (INPUT_PULLDOWN, HIGH = active) |
| Flow  | 3  | Flow sensor (INPUT_PULLDOWN, HIGH = flowing) |
| LED_G | 8  | Green status LED |
| LED_Y | 7  | Yellow status LED |
| Fan   | 21 | Fan PWM (0–255) |
| Sonic | 4  | Ultrasonic generator relay (active-low, LOW = ON) |

See [HARDWARE.md](HARDWARE.md) for complete wiring details.

## Sensor Behavior

Spray/flow sensors are read when **Debug** is `OFF` (`SENSOR_INPUTS_ENABLED = true`).
In the default Debug `ON` mode the spray/flow inputs are ignored and the cycle is driven
entirely from the menu.

### Limit Switch (GPIO 28)
- **Purpose**: home/reference position detection
- **Activation**: LOW (to GND when pressed); 20 ms debounce
- **Used in**: `HOMING` to establish the zero position; always re-homed before a run

### Spray Valve (GPIO 2)
- **Purpose**: detect cleaning-fluid spray activation
- **Activation**: HIGH (3.3 V when the valve opens)
- **Used in**: `IDLE → HOMING` trigger (sensor mode)

### Flow Sensor (GPIO 3)
- **Purpose**: detect liquid flow through the system
- **Activation**: HIGH (3.3 V when flowing)
- **Used in**: `PARKED`/`WAITING_SPRAY → SPRAY_ACTIVE` (sensor mode)

## Motion Model

Motion is expressed as **arm angle** rather than linear travel. Steps are derived from
the angle, the motor's full-steps-per-revolution, and the current microstep setting:

```cpp
steps = degrees × FULL_STEPS_PER_REV × microsteps / 360
```

Default parameters (all editable on-device and persisted to flash):

```cpp
const int FULL_STEPS_PER_REV = 200;   // 1.8° NEMA17
int  PARK_DEG_X10   = 70;             // 7.0° — park angle near the limit
int  CENTER_DEG_X10 = 260;            // 26.0° — sweep centre (over wafer)
int  ARM_LENGTH_MM  = 250;            // arm length (transducer radius)
unsigned long SWEEP_TIME_MS    = 4000; // time for one full back-forward-back cycle
unsigned long OSCILLATION_CYCLES = 4;  // full cycles to run (0 = run forever)
```

The **sweep angle** is computed from the selected wafer diameter and the arm length so
the sweep extremes land on the wafer edges:

```
sweep = 2 · asin( (wafer_diameter / 2) / arm_length )
```

- **Sweep type** `Edge↔(•)`: arm travels edge → centre (half the sweep).
- **Sweep type** `Edge↔Edge`: arm travels edge → edge (full sweep).
- **Speed profile**: `Sine`, `Sawtooth`, or `Cosecant` — velocity-shaping across each sweep
  (`Cosecant` is slowest at the wafer centre and fastest at the edges, for either sweep type).

The ultrasonic generator is energised **only while the arm tip is over the wafer disk**.

## Dependencies

- **earlephilhower/arduino-pico** – RP2040 Arduino core: dual-core `setup1()`/`loop1()`, hardware SPI, accurate `analogWrite`
- **TMCStepper** (teemuatlut) – TMC2130 SPI interface
- **Adafruit GFX Library** – graphics primitives
- **Adafruit ST7735/ST7789** – LCD driver (`Adafruit_ST7789`)

## Development Status

### ✅ Implemented
- [x] Non-blocking dual-core state machine
- [x] Angle-based motor control (step/dir) with microstepping
- [x] TMC2130 driver configuration over SPI1, with run/park hold-current modes
- [x] Driver fault detection (OT / short-to-ground / charge-pump UV) with park-then-disable
- [x] Home-search safety timeout and re-home before every run
- [x] Sensor reading (limit, spray, flow) with debounce + sensor-bypass DEBUG mode
- [x] LED indicators, fan PWM, ultrasonic relay (energised only over the wafer)
- [x] LCD UI — hardware SPI 20 MHz, partial redraw, ~20 fps on Core 1, arm-position animation
- [x] Rotary encoder — interrupt-based quadrature/button input, acceleration, basic/advanced menu unlock
- [x] On-device Settings/Setup editors
- [x] Persistent settings in flash (versioned, checksummed, compatible record loading)
- [x] Recoverable ERROR state (START re-homes and clears the fault latch)

### 🔄 Possible future work
- [ ] StallGuard-based stall/load detection
- [ ] Per-profile sweep tuning UI

## Troubleshooting

| Issue | Cause | Fix |
|-------|-------|-----|
| No serial output | Baud mismatch | Use 115200 baud |
| Motor won't move | EN pin HIGH | Check GPIO 1 is LOW when enabled |
| Won't start in DEBUG mode | Not at a known position | Use START — it re-homes first |
| Stuck in IDLE (sensor mode) | Spray sensor LOW | Check GPIO 2 reads HIGH, or set Debug = ON |
| ERROR right after start | Driver fault or home not found | Check TMC2130 (About screen) and limit switch wiring; press START to retry |
| Stuck in HOMING | Limit never pressed | Check GPIO 28; home search times out after 70° → ERROR |
| LCD not showing | Wrong pins or lib | See HARDWARE.md (LCD is on SPI0: SCK 18, SDA 19) |

## License

MIT — see [LICENSE](LICENSE).

## References

- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [TMC2130 Datasheet](https://www.trinamic.com/products/integrated-circuits/details/tmc2130-ta/)
- [Adafruit GFX Docs](https://github.com/adafruit/Adafruit-GFX-Library)
</content>
</invoke>
