# Stepper Controller for Megasonic Ultrasonic Cleaner

RP2040-based stepper motor controller featuring TMC2130 driver, SPI LCD display, rotary encoder input, and automatic oscillation control for ultrasonic wafer cleaning.

## Features

- **RP2040 Microcontroller** – Dual-core ARM, 133 MHz, 264 KB RAM
- **TMC2130 Stepper Driver** – SPI-controlled stepper with current limiting and stall detection
- **SPI LCD Display** – GMT147SPI 1.47" SPI 172×320 (ST7789 controller)
- **Rotary Encoder** – Quadrature input + push-button for menu navigation
- **Sensor Integration** – Limit switch, spray valve, flow sensor
- **Non-Blocking State Machine** – Smooth operation with millisecond-based timing
- **Safety Features** – Motor enable/disable, ultrasonic remote control, fan speed PWM

## Quick Start

### 1. Hardware Setup
See [HARDWARE.md](HARDWARE.md) for:
- Pin assignments (TMC2130, LCD, encoder, sensors)
- SPI wiring diagram
- Power requirements

### 2. Build & Upload
```bash
# Using PlatformIO
pio run                    # Build
pio run --target upload    # Flash to Pico (requires bootloader mode)
```

### 3. Monitor Serial Output
```bash
pio device monitor --baud 115200
```

Expected startup:
```
=== Stepper Controller Initializing ===
Hardware initialized
SPI initialized: SCK=18 MOSI=19 MISO=16
Encoder initialized: A=26 B=27
TMC2130 driver: Configure via SPI - TODO
Initialization complete!
State: IDLE | Pos: 0 | Spray: OFF | Flow: NO
```

## System States

The controller operates as a state machine with 7 states:

| State | Behavior |
|-------|----------|
| `IDLE` | Waiting for spray valve activation |
| `HOMING` | Moving motor toward limit switch (position 0) |
| `PARKED` | Motor at park position, waiting for spray + flow |
| `WAITING_SPRAY` | Spray on but no flow detected |
| `SPRAY_ACTIVE` | Preparing for oscillation (fan on, ultrasonic on) |
| `OSCILLATING` | Motor oscillating back/forth for cleaning |
| `ERROR` | Fault condition detected |

See [STATE_MACHINE.md](STATE_MACHINE.md) for detailed flow diagram.

## Pin Assignments

### Motor Control (TMC2130)
| Pin | GPIO | Function |
|-----|------|----------|
| CS  | 17   | SPI chip select |
| SCK | 18   | SPI clock |
| MOSI| 19   | SPI data out (controller → driver) |
| MISO| 16   | SPI data in (driver → controller) |
| STEP| 14   | Step pulse input |
| DIR | 15   | Direction control |
| EN  | 13   | Enable (LOW = active) |

### LCD Display — GMT147SPI 1.47" 172×320 (SPI)
| Board Label | GPIO | Function |
|-------------|------|----------|
| CS          | 9    | Chip select |
| DC          | 10   | Data/Command |
| RES         | 11   | Reset |
| SCL         | 18   | Shared SPI clock |
| SDA         | 19   | Shared SPI data |
| BL          | 20   | Backlight (HIGH = on) |

### Rotary Encoder (KY-040)
| Board Label | GPIO | Function |
|-------------|------|----------|
| CLK         | 26   | Quadrature A |
| DT          | 27   | Quadrature B |
| SW          | 22   | Push-button (LOW when pressed) |

### Sensors & Outputs
| Pin | GPIO | Function |
|-----|------|----------|
| Limit | 28 | End position (LOW when pressed) |
| Spray | 2  | Spray valve status (HIGH = active) |
| Flow  | 3  | Flow sensor input (HIGH = flowing) |
| LED_G | 8  | Green status LED |
| LED_Y | 7  | Yellow status LED |
| Fan   | 12 | Fan PWM (0-255) |
| Sonic | 4  | Ultrasonic remote (LOW = ON) |

See [HARDWARE.md](HARDWARE.md) for complete wiring details.

## Sensor Behavior

### Limit Switch (GPIO 28)
- **Purpose**: Home/reference position detection
- **Activation**: LOW (connected to GND when pressed)
- **Used in**: STATE_HOMING to establish zero position

### Spray Valve (GPIO 2)
- **Purpose**: Detect developer spray activation
- **Activation**: HIGH (3.3V when valve opens)
- **Used in**: IDLE → HOMING transition trigger

### Flow Sensor (GPIO 3)
- **Purpose**: Detect liquid flow through system
- **Activation**: HIGH (3.3V when flowing)
- **Used in**: PARKED → SPRAY_ACTIVE transition

## Motion Parameters

```cpp
const int STEPS_PER_REVOLUTION = 200;  // NEMA17 typical
const int PARK_POSITION = 7;            // mm from limit
const int CENTER_POSITION = 26;         // Park + 34 mm
const int OSCILLATION_STEPS = 16;       // Steps per oscillation cycle
const unsigned long OSCILLATION_DELAY = 10000;  // ms between steps
const unsigned long OSCILLATION_CYCLES = 20;    // Total oscillations
```

**Oscillation Pattern:**
- Moves from CENTER_POSITION toward limit switch
- One step every 10 seconds
- 16 steps total per cycle = ~2.7 minutes per cycle
- Repeats 20 times = ~54 minutes total cleaning

## Dependencies

- **TMCStepper** (teemuatlut) – TMC2130 SPI interface
- **Adafruit GFX Library** – Graphics primitives
- **Adafruit ST7735/ST7789** – LCD driver (for ST7789V3 use `Adafruit_ST7789`)
- Arduino Framework for RP2040

## Development Status

### ✅ Implemented
- [x] State machine core logic
- [x] Motor control (step/dir)
- [x] Sensor reading (limit, spray, flow)
- [x] LED indicators
- [x] Fan PWM control
- [x] SPI initialization for TMC2130
- [x] Non-blocking timing system

### 🔄 TODO
- [ ] TMC2130 SPI register configuration (current, microsteps)
- [ ] Rotary encoder quadrature decoding
- [ ] SPI LCD display rendering
- [ ] Error recovery logic
- [ ] EEPROM parameter storage
- [ ] Serial debug interface

## Troubleshooting

| Issue | Cause | Fix |
|-------|-------|-----|
| No serial output | Baud mismatch | Use 115200 baud |
| Motor won't move | EN pin HIGH | Check GPIO 13 is LOW when enabled |
| Stuck in IDLE | Spray sensor LOW | Check GPIO 2 reads HIGH |
| Stuck in HOMING | Limit never pressed | Check GPIO 28 with multimeter |
| LCD not showing | Wrong pins or lib | See HARDWARE.md for SPI check |

## License

MIT

## References

- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [TMC2130 Datasheet](https://www.trinamic.com/products/integrated-circuits/details/tmc2130-ta/)
- [Adafruit GFX Docs](https://github.com/adafruit/Adafruit-GFX-Library)
