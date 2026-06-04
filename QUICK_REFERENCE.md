# Quick Reference Card

## File Map

| File | Purpose |
|------|---------|
| [README.md](README.md) | Project overview & features |
| [HARDWARE.md](HARDWARE.md) | Pin assignments & wiring |
| [STATE_MACHINE.md](STATE_MACHINE.md) | State diagram & behavior |
| [SETUP.md](SETUP.md) | Development environment & API |
| [TESTING.md](TESTING.md) | Validation & troubleshooting |
| [src/main.cpp](src/main.cpp) | Firmware (400+ lines) |
| [platformio.ini](platformio.ini) | Project config |

## Build Commands

```bash
# Build only
pio run

# Build and upload
pio run --target upload

# Monitor serial (115200 baud)
pio device monitor --baud 115200

# Clean build
pio run --target clean

# Monitor only (without building)
pio device monitor --baud 115200
```

## Pin Quick Map

| Function | GPIO |
|----------|------|
| TMC_CS | 17 |
| TMC_SCK | 18 |
| TMC_MOSI | 19 |
| TMC_MISO | 16 |
| TMC_STEP | 14 |
| TMC_DIR | 15 |
| TMC_EN | 13 |
| LCD_CS | 9 |
| LCD_DC | 10 |
| LCD_RST | 11 |
| ENC_A | 26 |
| ENC_B | 27 |
| LIMIT_SWITCH | 28 |
| SPRAY_VALVE | 2 |
| FLOW_SENSOR | 3 |
| LED_GREEN | 8 |
| LED_YELLOW | 7 |
| FAN_PWM | 12 |
| ULTRASONIC | 4 |

## State Machine States

```
IDLE → HOMING → PARKED → WAITING_SPRAY → SPRAY_ACTIVE → OSCILLATING → IDLE
```

| State | Duration |
|-------|----------|
| IDLE | Indefinite (waiting) |
| HOMING | ~2-5 seconds (to limit) |
| PARKED | Indefinite (waiting for spray+flow) |
| WAITING_SPRAY | ~30 seconds max (waiting for flow) |
| SPRAY_ACTIVE | 2 seconds fixed |
| OSCILLATING | ~54 minutes (20 cycles × 2.7 min each) |

## Key Functions

```cpp
// Motor
motorStep(int dir);           // ±1 step
motorSetEnable(bool en);      // Enable/disable

// Sensors
readSensors();                // Poll all inputs
readEncoder();                // TODO

// Outputs
setLED(pin, state);           // On/off
setFan(speed);                // 0-255 PWM
setUltrasonic(active);        // On/off

// SPI
tmcWriteReg(addr, val);       // Write register
tmcReadReg(addr);             // Read register

// State machine
updateStateMachine();         // Check transitions
handleState();                // Execute state action

// Display
updateDisplay();              // Serial output

// Display module used in this project:
// ST7789V3 1.69" (240x280). Use the `Adafruit_ST7789` library and initialize with `tft.init(240, 280);` in `initDisplay()`.
```

## Motion Parameters

```cpp
STEPS_PER_REVOLUTION = 200    // NEMA17 stepper
PARK_POSITION = 7             // mm from limit
CENTER_POSITION = 26          // mm from limit
OSCILLATION_STEPS = 16        // steps per cycle
OSCILLATION_DELAY = 10000     // ms per step
OSCILLATION_CYCLES = 20       // total cycles
```

**Calculation**:
- 1 cycle = 16 steps × 10 sec = 160 sec = 2.7 min
- 20 cycles = 54 min total

## Serial Monitor Output

```
=== Stepper Controller Initializing ===
Hardware initialized
SPI initialized: SCK=18 MOSI=19 MISO=16
Encoder initialized: A=26 B=27
TMC2130 driver: Configure via SPI - TODO
Initialization complete!

State: IDLE | Pos: 0 | Spray: OFF | Flow: NO
State: IDLE | Pos: 0 | Spray: OFF | Flow: NO
[... repeated every 50ms ...]
```

## Sensor Logic

| Sensor | Active | Reading | GPIO |
|--------|--------|---------|------|
| Limit Switch | Pressed | LOW | 28 |
| Spray Valve | Flowing | HIGH | 2 |
| Flow Sensor | Flowing | HIGH | 3 |

## LED Behavior

| State | Green | Yellow |
|-------|-------|--------|
| IDLE | OFF | OFF |
| HOMING | ON | OFF |
| PARKED | ON | OFF |
| WAITING_SPRAY | OFF | ON |
| SPRAY_ACTIVE | ON | OFF |
| OSCILLATING | ON | OFF |
| ERROR | OFF | ON |

## Fan Speed

| State | PWM | % |
|-------|-----|---|
| IDLE | 0 | 0% |
| WAITING_SPRAY | 128 | 50% |
| SPRAY_ACTIVE | 255 | 100% |
| OSCILLATING | 255 | 100% |

## Ultrasonic Control

| State | GPIO 4 | Relay |
|-------|--------|-------|
| IDLE → WAITING_SPRAY | HIGH | OFF |
| SPRAY_ACTIVE → OSCILLATING | LOW | ON |
| OSCILLATING → IDLE | HIGH | OFF |

## Common Tasks

### Change Oscillation Speed
```cpp
// In src/main.cpp line 47:
const unsigned long OSCILLATION_DELAY = 10000;  // milliseconds per step
// Try 5000 for 2× faster, 20000 for 2× slower
```

### Change Oscillation Duration
```cpp
// In src/main.cpp line 48:
const unsigned long OSCILLATION_CYCLES = 20;  // repetitions
// Try 30 for longer, 10 for shorter
```

### Change Park Position
```cpp
// In src/main.cpp line 45:
const int PARK_POSITION = 7;  // mm from limit
// Adjust based on where you want motor to rest
```

### Enable Debug Output
Add to `updateDisplay()`:
```cpp
Serial.print("DEBUG: variable_name = ");
Serial.println(variable_value);
```

Then check with:
```bash
pio device monitor --baud 115200
```

## Typical Development Workflow

1. **Modify Code**
   ```bash
   # Edit src/main.cpp in VS Code
   ```

2. **Compile**
   ```bash
   pio run
   ```

3. **Upload**
   ```bash
   pio run --target upload
   ```

4. **Monitor**
   ```bash
   pio device monitor --baud 115200
   ```

5. **Repeat** as needed

## Libraries

```
platformio.ini:
  - TMCStepper (teemuatlut/TMCStepper)
  - Adafruit GFX Library
  - Adafruit ST7735 and ST7789 Library
```

Install:
```bash
pio lib install "teemuatlut/TMCStepper" "Adafruit GFX Library" "Adafruit ST7735 and ST7789 Library"
```

## Known Issues & TODO

| Item | Status | Impact |
|------|--------|--------|
| TMC2130 register config | TODO | Driver runs in default mode (may need tuning) |
| Encoder quadrature | TODO | No menu control (not used yet) |
| LCD rendering | TODO | No visual display (serial only) |
| Error recovery | TODO | No fault state handling |

## Hardware Requirements

- **RP2040 Pico**: ~$15
- **TMC2130 Stepper Driver**: ~$20
- **SPI LCD Display** (ST7735/ST7789): ~$10
- **NEMA17 Stepper Motor**: ~$15
- **Rotary Encoder**: ~$5
- **Supporting Electronics**: ~$20 (resistors, capacitors, connectors)

**Total BOM**: ~$85

## Performance

| Metric | Value |
|--------|-------|
| Flash Usage | 0.2% (4.2 KB of 2 MB) |
| RAM Usage | 15.4% (41.5 KB of 264 KB) |
| CPU Load | ~5-10% per state |
| Polling Interval | 50 ms |
| SPI Speed | 1 MHz |
| Step Pulse Rate | ~100 Hz (10 ms/step) |

## Debugging Tips

### Check Compilation
```bash
pio run 2>&1 | grep -i error
```

### Monitor Serial with Filtering
```bash
# Show only state changes
pio device monitor --baud 115200 | grep "State:"
```

### Test GPIO Manually
Use multimeter to check GPIO voltage (0V = LOW, 3.3V = HIGH)

### Capture SPI with Oscilloscope
- SCK should pulse at 1 MHz
- MOSI/MISO should toggle with data
- CS should go LOW during transactions

## Links

- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [TMC2130 Datasheet](https://www.trinamic.com/fileadmin/assets/Documents/TMC2130_Datasheet_Rev1.35.pdf)
- [GitHub Repository](https://github.com/GhVost/Stepper-controller)
- [PlatformIO Docs](https://docs.platformio.org/)

## Contact & Support

- **Author**: GhVost
- **Repository**: https://github.com/GhVost/Stepper-controller
- **Issues**: GitHub Issues

---

**Last Updated**: 2024  
**Firmware Version**: 1.0  
**Status**: Development (pre-hardware testing)
