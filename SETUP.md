# Developer Setup & API Reference

## Development Environment

### Requirements
- **PlatformIO Core** or **PlatformIO IDE** (VS Code extension recommended)
- **Python 3.7+** (required by PlatformIO)
- **Git** (for version control)
- **RP2040 Pico** board with USB cable
- **Terminal**: PowerShell, Command Prompt, or Git Bash

### Installation

#### 1. Install PlatformIO
```bash
# Via VS Code
# Install extension: "PlatformIO IDE" (platformio.platformio-ide)

# Or via pip
pip install platformio
```

#### 2. Clone Repository
```bash
cd c:\Users\glukhovskoy\Documents\PlatformIO\Projects
git clone https://github.com/GhVost/Stepper-controller.git
cd Stepper-controller
```

#### 3. Install Dependencies
```bash
pio lib install "teemuatlut/TMCStepper" "Adafruit GFX Library" "Adafruit ST7735 and ST7789 Library"
```

### Build & Test

```bash
# Build project
pio run

# Build and upload to Pico (use bootloader mode or drag .uf2)
pio run --target upload

# Monitor serial output at 115200 baud
pio device monitor --baud 115200

# Clean build artifacts
pio run --target clean
```

## Project Structure

```
Megasonic/
├── platformio.ini          # Project configuration
├── src/
│   └── main.cpp           # Main firmware (400+ lines)
├── include/               # Header files (for future modularization)
├── lib/                   # Local libraries (if needed)
├── README.md              # Project overview
├── HARDWARE.md            # Pin assignments & wiring
├── STATE_MACHINE.md       # State machine documentation
└── SETUP.md              # This file
```

## Code Organization

### src/main.cpp

#### Pin Definitions (Lines 3-26)
```cpp
// Motor (TMC2130)
const int TMC_CS = 17;      // SPI chip select
const int TMC_MOSI = 19;    // SPI data out
const int TMC_MISO = 16;    // SPI data in
const int TMC_SCK = 18;     // SPI clock
const int TMC_STEP = 14;    // Step pulse
const int TMC_DIR = 15;     // Direction
const int TMC_EN = 13;      // Enable (LOW = active)

// LCD Display
const int LCD_CS = 9;
const int LCD_DC = 10;
const int LCD_RST = 11;

// Encoder
const int ENC_A = 26;
const int ENC_B = 27;

// Sensors
const int LIMIT_SWITCH = 28;
const int SPRAY_VALVE = 2;
const int FLOW_SENSOR = 3;

// Outputs
const int LED_GREEN = 8;
const int LED_YELLOW = 7;
const int FAN_PWM = 12;
const int ULTRASONIC = 4;
```

#### Motion Parameters (Lines 41-49)
```cpp
const int STEPS_PER_REVOLUTION = 200;
const int PARK_POSITION = 7;
const int CENTER_POSITION = 26;
const int OSCILLATION_STEPS = 16;
const unsigned long OSCILLATION_DELAY = 10000;  // ms
const unsigned long OSCILLATION_CYCLES = 20;
```

#### State Machine (Lines 29-38)
```cpp
enum SystemState {
    STATE_IDLE,           // 0
    STATE_HOMING,         // 1
    STATE_PARKED,         // 2
    STATE_WAITING_SPRAY,  // 3
    STATE_SPRAY_ACTIVE,   // 4
    STATE_OSCILLATING,    // 5
    STATE_ERROR           // 6
};
```

### Function API

#### Motor Control

**`motorStep(int direction)`**
- **Purpose**: Move motor one step
- **Parameters**: 
  - `direction`: 1 (forward) or -1 (backward)
- **Returns**: void
- **Notes**: Updates internal position counter
```cpp
motorStep(1);   // One step forward
motorStep(-1);  // One step backward
```

**`motorSetEnable(bool enable)`**
- **Purpose**: Enable/disable motor torque
- **Parameters**: 
  - `enable`: true (motor ON) or false (motor OFF)
- **Returns**: void
- **Notes**: When disabled, motor coils de-energize
```cpp
motorSetEnable(true);   // Motor ready to move
motorSetEnable(false);  // Motor disabled
```

#### Sensor Reading

**`readSensors()`**
- **Purpose**: Poll all input sensors
- **Returns**: void
- **Updates**: Global state variables
  - `spray_valve` (bool)
  - `flow_sensor` (bool)
  - `limit_switch` (bool)
```cpp
readSensors();
if (spray_valve) {
    // Spray activated
}
```

**`readEncoder()`**
- **Purpose**: Read rotary encoder (TODO)
- **Current State**: Placeholder only
- **Future**: Return delta (-1, 0, +1)
```cpp
int delta = readEncoder();  // -1 left, 0 no change, +1 right
```

#### Output Control

**`setLED(int pin, bool state)`**
- **Purpose**: Control LED output
- **Parameters**: 
  - `pin`: GPIO pin (LED_GREEN or LED_YELLOW)
  - `state`: true (ON) or false (OFF)
- **Returns**: void
```cpp
setLED(LED_GREEN, true);    // Green LED on
setLED(LED_YELLOW, false);  // Yellow LED off
```

**`setFan(int speed)`**
- **Purpose**: Control fan PWM
- **Parameters**: 
  - `speed`: 0-255 (0% to 100%)
- **Returns**: void
```cpp
setFan(0);      // Fan off
setFan(128);    // Fan 50%
setFan(255);    // Fan full speed
```

**`setUltrasonic(bool active)`**
- **Purpose**: Control ultrasonic relay
- **Parameters**: 
  - `active`: true (ON) or false (OFF)
- **Returns**: void
- **Notes**: LOW = ON, HIGH = OFF (active low logic)
```cpp
setUltrasonic(true);   // Ultrasonic ON
setUltrasonic(false);  // Ultrasonic OFF
```

#### SPI Communication

**`tmcWriteReg(uint8_t addr, uint32_t datagram)`**
- **Purpose**: Write register to TMC2130 driver
- **Parameters**: 
  - `addr`: Register address (0x00-0x7F)
  - `datagram`: 32-bit value to write
- **Returns**: void
- **Protocol**: 
  - Assert CS LOW
  - Send address byte with write bit set
  - Send 4 data bytes (MSB first)
  - Release CS HIGH
- **Timing**: Respects TMC2130 setup/hold times

```cpp
// Example: Write GCONF register
uint32_t gconf_value = 0x000000A0;  // StealthChop enabled
tmcWriteReg(0x00, gconf_value);
```

**`tmcReadReg(uint8_t addr)`**
- **Purpose**: Read register from TMC2130 driver
- **Parameters**: 
  - `addr`: Register address (0x00-0x7F)
- **Returns**: uint32_t (32-bit register value)
- **Protocol**: 
  - Two-phase read (write address, then read data)
  - First call returns previous read
  - Handle timing per TMC2130 datasheet

```cpp
// Example: Read GSTAT register
uint32_t gstat = tmcReadReg(0x01);
```

#### State Machine

**`updateStateMachine()`**
- **Purpose**: Check state transitions
- **Returns**: void
- **Updates**: Global `current_state`
- **Called**: Every 50ms in main loop

```cpp
updateStateMachine();  // Check all transition conditions
```

**`handleState()`**
- **Purpose**: Execute per-state actions
- **Returns**: void
- **Examples**: 
  - Pulse motor in HOMING/OSCILLATING
  - Set LED colors per state
  - Manage fan speed
- **Called**: Every 50ms in main loop

```cpp
handleState();  // Do whatever the current state requires
```

#### Initialization

**`initHardware()`**
- **Purpose**: Setup GPIO pins
- **Returns**: void
- **Actions**: 
  - Configure all pins as INPUT or OUTPUT
  - Set motor disabled (EN pin HIGH)
  - All outputs OFF
- **Called**: Once in setup()

```cpp
initHardware();
```

**`initSPI()`**
- **Purpose**: Configure SPI0
- **Returns**: void
- **Settings**: 
  - Frequency: 1 MHz (TMC2130 compatible)
  - Format: 8 bits per word, mode 3
- **Called**: Once in setup()

```cpp
initSPI();
```

**`initEncoder()`**
- **Purpose**: Setup rotary encoder pins
- **Returns**: void
- **Current**: Placeholder (TODO: quadrature logic)
- **Called**: Once in setup()

```cpp
initEncoder();
```

**`initTMC2130()`**
- **Purpose**: Configure stepper driver via SPI (TODO)
- **Returns**: void
- **Future**: Set current limits, microsteps, chopper mode
- **Called**: Once in setup()

```cpp
initTMC2130();  // Currently a placeholder
```

#### Debug Output

**`updateDisplay()`**
- **Purpose**: Output system status
- **Current**: Serial to terminal (115200 baud)
- **Future**: Drive SPI LCD via Adafruit library
- **Output**: State, position, sensor status
- **Called**: Every 50ms

```cpp
updateDisplay();
// Output example:
// State: OSCILLATING | Pos: 15 | Spray: ON | Flow: YES
```

## Adding Features

### 1. Implement Rotary Encoder Reading

**File**: `src/main.cpp`, function `readEncoder()`

**Current**: Returns 0 (no change)

**TODO**: Implement quadrature decoding
```cpp
int readEncoder() {
    static int last_a = 0, last_b = 0;
    int curr_a = digitalRead(ENC_A);
    int curr_b = digitalRead(ENC_B);
    int delta = 0;

    // Quadrature logic: detect transitions
    if (curr_a != last_a) {
        if (curr_b != last_a) {
            delta = 1;  // Clockwise
        } else {
            delta = -1; // Counter-clockwise
        }
    }
    last_a = curr_a;
    last_b = curr_b;
    return delta;
}
```

### 2. Configure TMC2130 Driver

**File**: `src/main.cpp`, function `initTMC2130()`

**Current**: Empty

**TODO**: Initialize via SPI
```cpp
void initTMC2130() {
    // Example: Set microsteps to 16x
    // CHOPCONF register address 0x6C
    uint32_t chopconf = 0x00010115;  // MRES=4 (16x microsteps)
    tmcWriteReg(0x6C, chopconf);

    // Set currents: IHOLD_IRUN at 0x10
    uint32_t ihold_irun = 0x00080400;  // IHOLD=8, IRUN=4
    tmcWriteReg(0x10, ihold_irun);
}
```

### 3. Integrate SPI LCD

**File**: `src/main.cpp`, function `updateDisplay()`

**Current**: Serial output only

**TODO**: Use Adafruit_ST7789 library
```cpp
#include <Adafruit_ST7789.h>

// For the ST7789V3 1.69" (240x280) display use the Adafruit_ST7789 constructor
// and initialize with the correct width/height.
Adafruit_ST7789 tft = Adafruit_ST7789(LCD_CS, LCD_DC, LCD_RST);

void initDisplay() {
  // Initialize with 240x280 for the ST7789V3 module
  tft.init(240, 280);
  tft.fillScreen(ST7789_BLACK);
  // Optional: adjust rotation if the display appears rotated or clipped
  // tft.setRotation(1);
}

void updateDisplay() {
  tft.setCursor(0, 0);
  tft.setTextColor(ST7789_WHITE, ST7789_BLACK);
  tft.print("State: ");
  tft.println(state_names[current_state]);
  // ... more display updates
}
```

### 4. Add Error Recovery

**File**: `src/main.cpp`, function `handleState()` STATE_ERROR case

**Current**: Not implemented

**TODO**: Add recovery logic
```cpp
case STATE_ERROR:
    setLED(LED_YELLOW, true);
    motorSetEnable(false);
    setFan(0);
    setUltrasonic(false);
    
    // Reset after 10 seconds
    if (millis() - error_time > 10000) {
        current_state = STATE_IDLE;
    }
    break;
```

## Debugging Tips

### 1. Serial Monitor
```bash
pio device monitor --baud 115200

# Output shows:
# State transitions
# Sensor changes
# Position updates
# Any errors
```

### 2. Debug Output
Add debug prints in code:
```cpp
Serial.print("DEBUG: Motor position = ");
Serial.println(motor_position);
```

### 3. Use Multimeter
- Test GPIO HIGH/LOW with multimeter
- Measure SPI clock with oscilloscope (if available)
- Check 3.3V and 12V rails under load

### 4. Manual Motor Test
1. Power on Pico (USB)
2. Motor should be disabled (EN pin HIGH)
3. Manually rotate: move freely (power off)
4. Upload firmware
5. Serial monitor shows initialization
6. Check GPIO 13 (EN) goes LOW in HOMING state

## Building & Deployment

### Create Production .uf2 Firmware

```bash
pio run --target buildfs
```

This creates `.pio/build/pico/firmware.uf2` - the file to load onto Pico via bootloader.

### Flash via Drag-and-Drop
1. Hold BOOTSEL button on Pico
2. Plug USB into computer
3. Pico appears as USB drive
4. Drag `firmware.uf2` onto drive
5. Pico reboots automatically

### Flash via PlatformIO
```bash
pio run --target upload
```

Requires bootloader to be accessible.

## Configuration Management

### Adjusting Motion Parameters

Edit `src/main.cpp` constants (lines 41-49):

```cpp
// For longer cleaning cycles:
const unsigned long OSCILLATION_CYCLES = 30;  // Was 20

// For faster steps:
const unsigned long OSCILLATION_DELAY = 5000;  // Was 10000 (now 5s per step)

// For wider oscillation range:
const int OSCILLATION_STEPS = 24;  // Was 16

// For different motor speed during homing:
// (Would need to adjust STEP pulse timing in handleState)
```

## Performance Metrics

Current build (as of last compile):
- **Flash**: 4,254 bytes (0.2% of 2MB)
- **RAM**: ~41,528 bytes (15.4% of 264KB)
- **CPU**: ~5-10% per state (mostly sleeping)

### Memory Headroom
- Plenty of room for LCD driver code (~20KB)
- Encoder library addition: ~2KB
- TMC2130 register config: <1KB

## Testing Checklist

- [ ] Code compiles without errors
- [ ] Motor disabled at startup
- [ ] Serial output at 115200 baud
- [ ] LEDs respond to state transitions
- [ ] Spray valve triggers HOMING
- [ ] Limit switch detected in HOMING
- [ ] Motor oscillates smoothly
- [ ] Fan PWM working (multimeter check)
- [ ] Ultrasonic relay clicks (GPIO 4 LOW)

## References

- [PlatformIO Docs](https://docs.platformio.org/)
- [RP2040 Arduino Core](https://github.com/earlephilhower/arduino-pico)
- [Adafruit ST7789 Library](https://github.com/adafruit/Adafruit-ST7789-Library)
- [TMCStepper Library](https://github.com/teemuatlut/TMCStepper)
