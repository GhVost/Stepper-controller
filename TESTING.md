# Testing & Validation Guide

## Pre-Hardware Testing

Before connecting to actual hardware, validate the firmware compiles and core logic is sound.

### Step 1: Build Verification

```bash
cd c:\Users\glukhovskoy\Documents\PlatformIO\Projects\Megasonic
pio run
```

**Expected Output**:
```
Processing pico (platform: raspberrypi; board: pico; framework: arduino)
...
Linking .pio/build/pico/firmware.elf
Checking size .pio/build/pico/firmware.elf
Memory Usage [=                    ] 0.2% (FLASH)
                [===                ] 15.4% (RAM)
```

**Success**: Exits with 0, no error messages

**Failure**: Check compiler errors, typically:
- Missing headers: `#include <...>`
- Undefined functions: Forward declarations needed
- Pin conflicts: Check duplicate GPIO assignments

### Step 2: Serial Monitor Startup

```bash
pio run --target upload    # Or drag .uf2 to Pico
pio device monitor --baud 115200
```

**Expected Output** (within 1-2 seconds):
```
=== Stepper Controller Initializing ===
Hardware initialized
SPI initialized: SCK=18 MOSI=19 MISO=16
Encoder initialized: A=26 B=27
TMC2130 driver: Configure via SPI - TODO
Initialization complete!

State: IDLE | Pos: 0 | Spray: OFF | Flow: NO
State: IDLE | Pos: 0 | Spray: OFF | Flow: NO
```

**Repeating output**: Good (polling every ~50ms)

**No output**: 
- Check USB cable and baud rate (115200)
- Verify Pico is in bootloader mode
- Try different COM port

## Hardware Integration Testing

Once firmware runs, connect hardware incrementally.

### Test Phase 1: Power & GPIOs

**Equipment**: Multimeter or scope

**Test Motor EN Pin**
```
GPIO 13 (TMC_EN) should be HIGH at startup (motor disabled)
In HOMING state: GPIO 13 should go LOW (motor enabled)
```

**Check with Multimeter**:
```bash
# Terminal 1: Monitor state
pio device monitor --baud 115200

# Terminal 2: Simulate spray on
# (Hold spray valve wire to 3.3V or connect sensor to GND)

# Observe: GPIO 13 should drop to ~0.1V when HOMING
```

**Step Control**
```
GPIO 14 (STEP) should pulse every 10ms during HOMING
GPIO 15 (DIR) should be LOW (toward limit)
```

Use oscilloscope to verify pulse pattern (optional, or just listen for motor clicks).

### Test Phase 2: Sensors

**Limit Switch (GPIO 28)**

Multimeter test:
```
Default (switch open): GPIO 28 reads 3.3V (pulled up)
Switch pressed: GPIO 28 reads 0V (connected to GND)
```

Serial test:
```bash
# Manually close/open limit switch
pio device monitor --baud 115200

# Watch serial output:
# State: HOMING | Pos: 0 | ...    (when limit pressed)
# State: PARKED | Pos: 7 | ...    (after moving to park)
```

**Spray Valve (GPIO 2)**

Multimeter test:
```
Spray OFF: GPIO 2 reads 0V
Spray ON: GPIO 2 reads 3.3V
```

Connect signal wire from spray valve controller to GPIO 2 (active HIGH).

**Flow Sensor (GPIO 3)**

Multimeter/logic test:
```
No flow: GPIO 3 reads 0V
Flowing: GPIO 3 reads 3.3V
```

Connect signal from flow sensor to GPIO 3.

### Test Phase 3: Outputs

**LEDs (GPIO 8 Green, GPIO 7 Yellow)**

```
Startup (IDLE): No LEDs on
HOMING: Green LED on
PARKED: Green LED on
SPRAY_ACTIVE/OSCILLATING: Green LED on
```

Check with multimeter or visual inspection (LEDs should illuminate).

**Fan PWM (GPIO 12)**

Scope/PWM meter:
```
IDLE: 0% (0V)
WAITING_SPRAY: 50% (2.5V average)
SPRAY_ACTIVE/OSCILLATING: 100% (3.3V)
```

Or use oscilloscope to see PWM frequency (~1kHz typical).

**Ultrasonic Relay (GPIO 4)**

```
Default (OFF): GPIO 4 reads 3.3V (high, relay inactive)
SPRAY_ACTIVE: GPIO 4 reads 0V (low, relay activated)
OSCILLATING: GPIO 4 reads 0V
```

Connect relay trigger circuit to GPIO 4.

### Test Phase 4: SPI Communication (TMC2130)

**Hardware Setup**:
- TMC2130 connected via SPI0 (CS=17, SCK=18, MOSI=19, MISO=16)
- Power: 12-24V for motor coils, 3.3V for logic

**Verify SPI Pins with Multimeter**:
```
SCK (GPIO 18): Should pulse at ~1 MHz during HOMING
MOSI (GPIO 19): Should toggle with data
MISO (GPIO 16): Should return data from driver
CS (GPIO 17): Should pulse LOW when accessing driver
```

**Oscilloscope Capture** (optional):
- SCK should show clean 1 MHz square wave
- MOSI/MISO should show SPI data pattern
- CS should be active (LOW) during transfers

**Serial Debug** (future enhancement):
Add debug print to `tmcWriteReg()` to log SPI operations.

### Test Phase 5: State Machine Transitions

**Simulate Full Cycle** (without actual motor):

Terminal 1: Monitor state
```bash
pio device monitor --baud 115200
```

Terminal 2: Trigger events by connecting wires
```
1. Connect GPIO 2 (Spray) to 3.3V
   → Watch: STATE_IDLE → STATE_HOMING

2. Connect GPIO 28 (Limit) to GND momentarily
   → Watch: STATE_HOMING → STATE_PARKED

3. Connect GPIO 3 (Flow) to 3.3V
   → Watch: STATE_PARKED → STATE_WAITING_SPRAY → STATE_SPRAY_ACTIVE

4. After 2 seconds:
   → Watch: STATE_SPRAY_ACTIVE → STATE_OSCILLATING

5. Disconnect GPIO 2 (Spray OFF)
   → Watch: STATE_OSCILLATING → STATE_IDLE
```

**Expected Full Log**:
```
State: IDLE | Pos: 0 | Spray: OFF | Flow: NO
[Trigger spray on]
State: HOMING | Pos: 0 | Spray: ON | Flow: NO
State: HOMING | Pos: 1 | Spray: ON | Flow: NO
State: HOMING | Pos: 2 | Spray: ON | Flow: NO
[Trigger limit switch]
State: PARKED | Pos: 7 | Spray: ON | Flow: NO
[Trigger flow on]
State: WAITING_SPRAY | Pos: 7 | Spray: ON | Flow: YES
State: SPRAY_ACTIVE | Pos: 7 | Spray: ON | Flow: YES
[Wait 2 seconds]
State: OSCILLATING | Pos: 8 | Spray: ON | Flow: YES
State: OSCILLATING | Pos: 9 | Spray: ON | Flow: YES
State: OSCILLATING | Pos: 10 | Spray: ON | Flow: YES
...
[After 20 cycles or spray off]
State: IDLE | Pos: 0 | Spray: OFF | Flow: NO
```

## Motor Mechanical Testing

### Test Phase 6: Stepper Motor Behavior

**Before Power**:
1. Motor should spin freely (motor disabled, no torque)
2. No unusual grinding sounds

**After Power-Up** (firmware running):

**HOMING State**:
- Motor pulls hard toward limit switch
- Should hear clicking (stepper pulses every ~10ms)
- Motor accelerates as it approaches limit

**At Limit Switch**:
- Motor stops (position = 0)
- LED indicator changes to PARKED

**During OSCILLATION**:
- Motor moves back and forth at 10-second intervals
- Smooth, consistent motion
- No stalls or noise changes

**Speed Adjustments** (if needed):
```cpp
// In src/main.cpp, modify OSCILLATION_DELAY:
const unsigned long OSCILLATION_DELAY = 10000;  // Try 5000 for faster

// Or modify step pulse width in handleState():
// Currently: digitalWrite(TMC_STEP, LOW/HIGH) with 10ms delay
```

## Integration Checklist

| Component | Test | Status |
|-----------|------|--------|
| **Firmware** | Builds cleanly | ☐ |
| **Serial** | Output at 115200 | ☐ |
| **Motor EN** | Pulses LOW in HOMING | ☐ |
| **Motor STEP** | Pulses during motion | ☐ |
| **Motor DIR** | Set correctly | ☐ |
| **Limit Switch** | Stops motor | ☐ |
| **Spray Valve** | Triggers HOMING | ☐ |
| **Flow Sensor** | Enables oscillation | ☐ |
| **Green LED** | Lights in HOMING/PARKED | ☐ |
| **Yellow LED** | Lights in WAITING_SPRAY | ☐ |
| **Fan PWM** | Varies with state | ☐ |
| **Ultrasonic** | LOW during spray | ☐ |
| **State Machine** | All transitions work | ☐ |
| **SPI Clock** | Runs at ~1 MHz | ☐ |

## Troubleshooting by Symptom

### Symptom: Motor won't move

**Check**:
1. GPIO 13 (EN) is LOW during HOMING
2. GPIO 14 (STEP) is pulsing
3. GPIO 15 (DIR) is set
4. Motor power connected
5. Motor coil connections correct (A, A~, B, B~)

**Fix**:
```cpp
// Verify motor is enabled in handleState():
if (current_state == STATE_HOMING) {
    motorSetEnable(true);  // EN pin LOW
    // ... pulsing logic
}
```

### Symptom: Stuck in HOMING state

**Check**:
1. Limit switch wired to GPIO 28
2. Limit switch shows 0V when pressed (multimeter)
3. Motor is moving toward limit

**Fix**:
```cpp
// In readSensors(), verify:
limit_switch = (digitalRead(LIMIT_SWITCH) == LOW);
```

### Symptom: LCD doesn't show anything

**Check**:
1. CS, DC, RST pins correct (9, 10, 11)
2. SPI SCK/MOSI running (scope)
3. LCD powered at 3.3V
4. Initialization code in setup() calls:
   - `initSPI()`
   - `updateDisplay()` in loop

**Current**: updateDisplay() only outputs to Serial
**Future**: Implement Adafruit_ST7789 integration

### Symptom: Fan always on or always off

**Check**:
1. Fan PWM connected to GPIO 12
2. State machine sets fan speed:
   ```cpp
   case STATE_WAITING_SPRAY:
       setFan(128);  // 50%
       break;
   ```

**Fix**:
```cpp
// Verify PWM is analog write, not digital:
analogWrite(FAN_PWM, speed);  // Not digitalWrite
```

### Symptom: Encoder not responding

**Current**: Encoder input implemented as TODO placeholder
- `readEncoder()` returns 0 always

**Future**: Implement quadrature decoding
- Read GPIO 26 (A) and GPIO 27 (B)
- Detect transitions to determine rotation direction
- Update position or menu selection

## Performance Validation

### Memory Usage

```bash
pio run
# Output shows:
# FLASH: X% of 2MB (should be < 50% for room)
# RAM:   X% of 264KB (should be < 50% for room)
```

**Current**: 0.2% FLASH, 15.4% RAM → Good headroom

### CPU Load

Monitor for stalls or lag:
- Serial output should update every 50ms constantly
- No random delays or stuttering
- Responsive to sensor changes

### Timing Accuracy

Compare oscillation with stopwatch:
```
Expected: 10 seconds per step
Actual: Measure with stopwatch during OSCILLATING state
```

Should be within ±10% (9-11 seconds per step).

## Final Sign-Off

After all tests pass:

1. ✅ Firmware compiles, no warnings
2. ✅ All GPIO states correct
3. ✅ All sensors triggering properly
4. ✅ Motor motion smooth and controlled
5. ✅ State machine transitions work
6. ✅ LEDs indicate state correctly
7. ✅ PWM outputs have correct duty cycles
8. ✅ Serial output shows real-time updates
9. ✅ No runtime errors or crashes

**Next Steps**:
- Implement remaining TODO features (TMC2130 config, encoder, LCD)
- Calibrate motion parameters for specific application
- Create production firmware release
- Document any hardware modifications or customizations

## Appendix: Test Code Snippets

### Quick GPIO Test
```cpp
// Add to setup() to test GPIO toggling
void testGPIO() {
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    for (int i = 0; i < 5; i++) {
        digitalWrite(LED_GREEN, HIGH);
        delay(500);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_YELLOW, HIGH);
        delay(500);
        digitalWrite(LED_YELLOW, LOW);
    }
}
```

### Quick SPI Test
```cpp
// Test SPI communication (basic)
void testSPI() {
    SPI.begin();
    digitalWrite(TMC_CS, LOW);
    uint8_t response = SPI.transfer(0x01);  // Read GSTAT
    digitalWrite(TMC_CS, HIGH);
    Serial.print("SPI Response: 0x");
    Serial.println(response, HEX);
}
```

### Quick PWM Test
```cpp
// Test PWM on fan pin
void testPWM() {
    for (int duty = 0; duty <= 255; duty += 25) {
        analogWrite(FAN_PWM, duty);
        Serial.print("PWM: ");
        Serial.println(duty);
        delay(1000);
    }
}
```
