# Hardware Setup & Wiring Guide

## Overview

The Stepper Controller uses two SPI buses and numerous GPIO pins on the RP2040 Pico.

### Power Requirements
- **RP2040 Pico**: 5V USB (or 3.3V regulated supply)
- **TMC2130 Driver**: 12-24V stepper motor supply + 3.3V logic
- **LCD Display**: 3.3V
- **Sensors**: 3.3V
- **LEDs**: 5V (with 220Ω resistor) or 3.3V (with 470Ω)

## Pin Assignment Map

### SPI Bus 0 (Default) - TMC2130 & Potential Expansion
```
SCK  = GPIO 18
MOSI = GPIO 19  (DI on TMC2130)
MISO = GPIO 16  (DO on TMC2130)
CS   = GPIO 17  (TMC2130)
```

### SPI Bus 1 (Optional) - LCD Display
```
SCK  = GPIO 10
MOSI = GPIO 11
MISO = GPIO 12
CS   = GPIO 9   (LCD)
DC   = GPIO 8   (Data/Command)
RST  = GPIO 7   (Reset)
```

*Note: Current code uses SPI0 for LCD. SPI1 pins listed for future expansion.*

## Wiring Diagram

### TMC2130 Stepper Driver

```
RP2040 Pico                TMC2130
─────────────────          ────────
GPIO 17 ────────── CS     (Chip Select)
GPIO 18 ────────── SCK    (SPI Clock)
GPIO 19 ────────── DI     (SPI Data In)
GPIO 16 ────────── DO     (SPI Data Out)
GPIO 14 ────────── STEP   (Step Pulse)
GPIO 15 ────────── DIR    (Direction)
GPIO 13 ────────── EN     (Enable, LOW=active)
GND     ────────── GND    
3V3     ────────── VCC_IO (Logic supply)
```

**Motor Connections:**
```
TMC2130 A  → NEMA17 A (coil 1+)
TMC2130 A~ → NEMA17 A~ (coil 1-)
TMC2130 B  → NEMA17 B (coil 2+)
TMC2130 B~ → NEMA17 B~ (coil 2-)
```

### LCD Display (ST7789V3 — 1.69" 240x280)
// This project targets a 1.69" ST7789V3 SPI display (240x280, rounded corners). Use the Adafruit_ST7789 library and initialize with the 240x280 dimensions. Ensure the module is powered at 3.3V.

```
RP2040 Pico         Display Module
─────────────       ───────────────
GPIO 9  ────────── CS   (Chip Select)
GPIO 10 ────────── DC   (Data/Command)
GPIO 11 ────────── RST  (Reset)
GPIO 18 ────────── SCK  (SPI Clock, shared with TMC2130)
GPIO 19 ────────── SDA  (SPI Data, shared with TMC2130)
3V3     ────────── VCC (Required)
GND     ────────── GND

Notes:
- Use 3.3V logic only. Do not connect display VCC to 5V.
- Example software init (see SETUP.md): `tft.init(240, 280);` and adjust rotation/offsets as needed.
```

### Rotary Encoder

```
Encoder             RP2040 Pico
───────             ───────────
A (Phase 1) ─────── GPIO 26
B (Phase 2) ─────── GPIO 27
GND         ─────── GND
VCC (opt)   ─────── 3V3
```

### Sensors & Switches

```
Component           RP2040 Pico         Notes
──────────          ───────────         ─────
Limit Switch ────── GPIO 28 ────────── Connect to GND when pressed
                    Pull-up enabled

Spray Valve ─────── GPIO 2  ────────── Input: HIGH when spraying
Flow Sensor ─────── GPIO 3  ────────── Input: HIGH when flowing
```

### Outputs

```
Component           RP2040 Pico         Supply
──────────          ───────────         ──────
LED Green  ─────── GPIO 8  ────────── 3V3 + 470Ω resistor
LED Yellow ─────── GPIO 7  ────────── 3V3 + 470Ω resistor
Fan PWM    ─────── GPIO 12 ────────── 12V motor driver input
Ultrasonic ─────── GPIO 4  ────────── 5V relay/trigger (active LOW)
```

## Physical Layout

```
┌─────────────────────────────────────┐
│    Raspberry Pi Pico RP2040         │
├─────────────────────────────────────┤
│  USB                                │
│  ┌─────────────────────────────────┐│
│  │ GPIO  3.3V GND VBUS VSYS        ││
│  │  0     1   2   3    4           ││
│  │  5     6   7  (EN)  8 (LCD_RST)││
│  │  9     10  11  12   13(TMC_EN) ││
│  │ 14(S) 15(D) 16(MISO)17(TMC_CS)││
│  │ 18(S) 19(M) 20  21  22         ││
│  │ 23    24   25  26(ENC_A)       ││
│  │ 27(EB) 28(LIM) GND            ││
│  └─────────────────────────────────┘│
└─────────────────────────────────────┘
Legend: (S)=SCK, (D)=DIR, (M)=MOSI, (B)=B, (LIM)=Limit
```

## Connector Types

### Recommended

- **TMC2130 Stepper Driver**: 
  - Connector: 1.27mm pitch 2×4 header or 0.1" equivalent
  - Or direct solder for reliability

- **LCD Display Module**: 
  - Connector: 0.1" (2.54mm) header
  - Or SPI breakout board with 4-pin connector

- **Motor Connector**: 
  - Use screw terminal block (4-pin, 5mm pitch)
  - Label A, A~, B, B~

- **Sensor Connectors**: 
  - 3-pin JST or 0.1" headers
  - Limit switch: NO (normally open)
  - Spray valve: TTL input (HIGH when active)
  - Flow sensor: TTL input (HIGH when flowing)

## Software SPI Configuration

### SPI0 (TMC2130)
```cpp
// Defined in main.cpp
const int TMC_CS   = 17;    // Chip Select
const int TMC_MOSI = 19;    // SPI MOSI (DI)
const int TMC_MISO = 16;    // SPI MISO (DO)
const int TMC_SCK  = 18;    // SPI Clock
const int TMC_STEP = 14;    // Step pulse
const int TMC_DIR  = 15;    // Direction
const int TMC_EN   = 13;    // Enable
```

### SPI0 (LCD) - Shared Pins
```cpp
const int LCD_CS  = 9;      // Chip Select (separate from TMC2130)
const int LCD_DC  = 10;     // Data/Command
const int LCD_RST = 11;     // Reset
// SCK, MOSI shared with TMC2130 on SPI0
```

## TMC2130 Configuration (TODO)

### Key Registers
- **GCONF** (0x00): General configuration
  - `en_pwm_mode`: Enable StealthChop
  - `spreadCycle`: Enable spreadCycle chopper
  
- **IHOLD_IRUN** (0x10): Current settings
  - `IHOLD`: Hold current
  - `IRUN`: Running current
  
- **CHOPCONF** (0x6C): Chopper configuration
  - `MRES`: Microsteps (0=256, ..., 8=1)
  - `TPWMTHRS`: Threshold for mode switching

### Example Initialization (Future)
```cpp
// Configure 16x microsteps, 800mA holding current
TMC2130Stepper driver(...);
driver.microsteps(16);
driver.rms_current(800);
driver.enable();
```

## Testing Checklist

- [ ] **Power**: 12V at motor, 3.3V on logic side
- [ ] **SPI**: Verify CS, SCK, MOSI, MISO with multimeter/scope
- [ ] **Motor**: Manual spin (motor enabled, no power)
- [ ] **Sensors**: 
  - [ ] Limit switch triggers correctly
  - [ ] Spray valve reads HIGH when activated
  - [ ] Flow sensor reads HIGH when flowing
- [ ] **LEDs**: Green and Yellow illuminate correctly
- [ ] **Serial**: 115200 baud output from Pico

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| Motor jitters | Micro-vibrations in STEP pulses | Increase STEP timing or reduce acceleration |
| SPI not working | Incorrect pin assignment | Verify pins match SPI0 defaults |
| LCD blank | Wrong CS or data pins | Check GPIO 9, 10, 11 |
| Sensor always HIGH | Wrong logic (inverted) | Adjust readSensors() logic |
| Encoder not reading | Missing pull-ups | Add internal pull-ups via pinMode(ENC_A, INPUT_PULLUP) |

## References

- [RP2040 Pinout](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf)
- [TMC2130 Pinout](https://www.trinamic.com/fileadmin/assets/Documents/TMC2130_Datasheet_Rev1.35.pdf)
- [ST7735/ST7789 LCD Datasheets](https://github.com/adafruit/Adafruit-ST7735-Library)
