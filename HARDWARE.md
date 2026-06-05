# Hardware Setup & Wiring Guide

## Overview

The Stepper Controller uses one shared SPI bus (SPI0) and numerous GPIO pins on the RP2040 Pico. The TMC2130 stepper driver and the ST7789 LCD share the same SPI0 clock and data lines; they are distinguished by separate Chip Select pins.

### Power Requirements

| Component | Supply |
|-----------|--------|
| RP2040 Pico | 5 V USB or 3.3 V regulated |
| TMC2130 driver | 12–24 V motor supply + 3.3 V logic (VCC_IO) |
| LCD display | 3.3 V |
| Sensors | 3.3 V |
| LEDs | 3.3 V with 470 Ω resistor |

---

## Pin Assignment Map

### SPI0 — Shared by TMC2130 and LCD

```
SCK  = GPIO 18   (shared)
MOSI = GPIO 19   (shared)
MISO = GPIO 16   (TMC2130 only — LCD is write-only)
CS   = GPIO 17   (TMC2130 chip select)
CS   = GPIO 9    (LCD chip select)
```

Each device is selected exclusively via its own CS pin. The firmware uses
`SPI.beginTransaction()` (handled internally by the TMCStepper and Adafruit
libraries) to switch SPI mode between devices:

- TMC2130 requires **SPI Mode 3** (CPOL=1, CPHA=1)
- ST7789 requires **SPI Mode 0** (CPOL=0, CPHA=0)

---

## Wiring Diagrams

### TMC2130 Stepper Driver (BigTreeTech V1.1)

Board silkscreen labels (right side): EN, SDI, SCK, CSN, SDO, NC, STP, DIR  
Board silkscreen labels (left side): VM, GND, 2B, 1B, 1A, 2A, VCC, GND

```
RP2040 Pico                BTT TMC2130 V1.1
─────────────────          ────────────────────────────────
GPIO 17 ────────── CSN     (Chip Select — active LOW)
GPIO 18 ────────── SCK     (SPI Clock)
GPIO 19 ────────── SDI     (SPI Data In: Pico MOSI → driver)
GPIO 16 ────────── SDO     (SPI Data Out: driver → Pico MISO)
GPIO 14 ────────── STP     (Step pulse)
GPIO 15 ────────── DIR     (Direction)
GPIO 13 ────────── EN      (Enable — LOW = active)
GND     ────────── GND
3V3     ────────── VCC     (Logic supply)
12–24 V ────────── VM      (Motor supply — separate power rail)
```

**Sense resistor (R_SENSE):** Most TMC2130 carrier boards (BigTreeTech, FYSETC)
use **0.11 Ω**. Check your board schematic. The value is set in `main.cpp` as
`#define R_SENSE 0.11f` — update it if your board differs.

**Motor Connections:**
```
TMC2130 A  → NEMA17 Coil 1+
TMC2130 A~ → NEMA17 Coil 1−
TMC2130 B  → NEMA17 Coil 2+
TMC2130 B~ → NEMA17 Coil 2−
```

---

### LCD Display (GMT147SPI — 1.47" 172×320 ST7789)

Module: **GMT147SPI** 1.47" SPI display, ST7789 controller, 172×320 pixels.
Use the `Adafruit_ST7789` library and initialise with `tft.init(172, 320)`.
Power the module from 3.3 V only.

Board pin labels and their logical meaning:

| Board Label | Function          |
|-------------|-------------------|
| SCL         | SPI Clock (SCK)   |
| SDA         | SPI Data (MOSI)   |
| RES         | Reset             |
| DC          | Data/Command      |
| CS          | Chip Select       |
| BL          | Backlight enable  |
| VDD         | 3.3 V supply      |
| GND         | Ground            |

```
RP2040 Pico         Display Module
───────────         ──────────────
GPIO 9  ────────── CS   (Chip Select)
GPIO 10 ────────── DC   (Data/Command)
GPIO 11 ────────── RES  (Reset)
GPIO 18 ────────── SCL  (SPI Clock — shared with TMC2130)
GPIO 19 ────────── SDA  (SPI Data  — shared with TMC2130)
GPIO 20 ────────── BL   (Backlight — HIGH = on)
3V3     ────────── VDD
GND     ────────── GND
```

> Do not connect the display VDD to 5 V — the module is 3.3 V only.
> BL can also be tied directly to 3V3 for always-on backlight (omit GPIO 20 connection).

---

### Rotary Encoder (KY-040 type)

All encoder pins use internal pull-ups — no external resistors needed.

| Board Label | Function            |
|-------------|---------------------|
| CLK         | Quadrature A (ENC_A)|
| DT          | Quadrature B (ENC_B)|
| SW          | Push-button (ENC_SW)|
| +           | 3.3 V supply        |
| GND         | Ground              |

```
Encoder             RP2040 Pico
───────             ───────────
CLK (Phase A) ───── GPIO 26  (INPUT_PULLUP)
DT  (Phase B) ───── GPIO 27  (INPUT_PULLUP)
SW  (Button)  ───── GPIO 22  (INPUT_PULLUP — LOW when pressed)
+             ───── 3V3
GND           ───── GND
```

---

### Sensors & Switches

```
Component           GPIO    Mode            Notes
──────────          ────    ────            ─────
Limit Switch ─────── 28     INPUT_PULLUP    Connect switch between GPIO 28 and GND.
                                            Reads LOW when pressed.
Spray Valve  ─────── 2      INPUT           External 0 / 3.3 V signal.
                                            HIGH = spray is active.
Flow Sensor  ─────── 3      INPUT           External 0 / 3.3 V signal.
                                            HIGH = liquid is flowing.
```

### Outputs

```
Component           GPIO    Notes
──────────          ────    ─────
LED Green    ─────── 8      3V3 through 470 Ω resistor
LED Yellow   ─────── 7      3V3 through 470 Ω resistor
Fan PWM      ─────── 12     Connect to fan driver/MOSFET gate (12 V fan)
Ultrasonic   ─────── 4      Active-low relay trigger. LOW = ultrasonic ON.
```

---

## TMC2130 Driver Configuration

The driver is configured via SPI at startup using the **TMCStepper** library.
Key settings (adjustable in `initTMC2130()` in `src/main.cpp`):

| Setting | Value | Notes |
|---------|-------|-------|
| `rms_current` | 600 mA | Reduce if driver or motor runs hot |
| `microsteps` | 16× | Update `STEPS_PER_MM` in `main.cpp` to match |
| `en_pwm_mode` | true | StealthChop — silent at low speed |
| `pwm_autoscale` | true | Auto-tune StealthChop amplitude |
| `toff` | 5 | Enables the chopper — required |

> **After changing `microsteps`:** recalculate and update `STEPS_PER_MM` in
> `main.cpp`. Formula: `(motor_steps_per_rev × microsteps) ÷ lead_screw_pitch_mm`.

---

## Steps-per-mm Calibration

`STEPS_PER_MM` defaults to `1` (uncalibrated). To calibrate:

1. Command the motor to move a known number of steps.
2. Measure actual travel with a ruler.
3. Calculate: `STEPS_PER_MM = steps_commanded / mm_travelled`.
4. Update `const int STEPS_PER_MM = <value>;` in `src/main.cpp`.

Example (200 step/rev motor, 2 mm lead screw, 16× microsteps):
```
STEPS_PER_MM = (200 × 16) / 2 = 1600
```

---

## Testing Checklist

- [ ] **Power**: 12–24 V at VM, 3.3 V on VCC_IO
- [ ] **SPI**: Scope or multimeter on CS, SCK, MOSI during startup
- [ ] **TMC2130**: Serial output shows "TMC2130 configured"
- [ ] **Motor**: Spins during HOMING state (EN goes LOW)
- [ ] **Limit switch**: GPIO 28 drops to 0 V when pressed
- [ ] **Spray valve**: GPIO 2 reads 3.3 V when spray is active
- [ ] **Flow sensor**: GPIO 3 reads 3.3 V when liquid is flowing
- [ ] **LEDs**: Green and Yellow illuminate per state table
- [ ] **Fan PWM**: Duty cycle varies with state (multimeter)
- [ ] **Ultrasonic relay**: GPIO 4 drops to 0 V in SPRAY_ACTIVE/OSCILLATING

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Motor jitters or stalls | Wrong `rms_current` | Reduce or increase in `initTMC2130()` |
| Motor won't move | EN pin not going LOW | Check `motorSetEnable(true)` is called |
| SPI not working | Pin mismatch | Verify GPIO 16/18/19 match SPI0 defaults |
| LCD blank | Wrong CS/DC/RST | Check GPIO 9, 10, 11; also verify BL (GPIO 20) is HIGH |
| Sensor always HIGH | Floating input | Verify external pull-down or signal source |
| Encoder not responding | No pull-up | Already fixed: firmware uses `INPUT_PULLUP` |
| Motor runs hot | Current too high | Lower `rms_current` in `initTMC2130()` |

---

## References

- [RP2040 / Pico Datasheet](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf)
- [TMC2130 Datasheet](https://www.trinamic.com/fileadmin/assets/Documents/TMC2130_Datasheet_Rev1.35.pdf)
- [TMCStepper Library](https://github.com/teemuatlut/TMCStepper)
- [Adafruit ST7789 Library](https://github.com/adafruit/Adafruit-ST7789-Library)
