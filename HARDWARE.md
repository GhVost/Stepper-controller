# Hardware Setup & Wiring Guide

## Overview

MEGASONIC runs on an RP2040 (Pico). The TMC2130 stepper driver and the ST7789 LCD are
on **two separate SPI buses** — the driver on **SPI1**, the display on **SPI0** — so
there is no bus sharing and no per-transaction SPI-mode switching between them. A single
firmware mutex serialises SPI access across the two cores.

### Power Requirements

| Component | Supply |
|-----------|--------|
| RP2040 Pico | 5 V USB or 3.3 V regulated |
| TMC2130 driver | 12–24 V motor supply (VM) + 3.3 V logic (VCC_IO) |
| LCD display | 3.3 V only |
| Sensors | 3.3 V |
| LEDs | 3.3 V with 470 Ω resistor |

---

## Pin Assignment Map

### SPI1 — TMC2130 stepper driver

```
SCK  = GPIO 10
MOSI = GPIO 11   (SDI: Pico → driver)
MISO = GPIO 12   (SDO: driver → Pico)
CS   = GPIO 13   (chip select, active LOW)
STEP = GPIO 14
DIR  = GPIO 15
EN   = GPIO 1    (LOW = motor active)
```

### SPI0 — ST7789 LCD

```
SCK  = GPIO 18   (SCL)
MOSI = GPIO 19   (SDA)   — display is write-only, no MISO
CS   = GPIO 9
DC   = GPIO 5
RES  = GPIO 6
BL   = GPIO 20   (backlight, HIGH = on)
```

The TMC2130 requires **SPI Mode 3** (CPOL=1, CPHA=1); the ST7789 uses Mode 0. Because
each device has its own bus, the firmware configures the mode per bus once — no runtime
switching.

---

## Wiring Diagrams

### TMC2130 Stepper Driver (BigTreeTech V1.1)

Board silkscreen labels (right side): EN, SDI, SCK, CSN, SDO, NC, STP, DIR
Board silkscreen labels (left side): VM, GND, 2B, 1B, 1A, 2A, VCC, GND

```
RP2040 Pico                BTT TMC2130 V1.1
─────────────────          ────────────────────────────────
GPIO 13 ────────── CSN     (Chip Select — active LOW)
GPIO 10 ────────── SCK     (SPI1 Clock)
GPIO 11 ────────── SDI     (SPI Data In: Pico MOSI → driver)
GPIO 12 ────────── SDO     (SPI Data Out: driver → Pico MISO)
GPIO 14 ────────── STP     (Step pulse)
GPIO 15 ────────── DIR     (Direction)
GPIO 1  ────────── EN      (Enable — LOW = active)
GND     ────────── GND
3V3     ────────── VCC     (Logic supply)
12–24 V ────────── VM      (Motor supply — separate power rail)
```

**Sense resistor (R_SENSE):** Most TMC2130 carrier boards (BigTreeTech, FYSETC) use
**0.11 Ω**. Check your board schematic. It is set in `main.cpp` as
`#define R_SENSE 0.11f` — update it if your board differs.

**Motor Connections:**
```
TMC2130 1A / 1B → NEMA17 coil 1 (A, A−)
TMC2130 2A / 2B → NEMA17 coil 2 (B, B−)
```

---

### LCD Display (GMT147SPI — 1.47" 172×320 ST7789)

Module: **GMT147SPI** 1.47" SPI display, ST7789 controller, 172×320 pixels.
Driven with `Adafruit_ST7789`, initialised as `tft.init(172, 320)` then
`tft.setRotation(1)` (landscape 320×172). Power from 3.3 V only.

```
RP2040 Pico         Display Module
───────────         ──────────────
GPIO 9  ────────── CS   (Chip Select)
GPIO 5  ────────── DC   (Data/Command)
GPIO 6  ────────── RES  (Reset)
GPIO 18 ────────── SCL  (SPI0 Clock)
GPIO 19 ────────── SDA  (SPI0 Data / MOSI)
GPIO 20 ────────── BL   (Backlight — HIGH = on)
3V3     ────────── VDD
GND     ────────── GND
```

> Do not connect the display VDD to 5 V — the module is 3.3 V only.
> BL can be tied directly to 3V3 for an always-on backlight (omit GPIO 20).

---

### Rotary Encoder (KY-040 type)

All encoder pins use internal pull-ups — no external resistors needed.

```
Encoder             RP2040 Pico
───────             ───────────
CLK (Phase A) ───── GPIO 26  (INPUT_PULLUP)
DT  (Phase B) ───── GPIO 27  (INPUT_PULLUP)
SW  (Button)  ───── GPIO 22  (INPUT_PULLUP — LOW when pressed)
+             ───── 3V3
GND           ───── GND
```

**Recommended: RC debounce.** Add a 100nF (0.1µF) ceramic capacitor from CLK (GPIO 26)
to GND and another from DT (GPIO 27) to GND, as close to the Pico's pins as practical.
Combined with the ~50-80kΩ internal pull-ups this gives a ~5-8ms time constant that
filters mechanical contact bounce, making rotation noticeably more reliable. The
firmware's polled quadrature decoder works without the capacitors, but is much
"snappier" with them.

**UI gestures:** rotate to navigate / edit, click to select. On the main menu a short
click immediately followed by a long press (≥ 1000 ms, within 400 ms of the release)
toggles the advanced menu (Setup / About).

---

### Sensors & Switches

```
Component           GPIO    Mode             Notes
──────────          ────    ────             ─────
Limit Switch ─────── 28     INPUT_PULLUP     Switch between GPIO 28 and GND.
                                             Reads LOW when pressed. 20 ms debounce.
Spray Valve  ─────── 2      INPUT_PULLDOWN   External 0 / 3.3 V signal.
                                             HIGH = spray active.
Flow Sensor  ─────── 3      INPUT_PULLDOWN   External 0 / 3.3 V signal.
                                             HIGH = liquid flowing.
```

> Spray/flow are only read when **Debug** is `OFF`. In the default Debug `ON` mode they
> are ignored and forced inactive in firmware.

### Outputs

```
Component           GPIO    Notes
──────────          ────    ─────
LED Green    ─────── 8      3V3 through 470 Ω resistor
LED Yellow   ─────── 7      3V3 through 470 Ω resistor
Fan PWM      ─────── 21     To fan driver / MOSFET gate (12 V fan)
Ultrasonic   ─────── 4      Active-low relay trigger. LOW = generator ON.
                            Energised only while the arm is over the wafer.
```

---

## TMC2130 Driver Configuration

Configured over SPI1 at startup in `initTMC2130()` using the **TMCStepper** library.
Settings persist to flash and can be changed on-device in the **Setup** menu (Current,
RunHold, PrkHold, Chop, Mstep, Invert).

| Setting | Default | Notes |
|---------|---------|-------|
| `rms_current` | 800 mA | Editable (`Current`). Run/park hold % below |
| `microsteps` | 256× | Editable (`Mstep`); motion is angle-based, so no recalibration needed |
| `en_pwm_mode` | true | Chopper mode, editable (`Chop`): `true` = StealthChop (quiet), `false` = SpreadCycle (more torque/high-speed) |
| `pwm_autoscale` | true | Auto-tune StealthChop amplitude |
| `intpol` | true | 256-microstep interpolation |
| `toff` | 5 | Enables the chopper — required |

Two hold-current modes reduce heat and holding torque when idle (both editable as a
percentage of run current in **Setup**):
- **Run hold** (`driverRunHoldPct`, default 25 %) — while moving / staging.
- **Park hold** (`driverParkHoldPct`, default 10 %) — once settled at the park angle.

The active hold mode (run/park) is reported over the serial log.

---

## Angle-Based Motion (no steps-per-mm)

There is **no `STEPS_PER_MM` calibration**. Motion is commanded in arm angle and
converted to steps using the live microstep setting and the motor→arm gear reduction:

```
steps = degrees × FULL_STEPS_PER_REV × microsteps × (gearOutTeeth / gearInTeeth) / 360
```

The gear ratio is set in **Setup** (`Gear in` / `Gear out`, default **15:108** = 7.2:1).
Changing microsteps or gearing therefore needs no recalibration — arm angles stay correct.
The sweep width is derived from the selected wafer diameter and the arm length
(`ARM_LENGTH_MM`, set in Setup).

---

## Testing Checklist

- [ ] **Power**: 12–24 V at VM, 3.3 V on VCC_IO
- [ ] **SPI1 (TMC)**: scope CS/SCK/MOSI on GPIO 13/10/11 during startup
- [ ] **SPI0 (LCD)**: display initialises (title visible)
- [ ] **TMC2130**: serial shows "TMC2130 configured…"; About screen shows live status
- [ ] **Motor**: spins during HOMING (EN GPIO 1 goes LOW)
- [ ] **Limit switch**: GPIO 28 drops to 0 V when pressed
- [ ] **Spray valve** (Debug = OFF): GPIO 2 reads 3.3 V when active
- [ ] **Flow sensor** (Debug = OFF): GPIO 3 reads 3.3 V when flowing
- [ ] **LEDs**: green/yellow follow the state table
- [ ] **Fan PWM**: duty varies with state on GPIO 21
- [ ] **Ultrasonic relay**: GPIO 4 drops to 0 V while the arm is over the wafer

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Motor jitters or stalls | Wrong current | Adjust Current in Setup or `driverCurrent` |
| Motor won't move | EN not LOW | Check GPIO 1; `motorSetEnable(true)` in active states |
| TMC SPI dead | Pin/bus mismatch | Verify SPI1 on GPIO 10/11/12/13 and 3.3 V VCC_IO |
| LCD blank | Wrong CS/DC/RES | Check GPIO 9/5/6; ensure BL (GPIO 20) is HIGH |
| Sensor always inactive | Debug = ON | Set Debug = OFF in Setup to read GPIO 2/3 |
| Sensor floating | No source | Inputs use pull-downs; drive with a real 0/3.3 V signal |
| ERROR after start | Driver fault / home timeout | Check About status + limit wiring; press START to retry |
| Motor runs hot | Current too high | Lower Current in Setup |

---

## References

- [RP2040 / Pico Datasheet](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf)
- [TMC2130 Datasheet](https://www.trinamic.com/fileadmin/assets/Documents/TMC2130_Datasheet_Rev1.35.pdf)
- [TMCStepper Library](https://github.com/teemuatlut/TMCStepper)
- [Adafruit ST7789 Library](https://github.com/adafruit/Adafruit-ST7789-Library)
</content>
