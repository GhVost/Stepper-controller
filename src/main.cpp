#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// ============= PIN DEFINITIONS =============
// TMC2130 stepper driver (SPI0)
const int TMC_CS   = 17;
const int TMC_MOSI = 19;
const int TMC_MISO = 16;
const int TMC_SCK  = 18;
const int TMC_STEP = 14;
const int TMC_DIR  = 15;
const int TMC_EN   = 13;

// SPI LCD — CS/DC/RST/BL separate; SCK/MOSI shared with TMC2130 on SPI0
// Module: GMT147SPI 1.47" 172x320 ST7789
// Board labels: SCL=SCK, SDA=MOSI, RES=RST, BL=backlight
const int LCD_CS  = 9;
const int LCD_DC  = 10;
const int LCD_RST = 11;
const int LCD_BL  = 20;  // Backlight — HIGH = on. Tie to 3V3 if not needed.

// Rotary encoder (KY-040 type): CLK=ENC_A, DT=ENC_B, SW=ENC_SW
const int ENC_A  = 26;
const int ENC_B  = 27;
const int ENC_SW = 22;  // Push-button (INPUT_PULLUP, LOW when pressed)

// Sensors & outputs
const int LIMIT_SWITCH = 28;   // LOW when pressed (INPUT_PULLUP)
const int SPRAY_VALVE  = 2;    // HIGH when spray is active
const int FLOW_SENSOR  = 3;    // HIGH when liquid is flowing
const int LED_GREEN    = 8;
const int LED_YELLOW   = 7;
const int FAN_PWM      = 12;
const int ULTRASONIC   = 4;    // Active-low relay: LOW = ultrasonic ON

// ============= TMC2130 DRIVER =============
// R_SENSE: sense resistor in ohms. 0.11 Ω for most TMC2130 carrier boards
// (BigTreeTech, FYSETC). Check your board schematic if motor runs hot or weak.
#define R_SENSE 0.11f
TMC2130Stepper driver(TMC_CS, R_SENSE);

// ============= STATE MACHINE =============
enum SystemState {
    STATE_IDLE,
    STATE_HOMING,
    STATE_PARKED,
    STATE_WAITING_SPRAY,
    STATE_SPRAY_ACTIVE,
    STATE_OSCILLATING,
    STATE_ERROR
};

volatile SystemState currentState = STATE_IDLE;
unsigned long lastStateChange = 0;

// ============= MOTION PARAMETERS =============
// STEPS_PER_MM: calibrate to your hardware.
// Formula: (motor steps/rev × microsteps) ÷ lead-screw pitch (mm/rev)
// Example: 200 steps/rev, 16× microsteps, 2 mm pitch → (200 × 16) / 2 = 1600
// Default 1 preserves original uncalibrated behaviour; change once mechanically verified.
const int STEPS_PER_MM      = 1;
const int PARK_MM           = 7;    // Park position: mm from limit switch
const int CENTER_MM         = 26;   // Oscillation centre: mm from limit switch
const int PARK_STEPS        = PARK_MM  * STEPS_PER_MM;
const int CENTER_STEPS      = CENTER_MM * STEPS_PER_MM;
const int OSCILLATION_STEPS = 16;   // Steps per directional sweep
const unsigned long OSCILLATION_DELAY  = 10000; // ms between oscillation steps
const unsigned long OSCILLATION_CYCLES = 20;    // Sweeps before stopping
                                                 // Total: 20 × 16 × 10 s ≈ 53 min
const unsigned long SPRAY_ACTIVE_WAIT  = 2000;  // ms to stabilise before oscillating

// Fan PWM levels
const int FAN_OFF     = 0;
const int FAN_WAITING = 128;  // 50 % while priming
const int FAN_FULL    = 255;  // 100 % during cleaning

// ============= STATUS VARIABLES =============
volatile int  motorPosition      = 0;
volatile bool limitSwitchPressed = false;
volatile bool sprayActive        = false;
volatile bool flowDetected       = false;

unsigned long oscillationCount    = 0;
int           oscillationDir      = -1;  // -1 = toward limit, +1 = away
int           oscillationStepCount = 0;  // Steps taken in current sweep

// ============= TIMING =============
unsigned long lastMotorUpdate = 0;
unsigned long lastEncoderRead = 0;
const unsigned long MOTOR_UPDATE_INTERVAL  = 50;
const unsigned long ENCODER_READ_INTERVAL  = 20;

// ============= DISPLAY / MENU =============
// Hardware SPI on earlephilhower: fast, low-overhead transactions.
Adafruit_ST7789 tft = Adafruit_ST7789(LCD_CS, LCD_DC, LCD_RST);

const char* menuItems[] = { "START", "HOME", "SETTINGS", "ABOUT" };
const int   MENU_COUNT  = sizeof(menuItems) / sizeof(menuItems[0]);
volatile int menuIndex = 0;

// Set to true by Core 0 once hardware is fully initialised.
// Core 1 waits on this before touching the display.
volatile bool core0_ready = false;

// ============= FUNCTION DECLARATIONS =============
void initHardware();
void initSPI();
void initEncoder();
void initTMC2130();
void initDisplay();
void readSensors();
void readEncoder();
void updateStateMachine();
void handleState();
void motorStep(int direction);
void motorSetEnable(bool enable);
void motorMoveTo(int target);
void updateDisplay();
void drawMenuRow(int i, int y, bool selected);
void drawMenu();
void setLED(int ledPin, bool state);
void setFan(int speed);
void setUltrasonic(bool on);

// ============= SETUP =============
void setup() {
    Serial.begin(115200);
    // Wait up to 2 s for USB-CDC to enumerate so early prints are not lost.
    { unsigned long t = millis(); while (!Serial && millis() - t < 2000) delay(10); }
    Serial.println("=== Stepper Controller Initializing ===");

    initHardware();
    initSPI();
    // initTMC2130();  // Uncomment once the TMC2130 is wired — leaving it out
                       // prevents SPI bus corruption when the chip is absent.
    initEncoder();
    initDisplay();

    currentState    = STATE_IDLE;
    lastStateChange = millis();
    Serial.println("Initialization complete!");
    core0_ready = true;  // Signal Core 1 to start rendering
}

// ============= CORE 1 — LCD RENDERING =============
void setup1() {
    while (!core0_ready) tight_loop_contents();  // Wait for Core 0 hardware init
}

void loop1() {
    updateDisplay();
    delay(50);  // ~20 fps; non-blocking for Core 1
}

// ============= MAIN LOOP =============
void loop() {
    unsigned long now = millis();

    if (now - lastEncoderRead >= ENCODER_READ_INTERVAL) {
        readEncoder();
        lastEncoderRead = now;
    }

    readSensors();
    updateStateMachine();
    handleState();
    // Display is handled by Core 1 (loop1).
}

// ============= HARDWARE INITIALIZATION =============
void initHardware() {
    // Motor
    pinMode(TMC_STEP, OUTPUT);
    pinMode(TMC_DIR,  OUTPUT);
    pinMode(TMC_EN,   OUTPUT);
    digitalWrite(TMC_EN, HIGH);  // Motor disabled at startup

    // Sensors
    pinMode(LIMIT_SWITCH, INPUT_PULLUP);
    pinMode(SPRAY_VALVE,  INPUT);
    pinMode(FLOW_SENSOR,  INPUT);

    // Outputs
    pinMode(LED_GREEN,  OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(FAN_PWM,    OUTPUT);
    pinMode(ULTRASONIC, OUTPUT);
    pinMode(LCD_CS, OUTPUT);
    digitalWrite(LCD_CS, HIGH);
    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, HIGH);  // Backlight on

    digitalWrite(LED_GREEN,  LOW);
    digitalWrite(LED_YELLOW, LOW);
    analogWrite(FAN_PWM, 0);
    digitalWrite(ULTRASONIC, HIGH);  // Active-low: HIGH = ultrasonic OFF

    Serial.println("Hardware initialized");
}

void initSPI() {
    // earlephilhower: explicit pin assignment so SPI0 uses our wired pins.
    SPI.setRX(TMC_MISO);
    SPI.setTX(TMC_MOSI);
    SPI.setSCK(TMC_SCK);
    SPI.begin();
    Serial.print("SPI initialized: SCK=");
    Serial.print(TMC_SCK);
    Serial.print(" MOSI=");
    Serial.print(TMC_MOSI);
    Serial.print(" MISO=");
    Serial.println(TMC_MISO);
}

void initEncoder() {
    // Pull-ups required for open-collector encoder outputs and push-button
    pinMode(ENC_A,  INPUT_PULLUP);
    pinMode(ENC_B,  INPUT_PULLUP);
    pinMode(ENC_SW, INPUT_PULLUP);
    Serial.print("Encoder initialized: CLK=");
    Serial.print(ENC_A);
    Serial.print(" DT=");
    Serial.print(ENC_B);
    Serial.print(" SW=");
    Serial.println(ENC_SW);
}

void initTMC2130() {
    driver.begin();
    driver.toff(5);              // Enable chopper (mandatory to activate driver)
    driver.rms_current(600);     // 600 mA RMS — reduce if motor runs hot
    driver.microsteps(16);       // 16× microsteps for smooth motion
    driver.en_pwm_mode(true);    // StealthChop: quiet operation at low speed
    driver.pwm_autoscale(true);  // Auto-tune StealthChop amplitude
    Serial.println("TMC2130 configured: 600 mA, 16x microsteps, StealthChop");
}

void initDisplay() {
    tft.init(172, 320);      // GMT147SPI 1.47" 172x320
    tft.setSPISpeed(20000000);
    tft.setRotation(1);
    tft.fillScreen(ST77XX_BLACK);  // one clear at boot; drawMenu() reuses this black background
    tft.setTextWrap(false);
    Serial.println("Display initialized (GMT147SPI 1.47\" 172x320)");
}

// ============= SENSOR READING =============
void readSensors() {
    // 20 ms hardware debounce for the mechanical limit switch
    static bool          lastLimitRaw    = false;
    static unsigned long lastLimitChange = 0;
    bool rawLimit = (digitalRead(LIMIT_SWITCH) == LOW);
    if (rawLimit != lastLimitRaw) {
        lastLimitRaw    = rawLimit;
        lastLimitChange = millis();
    }
    if (millis() - lastLimitChange >= 20) {
        limitSwitchPressed = lastLimitRaw;
    }

    sprayActive  = (digitalRead(SPRAY_VALVE) == HIGH);
    flowDetected = (digitalRead(FLOW_SENSOR) == HIGH);
}

void readEncoder() {
    static int           lastEncoded  = 0;
    static int           stepAccum    = 0;
    static unsigned long lastStepTime = 0;

    int a       = digitalRead(ENC_A);
    int b       = digitalRead(ENC_B);
    int encoded = (a << 1) | b;
    int sum     = (lastEncoded << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) stepAccum++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) stepAccum--;

    lastEncoded = encoded;

    // KY-040 produces 4 quadrature transitions per physical detent click.
    // Require a full detent (±4) before registering a step — contact bounce
    // never accumulates that many consistent transitions in one direction.
    if (abs(stepAccum) >= 4) {
        unsigned long now = millis();
        if (now - lastStepTime >= 50) {
            menuIndex    = (menuIndex + (stepAccum > 0 ? 1 : -1) + MENU_COUNT) % MENU_COUNT;
            lastStepTime = now;
        }
        stepAccum = 0;
    }
}

// ============= STATE MACHINE TRANSITIONS =============
void updateStateMachine() {
    unsigned long now = millis();

    switch (currentState) {
        case STATE_IDLE:
            if (sprayActive) {
                currentState    = STATE_HOMING;
                lastStateChange = now;
                Serial.println("→ HOMING");
            }
            break;

        case STATE_HOMING:
            if (limitSwitchPressed) {
                motorPosition   = 0;
                currentState    = STATE_PARKED;
                lastStateChange = now;
                Serial.println("→ PARKED (moving to park position)");
            }
            break;

        case STATE_PARKED:
            // Only evaluate spray/flow transitions once at park position
            if (motorPosition >= PARK_STEPS) {
                if (sprayActive && !flowDetected) {
                    currentState    = STATE_WAITING_SPRAY;
                    lastStateChange = now;
                    Serial.println("→ WAITING_SPRAY");
                } else if (sprayActive && flowDetected) {
                    oscillationCount     = 0;
                    oscillationDir       = -1;
                    oscillationStepCount = 0;
                    currentState         = STATE_SPRAY_ACTIVE;
                    lastStateChange      = now;
                    Serial.println("→ SPRAY_ACTIVE");
                }
            }
            break;

        case STATE_WAITING_SPRAY:
            if (!sprayActive) {
                currentState    = STATE_PARKED;
                lastStateChange = now;
                Serial.println("→ PARKED (spray lost)");
            } else if (flowDetected) {
                oscillationCount     = 0;
                oscillationDir       = -1;
                oscillationStepCount = 0;
                currentState         = STATE_SPRAY_ACTIVE;
                lastStateChange      = now;
                Serial.println("→ SPRAY_ACTIVE");
            }
            break;

        case STATE_SPRAY_ACTIVE:
            if (!sprayActive || !flowDetected) {
                currentState    = STATE_IDLE;
                lastStateChange = now;
                Serial.println("→ IDLE (spray or flow lost)");
            } else if (now - lastStateChange >= SPRAY_ACTIVE_WAIT &&
                       motorPosition >= CENTER_STEPS) {
                // System stabilised and motor at centre — begin oscillation
                currentState    = STATE_OSCILLATING;
                lastStateChange = now;
                Serial.println("→ OSCILLATING");
            }
            break;

        case STATE_OSCILLATING:
            if (!sprayActive || !flowDetected) {
                currentState    = STATE_IDLE;
                lastStateChange = now;
                Serial.println("→ IDLE (spray or flow lost)");
            } else if (oscillationCount >= OSCILLATION_CYCLES) {
                currentState    = STATE_IDLE;
                lastStateChange = now;
                Serial.println("→ IDLE (cleaning cycle complete)");
            }
            break;

        case STATE_ERROR:
            // Requires power-cycle or manual reset — no automatic recovery
            break;
    }
}

// ============= STATE ACTIONS =============
void handleState() {
    unsigned long now = millis();

    switch (currentState) {
        case STATE_IDLE:
            motorSetEnable(false);
            setLED(LED_GREEN,  false);
            setLED(LED_YELLOW, false);
            setFan(FAN_OFF);
            setUltrasonic(false);
            break;

        case STATE_HOMING:
            // Step toward limit switch at MOTOR_UPDATE_INTERVAL
            motorSetEnable(true);
            setLED(LED_GREEN,  false);
            setLED(LED_YELLOW, true);
            if (now - lastMotorUpdate >= MOTOR_UPDATE_INTERVAL) {
                motorStep(-1);
                lastMotorUpdate = now;
            }
            break;

        case STATE_PARKED:
            // Move forward to park position, then hold with motor disabled
            if (motorPosition < PARK_STEPS) {
                motorSetEnable(true);
                if (now - lastMotorUpdate >= MOTOR_UPDATE_INTERVAL) {
                    motorStep(1);
                    lastMotorUpdate = now;
                }
            } else {
                motorSetEnable(false);
            }
            setLED(LED_GREEN,  true);
            setLED(LED_YELLOW, false);
            setFan(FAN_OFF);
            setUltrasonic(false);
            break;

        case STATE_WAITING_SPRAY:
            motorSetEnable(false);
            setLED(LED_GREEN,  false);
            setLED(LED_YELLOW, true);
            setFan(FAN_WAITING);  // 50 % to prime circulation
            setUltrasonic(false);
            break;

        case STATE_SPRAY_ACTIVE:
            // Fan and ultrasonic on; motor moves toward centre position
            setFan(FAN_FULL);
            setUltrasonic(true);
            setLED(LED_GREEN,  true);
            setLED(LED_YELLOW, false);
            motorSetEnable(true);
            if (now - lastMotorUpdate >= MOTOR_UPDATE_INTERVAL) {
                motorMoveTo(CENTER_STEPS);
                lastMotorUpdate = now;
            }
            break;

        case STATE_OSCILLATING:
            motorSetEnable(true);
            setFan(FAN_FULL);
            setUltrasonic(true);
            setLED(LED_GREEN,  true);
            setLED(LED_YELLOW, false);
            if (now - lastMotorUpdate >= OSCILLATION_DELAY) {
                motorStep(oscillationDir);
                oscillationStepCount++;
                lastMotorUpdate = now;

                if (oscillationStepCount >= OSCILLATION_STEPS) {
                    // Completed one sweep — reverse direction
                    oscillationStepCount = 0;
                    oscillationDir       = -oscillationDir;
                    oscillationCount++;
                    Serial.print("Sweep ");
                    Serial.print(oscillationCount);
                    Serial.print("/");
                    Serial.println(OSCILLATION_CYCLES);
                }
            }
            break;

        case STATE_ERROR:
            motorSetEnable(false);
            setFan(FAN_OFF);
            setUltrasonic(false);
            setLED(LED_GREEN, false);
            setLED(LED_YELLOW, (millis() % 1000) < 500);  // 1 Hz blink
            break;
    }
}

// ============= MOTOR CONTROL =============
void motorStep(int direction) {
    digitalWrite(TMC_DIR,  direction > 0 ? HIGH : LOW);
    digitalWrite(TMC_STEP, HIGH);
    delayMicroseconds(10);
    digitalWrite(TMC_STEP, LOW);
    delayMicroseconds(10);
    motorPosition += (direction > 0) ? 1 : -1;
}

void motorSetEnable(bool enable) {
    // TMC2130 EN is active-low
    digitalWrite(TMC_EN, enable ? LOW : HIGH);
}

void motorMoveTo(int target) {
    if      (motorPosition < target) motorStep( 1);
    else if (motorPosition > target) motorStep(-1);
}

// ============= OUTPUT CONTROL =============
void setLED(int ledPin, bool state) {
    digitalWrite(ledPin, state ? HIGH : LOW);
}

void setFan(int speed) {
    analogWrite(FAN_PWM, constrain(speed, 0, 255));
}

void setUltrasonic(bool on) {
    // Active-low relay: drive LOW to switch ultrasonic ON
    digitalWrite(ULTRASONIC, on ? LOW : HIGH);
}

// ============= DISPLAY =============
void drawMenuRow(int i, int y, bool selected) {
    uint16_t bg = selected ? ST77XX_BLUE : ST77XX_BLACK;
    tft.fillRect(6, y - 2, tft.width() - 12, 28, bg);
    tft.setTextColor(ST77XX_WHITE, bg);
    tft.setTextSize(3);
    tft.setCursor(12, y);
    tft.print(menuItems[i]);
}

void drawMenu() {
    // Landscape 320×172. Layout with textSize 3 (24 px/row):
    //   Title  y=2  (size 2, 16 px)
    //   Items  y=22,50,78,106  step=28  (24 px text + 4 px gap)
    //   Status y=150 (size 2, 16 px, bottom-aligned)
    static int lastDrawnMenu = -1;

    // Snapshot volatile once — prevents Core 0 changing it mid-function
    // and leaving two rows simultaneously highlighted.
    int mi = menuIndex;

    if (lastDrawnMenu == -1) {
        // First draw: screen already black from initDisplay(); paint title + all rows.
        tft.setTextSize(2);
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.setCursor(8, 2);
        tft.print("MEGASONIC");
        int y = 22;
        for (int i = 0; i < MENU_COUNT; ++i) {
            drawMenuRow(i, y, i == mi);
            y += 28;
        }
    } else if (lastDrawnMenu != mi) {
        // Selection changed: repaint only the two affected rows.
        drawMenuRow(lastDrawnMenu, 22 + lastDrawnMenu * 28, false);
        drawMenuRow(mi,            22 + mi             * 28, true);
    }

    lastDrawnMenu = mi;
}

void updateDisplay() {
    // Only redraw when something visible has changed.
    // Snapshot all volatile shared variables once to avoid tearing.
    static SystemState lastState = (SystemState)-1;
    static int         lastPos   = -1;
    static int         lastMenu  = -1;

    int         mi    = menuIndex;
    SystemState state = currentState;
    int         pos   = motorPosition;

    bool menuChanged   = (mi    != lastMenu);
    bool statusChanged = (state != lastState || pos != lastPos);

    if (!menuChanged && !statusChanged) return;

    // Menu rows — only repaints changed rows, no fillScreen.
    if (menuChanged) {
        drawMenu();
        lastMenu = mi;
    }

    // Status bar — fast text-only update, independent of menu redraws.
    if (statusChanged) {
        tft.setTextSize(2);
        tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
        tft.setCursor(6, 150);
        tft.print("State:");
        switch (state) {
            case STATE_IDLE:          tft.print("IDLE  "); break;
            case STATE_HOMING:        tft.print("HOMING"); break;
            case STATE_PARKED:        tft.print("PARKED"); break;
            case STATE_WAITING_SPRAY: tft.print("WAIT  "); break;
            case STATE_SPRAY_ACTIVE:  tft.print("SPRAY "); break;
            case STATE_OSCILLATING:   tft.print("OSC   "); break;
            case STATE_ERROR:         tft.print("ERROR "); break;
        }
        tft.print(" Pos:");
        tft.print(pos);
        tft.print("   ");

        lastState = state;
        lastPos   = pos;

        Serial.print("State:");
        switch (state) {
            case STATE_IDLE:          Serial.print("IDLE");         break;
            case STATE_HOMING:        Serial.print("HOMING");       break;
            case STATE_PARKED:        Serial.print("PARKED");       break;
            case STATE_WAITING_SPRAY: Serial.print("WAITING");      break;
            case STATE_SPRAY_ACTIVE:  Serial.print("SPRAY_ACTIVE"); break;
            case STATE_OSCILLATING:   Serial.print("OSCILLATING");  break;
            case STATE_ERROR:         Serial.print("ERROR");        break;
        }
        Serial.print(" | Pos:");
        Serial.print(pos);
        Serial.print(" | Spray:");
        Serial.print(sprayActive  ? "ON"  : "OFF");
        Serial.print(" | Flow:");
        Serial.println(flowDetected ? "YES" : "NO");
    }
}
