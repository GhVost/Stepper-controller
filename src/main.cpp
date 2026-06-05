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

// SPI LCD — CS/DC/RST separate; SCK/MOSI shared with TMC2130 on SPI0
const int LCD_CS  = 9;
const int LCD_DC  = 10;
const int LCD_RST = 11;

// Rotary encoder
const int ENC_A = 26;
const int ENC_B = 27;

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

SystemState currentState = STATE_IDLE;
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
int  motorPosition      = 0;
bool limitSwitchPressed = false;
bool sprayActive        = false;
bool flowDetected       = false;

unsigned long oscillationCount    = 0;
int           oscillationDir      = -1;  // -1 = toward limit, +1 = away
int           oscillationStepCount = 0;  // Steps taken in current sweep

// ============= TIMING =============
unsigned long lastMotorUpdate   = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastEncoderRead   = 0;
const unsigned long MOTOR_UPDATE_INTERVAL   = 50;
const unsigned long DISPLAY_UPDATE_INTERVAL = 100;
const unsigned long ENCODER_READ_INTERVAL   = 20;

// ============= DISPLAY / MENU =============
Adafruit_ST7789 tft = Adafruit_ST7789(LCD_CS, LCD_DC, LCD_RST);

const char* menuItems[] = { "START", "HOME", "SETTINGS", "ABOUT" };
const int   MENU_COUNT  = sizeof(menuItems) / sizeof(menuItems[0]);
int menuIndex = 0;

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
void drawMenu();
void setLED(int ledPin, bool state);
void setFan(int speed);
void setUltrasonic(bool on);

// ============= SETUP =============
void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("=== Stepper Controller Initializing ===");

    initHardware();
    initSPI();
    initTMC2130();
    initEncoder();
    initDisplay();

    currentState    = STATE_IDLE;
    lastStateChange = millis();
    Serial.println("Initialization complete!");
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

    if (now - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
        updateDisplay();
        lastDisplayUpdate = now;
    }
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
    pinMode(SPRAY_VALVE,  INPUT);   // Expects external 0 / 3.3 V signal
    pinMode(FLOW_SENSOR,  INPUT);

    // Outputs
    pinMode(LED_GREEN,  OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(FAN_PWM,    OUTPUT);
    pinMode(ULTRASONIC, OUTPUT);
    pinMode(LCD_CS,     OUTPUT);
    digitalWrite(LCD_CS, HIGH);

    digitalWrite(LED_GREEN,  LOW);
    digitalWrite(LED_YELLOW, LOW);
    analogWrite(FAN_PWM, 0);
    digitalWrite(ULTRASONIC, HIGH);  // Active-low: HIGH = ultrasonic OFF

    Serial.println("Hardware initialized");
}

void initSPI() {
    SPI.begin();
    Serial.print("SPI initialized: SCK=");
    Serial.print(TMC_SCK);
    Serial.print(" MOSI=");
    Serial.print(TMC_MOSI);
    Serial.print(" MISO=");
    Serial.println(TMC_MISO);
}

void initEncoder() {
    // Pull-ups required for open-collector encoder outputs
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    Serial.print("Encoder initialized: A=");
    Serial.print(ENC_A);
    Serial.print(" B=");
    Serial.println(ENC_B);
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
    tft.init(240, 280);
    tft.setRotation(1);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextWrap(false);
    Serial.println("Display initialized (ST7789V3 240x280)");
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
    static int lastEncoded = 0;
    int a       = digitalRead(ENC_A);
    int b       = digitalRead(ENC_B);
    int encoded = (a << 1) | b;
    int sum     = (lastEncoded << 2) | encoded;
    int delta   = 0;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) delta =  1;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) delta = -1;

    if (delta != 0) {
        menuIndex = (menuIndex + delta + MENU_COUNT) % MENU_COUNT;
    }
    lastEncoded = encoded;
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
void drawMenu() {
    tft.fillScreen(ST77XX_BLACK);

    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(8, 4);
    tft.print("MEGASONIC");

    int y = 40;
    for (int i = 0; i < MENU_COUNT; ++i) {
        if (i == menuIndex) {
            tft.fillRect(6, y - 2, tft.width() - 12, 40, ST77XX_BLUE);
            tft.setTextColor(ST77XX_WHITE, ST77XX_BLUE);
        } else {
            tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        }
        tft.setTextSize(4);
        tft.setCursor(12, y);
        tft.print(menuItems[i]);
        y += 48;
    }
}

void updateDisplay() {
    // Only redraw when something visible has changed
    static SystemState lastState = (SystemState)-1;
    static int         lastPos   = -1;
    static int         lastMenu  = -1;

    if (currentState == lastState &&
        motorPosition == lastPos  &&
        menuIndex     == lastMenu) return;

    drawMenu();

    tft.setTextSize(2);
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    tft.setCursor(6, tft.height() - 28);
    tft.print("State:");
    switch (currentState) {
        case STATE_IDLE:          tft.print("IDLE");    break;
        case STATE_HOMING:        tft.print("HOMING");  break;
        case STATE_PARKED:        tft.print("PARKED");  break;
        case STATE_WAITING_SPRAY: tft.print("WAIT");    break;
        case STATE_SPRAY_ACTIVE:  tft.print("SPRAY");   break;
        case STATE_OSCILLATING:   tft.print("OSC");     break;
        case STATE_ERROR:         tft.print("ERROR");   break;
    }
    tft.print(" Pos:");
    tft.print(motorPosition);
    tft.print("   ");  // Overwrite leftover digits from longer numbers

    lastState = currentState;
    lastPos   = motorPosition;
    lastMenu  = menuIndex;

    // Mirror to serial for bench debugging
    Serial.print("State:");
    switch (currentState) {
        case STATE_IDLE:          Serial.print("IDLE");         break;
        case STATE_HOMING:        Serial.print("HOMING");       break;
        case STATE_PARKED:        Serial.print("PARKED");       break;
        case STATE_WAITING_SPRAY: Serial.print("WAITING");      break;
        case STATE_SPRAY_ACTIVE:  Serial.print("SPRAY_ACTIVE"); break;
        case STATE_OSCILLATING:   Serial.print("OSCILLATING");  break;
        case STATE_ERROR:         Serial.print("ERROR");        break;
    }
    Serial.print(" | Pos:");
    Serial.print(motorPosition);
    Serial.print(" | Spray:");
    Serial.print(sprayActive  ? "ON"  : "OFF");
    Serial.print(" | Flow:");
    Serial.println(flowDetected ? "YES" : "NO");
}
