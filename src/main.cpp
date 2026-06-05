#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <pico/mutex.h>

// ============= PIN DEFINITIONS =============
const int TMC_CS   = 17;
const int TMC_MOSI = 19;
const int TMC_MISO = 16;
const int TMC_SCK  = 18;
const int TMC_STEP = 14;
const int TMC_DIR  = 15;
const int TMC_EN   = 13;

// GMT147SPI 1.47" 172x320 ST7789 — SCK/MOSI shared with TMC2130 on SPI0
const int LCD_CS  = 9;
const int LCD_DC  = 10;
const int LCD_RST = 11;
const int LCD_BL  = 20;

// KY-040 rotary encoder: CLK=A, DT=B, SW=push-button
const int ENC_A  = 26;
const int ENC_B  = 27;
const int ENC_SW = 22;

const int LIMIT_SWITCH = 28;   // INPUT_PULLUP, LOW = pressed
const int SPRAY_VALVE  = 2;    // HIGH = spray active
const int FLOW_SENSOR  = 3;    // HIGH = flowing
const int LED_GREEN    = 8;
const int LED_YELLOW   = 7;
const int FAN_PWM      = 12;
const int ULTRASONIC   = 4;    // Active-low relay: LOW = ON

// ============= TMC2130 DRIVER =============
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

// When true: after reaching limit switch go IDLE instead of PARKED (abort path)
volatile bool homingToStop = false;

// ============= MOTION PARAMETERS =============
const int STEPS_PER_MM = 1;
int PARK_MM           = 7;
int CENTER_MM         = 26;
int OSCILLATION_STEPS = 16;
unsigned long OSCILLATION_DELAY  = 10000;
unsigned long OSCILLATION_CYCLES = 20;
const unsigned long SPRAY_ACTIVE_WAIT = 2000;

// ============= SETTINGS (sweep params) =============
// Speed 1–10 maps to OSCILLATION_DELAY = (11-speed)*1000 ms
int sweepSpeed   = 5;
int sampleIndex  = 4;   // index into SAMPLE_TABLE
int sweepType    = 0;   // 0=centre↔back, 1=edge→edge
int sweepProfile = 0;   // 0=linear, 1=harmonic, 2=swing

const int SAMPLE_TABLE[] = {10, 20, 50, 75, 100, 150};
const int SAMPLE_COUNT   = 6;

// ============= SETUP (hardware/driver params) =============
int driverCurrent    = 600;
int driverMicrosteps = 16;

const int MICROSTEP_TABLE[] = {1, 2, 4, 8, 16, 32, 64, 128, 256};
const int MICROSTEP_COUNT   = 9;

volatile uint32_t driverDrvStatus = 0;
volatile uint32_t driverGStat     = 0;
unsigned long lastDriverPoll = 0;
const unsigned long DRIVER_POLL_INTERVAL = 500;

const int FAN_OFF     = 0;
const int FAN_WAITING = 128;
const int FAN_FULL    = 255;

// ============= STATUS =============
volatile int  motorPosition      = 0;
volatile bool limitSwitchPressed = false;
volatile bool sprayActive        = false;
volatile bool flowDetected       = false;

unsigned long oscillationCount    = 0;
int           oscillationDir      = -1;
int           oscillationStepCount = 0;

unsigned long lastMotorUpdate = 0;
unsigned long lastEncoderRead = 0;
const unsigned long MOTOR_UPDATE_INTERVAL = 50;
const unsigned long ENCODER_READ_INTERVAL = 20;

// ============= DISPLAY / MENU =============
Adafruit_ST7789 tft = Adafruit_ST7789(LCD_CS, LCD_DC, LCD_RST);

// Main menu — 4 items, no HOME
const char* menuItems[] = { "START/STOP", "Settings", "Setup", "About" };
const int   MENU_COUNT  = 4;
volatile int menuIndex  = 0;

enum DisplayMode { DISP_MENU, DISP_SETTINGS, DISP_SETUP, DISP_ABOUT };
volatile DisplayMode displayMode = DISP_MENU;

volatile bool menuButtonPressed = false;
int menuDrawState = -1;  // -1 = full redraw needed

// Settings screen (sweep): Speed, Sample, Sweep, Profile, < Back
const int SETTINGS_COUNT = 5;
volatile int  settingsIndex       = 0;
volatile bool editingSettings     = false;
volatile bool settingsNeedsRedraw = false;

// Setup screen (hardware): Park, Centre, Steps, Delay, Cycles, Current, Mstep, < Back
const int SETUP_COUNT = 8;
volatile int  setupIndex       = 0;
volatile bool editingSetup     = false;
volatile bool setupNeedsRedraw = false;

volatile bool aboutNeedsRedraw = false;
volatile bool core0_ready = false;

mutex_t spi_mutex;

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
void handleMenuSelect();
void adjustSettingsValue(int delta);
void adjustSetupValue(int delta);
void applyDriverSettings();
void pollDriverStatus();
void updateDisplay();
void drawIcon(int id, int x, int y, uint16_t color);
void drawMenuRow(int i, int y, bool selected);
void drawMenu();
void drawSettingsRow(int i, int y, bool selected, bool editing);
void drawSettings();
void drawSetupRow(int i, int y, bool selected, bool editing);
void drawSetup();
void drawAbout();
void setLED(int ledPin, bool state);
void setFan(int speed);
void setUltrasonic(bool on);

// ============= SETUP =============
void setup() {
    Serial.begin(115200);
    { unsigned long t = millis(); while (!Serial && millis() - t < 2000) delay(10); }
    Serial.println("=== Stepper Controller Initializing ===");

    initHardware();
    initSPI();
    initTMC2130();
    initEncoder();
    initDisplay();

    currentState    = STATE_IDLE;
    lastStateChange = millis();
    Serial.println("Initialization complete!");
    core0_ready = true;
}

// ============= CORE 1 — LCD RENDERING =============
void setup1() {
    while (!core0_ready) tight_loop_contents();
}

void loop1() {
    updateDisplay();
    delay(50);  // ~20 fps
}

// ============= MAIN LOOP =============
void loop() {
    unsigned long now = millis();

    if (now - lastEncoderRead >= ENCODER_READ_INTERVAL) {
        readEncoder();
        lastEncoderRead = now;
    }

    if (menuButtonPressed) {
        menuButtonPressed = false;
        handleMenuSelect();
    }

    readSensors();
    updateStateMachine();
    handleState();
    pollDriverStatus();
}

// ============= HARDWARE INIT =============
void initHardware() {
    pinMode(TMC_STEP, OUTPUT);
    pinMode(TMC_DIR,  OUTPUT);
    pinMode(TMC_EN,   OUTPUT);
    digitalWrite(TMC_EN, HIGH);

    pinMode(LIMIT_SWITCH, INPUT_PULLUP);
    pinMode(SPRAY_VALVE,  INPUT);
    pinMode(FLOW_SENSOR,  INPUT);

    pinMode(LED_GREEN,  OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(FAN_PWM,    OUTPUT);
    pinMode(ULTRASONIC, OUTPUT);
    pinMode(LCD_CS, OUTPUT);  digitalWrite(LCD_CS, HIGH);
    pinMode(LCD_BL, OUTPUT);  digitalWrite(LCD_BL, HIGH);

    digitalWrite(LED_GREEN,  LOW);
    digitalWrite(LED_YELLOW, LOW);
    analogWrite(FAN_PWM, 0);
    digitalWrite(ULTRASONIC, HIGH);  // relay OFF at startup

    Serial.println("Hardware initialized");
}

void initSPI() {
    SPI.setRX(TMC_MISO);
    SPI.setTX(TMC_MOSI);
    SPI.setSCK(TMC_SCK);
    SPI.begin();
    mutex_init(&spi_mutex);
    Serial.printf("SPI initialized: SCK=%d MOSI=%d MISO=%d\n", TMC_SCK, TMC_MOSI, TMC_MISO);
}

void initEncoder() {
    pinMode(ENC_A,  INPUT_PULLUP);
    pinMode(ENC_B,  INPUT_PULLUP);
    pinMode(ENC_SW, INPUT_PULLUP);
    Serial.printf("Encoder initialized: CLK=%d DT=%d SW=%d\n", ENC_A, ENC_B, ENC_SW);
}

void initTMC2130() {
    driver.begin();
    driver.toff(5);
    driver.rms_current(600);
    driver.microsteps(16);
    driver.en_pwm_mode(true);
    driver.pwm_autoscale(true);
    Serial.println("TMC2130 configured: 600 mA, 16x microsteps, StealthChop");
}

void initDisplay() {
    tft.init(172, 320);
    tft.setSPISpeed(20000000);
    tft.setRotation(1);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextWrap(false);
    Serial.println("Display initialized (GMT147SPI 1.47\" 172x320)");
}

// ============= SENSORS =============
void readSensors() {
    static bool          lastLimitRaw    = false;
    static unsigned long lastLimitChange = 0;
    bool raw = (digitalRead(LIMIT_SWITCH) == LOW);
    if (raw != lastLimitRaw) { lastLimitRaw = raw; lastLimitChange = millis(); }
    if (millis() - lastLimitChange >= 20) limitSwitchPressed = lastLimitRaw;

    sprayActive  = (digitalRead(SPRAY_VALVE) == HIGH);
    flowDetected = (digitalRead(FLOW_SENSOR) == HIGH);
}

// ============= ENCODER =============
void readEncoder() {
    static int           lastEncoded  = 0;
    static int           stepAccum    = 0;
    static unsigned long lastStepTime = 0;

    int a = digitalRead(ENC_A), b = digitalRead(ENC_B);
    int encoded = (a << 1) | b;
    int sum     = (lastEncoded << 2) | encoded;
    lastEncoded = encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) stepAccum++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) stepAccum--;

    if (abs(stepAccum) >= 4) {
        unsigned long now = millis();
        if (now - lastStepTime >= 50) {
            int d = stepAccum > 0 ? 1 : -1;
            switch (displayMode) {
                case DISP_MENU:
                    menuIndex = (menuIndex + d + MENU_COUNT) % MENU_COUNT;
                    break;
                case DISP_SETTINGS:
                    if (editingSettings) adjustSettingsValue(d);
                    else settingsIndex = constrain(settingsIndex + d, 0, SETTINGS_COUNT - 1);
                    break;
                case DISP_SETUP:
                    if (editingSetup) adjustSetupValue(d);
                    else setupIndex = constrain(setupIndex + d, 0, SETUP_COUNT - 1);
                    break;
                case DISP_ABOUT:
                    break;
            }
            lastStepTime = now;
        }
        stepAccum = 0;
    }

    static bool          lastBtn     = HIGH;
    static unsigned long lastBtnTime = 0;
    bool btn = digitalRead(ENC_SW);
    if (lastBtn == HIGH && btn == LOW) {
        unsigned long now = millis();
        if (now - lastBtnTime >= 200) { menuButtonPressed = true; lastBtnTime = now; }
    }
    lastBtn = btn;
}

// ============= STATE MACHINE =============
void updateStateMachine() {
    unsigned long now = millis();

    switch (currentState) {
        case STATE_IDLE:
            if (sprayActive) {
                currentState = STATE_HOMING; lastStateChange = now;
                Serial.println("→ HOMING");
            }
            break;

        case STATE_HOMING:
            if (limitSwitchPressed) {
                motorPosition = 0;
                if (homingToStop) {
                    homingToStop = false;
                    motorSetEnable(false);
                    currentState = STATE_IDLE; lastStateChange = now;
                    Serial.println("→ IDLE (aborted, parked at limit)");
                } else {
                    currentState = STATE_PARKED; lastStateChange = now;
                    Serial.println("→ PARKED (moving to park position)");
                }
            }
            break;

        case STATE_PARKED:
            if (motorPosition >= PARK_MM * STEPS_PER_MM) {
                if (sprayActive && !flowDetected) {
                    currentState = STATE_WAITING_SPRAY; lastStateChange = now;
                    Serial.println("→ WAITING_SPRAY");
                } else if (sprayActive && flowDetected) {
                    oscillationCount = 0; oscillationDir = -1; oscillationStepCount = 0;
                    currentState = STATE_SPRAY_ACTIVE; lastStateChange = now;
                    Serial.println("→ SPRAY_ACTIVE");
                }
            }
            break;

        case STATE_WAITING_SPRAY:
            if (!sprayActive) {
                currentState = STATE_PARKED; lastStateChange = now;
                Serial.println("→ PARKED (spray lost)");
            } else if (flowDetected) {
                oscillationCount = 0; oscillationDir = -1; oscillationStepCount = 0;
                currentState = STATE_SPRAY_ACTIVE; lastStateChange = now;
                Serial.println("→ SPRAY_ACTIVE");
            }
            break;

        case STATE_SPRAY_ACTIVE:
            if (!sprayActive || !flowDetected) {
                currentState = STATE_IDLE; lastStateChange = now;
                Serial.println("→ IDLE (spray or flow lost)");
            } else if (now - lastStateChange >= SPRAY_ACTIVE_WAIT &&
                       motorPosition >= CENTER_MM * STEPS_PER_MM) {
                currentState = STATE_OSCILLATING; lastStateChange = now;
                Serial.println("→ OSCILLATING");
            }
            break;

        case STATE_OSCILLATING:
            if (!sprayActive || !flowDetected) {
                currentState = STATE_IDLE; lastStateChange = now;
                Serial.println("→ IDLE (spray or flow lost)");
            } else if (oscillationCount >= OSCILLATION_CYCLES) {
                currentState = STATE_IDLE; lastStateChange = now;
                Serial.println("→ IDLE (cleaning cycle complete)");
            }
            break;

        case STATE_ERROR:
            break;
    }
}

// ============= STATE ACTIONS =============
void handleState() {
    unsigned long now = millis();

    switch (currentState) {
        case STATE_IDLE:
            motorSetEnable(false);
            setLED(LED_GREEN, false); setLED(LED_YELLOW, false);
            setFan(FAN_OFF); setUltrasonic(false);
            break;

        case STATE_HOMING:
            motorSetEnable(true);
            setLED(LED_GREEN, false); setLED(LED_YELLOW, true);
            if (now - lastMotorUpdate >= MOTOR_UPDATE_INTERVAL) {
                motorStep(-1); lastMotorUpdate = now;
            }
            break;

        case STATE_PARKED:
            if (motorPosition < PARK_MM * STEPS_PER_MM) {
                motorSetEnable(true);
                if (now - lastMotorUpdate >= MOTOR_UPDATE_INTERVAL) {
                    motorStep(1); lastMotorUpdate = now;
                }
            } else {
                motorSetEnable(false);
            }
            setLED(LED_GREEN, true); setLED(LED_YELLOW, false);
            setFan(FAN_OFF); setUltrasonic(false);
            break;

        case STATE_WAITING_SPRAY:
            motorSetEnable(false);
            setLED(LED_GREEN, false); setLED(LED_YELLOW, true);
            setFan(FAN_WAITING); setUltrasonic(false);
            break;

        case STATE_SPRAY_ACTIVE:
            setFan(FAN_FULL); setUltrasonic(true);
            setLED(LED_GREEN, true); setLED(LED_YELLOW, false);
            motorSetEnable(true);
            if (now - lastMotorUpdate >= MOTOR_UPDATE_INTERVAL) {
                motorMoveTo(CENTER_MM * STEPS_PER_MM); lastMotorUpdate = now;
            }
            break;

        case STATE_OSCILLATING:
            motorSetEnable(true);
            setFan(FAN_FULL); setUltrasonic(true);
            setLED(LED_GREEN, true); setLED(LED_YELLOW, false);
            if (now - lastMotorUpdate >= OSCILLATION_DELAY) {
                motorStep(oscillationDir);
                oscillationStepCount++;
                lastMotorUpdate = now;
                if (oscillationStepCount >= OSCILLATION_STEPS) {
                    oscillationStepCount = 0;
                    oscillationDir       = -oscillationDir;
                    oscillationCount++;
                    Serial.printf("Sweep %lu/%lu\n", oscillationCount, OSCILLATION_CYCLES);
                }
            }
            break;

        case STATE_ERROR:
            motorSetEnable(false); setFan(FAN_OFF); setUltrasonic(false);
            setLED(LED_GREEN, false);
            setLED(LED_YELLOW, (millis() % 1000) < 500);
            break;
    }
}

// ============= MENU ACTIONS =============
void handleMenuSelect() {
    if (displayMode == DISP_SETTINGS) {
        if (editingSettings)                       { editingSettings = false; }
        else if (settingsIndex == SETTINGS_COUNT - 1) {
            settingsIndex = 0; displayMode = DISP_MENU; menuDrawState = -1;
        } else                                     { editingSettings = true; }
        return;
    }

    if (displayMode == DISP_SETUP) {
        if (editingSetup)                    { editingSetup = false; }
        else if (setupIndex == SETUP_COUNT - 1) {
            applyDriverSettings();
            setupIndex = 0; displayMode = DISP_MENU; menuDrawState = -1;
        } else                               { editingSetup = true; }
        return;
    }

    if (displayMode == DISP_ABOUT) {
        displayMode = DISP_MENU; menuDrawState = -1; return;
    }

    switch (menuIndex) {
        case 0:  // START / STOP
            if (currentState == STATE_IDLE) {
                homingToStop = false;
                currentState = STATE_HOMING; lastStateChange = millis();
                Serial.println("Menu: START → HOMING");
            } else {
                // Abort: home to limit switch, then IDLE
                homingToStop = true;
                motorSetEnable(true);
                currentState = STATE_HOMING; lastStateChange = millis();
                Serial.println("Menu: STOP → homing to limit");
            }
            break;

        case 1:  // Settings (sweep params)
            settingsNeedsRedraw = true; editingSettings = false;
            settingsIndex = 0; displayMode = DISP_SETTINGS;
            break;

        case 2:  // Setup (hardware params)
            setupNeedsRedraw = true; editingSetup = false;
            setupIndex = 0; displayMode = DISP_SETUP;
            break;

        case 3:  // About
            aboutNeedsRedraw = true; displayMode = DISP_ABOUT;
            break;
    }
}

// Adjust Settings (sweep) parameter by encoder delta
void adjustSettingsValue(int delta) {
    switch (settingsIndex) {
        case 0:
            sweepSpeed = constrain(sweepSpeed + delta, 1, 10);
            OSCILLATION_DELAY = (unsigned long)(11 - sweepSpeed) * 1000UL;
            break;
        case 1:
            sampleIndex = constrain(sampleIndex + delta, 0, SAMPLE_COUNT - 1);
            break;
        case 2:
            sweepType = constrain(sweepType + delta, 0, 1);
            break;
        case 3:
            sweepProfile = constrain(sweepProfile + delta, 0, 2);
            break;
    }
}

// Adjust Setup (hardware) parameter by encoder delta
void adjustSetupValue(int delta) {
    switch (setupIndex) {
        case 0: PARK_MM = constrain(PARK_MM + delta, 1, 50); break;
        case 1: CENTER_MM = constrain(CENTER_MM + delta, 1, 150); break;
        case 2: OSCILLATION_STEPS = constrain(OSCILLATION_STEPS + delta, 1, 200); break;
        case 3: OSCILLATION_DELAY = (unsigned long)constrain(
                    (long)OSCILLATION_DELAY + delta * 1000L, 1000L, 60000L); break;
        case 4: OSCILLATION_CYCLES = (unsigned long)constrain(
                    (long)OSCILLATION_CYCLES + delta, 1L, 100L); break;
        case 5: driverCurrent = constrain(driverCurrent + delta * 50, 100, 1500); break;
        case 6: {
            int idx = 4;
            for (int i = 0; i < MICROSTEP_COUNT; i++)
                if (MICROSTEP_TABLE[i] == driverMicrosteps) { idx = i; break; }
            idx = constrain(idx + delta, 0, MICROSTEP_COUNT - 1);
            driverMicrosteps = MICROSTEP_TABLE[idx];
            break;
        }
    }
}

void applyDriverSettings() {
    mutex_enter_blocking(&spi_mutex);
    driver.rms_current(driverCurrent);
    driver.microsteps(driverMicrosteps);
    mutex_exit(&spi_mutex);
    Serial.printf("Driver applied: %d mA, %dx microsteps\n", driverCurrent, driverMicrosteps);
}

void pollDriverStatus() {
    unsigned long now = millis();
    if (now - lastDriverPoll < DRIVER_POLL_INTERVAL) return;
    lastDriverPoll = now;
    if (mutex_try_enter(&spi_mutex, nullptr)) {
        driverDrvStatus = driver.DRV_STATUS();
        driverGStat     = driver.GSTAT();
        mutex_exit(&spi_mutex);
    }
}

// ============= MOTOR =============
void motorStep(int direction) {
    digitalWrite(TMC_DIR,  direction > 0 ? HIGH : LOW);
    digitalWrite(TMC_STEP, HIGH); delayMicroseconds(10);
    digitalWrite(TMC_STEP, LOW);  delayMicroseconds(10);
    motorPosition += (direction > 0) ? 1 : -1;
}

void motorSetEnable(bool enable) {
    digitalWrite(TMC_EN, enable ? LOW : HIGH);
}

void motorMoveTo(int target) {
    if      (motorPosition < target) motorStep( 1);
    else if (motorPosition > target) motorStep(-1);
}

// ============= OUTPUTS =============
void setLED(int ledPin, bool state) { digitalWrite(ledPin, state ? HIGH : LOW); }
void setFan(int speed) { analogWrite(FAN_PWM, constrain(speed, 0, 255)); }
void setUltrasonic(bool on) { digitalWrite(ULTRASONIC, on ? LOW : HIGH); }

// ============= DISPLAY =============

// Draw icon id into a 20×20 box at (x,y).
// id: 0=play/stop (START/STOP), 1=sliders (Settings), 2=gear (Setup), 3=info (About)
void drawIcon(int id, int x, int y, uint16_t color) {
    switch (id) {
        case 0:  // ▶ play triangle
            tft.fillTriangle(x, y+1, x, y+17, x+14, y+9, color);
            break;
        case 1:  // ≡ three-slider equaliser
            tft.fillRect(x,    y+2,  16, 2, color);
            tft.fillRect(x,    y+8,  16, 2, color);
            tft.fillRect(x,    y+14, 16, 2, color);
            tft.fillCircle(x+4,  y+3,  3, color);
            tft.fillCircle(x+12, y+9,  3, color);
            tft.fillCircle(x+8,  y+15, 3, color);
            break;
        case 2:  // ⚙ gear
            tft.fillCircle(x+8, y+8, 7, color);
            tft.fillRect(x+5, y,    6, 4, color);   // top tooth
            tft.fillRect(x+5, y+13, 6, 5, color);   // bottom tooth
            tft.fillRect(x,    y+5, 4, 6, color);   // left tooth
            tft.fillRect(x+13, y+5, 5, 6, color);   // right tooth
            tft.fillCircle(x+8, y+8, 3, ST77XX_BLACK);  // hole
            break;
        case 3:  // ℹ info circle
            tft.drawCircle(x+8, y+8, 8, color);
            tft.fillRect(x+6, y+3, 4, 3, color);  // dot
            tft.fillRect(x+6, y+8, 4, 6, color);  // stem
            break;
    }
}

void drawMenuRow(int i, int y, bool selected) {
    uint16_t bg   = selected ? ST77XX_BLUE : ST77XX_BLACK;
    uint16_t icnc = selected ? ST77XX_WHITE : ST77XX_CYAN;

    tft.fillRect(6, y - 2, tft.width() - 12, 28, bg);

    // Item 0: show stop square (■) when motor is running, play triangle otherwise
    if (i == 0 && currentState != STATE_IDLE) {
        tft.fillRect(9, y + 3, 14, 14, icnc);
    } else {
        drawIcon(i, 8, y + 2, icnc);
    }

    tft.setTextColor(ST77XX_WHITE, bg);
    tft.setTextSize(3);
    tft.setCursor(36, y);
    tft.print(menuItems[i]);
}

void drawMenu() {
    // Layout (landscape 320×172):
    //   Wave + title  y=2   (textSize 2, 16px tall)
    //   Items         y=22,50,78,106  step=28  (textSize 3, 24px)
    //   Status bar    y=150

    int mi = menuIndex;  // snapshot to avoid race with Core 0

    if (menuDrawState == -1) {
        tft.fillScreen(ST77XX_BLACK);

        // Ultrasonic wave logo: 3 zigzag bumps in cyan
        uint16_t wc = tft.color565(0, 200, 255);
        tft.drawLine(8, 12, 11,  6, wc);
        tft.drawLine(11, 6, 14, 12, wc);
        tft.drawLine(14,12, 17,  6, wc);
        tft.drawLine(17, 6, 20, 12, wc);
        tft.drawLine(20,12, 23,  6, wc);
        tft.drawLine(23, 6, 26, 12, wc);

        tft.setTextSize(2);
        tft.setTextColor(wc, ST77XX_BLACK);
        tft.setCursor(32, 2);
        tft.print("MEGASONIC");

        for (int i = 0; i < MENU_COUNT; i++)
            drawMenuRow(i, 22 + i * 28, i == mi);

    } else if (menuDrawState != mi) {
        drawMenuRow(menuDrawState, 22 + menuDrawState * 28, false);
        drawMenuRow(mi,            22 + mi             * 28, true);
    }

    menuDrawState = mi;
}

// ============= SETTINGS SCREEN (sweep params) =============
void drawSettingsRow(int i, int y, bool selected, bool editing) {
    const char* labels[] = { "Speed  ", "Sample ", "Sweep  ", "Profile", "< Back " };
    uint16_t bg = editing  ? tft.color565(0, 140, 0) :
                  selected ? ST77XX_BLUE : ST77XX_BLACK;
    tft.fillRect(0, y - 2, tft.width(), 26, bg);
    tft.setTextColor(ST77XX_WHITE, bg);
    tft.setTextSize(2);
    tft.setCursor(6, y);
    tft.print(labels[i]);
    if (i < SETTINGS_COUNT - 1) {
        tft.print(": ");
        switch (i) {
            case 0:
                tft.print(sweepSpeed);
                tft.print("    ");
                break;
            case 1:
                tft.print(SAMPLE_TABLE[sampleIndex]);
                tft.print("mm  ");
                break;
            case 2:
                tft.print(sweepType == 0 ? "Ctr<->Bk" : "Edge->Ed");
                break;
            case 3:
                if      (sweepProfile == 0) tft.print("Linear  ");
                else if (sweepProfile == 1) tft.print("Harmonic");
                else                        tft.print("Swing   ");
                break;
        }
    }
}

void drawSettings() {
    static int  lastIdx      = -1;
    static bool lastEdit     = false;
    static int  lastVals[4]  = {};

    if (settingsNeedsRedraw) { settingsNeedsRedraw = false; lastIdx = -1; }

    int  si = settingsIndex;
    bool ed = editingSettings;
    int  vals[4] = { sweepSpeed, sampleIndex, sweepType, sweepProfile };

    bool changed = (lastIdx == -1) || (si != lastIdx) || (ed != lastEdit);
    for (int i = 0; i < 4 && !changed; i++) changed = (vals[i] != lastVals[i]);
    if (!changed) return;

    if (lastIdx == -1) {
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextWrap(false);
        tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
        tft.setTextSize(2);
        tft.setCursor(6, 3);
        tft.print("SETTINGS");
    }

    for (int i = 0; i < SETTINGS_COUNT; i++)
        drawSettingsRow(i, 22 + i * 28, i == si, i == si && ed && i < SETTINGS_COUNT - 1);

    lastIdx = si; lastEdit = ed;
    for (int i = 0; i < 4; i++) lastVals[i] = vals[i];
}

// ============= SETUP SCREEN (hardware params) =============
void drawSetupRow(int i, int y, bool selected, bool editing) {
    const char* labels[] = {
        "Park   ", "Centre ", "Steps  ", "Delay  ",
        "Cycles ", "Current", "Mstep  ", "< Back "
    };
    uint16_t bg = editing  ? tft.color565(0, 140, 0) :
                  selected ? ST77XX_BLUE : ST77XX_BLACK;
    tft.fillRect(0, y - 2, tft.width(), 18, bg);
    tft.setTextColor(ST77XX_WHITE, bg);
    tft.setTextSize(2);
    tft.setCursor(6, y);
    tft.print(labels[i]);
    if (i < SETUP_COUNT - 1) {
        tft.print(": ");
        switch (i) {
            case 0: tft.print(PARK_MM);                     tft.print("mm  ");  break;
            case 1: tft.print(CENTER_MM);                   tft.print("mm  ");  break;
            case 2: tft.print(OSCILLATION_STEPS);           tft.print("      "); break;
            case 3: tft.print(OSCILLATION_DELAY / 1000);    tft.print("s     "); break;
            case 4: tft.print(OSCILLATION_CYCLES);          tft.print("      "); break;
            case 5: tft.print(driverCurrent);               tft.print("mA  ");  break;
            case 6: tft.print(driverMicrosteps);            tft.print("x     "); break;
        }
    }
}

void drawSetup() {
    static int  lastIdx     = -1;
    static bool lastEdit    = false;
    static int  lastVals[7] = {};

    if (setupNeedsRedraw) { setupNeedsRedraw = false; lastIdx = -1; }

    int  si = setupIndex;
    bool ed = editingSetup;
    int  vals[7] = {
        PARK_MM, CENTER_MM, OSCILLATION_STEPS,
        (int)(OSCILLATION_DELAY / 1000), (int)OSCILLATION_CYCLES,
        driverCurrent, driverMicrosteps
    };

    bool changed = (lastIdx == -1) || (si != lastIdx) || (ed != lastEdit);
    for (int i = 0; i < 7 && !changed; i++) changed = (vals[i] != lastVals[i]);
    if (!changed) return;

    if (lastIdx == -1) {
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextWrap(false);
        tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
        tft.setTextSize(2);
        tft.setCursor(6, 3);
        tft.print("SETUP");
    }

    for (int i = 0; i < SETUP_COUNT; i++)
        drawSetupRow(i, 22 + i * 18, i == si, i == si && ed && i < SETUP_COUNT - 1);

    lastIdx = si; lastEdit = ed;
    for (int i = 0; i < 7; i++) lastVals[i] = vals[i];
}

// ============= ABOUT SCREEN =============
void drawAbout() {
    static bool     titleDrawn = false;
    static uint32_t lastDrvSt  = 0xFFFFFFFF;
    static uint32_t lastGStat  = 0xFFFFFFFF;

    if (aboutNeedsRedraw) {
        aboutNeedsRedraw = false;
        titleDrawn = false;
        lastDrvSt  = 0xFFFFFFFF;
        lastGStat  = 0xFFFFFFFF;
    }

    if (!titleDrawn) {
        titleDrawn = true;
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextWrap(false);
        tft.setTextColor(ST77XX_CYAN,  ST77XX_BLACK);
        tft.setTextSize(2);
        tft.setCursor(8, 3);   tft.print("MEGASONIC v1.0");
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.setCursor(8, 22);  tft.print("RP2040 earlephilhower");
        tft.setCursor(8, 40);  tft.print("GMT147SPI 172x320");
        tft.setCursor(8, 58);  tft.print("Authors: LK & AG");
        tft.setTextColor(ST77XX_CYAN,  ST77XX_BLACK);
        tft.setCursor(8, 76);  tft.print("Driver Status:");
        tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
        tft.setCursor(8, 154); tft.print("Press to return");
    }

    uint32_t ds = driverDrvStatus, gs = driverGStat;
    if (ds == lastDrvSt && gs == lastGStat) return;
    lastDrvSt = ds; lastGStat = gs;

    bool     ot     = (ds >> 30) & 1;
    bool     otpw   = (ds >> 29) & 1;
    bool     stst   = (ds >> 24) & 1;
    bool     drverr = (gs >> 1)  & 1;
    uint8_t  cs     = (ds >> 16) & 0x1F;
    uint16_t sg     = ds & 0x3FF;

    tft.setTextSize(2);

    tft.setCursor(8, 94);
    tft.setTextColor(ot   ? ST77XX_RED    : ST77XX_WHITE, ST77XX_BLACK);
    tft.print("OT:");   tft.print(ot   ? "YES " : "NO  ");
    tft.setTextColor(otpw ? ST77XX_YELLOW : ST77XX_WHITE, ST77XX_BLACK);
    tft.print("OTPW:"); tft.print(otpw ? "YES " : "NO  ");

    tft.setCursor(8, 114);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.print("CS:"); tft.print(cs); tft.print("  ");
    tft.print("SG:"); tft.print(sg); tft.print("  ");

    tft.setCursor(8, 134);
    tft.print("STST:"); tft.print(stst ? "YES " : "NO  ");
    tft.setTextColor(drverr ? ST77XX_RED : ST77XX_WHITE, ST77XX_BLACK);
    tft.print("ERR:"); tft.print(drverr ? "YES" : "NO ");
}

// ============= UPDATE DISPLAY (Core 1) =============
void updateDisplay() {
    static SystemState lastState       = (SystemState)-1;
    static int         lastPos         = -1;
    static int         lastMenu        = -1;
    static bool        lastRunning     = false;
    static DisplayMode lastDisplayMode = DISP_MENU;

    DisplayMode dm    = displayMode;
    int         mi    = menuIndex;
    SystemState state = currentState;
    int         pos   = motorPosition;

    if (dm != DISP_MENU) {
        mutex_enter_blocking(&spi_mutex);
        if      (dm == DISP_SETTINGS) drawSettings();
        else if (dm == DISP_SETUP)    drawSetup();
        else                          drawAbout();
        mutex_exit(&spi_mutex);
        lastDisplayMode = dm;
        return;
    }

    if (lastDisplayMode != DISP_MENU) {
        lastState   = (SystemState)-1;
        lastPos     = -1;
        lastMenu    = -1;
        lastRunning = false;
        lastDisplayMode = DISP_MENU;
    }

    bool running = (state != STATE_IDLE);

    // State changed from idle↔running: force icon on row 0 to repaint
    if (running != lastRunning) {
        menuDrawState = -1;
        lastRunning   = running;
    }

    bool menuChanged  = (mi    != lastMenu);
    bool statusChanged = (state != lastState || pos != lastPos);

    if (!menuChanged && !statusChanged) return;

    mutex_enter_blocking(&spi_mutex);

    if (menuChanged) {
        drawMenu();
        lastMenu = mi;
    }

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
        tft.print(" Pos:"); tft.print(pos); tft.print("   ");

        lastState = state; lastPos = pos;

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
        Serial.printf(" | Pos:%d | Spray:%s | Flow:%s\n",
            pos, sprayActive ? "ON" : "OFF", flowDetected ? "YES" : "NO");
    }

    mutex_exit(&spi_mutex);
}
