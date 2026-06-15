#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <TMCStepper.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <pico/mutex.h>

// ============= PIN DEFINITIONS =============
const int TMC_CS   = 13;
const int TMC_MOSI = 11;
const int TMC_MISO = 12;
const int TMC_SCK  = 10;
const int TMC_STEP = 14;
const int TMC_DIR  = 15;
const int TMC_EN   = 1;

// GMT147SPI 1.47" 172x320 ST7789 on SPI0
const int LCD_CS  = 9;
const int LCD_DC  = 5;
const int LCD_RST = 6;
const int LCD_BL  = 20;
const int LCD_MOSI = 19;
const int LCD_SCK  = 18;

// KY-040 rotary encoder: CLK=A, DT=B, SW=push-button
const int ENC_A  = 26;
const int ENC_B  = 27;
const int ENC_SW = 22;

const int LIMIT_SWITCH = 28;   // INPUT_PULLUP, LOW = pressed
const int SPRAY_VALVE  = 2;    // HIGH = spray active
const int FLOW_SENSOR  = 3;    // HIGH = flowing
const int LED_GREEN    = 8;
const int LED_YELLOW   = 7;
const int FAN_PWM      = 21;
const int ULTRASONIC   = 4;    // Active-low relay: LOW = ON

// ============= TMC2130 DRIVER =============
#define R_SENSE 0.11f
TMC2130Stepper driver(TMC_CS, R_SENSE);

void TMC2130Stepper::beginTransaction() {
    if (TMC_SW_SPI == nullptr) {
        SPI1.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
    }
}

void TMC2130Stepper::endTransaction() {
    if (TMC_SW_SPI == nullptr) {
        SPI1.endTransaction();
    }
}

uint8_t TMC2130Stepper::transfer(const uint8_t data) {
    if (TMC_SW_SPI != nullptr) {
        return TMC_SW_SPI->transfer(data);
    }
    return SPI1.transfer(data);
}

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

// Home search safety: abort if the endstop is not found within this much arm travel.
const int HOMING_MAX_DEG_X10 = 700;     // 70.0 degrees of real arm motion
volatile int homingStartPos  = 0;       // motorPosition captured when homing begins

// Position is unknown (motor was disabled / never homed): re-home before a run.
// Set whenever the motor is de-energised; cleared once homing reaches the limit.
volatile bool needsHoming   = true;
// Park the arm, then disable the motor and return to IDLE (normal stop / end of cycle).
volatile bool stopRequested = false;
// A driver fault was detected: park the arm, then disable the motor and latch ERROR.
volatile bool faultLatched  = false;

// ============= MOTION PARAMETERS =============
const int FULL_STEPS_PER_REV = 200;  // typical 1.8 degree NEMA17 motor
int PARK_DEG_X10       = 70;
int CENTER_DEG_X10     = 260;
int ARM_LENGTH_MM      = 250;
unsigned long SWEEP_TIME_MS = 4000;
unsigned long OSCILLATION_CYCLES = 4;
const unsigned long SPRAY_ACTIVE_WAIT = 2000;
bool SENSOR_INPUTS_ENABLED = false;  // false = bypass/debug safety inputs (toggle in Setup)

// ============= SETTINGS (sweep params) =============
// Wafer diameter comes from SAMPLE_TABLE; sweep angle is calculated from arm length.
int sampleIndex  = 4;   // index into SAMPLE_TABLE
int sweepType    = 0;   // 0=back-centre-back, 1=back-front-back
int sweepProfile = 0;   // 0=linear, 1=harmonic, 2=inverse-distance

const int SAMPLE_TABLE[] = {10, 20, 50, 75, 100, 150};
const int SAMPLE_COUNT   = 6;

const int SWEEP_PATH_BACK_CENTER = 0;
const int SWEEP_PATH_BACK_FRONT  = 1;
const int SWEEP_PROFILE_LINEAR   = 0;
const int SWEEP_PROFILE_HARMONIC = 1;
const int SWEEP_PROFILE_INVDIST  = 2;
const int SWEEP_ENDPOINT_RAMP_DEG_X10 = 10;  // 1.0 degree accel/decel segment
const double SWEEP_ENDPOINT_SLOW_FACTOR = 3.0;

// ============= SETUP (hardware/driver params) =============
int driverCurrent    = 600;
int driverMicrosteps = 256;
bool motorDirectionInverted = false;
const float DRIVER_RUN_HOLD_MULTIPLIER = 0.25f;
const float DRIVER_PARK_HOLD_MULTIPLIER = 0.10f;
bool driverParkHoldMode = false;

const int MICROSTEP_TABLE[] = {1, 2, 4, 8, 16, 32, 64, 128, 256};
const int MICROSTEP_COUNT   = 9;

const uint32_t SETTINGS_MAGIC = 0x4D534743UL;  // "MSGC"
const uint16_t SETTINGS_VERSION = 6;
const size_t EEPROM_BYTES = 4096;

// Auto-save: every Setup/Settings change is persisted, but writes are debounced so a
// burst of encoder steps becomes a single flash write (wear protection). EEPROM.commit()
// also skips the write entirely when the stored bytes are unchanged.
volatile bool settingsDirty      = false;
unsigned long lastSettingsChange = 0;
const unsigned long SETTINGS_SAVE_DELAY = 2500;  // ms of quiet before committing to flash

struct StoredSettings {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    int16_t parkDegX10;
    int16_t centerDegX10;
    int16_t armLengthMm;
    uint32_t sweepTimeMs;
    uint32_t oscillationCycles;
    int16_t sampleIndex;
    int16_t sweepType;
    int16_t sweepProfile;
    int16_t driverCurrent;
    int16_t driverMicrosteps;
    int16_t motorDirectionInverted;
    int16_t sensorInputsEnabled;
    uint32_t checksum;
};

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
volatile bool debugMotorHold     = false;
volatile bool sensorBypassCycleArmed = false;
volatile bool ultrasonicActive   = false;   // generator on/off, for the LCD lightning sign

unsigned long oscillationCount    = 0;
int           oscillationDir      = -1;
int           oscillationStepCount = 0;

unsigned long lastMotorStepMicros = 0;
bool          sweepTimingActive = false;
int           sweepTimingStartSteps = 0;
int           sweepTimingTargetSteps = 0;
unsigned long sweepTimingStartMicros = 0;
double        sweepTimingCompletedFactorSum = 0.0;
double        sweepTimingBaseUs = 1.0;
unsigned long lastEncoderRead = 0;
volatile bool    encoderButtonEdgePending = false;
volatile bool    encoderButtonRawState = HIGH;
volatile unsigned long encoderButtonEdgeMillis = 0;

// Buxton quadrature state table: rows are states, columns are pinState = (B<<1 | A).
const uint8_t ENC_R_START   = 0x0;
const uint8_t ENC_R_CW_FIN  = 0x1;
const uint8_t ENC_R_CW_BEG  = 0x2;
const uint8_t ENC_R_CW_NEXT = 0x3;
const uint8_t ENC_R_CCW_BEG = 0x4;
const uint8_t ENC_R_CCW_FIN = 0x5;
const uint8_t ENC_R_CCW_NEXT = 0x6;
const uint8_t ENC_DIR_CW    = 0x10;
const uint8_t ENC_DIR_CCW   = 0x20;
const uint8_t ENC_TTABLE[7][4] = {
    {ENC_R_START,      ENC_R_CW_BEG,  ENC_R_CCW_BEG, ENC_R_START},
    {ENC_R_CW_NEXT,    ENC_R_START,   ENC_R_CW_FIN,  ENC_R_START | ENC_DIR_CW},
    {ENC_R_CW_NEXT,    ENC_R_CW_BEG,  ENC_R_START,   ENC_R_START},
    {ENC_R_CW_NEXT,    ENC_R_CW_BEG,  ENC_R_CW_FIN,  ENC_R_START},
    {ENC_R_CCW_NEXT,   ENC_R_START,   ENC_R_CCW_BEG, ENC_R_START},
    {ENC_R_CCW_NEXT,   ENC_R_CCW_FIN, ENC_R_START,   ENC_R_START | ENC_DIR_CCW},
    {ENC_R_CCW_NEXT,   ENC_R_CCW_FIN, ENC_R_CCW_BEG, ENC_R_START},
};
uint8_t encoderRotState = ENC_R_START;
bool encoderDiagEnabled = false;
// Tunable from the serial debug console ('[' / ']') while diagnosing encoder issues;
// normally 1ms is plenty since mechanical detents are far slower than that.
unsigned long encoderReadIntervalMs = 1;
const unsigned long MOTOR_UPDATE_INTERVAL_US = 500;
const unsigned long MIN_SWEEP_STEP_INTERVAL_US = 100;
const unsigned long ENCODER_STEP_MIN_INTERVAL = 5;
const unsigned long ENCODER_BUTTON_DEBOUNCE = 120;
const uint8_t ENCODER_STABLE_SAMPLES = 1;
const unsigned long SENSOR_DEBOUNCE = 50;

// ============= DISPLAY / MENU =============
Adafruit_ST7789 tft = Adafruit_ST7789(LCD_CS, LCD_DC, LCD_RST);

// Status column narrowed slightly (was 232/88) so the textSize-2 Sweep Settings rows fit
// their full "label:value" text in the content area to its left.
const int STATUS_X = 244;
const int STATUS_W = 76;
const int CONTENT_W = STATUS_X - 2;

// Arm-position animation region (main menu + Sweep Settings). ~1/3 of the 172 px screen
// height, anchored at the bottom so the rows above get the taller remaining area.
const int ANIM_X = 4;
const int ANIM_W = STATUS_X - 8;
const int ANIM_H = 57;             // ≈ 1/3 of the screen height
const int ANIM_Y = 172 - ANIM_H;   // 115 — flush to the bottom edge

// Main menu — 4 items, no HOME
const char* menuItems[] = { "START/STOP", "Settings", "Setup", "About" };
const int   MENU_COUNT  = 4;
const int   BASIC_MENU_COUNT = 2;
const unsigned long LONG_PRESS_MS = 1000;  // hold time that counts as a "long" click
const unsigned long COMBO_GAP_MS  = 400;   // max gap from short-click release to the long press
volatile int menuIndex  = 0;
volatile bool advancedMenuUnlocked = false;

enum DisplayMode { DISP_MENU, DISP_SETTINGS, DISP_SETUP, DISP_ABOUT };
volatile DisplayMode displayMode = DISP_MENU;

volatile bool menuButtonPressed = false;
volatile bool longPressBack    = false;  // long press on a sub-screen = back to the menu
int menuDrawState = -1;  // -1 = full redraw needed

// Deferred short-click + short/long combo state (toggles advanced menu on the main screen)
bool          pendingShortClick     = false;
unsigned long shortClickReleaseTime = 0;

// Sweep Settings rows: Time, Wafer, Sweep type, Speed profile. No "< Back" row —
// a long press returns to the menu. Values are shown live in the side status bar.
const int SETTINGS_COUNT = 4;
volatile int  settingsIndex       = 0;
volatile bool editingSettings     = false;
volatile bool settingsNeedsRedraw = false;

// Setup rows: Park, Centre, Arm, Cycles, Current, Mstep, Invert, Debug. No "< Back" row —
// a long press returns to the menu.
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
int8_t pollEncoderRotation();
void encoderButtonIsr();
void initTMC2130();
void initDisplay();
uint32_t settingsChecksum(const StoredSettings& settings);
bool isValidMicrostep(int value);
void loadSettings();
void saveSettings();
int waferDiameterMm();
int calculatedSweepDegX10();
int travelSweepDegX10();
const char* sweepTypeLabel();
const char* sweepProfileLabel();
bool armOverWafer();
int sweepLeftSteps();
int sweepRightSteps();
int sweepBackSteps();
int sweepForwardSteps();
double sweepProfileFactor(int currentSteps, int startSteps, int targetSteps);
double sweepLegFactorSum(int startSteps, int targetSteps);
unsigned long minimumSweepTimeMs();
bool enforceMinimumSweepTime(bool persist);
void resetSweepTiming();
bool sweepStepDue(unsigned long nowUs, int currentSteps, int targetSteps);
int degX10ToSteps(int degX10);
int stepsToDegX10(int steps);
void printDegX10(int degX10);
int encoderAcceleration(unsigned long intervalMs);
void readSensors();
void readEncoder();
void updateStateMachine();
void handleState();
int motorShaftRevSteps();
int degreesToSteps(int degrees);
int stepsToDegrees(int steps);
void motorStep(int direction);
void motorSetEnable(bool enable);
void motorMoveTo(int target);
void motorMoveToBlocking(int target, unsigned int stepDelayUs);
void handleMenuSelect();
void exitToMenu();
void adjustSettingsValue(int delta);
void adjustSetupValue(int delta);
void applyDriverSettings();
void setDriverParkHold(bool parkHold);
void pollDriverStatus();
void handleSerialDebug();
void dumpDriverStatus();
uint32_t rawTmcRead(uint8_t address);
void debugStepBurst(int direction, int steps, unsigned int stepDelayUs);
void updateDisplay();
const char* stateLabel(SystemState state);
void drawStatusColumn(SystemState state, int posDeg, bool forceRedraw);
void drawIcon(int id, int x, int y, uint16_t color);
void drawMenuRow(int i, int y, bool selected);
void drawMenu();
void drawArmAnim(bool fullRedraw, int posDegX10);
void drawCircleClipped(int cx, int cy, int r, int yTop, int yBot, uint16_t color);
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

    loadSettings();
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

    handleSerialDebug();

    if (now - lastEncoderRead >= encoderReadIntervalMs) {
        readEncoder();
        lastEncoderRead = now;
    }

    if (menuButtonPressed) {
        menuButtonPressed = false;
        handleMenuSelect();
    }

    if (longPressBack) {
        longPressBack = false;
        exitToMenu();
    }

    readSensors();
    updateStateMachine();
    handleState();
    pollDriverStatus();

    // Debounced auto-save: commit pending Setup/Settings changes once the encoder has
    // been quiet for SETTINGS_SAVE_DELAY (coalesces a burst of steps into one write).
    // Re-read millis() here rather than reusing `now`: markSettingsDirty() above (via
    // readEncoder) can set lastSettingsChange to a value >= now within this same
    // iteration, and now - lastSettingsChange would then underflow and fire every step.
    if (settingsDirty && millis() - lastSettingsChange >= SETTINGS_SAVE_DELAY) {
        saveSettings();
    }
}

// ============= HARDWARE INIT =============
void initHardware() {
    pinMode(TMC_STEP, OUTPUT);
    pinMode(TMC_DIR,  OUTPUT);
    pinMode(TMC_EN,   OUTPUT);
    pinMode(TMC_CS,   OUTPUT);
    digitalWrite(TMC_EN, HIGH);
    digitalWrite(TMC_CS, HIGH);

    pinMode(LIMIT_SWITCH, INPUT_PULLUP);
    pinMode(SPRAY_VALVE,  INPUT_PULLDOWN);
    pinMode(FLOW_SENSOR,  INPUT_PULLDOWN);

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
    SPI.setTX(LCD_MOSI);
    SPI.setSCK(LCD_SCK);
    SPI.begin();

    SPI1.setRX(TMC_MISO);
    SPI1.setTX(TMC_MOSI);
    SPI1.setSCK(TMC_SCK);
    SPI1.begin();

    mutex_init(&spi_mutex);
    Serial.printf("SPI0 LCD initialized: SCK=%d MOSI=%d\n", LCD_SCK, LCD_MOSI);
    Serial.printf("SPI1 TMC initialized: SCK=%d MOSI=%d MISO=%d\n", TMC_SCK, TMC_MOSI, TMC_MISO);
}

void initEncoder() {
    pinMode(ENC_A,  INPUT_PULLUP);
    pinMode(ENC_B,  INPUT_PULLUP);
    pinMode(ENC_SW, INPUT_PULLUP);
    encoderRotState = ENC_R_START;
    encoderButtonRawState = digitalRead(ENC_SW);
    encoderButtonEdgePending = false;
    attachInterrupt(digitalPinToInterrupt(ENC_SW), encoderButtonIsr, CHANGE);
    Serial.printf("Encoder initialized: CLK=%d DT=%d SW=%d (polled rotation, interrupt button)\n", ENC_A, ENC_B, ENC_SW);
}

void initTMC2130() {
    driver.setSPISpeed(1000000);
    driver.begin();
    driver.toff(5);
    driver.rms_current(driverCurrent, DRIVER_RUN_HOLD_MULTIPLIER);
    driver.microsteps(driverMicrosteps);
    driver.iholddelay(15);
    driver.TPOWERDOWN(20);
    driver.intpol(true);
    driver.en_pwm_mode(true);
    driver.pwm_autoscale(true);
    Serial.printf("TMC2130 configured: %d mA, run hold %.0f%%, park hold %.0f%%, %dx microsteps, interpolation, StealthChop\n",
                  driverCurrent, DRIVER_RUN_HOLD_MULTIPLIER * 100.0f,
                  DRIVER_PARK_HOLD_MULTIPLIER * 100.0f, driverMicrosteps);
    dumpDriverStatus();
}

void initDisplay() {
    tft.init(172, 320);
    tft.setSPISpeed(20000000);
    tft.setRotation(1);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextWrap(false);
    tft.cp437(true);   // CP437 glyphs so the sweep-type arrow (0x1D) and bullet (0x07) render
    Serial.println("Display initialized (GMT147SPI 1.47\" 172x320)");
}

// ============= PERSISTENT SETTINGS =============
uint32_t settingsChecksum(const StoredSettings& settings) {
    const uint8_t* bytes = (const uint8_t*)&settings;
    size_t len = sizeof(StoredSettings) - sizeof(settings.checksum);
    uint32_t hash = 2166136261UL;
    for (size_t i = 0; i < len; i++) {
        hash ^= bytes[i];
        hash *= 16777619UL;
    }
    return hash;
}

bool isValidMicrostep(int value) {
    for (int i = 0; i < MICROSTEP_COUNT; i++) {
        if (MICROSTEP_TABLE[i] == value) return true;
    }
    return false;
}

void loadSettings() {
    EEPROM.begin(EEPROM_BYTES);

    StoredSettings stored;
    EEPROM.get(0, stored);

    bool validMagic = stored.magic == SETTINGS_MAGIC;
    bool validVersion = stored.version > 0 && stored.version <= SETTINGS_VERSION;
    bool validSize = stored.size == sizeof(StoredSettings);
    bool validChecksum = stored.checksum == settingsChecksum(stored);
    bool validMicrosteps = isValidMicrostep(stored.driverMicrosteps);
    bool valid = validMagic && validVersion && validSize && validChecksum && validMicrosteps;

    if (!valid) {
        Serial.printf("Settings: using defaults (magic=%d version=%u size=%u checksum=%d microsteps=%d)\n",
                      validMagic ? 1 : 0,
                      stored.version,
                      stored.size,
                      validChecksum ? 1 : 0,
                      validMicrosteps ? 1 : 0);
        saveSettings();
        return;
    }

    PARK_DEG_X10 = constrain((int)stored.parkDegX10, 10, 1800);
    CENTER_DEG_X10 = constrain((int)stored.centerDegX10, 10, 1800);
    ARM_LENGTH_MM = constrain((int)stored.armLengthMm, 50, 1000);
    SWEEP_TIME_MS = (unsigned long)constrain((long)stored.sweepTimeMs, 500L, 60000L);
    OSCILLATION_CYCLES = (unsigned long)constrain((long)stored.oscillationCycles, 0L, 100L);
    sampleIndex = constrain((int)stored.sampleIndex, 0, SAMPLE_COUNT - 1);
    sweepType = constrain((int)stored.sweepType, 0, 1);
    sweepProfile = constrain((int)stored.sweepProfile, 0, 2);
    driverCurrent = constrain((int)stored.driverCurrent, 100, 1500);
    driverMicrosteps = stored.driverMicrosteps;
    motorDirectionInverted = stored.motorDirectionInverted != 0;
    SENSOR_INPUTS_ENABLED = stored.sensorInputsEnabled != 0;
    enforceMinimumSweepTime(true);

    Serial.printf("Settings: loaded from flash (v%u)\n", stored.version);
}

void saveSettings() {
    StoredSettings stored = {};
    stored.magic = SETTINGS_MAGIC;
    stored.version = SETTINGS_VERSION;
    stored.size = sizeof(StoredSettings);
    stored.parkDegX10 = PARK_DEG_X10;
    stored.centerDegX10 = CENTER_DEG_X10;
    stored.armLengthMm = ARM_LENGTH_MM;
    stored.sweepTimeMs = SWEEP_TIME_MS;
    stored.oscillationCycles = OSCILLATION_CYCLES;
    stored.sampleIndex = sampleIndex;
    stored.sweepType = sweepType;
    stored.sweepProfile = sweepProfile;
    stored.driverCurrent = driverCurrent;
    stored.driverMicrosteps = driverMicrosteps;
    stored.motorDirectionInverted = motorDirectionInverted ? 1 : 0;
    stored.sensorInputsEnabled = SENSOR_INPUTS_ENABLED ? 1 : 0;
    stored.checksum = settingsChecksum(stored);

    EEPROM.put(0, stored);
    if (EEPROM.commit()) {
        Serial.println("Settings: saved to flash");
    } else {
        Serial.println("Settings: save failed");
    }
    settingsDirty = false;
}

// Mark settings changed; the debounced auto-save in loop() commits them once the
// encoder has been quiet for SETTINGS_SAVE_DELAY (avoids a flash write per step).
void markSettingsDirty() {
    settingsDirty = true;
    lastSettingsChange = millis();
}

int waferDiameterMm() {
    return SAMPLE_TABLE[sampleIndex];
}

int calculatedSweepDegX10() {
    float ratio = ((float)waferDiameterMm() * 0.5f) / (float)ARM_LENGTH_MM;
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    float sweep = 2.0f * asinf(ratio) * 180.0f / PI;
    return constrain((int)(sweep * 10.0f + 0.5f), 10, 1800);
}

// Angle the arm actually travels: full edge-to-edge for Back-Front,
// half (edge-to-centre) for Back-Centre.
int travelSweepDegX10() {
    int full = calculatedSweepDegX10();
    return sweepType == SWEEP_PATH_BACK_CENTER ? (full + 1) / 2 : full;
}

// Display labels for the sweep type / speed profile (shared by the Settings rows and
// the status bar). The sweep-type strings use CP437 0x1D (↔) and 0x07 (•) glyphs.
const char* sweepTypeLabel() {
    return sweepType == SWEEP_PATH_BACK_CENTER ? "Edge\x1D(\x07)" : "Edge\x1D" "Edge";
}
const char* sweepProfileLabel() {
    if (sweepProfile == SWEEP_PROFILE_LINEAR)   return "Sawtooth";
    if (sweepProfile == SWEEP_PROFILE_HARMONIC) return "Sine";
    return "Reciprocal";
}

int sweepLeftSteps() {
    int halfSweepDegX10 = (calculatedSweepDegX10() + 1) / 2;
    return degX10ToSteps(constrain(CENTER_DEG_X10 - halfSweepDegX10, 10, 1800));
}

int sweepRightSteps() {
    int halfSweepDegX10 = (calculatedSweepDegX10() + 1) / 2;
    return degX10ToSteps(constrain(CENTER_DEG_X10 + halfSweepDegX10, 10, 1800));
}

int sweepBackSteps() {
    return sweepLeftSteps();
}

int sweepForwardSteps() {
    return sweepType == SWEEP_PATH_BACK_CENTER ? degX10ToSteps(CENTER_DEG_X10) : sweepRightSteps();
}

double sweepEndpointRampFactor(int currentSteps, int startSteps, int targetSteps) {
    int sweepSteps = abs(targetSteps - startSteps);
    int rampSteps = degX10ToSteps(SWEEP_ENDPOINT_RAMP_DEG_X10);
    if (sweepSteps <= 0 || rampSteps <= 0) return 1.0;

    rampSteps = min(rampSteps, max(1, sweepSteps / 2));
    int progressSteps = abs(currentSteps - startSteps);
    int edgeDistance = min(progressSteps, max(0, sweepSteps - progressSteps));
    if (edgeDistance >= rampSteps) return 1.0;

    double x = (double)edgeDistance / (double)rampSteps;
    double smooth = x * x * (3.0 - 2.0 * x);
    return 1.0 + (SWEEP_ENDPOINT_SLOW_FACTOR - 1.0) * (1.0 - smooth);
}

double sweepProfileFactor(int currentSteps, int startSteps, int targetSteps) {
    int sweepSteps = abs(targetSteps - startSteps);
    if (sweepSteps <= 0) return 1.0f;

    if (sweepProfile == SWEEP_PROFILE_HARMONIC) {
        // Sine: speed is highest at the wafer centre and falls off toward the edge,
        // following a sine curve (gentler than Reciprocal's 1/distance falloff).
        // Keyed off |position - centre|, so it behaves the same for either sweep type.
        int centerSteps = degX10ToSteps(CENTER_DEG_X10);
        int maxDistance = max(abs(sweepBackSteps() - centerSteps), abs(sweepForwardSteps() - centerSteps));
        if (maxDistance > 0) {
            double distance = (double)abs(currentSteps - centerSteps) / (double)maxDistance;  // 0 centre … 1 edge
            if (distance > 1.0) distance = 1.0;
            double velocity = cos(PI / 2.0 * distance);  // 1 at centre … 0 at edge
            if (velocity < 0.20) velocity = 0.20;
            return (1.0 / velocity) * sweepEndpointRampFactor(currentSteps, startSteps, targetSteps);
        }
    }

    if (sweepProfile == SWEEP_PROFILE_INVDIST) {
        // Reciprocal: speed is highest at the wafer centre and falls off toward the edge,
        // proportional to distance from the centre, so the step interval (dwell time)
        // is the reciprocal of the remaining distance to the edge, growing toward it.
        // This keeps dwell-time/area roughly constant as the wafer spins underneath:
        // a point at radius r sweeps past the arm at a rate ∝ r, so the arm must
        // linger longer at larger r to deposit the same dose per unit area.
        // Keyed off |position − centre|, so it behaves the same for either sweep type.
        int centerSteps = degX10ToSteps(CENTER_DEG_X10);
        int maxDistance = max(abs(sweepBackSteps() - centerSteps), abs(sweepForwardSteps() - centerSteps));
        if (maxDistance > 0) {
            double distance = (double)abs(currentSteps - centerSteps) / (double)maxDistance;  // 0 centre … 1 edge
            if (distance > 1.0) distance = 1.0;
            double edgeDistance = 1.0 - distance;     // 1 centre … 0 edge
            if (edgeDistance < 0.10) edgeDistance = 0.10;   // clamp so the arm still moves at the edge
            return (1.0 / edgeDistance) * sweepEndpointRampFactor(currentSteps, startSteps, targetSteps);
        }
    }

    return sweepEndpointRampFactor(currentSteps, startSteps, targetSteps);
}

double sweepLegFactorSum(int startSteps, int targetSteps) {
    int sweepSteps = abs(targetSteps - startSteps);
    if (sweepSteps <= 0) return 1.0f;

    int direction = targetSteps > startSteps ? 1 : -1;
    double sum = 0.0;
    for (int i = 0; i < sweepSteps; i++) {
        sum += sweepProfileFactor(startSteps + i * direction, startSteps, targetSteps);
    }
    return sum > 1.0 ? sum : 1.0;
}

unsigned long minimumSweepTimeMs() {
    int backSteps = sweepBackSteps();
    int forwardSteps = sweepForwardSteps();
    if (backSteps == forwardSteps) return 500UL;

    double forwardSum = sweepLegFactorSum(backSteps, forwardSteps);
    double returnSum = sweepLegFactorSum(forwardSteps, backSteps);
    double fullCycleFactorSum = forwardSum + returnSum;
    double minFactor = 1.0;   // smallest profile factor (fastest step) is ~1.0 for every profile

    double minTimeMs =
        (fullCycleFactorSum * (double)MIN_SWEEP_STEP_INTERVAL_US / minFactor) / 1000.0;
    unsigned long roundedMs = (unsigned long)(minTimeMs + 0.999);
    return constrain(roundedMs, 500UL, 60000UL);
}

bool enforceMinimumSweepTime(bool persist) {
    unsigned long minimumMs = minimumSweepTimeMs();
    if (SWEEP_TIME_MS >= minimumMs) return false;

    SWEEP_TIME_MS = minimumMs;
    Serial.printf("Settings: Time raised to %.3fs minimum for this sweep/profile\n",
                  SWEEP_TIME_MS / 1000.0f);
    resetSweepTiming();
    if (persist) {
        saveSettings();
    } else {
        markSettingsDirty();
    }
    return true;
}

void resetSweepTiming() {
    sweepTimingActive = false;
}

static void beginSweepLegTiming(unsigned long nowUs, int startSteps, int targetSteps) {
    double fullCycleFactorSum = sweepLegFactorSum(sweepBackSteps(), sweepForwardSteps()) +
                                sweepLegFactorSum(sweepForwardSteps(), sweepBackSteps());
    if (fullCycleFactorSum < 1.0) fullCycleFactorSum = 1.0;

    sweepTimingActive = true;
    sweepTimingStartSteps = startSteps;
    sweepTimingTargetSteps = targetSteps;
    sweepTimingStartMicros = nowUs;
    sweepTimingCompletedFactorSum = 0.0;
    sweepTimingBaseUs = ((double)SWEEP_TIME_MS * 1000.0) / fullCycleFactorSum;
}

bool sweepStepDue(unsigned long nowUs, int currentSteps, int targetSteps) {
    if (currentSteps == targetSteps) return true;

    bool targetChanged = !sweepTimingActive || targetSteps != sweepTimingTargetSteps;
    if (targetChanged) {
        // Start each leg's timing fresh from "now" rather than chaining off the
        // previous leg's scheduled end time: any rounding slack between the
        // scheduled and actual duration of the previous leg would otherwise carry
        // over as a time "debt", making the new leg's early steps already overdue
        // and firing in a rapid burst right after the direction reversal.
        beginSweepLegTiming(nowUs, currentSteps, targetSteps);
    }

    int legDirection = sweepTimingTargetSteps > sweepTimingStartSteps ? 1 : -1;
    int completedSteps = abs(currentSteps - sweepTimingStartSteps);
    int legSteps = abs(sweepTimingTargetSteps - sweepTimingStartSteps);
    int expectedSteps = sweepTimingStartSteps + completedSteps * legDirection;
    if (completedSteps >= legSteps || currentSteps != expectedSteps) {
        beginSweepLegTiming(nowUs, currentSteps, targetSteps);
    }

    double nextFactor = sweepProfileFactor(currentSteps, sweepTimingStartSteps, sweepTimingTargetSteps);
    unsigned long dueUs =
        sweepTimingStartMicros +
        (unsigned long)(sweepTimingBaseUs * (sweepTimingCompletedFactorSum + nextFactor) + 0.5);

    if ((long)(nowUs - dueUs) < 0) return false;

    sweepTimingCompletedFactorSum += nextFactor;
    return true;
}

// ============= SENSORS =============
void readSensors() {
    static bool          lastLimitRaw    = false;
    static unsigned long lastLimitChange = 0;
    bool raw = (digitalRead(LIMIT_SWITCH) == LOW);
    if (raw != lastLimitRaw) { lastLimitRaw = raw; lastLimitChange = millis(); }
    if (millis() - lastLimitChange >= 20) limitSwitchPressed = lastLimitRaw;

    if (!SENSOR_INPUTS_ENABLED) {
        sprayActive = false;
        flowDetected = false;
        return;
    }

    static bool          lastSprayRaw    = false;
    static bool          lastFlowRaw     = false;
    static unsigned long lastSprayChange = 0;
    static unsigned long lastFlowChange  = 0;
    unsigned long now = millis();

    bool sprayRaw = (digitalRead(SPRAY_VALVE) == HIGH);
    bool flowRaw  = (digitalRead(FLOW_SENSOR) == HIGH);

    if (sprayRaw != lastSprayRaw) {
        lastSprayRaw = sprayRaw;
        lastSprayChange = now;
    }
    if (flowRaw != lastFlowRaw) {
        lastFlowRaw = flowRaw;
        lastFlowChange = now;
    }

    if (now - lastSprayChange >= SENSOR_DEBOUNCE) sprayActive = lastSprayRaw;
    if (now - lastFlowChange >= SENSOR_DEBOUNCE)  flowDetected = lastFlowRaw;
}

// ============= ENCODER =============
// Polled full quadrature decode (Buxton state table): readEncoder() is called every
// encoderReadIntervalMs from the main loop and samples both A/B pins directly, so there
// is no reliance on GPIO interrupts (which the RP2040 can coalesce on simultaneous edges)
// and no fixed debounce window. Invalid/bouncy pin sequences simply route back to
// ENC_R_START without emitting a step, while a full valid 4-phase rotation in either
// direction emits exactly one detent.
int8_t pollEncoderRotation() {
    uint8_t pinState = (digitalRead(ENC_B) << 1) | digitalRead(ENC_A);
    uint8_t prevState = encoderRotState & 0x0F;
    encoderRotState = ENC_TTABLE[prevState][pinState];
    uint8_t dir = encoderRotState & 0x30;
    int8_t result = 0;
    // Inverted vs. the table's nominal CW/CCW so increasing values correspond to the
    // knob's expected turn direction on this hardware.
    if (dir == ENC_DIR_CW)  result = -1;
    if (dir == ENC_DIR_CCW) result =  1;

    if (encoderDiagEnabled && (encoderRotState & 0x0F) != prevState) {
        Serial.printf("ENC: A=%d B=%d state %u->%u result=%d\n",
                      pinState & 1, (pinState >> 1) & 1, prevState, encoderRotState & 0x0F, result);
    }
    return result;
}

void encoderButtonIsr() {
    encoderButtonRawState = digitalRead(ENC_SW);
    encoderButtonEdgeMillis = millis();
    encoderButtonEdgePending = true;
}

void readEncoder() {
    static unsigned long lastStepTime     = 0;
    static int           pendingDetents   = 0;

    pendingDetents += pollEncoderRotation();

    if (pendingDetents != 0) {
        unsigned long now = millis();
        if (now - lastStepTime >= ENCODER_STEP_MIN_INTERVAL) {
            unsigned long stepInterval = now - lastStepTime;
            int d = pendingDetents > 0 ? 1 : -1;
            int steps = abs(pendingDetents);
            int accel = encoderAcceleration(stepInterval);
            int editDelta = d * steps * accel;
            if (encoderDiagEnabled) {
                Serial.printf("ENC step: pending=%d interval=%lums accel=%d editDelta=%d\n",
                              pendingDetents, stepInterval, accel, editDelta);
            }
            switch (displayMode) {
                case DISP_MENU:
                    {
                        int visibleCount = advancedMenuUnlocked ? MENU_COUNT : BASIC_MENU_COUNT;
                        menuIndex = (menuIndex + d * steps + visibleCount * steps) % visibleCount;
                    }
                    break;
                case DISP_SETTINGS:
                    if (editingSettings) adjustSettingsValue(editDelta);
                    else settingsIndex = (settingsIndex + d * steps + SETTINGS_COUNT * steps) % SETTINGS_COUNT;
                    break;
                case DISP_SETUP:
                    if (editingSetup) adjustSetupValue(editDelta);
                    else setupIndex = (setupIndex + d * steps + SETUP_COUNT * steps) % SETUP_COUNT;
                    break;
                case DISP_ABOUT:
                    break;
            }
            lastStepTime = now;
            pendingDetents = 0;
        }
    }

    static bool          stableBtn    = HIGH;
    static unsigned long lastBtnTime  = 0;
    static unsigned long pressStart   = 0;
    static bool          longFired    = false;

    bool edgePending = false;
    bool rawBtn = HIGH;
    unsigned long edgeTime = 0;
    noInterrupts();
    edgePending = encoderButtonEdgePending;
    if (edgePending) {
        rawBtn = encoderButtonRawState;
        edgeTime = encoderButtonEdgeMillis;
        encoderButtonEdgePending = false;
    }
    interrupts();

    if (edgePending && rawBtn != stableBtn && edgeTime - lastBtnTime >= ENCODER_BUTTON_DEBOUNCE) {
        bool oldStableBtn = stableBtn;
        stableBtn   = rawBtn;
        lastBtnTime = edgeTime;

        if (oldStableBtn == HIGH && stableBtn == LOW) {
            // ---- pressed (falling edge) ----
            pressStart = edgeTime;
            longFired  = false;
        } else {
            // ---- released (rising edge) ----
            if (!longFired) {
                if (displayMode == DISP_MENU) {
                    // defer: a long press may follow to toggle the advanced menu
                    pendingShortClick     = true;
                    shortClickReleaseTime = edgeTime;
                } else {
                    menuButtonPressed = true;
                }
            }
        }
    }

    unsigned long now = millis();

    // Long press while held: on the main menu a short-click-then-long-press toggles the
    // advanced menu; on a sub-screen a long press means "Back to menu".
    if (stableBtn == LOW && !longFired && now - pressStart >= LONG_PRESS_MS) {
        longFired = true;
        if (displayMode == DISP_MENU) {
            if (pendingShortClick && pressStart - shortClickReleaseTime <= COMBO_GAP_MS) {
                advancedMenuUnlocked = !advancedMenuUnlocked;
                if (!advancedMenuUnlocked && menuIndex >= BASIC_MENU_COUNT) menuIndex = 0;
                pendingShortClick = false;
                menuDrawState = -1;
                Serial.printf("Advanced menu %s\n", advancedMenuUnlocked ? "shown" : "hidden");
            }
            // a plain long press on the menu otherwise does nothing
        } else {
            longPressBack = true;
        }
    }

    // No long press followed in time -> execute the deferred short click
    if (pendingShortClick && stableBtn == HIGH && now - shortClickReleaseTime > COMBO_GAP_MS) {
        pendingShortClick = false;
        menuButtonPressed = true;
    }
}

// ============= STATE MACHINE =============
void updateStateMachine() {
    unsigned long now = millis();
    int parkSteps = degX10ToSteps(PARK_DEG_X10);

    // Driver fault: park first if actively sweeping (motor energised, position known),
    // otherwise latch ERROR straight away.
    if (faultLatched) {
        if (currentState == STATE_SPRAY_ACTIVE || currentState == STATE_OSCILLATING) {
            currentState = STATE_PARKED; lastStateChange = now;
            Serial.println("FAULT → PARKED (will disable)");
        } else if (currentState == STATE_WAITING_SPRAY) {
            needsHoming = true;
            currentState = STATE_ERROR; lastStateChange = now;
            Serial.println("FAULT → ERROR (motor disabled)");
        }
    }

    switch (currentState) {
        case STATE_IDLE:
            if (faultLatched) {
                needsHoming = true;
                currentState = STATE_ERROR; lastStateChange = now;
                Serial.println("→ ERROR (driver fault)");
            } else if (SENSOR_INPUTS_ENABLED && sprayActive) {
                homingStartPos = motorPosition;
                currentState = STATE_HOMING; lastStateChange = now;
                Serial.println("→ HOMING");
            }
            break;

        case STATE_HOMING:
            // Safety: endstop must be found within HOMING_MAX_DEG_X10 of real travel.
            if (!limitSwitchPressed &&
                (homingStartPos - motorPosition) >= degX10ToSteps(HOMING_MAX_DEG_X10)) {
                motorSetEnable(false);
                needsHoming = true;
                currentState = STATE_ERROR; lastStateChange = now;
                Serial.println("→ ERROR (home endstop not found within 70°)");
                break;
            }
            if (limitSwitchPressed) {
                motorPosition = 0;
                needsHoming = false;                    // position re-established
                if (faultLatched) {
                    needsHoming = true;
                    motorSetEnable(false);
                    currentState = STATE_ERROR; lastStateChange = now;
                    Serial.println("→ ERROR (homed, motor disabled)");
                } else if (homingToStop) {
                    homingToStop = false;
                    needsHoming = true;
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
            // Park is reached at the park setpoint (≤ PARK_DEG_X10, a small angle).
            if (motorPosition == parkSteps) {
                if (faultLatched) {
                    needsHoming = true;
                    currentState = STATE_ERROR; lastStateChange = now;
                    Serial.println("→ ERROR (parked, motor disabled)");
                } else if (stopRequested) {
                    stopRequested = false; needsHoming = true;
                    currentState = STATE_IDLE; lastStateChange = now;
                    Serial.println("→ IDLE (parked, motor disabled)");
                } else if (!SENSOR_INPUTS_ENABLED && sensorBypassCycleArmed) {
                    oscillationCount = 0; oscillationDir = -1; oscillationStepCount = 0;
                    currentState = STATE_SPRAY_ACTIVE; lastStateChange = now;
                    Serial.println("→ SPRAY_ACTIVE (sensor bypass)");
                } else if (sprayActive && !flowDetected) {
                    currentState = STATE_WAITING_SPRAY; lastStateChange = now;
                    Serial.println("→ WAITING_SPRAY");
                } else if (sprayActive && flowDetected) {
                    oscillationCount = 0; oscillationDir = -1; oscillationStepCount = 0;
                    currentState = STATE_SPRAY_ACTIVE; lastStateChange = now;
                    Serial.println("→ SPRAY_ACTIVE");
                } else {
                    // Parked with nothing pending: disable the motor and idle.
                    needsHoming = true;
                    currentState = STATE_IDLE; lastStateChange = now;
                    Serial.println("→ IDLE (parked, motor disabled)");
                }
            }
            break;

        case STATE_WAITING_SPRAY:
            if (!SENSOR_INPUTS_ENABLED && sensorBypassCycleArmed) {
                oscillationCount = 0; oscillationDir = -1; oscillationStepCount = 0;
                currentState = STATE_SPRAY_ACTIVE; lastStateChange = now;
                Serial.println("→ SPRAY_ACTIVE (sensor bypass)");
            } else if (!sprayActive) {
                currentState = STATE_PARKED; lastStateChange = now;
                Serial.println("→ PARKED (spray lost)");
            } else if (flowDetected) {
                oscillationCount = 0; oscillationDir = -1; oscillationStepCount = 0;
                currentState = STATE_SPRAY_ACTIVE; lastStateChange = now;
                Serial.println("→ SPRAY_ACTIVE");
            }
            break;

        case STATE_SPRAY_ACTIVE:
            if (SENSOR_INPUTS_ENABLED && (!sprayActive || !flowDetected)) {
                currentState = STATE_PARKED; lastStateChange = now;
                Serial.println("→ PARKED (spray or flow lost)");
            } else if (now - lastStateChange >= SPRAY_ACTIVE_WAIT &&
                       motorPosition == sweepBackSteps()) {
                oscillationDir = 1;
                oscillationCount = 0;
                oscillationStepCount = 0;
                currentState = STATE_OSCILLATING; lastStateChange = now;
                Serial.println("→ OSCILLATING");
            }
            break;

        case STATE_OSCILLATING:
            if (SENSOR_INPUTS_ENABLED && (!sprayActive || !flowDetected)) {
                currentState = STATE_PARKED; lastStateChange = now;
                Serial.println("→ PARKED (spray or flow lost)");
            } else if (OSCILLATION_CYCLES > 0 && oscillationCount >= OSCILLATION_CYCLES) {
                sensorBypassCycleArmed = false;
                currentState = STATE_PARKED; lastStateChange = now;
                Serial.println("→ PARKED (cleaning cycle complete)");
            }
            break;

        case STATE_ERROR:
            break;
    }
}

// ============= STATE ACTIONS =============
void handleState() {
    unsigned long now = millis();
    unsigned long stepNow = micros();

    if (displayMode == DISP_SETUP && editingSetup && setupIndex == 1) {
        setDriverParkHold(false);
        motorSetEnable(true);
        setLED(LED_GREEN, true); setLED(LED_YELLOW, false);
        setFan(FAN_OFF); setUltrasonic(false);
        return;
    }

    if (currentState != STATE_OSCILLATING) resetSweepTiming();

    switch (currentState) {
        case STATE_IDLE:
            setDriverParkHold(false);
            motorSetEnable(debugMotorHold);
            setLED(LED_GREEN, false); setLED(LED_YELLOW, false);
            setFan(FAN_OFF); setUltrasonic(false);
            break;

        case STATE_HOMING:
            setDriverParkHold(false);
            motorSetEnable(true);
            setLED(LED_GREEN, false); setLED(LED_YELLOW, true);
            if (stepNow - lastMotorStepMicros >= MOTOR_UPDATE_INTERVAL_US) {
                motorStep(-1); lastMotorStepMicros = stepNow;
            }
            break;

        case STATE_PARKED:
            motorSetEnable(true);
            if (motorPosition != degX10ToSteps(PARK_DEG_X10)) {
                setDriverParkHold(false);
                if (stepNow - lastMotorStepMicros >= MOTOR_UPDATE_INTERVAL_US) {
                    motorMoveTo(degX10ToSteps(PARK_DEG_X10)); lastMotorStepMicros = stepNow;
                }
            } else {
                setDriverParkHold(true);
            }
            setLED(LED_GREEN, true); setLED(LED_YELLOW, false);
            setFan(FAN_OFF); setUltrasonic(false);
            break;

        case STATE_WAITING_SPRAY:
            setDriverParkHold(false);
            motorSetEnable(false);
            setLED(LED_GREEN, false); setLED(LED_YELLOW, true);
            setFan(FAN_WAITING); setUltrasonic(false);
            break;

        case STATE_SPRAY_ACTIVE:
            setDriverParkHold(false);
            setFan(FAN_FULL); setUltrasonic(armOverWafer());   // generator only over the wafer
            setLED(LED_GREEN, true); setLED(LED_YELLOW, false);
            motorSetEnable(true);
            if (stepNow - lastMotorStepMicros >= MOTOR_UPDATE_INTERVAL_US) {
                motorMoveTo(sweepBackSteps()); lastMotorStepMicros = stepNow;
            }
            break;

        case STATE_OSCILLATING:
            setDriverParkHold(false);
            motorSetEnable(true);
            setFan(FAN_FULL); setUltrasonic(armOverWafer());   // generator only over the wafer
            setLED(LED_GREEN, true); setLED(LED_YELLOW, false);
            {
                int target = oscillationDir > 0 ? sweepForwardSteps() : sweepBackSteps();
                if (!sweepStepDue(stepNow, motorPosition, target)) break;
                motorMoveTo(target);
                lastMotorStepMicros = stepNow;
                if (motorPosition == target) {
                    if (oscillationDir < 0) {
                        oscillationCount++;
                    }
                    oscillationDir = -oscillationDir;
                }
            }
            break;

        case STATE_ERROR:
            setDriverParkHold(false);
            motorSetEnable(false); setFan(FAN_OFF); setUltrasonic(false);
            setLED(LED_GREEN, false);
            setLED(LED_YELLOW, (millis() % 1000) < 500);
            break;
    }
}

// ============= MENU ACTIONS =============
void handleMenuSelect() {
    int visibleCount = advancedMenuUnlocked ? MENU_COUNT : BASIC_MENU_COUNT;
    if (displayMode == DISP_MENU && menuIndex >= visibleCount) {
        menuIndex = 0;
        menuDrawState = -1;
        return;
    }

    // Sweep Settings / Setup: a click toggles edit on the selected row; there is no
    // "< Back" row — a long press of the encoder returns to the menu (see exitToMenu()).
    if (displayMode == DISP_SETTINGS) {
        if (editingSettings) { editingSettings = false; saveSettings(); }
        else                 { editingSettings = true; }
        return;
    }

    if (displayMode == DISP_SETUP) {
        if (editingSetup) { editingSetup = false; saveSettings(); }
        else {
            if (setupIndex == 1) {
                CENTER_DEG_X10 = constrain(stepsToDegX10(motorPosition), 10, 1800);
                setupNeedsRedraw = true;
                Serial.printf("Setup: Centre live jog starts at %.1f deg\n", CENTER_DEG_X10 * 0.1f);
            }
            editingSetup = true;
        }
        return;
    }

    if (displayMode == DISP_ABOUT) {
        displayMode = DISP_MENU; menuDrawState = -1; return;
    }

    switch (menuIndex) {
        case 0:  // START / STOP
            if (currentState == STATE_IDLE || currentState == STATE_ERROR) {
                // START: always home first to re-establish position, then park + run.
                faultLatched = false; stopRequested = false; homingToStop = false;
                sensorBypassCycleArmed = !SENSOR_INPUTS_ENABLED;
                homingStartPos = motorPosition;
                currentState = STATE_HOMING; lastStateChange = millis();
                Serial.println("Menu: START → HOMING");
            } else if (currentState == STATE_PARKED && !needsHoming) {
                // Already parked with known position: start the cleaning cycle.
                if (!SENSOR_INPUTS_ENABLED) {
                    sensorBypassCycleArmed = true;
                    oscillationCount = 0; oscillationDir = -1; oscillationStepCount = 0;
                    currentState = STATE_SPRAY_ACTIVE; lastStateChange = millis();
                    Serial.println("Menu: START → SPRAY_ACTIVE (sensor bypass)");
                } else {
                    Serial.println("Menu: already parked");
                }
            } else {
                // STOP: park the arm, then disable the motor.
                homingToStop = false;
                sensorBypassCycleArmed = false;
                stopRequested = true;
                if (currentState != STATE_HOMING) {
                    currentState = STATE_PARKED; lastStateChange = millis();
                }
                Serial.println("Menu: STOP → PARK then disable");
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

// Long press on a sub-screen: save and return to the main menu (replaces the old
// "< Back" row). Setup also re-applies the driver settings on the way out.
void exitToMenu() {
    if (displayMode == DISP_SETTINGS) {
        editingSettings = false;
        saveSettings();
        settingsIndex = 0;
    } else if (displayMode == DISP_SETUP) {
        editingSetup = false;
        applyDriverSettings();
        saveSettings();
        setupIndex = 0;
    }
    displayMode = DISP_MENU;
    menuDrawState = -1;
    Serial.println("Long press → back to menu");
}

// Adjust Settings (sweep) parameter by encoder delta
void adjustSettingsValue(int delta) {
    int singleStep = delta > 0 ? 1 : -1;
    switch (settingsIndex) {
        case 0:
            SWEEP_TIME_MS = (unsigned long)constrain((long)SWEEP_TIME_MS + delta * 1000L, 500L, 60000L);
            break;
        case 1:
            sampleIndex = (sampleIndex + singleStep + SAMPLE_COUNT) % SAMPLE_COUNT;
            break;
        case 2:
            sweepType = (sweepType + singleStep + 2) % 2;
            break;
        case 3:
            sweepProfile = (sweepProfile + singleStep + 3) % 3;
            break;
    }
    enforceMinimumSweepTime(false);
    markSettingsDirty();
}

// Adjust Setup (hardware) parameter by encoder delta
void adjustSetupValue(int delta) {
    switch (setupIndex) {
        case 0: PARK_DEG_X10 = constrain(PARK_DEG_X10 + delta, 10, 1800); break;
        case 1:
            CENTER_DEG_X10 = constrain(CENTER_DEG_X10 + delta, 10, 1800);
            motorMoveToBlocking(degX10ToSteps(CENTER_DEG_X10), MOTOR_UPDATE_INTERVAL_US);
            break;
        case 2: ARM_LENGTH_MM = constrain(ARM_LENGTH_MM + delta * 5, 50, 1000); break;
        case 3: OSCILLATION_CYCLES = (unsigned long)constrain(
                    (long)OSCILLATION_CYCLES + delta, 0L, 100L); break;
        case 4: driverCurrent = constrain(driverCurrent + delta * 50, 100, 1500); break;
        case 5: {
            int singleStep = delta > 0 ? 1 : -1;
            int idx = 4;
            for (int i = 0; i < MICROSTEP_COUNT; i++)
                if (MICROSTEP_TABLE[i] == driverMicrosteps) { idx = i; break; }
            idx = (idx + singleStep + MICROSTEP_COUNT) % MICROSTEP_COUNT;
            driverMicrosteps = MICROSTEP_TABLE[idx];
            break;
        }
        case 6:
            motorDirectionInverted = !motorDirectionInverted;
            break;
        case 7:
            SENSOR_INPUTS_ENABLED = !SENSOR_INPUTS_ENABLED;
            break;
    }
    enforceMinimumSweepTime(false);
    markSettingsDirty();
}

void applyDriverSettings() {
    mutex_enter_blocking(&spi_mutex);
    driver.rms_current(driverCurrent, driverParkHoldMode ? DRIVER_PARK_HOLD_MULTIPLIER : DRIVER_RUN_HOLD_MULTIPLIER);
    driver.microsteps(driverMicrosteps);
    driver.intpol(true);
    mutex_exit(&spi_mutex);
    Serial.printf("Driver applied: %d mA, hold %.0f%%, %dx microsteps\n",
                  driverCurrent,
                  (driverParkHoldMode ? DRIVER_PARK_HOLD_MULTIPLIER : DRIVER_RUN_HOLD_MULTIPLIER) * 100.0f,
                  driverMicrosteps);
}

void setDriverParkHold(bool parkHold) {
    if (driverParkHoldMode == parkHold) return;

    driverParkHoldMode = parkHold;
    mutex_enter_blocking(&spi_mutex);
    driver.rms_current(driverCurrent, parkHold ? DRIVER_PARK_HOLD_MULTIPLIER : DRIVER_RUN_HOLD_MULTIPLIER);
    mutex_exit(&spi_mutex);
    Serial.printf("Driver hold mode: %s hold %.0f%%\n",
                  parkHold ? "park" : "run",
                  (parkHold ? DRIVER_PARK_HOLD_MULTIPLIER : DRIVER_RUN_HOLD_MULTIPLIER) * 100.0f);
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

    // Failure detection: overtemperature, short-to-ground, or charge-pump undervoltage.
    bool ot   = (driverDrvStatus >> 25) & 1;                 // overtemp shutdown
    bool s2g  = (driverDrvStatus >> 27) & 1 || (driverDrvStatus >> 28) & 1;  // short to GND
    bool uvcp = (driverGStat >> 2) & 1;                      // charge-pump undervoltage
    if ((ot || s2g || uvcp) && !faultLatched &&
        currentState != STATE_IDLE && currentState != STATE_ERROR) {
        faultLatched = true;
        Serial.printf("FAULT: OT=%d S2G=%d UVCP=%d → park + disable\n", ot, s2g, uvcp);
    }
}

// ============= SERIAL DEBUG =============
void handleSerialDebug() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        switch (c) {
            case 'd':
            case 'D':
                dumpDriverStatus();
                break;
            case 'e':
            case 'E':
                debugMotorHold = true;
                motorSetEnable(true);
                Serial.println("DBG: motor enabled (EN=LOW)");
                break;
            case 'x':
            case 'X':
                debugMotorHold = false;
                motorSetEnable(false);
                Serial.println("DBG: motor disabled (EN=HIGH)");
                break;
            case '+':
                debugMotorHold = true;
                motorSetEnable(true);
                motorStep(1);
                Serial.printf("DBG: step + pos=%d\n", motorPosition);
                break;
            case '-':
                debugMotorHold = true;
                motorSetEnable(true);
                motorStep(-1);
                Serial.printf("DBG: step - pos=%d\n", motorPosition);
                break;
            case 'f':
            case 'F':
                debugStepBurst(1, 400, 1000);
                break;
            case 'b':
            case 'B':
                debugStepBurst(-1, 400, 1000);
                break;
            case 'r':
            case 'R': {
                int revSteps = motorShaftRevSteps();
                Serial.printf("DBG: one shaft revolution = %d microsteps at %dx\n",
                              revSteps, driverMicrosteps);
                debugStepBurst(1, revSteps, 150);
                break;
            }
            case 'k':
            case 'K':
                encoderDiagEnabled = !encoderDiagEnabled;
                Serial.printf("DBG: encoder diagnostics %s (read interval %lums)\n",
                              encoderDiagEnabled ? "ON" : "OFF", encoderReadIntervalMs);
                break;
            case '[':
                if (encoderReadIntervalMs > 1) encoderReadIntervalMs--;
                Serial.printf("DBG: encoder read interval = %lums\n", encoderReadIntervalMs);
                break;
            case ']':
                if (encoderReadIntervalMs < 20) encoderReadIntervalMs++;
                Serial.printf("DBG: encoder read interval = %lums\n", encoderReadIntervalMs);
                break;
            case '?':
                Serial.println("DBG commands: d=dump TMC, e=enable, x=disable, +=one forward, -=one back, f=400 forward, b=400 back, r=one shaft rev, k=toggle encoder diag, [/]=encoder read interval -/+");
                break;
        }
    }
}

void dumpDriverStatus() {
    uint32_t gstat = 0;
    uint32_t drvStatus = 0;
    uint32_t rawGstat = 0;
    uint32_t rawIoin = 0;
    uint32_t rawDrvStatus = 0;
    uint32_t rawChopconf = 0;

    mutex_enter_blocking(&spi_mutex);
    gstat = driver.GSTAT();
    drvStatus = driver.DRV_STATUS();
    mutex_exit(&spi_mutex);

    rawGstat = rawTmcRead(0x01);
    rawIoin = rawTmcRead(0x04);
    rawChopconf = rawTmcRead(0x6C);
    rawDrvStatus = rawTmcRead(0x6F);

    driverGStat = gstat;
    driverDrvStatus = drvStatus;

    Serial.println("=== TMC2130 DEBUG ===");
    Serial.printf("Pins: CS=%d SCK=%d MOSI=%d MISO=%d STEP=%d DIR=%d EN=%d\n",
                  TMC_CS, TMC_SCK, TMC_MOSI, TMC_MISO, TMC_STEP, TMC_DIR, TMC_EN);
    Serial.printf("EN pin: %s\n", digitalRead(TMC_EN) == LOW ? "LOW enabled" : "HIGH disabled");
    Serial.printf("GSTAT:     0x%08lX\n", (unsigned long)gstat);
    Serial.printf("DRV_STAT:  0x%08lX\n", (unsigned long)drvStatus);
    Serial.printf("RAW GSTAT: 0x%08lX\n", (unsigned long)rawGstat);
    Serial.printf("RAW IOIN:  0x%08lX version=0x%02lX\n",
                  (unsigned long)rawIoin, (unsigned long)((rawIoin >> 24) & 0xFF));
    Serial.printf("RAW CHOP:  0x%08lX toff=%lu mres=%lu\n",
                  (unsigned long)rawChopconf,
                  (unsigned long)(rawChopconf & 0x0F),
                  (unsigned long)((rawChopconf >> 24) & 0x0F));
    Serial.printf("RAW DRV:   0x%08lX\n", (unsigned long)rawDrvStatus);

    if (gstat == 0xFFFFFFFFUL || drvStatus == 0xFFFFFFFFUL ||
        rawGstat == 0xFFFFFFFFUL || rawIoin == 0xFFFFFFFFUL || rawDrvStatus == 0xFFFFFFFFUL) {
        Serial.println("SPI: suspicious all-ones read; check CS/MISO/SCK/MOSI wiring and driver VCC_IO.");
    } else if (gstat == 0 && drvStatus == 0 && rawGstat == 0 && rawIoin == 0 && rawDrvStatus == 0) {
        Serial.println("SPI: all-zero read; check driver power, CS wiring, and SPI mode/jumpers.");
    } else {
        Serial.println("SPI: non-zero register data received.");
    }

    Serial.printf("GSTAT bits: reset=%d drv_err=%d uv_cp=%d\n",
                  (int)(gstat & 0x01), (int)((gstat >> 1) & 0x01), (int)((gstat >> 2) & 0x01));
    Serial.printf("IOIN bits: step=%d dir=%d drv_enn=%d cfg5=%d cfg4=%d\n",
                  (int)(rawIoin & 0x01),
                  (int)((rawIoin >> 1) & 0x01),
                  (int)((rawIoin >> 4) & 0x01),
                  (int)((rawIoin >> 3) & 0x01),
                  (int)((rawIoin >> 2) & 0x01));
    Serial.printf("Config: irun=%d ihold=%d toff=%d microsteps=%d stealth=%d pwm_auto=%d\n",
                  driver.irun(), driver.ihold(), driver.toff(),
                  driver.microsteps(), driver.en_pwm_mode(), driver.pwm_autoscale());
    Serial.printf("DRV bits: stst=%d olb=%d ola=%d s2gb=%d s2ga=%d otpw=%d ot=%d cs_actual=%lu sg=%lu\n",
                  (int)((drvStatus >> 31) & 0x01),
                  (int)((drvStatus >> 30) & 0x01),
                  (int)((drvStatus >> 29) & 0x01),
                  (int)((drvStatus >> 28) & 0x01),
                  (int)((drvStatus >> 27) & 0x01),
                  (int)((drvStatus >> 26) & 0x01),
                  (int)((drvStatus >> 25) & 0x01),
                  (unsigned long)((drvStatus >> 16) & 0x1F),
                  (unsigned long)(drvStatus & 0x3FF));
    if ((drvStatus >> 29) & 0x03) {
        Serial.println("DRV: open-load on motor outputs; check VM power and both motor coil pairs.");
    }
    Serial.println("Commands: d=dump, e=enable, x=disable, +=one forward, -=one back, f=400 forward, b=400 back");
}

uint32_t rawTmcRead(uint8_t address) {
    uint32_t out = 0;

    mutex_enter_blocking(&spi_mutex);
    SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

    digitalWrite(LCD_CS, HIGH);
    digitalWrite(TMC_CS, LOW);
    SPI1.transfer(address & 0x7F);
    SPI1.transfer(0x00);
    SPI1.transfer(0x00);
    SPI1.transfer(0x00);
    SPI1.transfer(0x00);
    digitalWrite(TMC_CS, HIGH);
    delayMicroseconds(10);

    digitalWrite(TMC_CS, LOW);
    SPI1.transfer(address & 0x7F);
    out  = (uint32_t)SPI1.transfer(0x00) << 24;
    out |= (uint32_t)SPI1.transfer(0x00) << 16;
    out |= (uint32_t)SPI1.transfer(0x00) << 8;
    out |= (uint32_t)SPI1.transfer(0x00);
    digitalWrite(TMC_CS, HIGH);

    SPI1.endTransaction();
    mutex_exit(&spi_mutex);
    return out;
}

void debugStepBurst(int direction, int steps, unsigned int stepDelayUs) {
    debugMotorHold = true;
    motorSetEnable(true);
    bool dirLevel = direction > 0;
    if (motorDirectionInverted) dirLevel = !dirLevel;
    digitalWrite(TMC_DIR, dirLevel ? HIGH : LOW);
    delayMicroseconds(20);

    for (int i = 0; i < steps; i++) {
        digitalWrite(TMC_STEP, HIGH);
        delayMicroseconds(10);
        digitalWrite(TMC_STEP, LOW);
        delayMicroseconds(stepDelayUs);
        motorPosition += (direction > 0) ? 1 : -1;
    }

    Serial.printf("DBG: burst %c %d steps pos=%d\n",
                  direction > 0 ? '+' : '-', steps, motorPosition);
    dumpDriverStatus();
}

// ============= MOTOR =============
int motorShaftRevSteps() {
    return FULL_STEPS_PER_REV * driverMicrosteps;
}

int degreesToSteps(int degrees) {
    long steps = (long)degrees * FULL_STEPS_PER_REV * driverMicrosteps;
    return (int)((steps + 180L) / 360L);
}

int degX10ToSteps(int degX10) {
    long steps = (long)degX10 * FULL_STEPS_PER_REV * driverMicrosteps;
    return (int)((steps + 1800L) / 3600L);
}

int stepsToDegrees(int steps) {
    long denom = (long)FULL_STEPS_PER_REV * driverMicrosteps;
    long numerator = (long)steps * 360L;
    if (numerator >= 0) {
        return (int)((numerator + denom / 2) / denom);
    }
    return (int)((numerator - denom / 2) / denom);
}

int stepsToDegX10(int steps) {
    long denom = (long)FULL_STEPS_PER_REV * driverMicrosteps;
    long numerator = (long)steps * 3600L;
    if (numerator >= 0) {
        return (int)((numerator + denom / 2) / denom);
    }
    return (int)((numerator - denom / 2) / denom);
}

void printDegX10(int degX10) {
    tft.print(degX10 / 10);
    tft.print(".");
    tft.print(abs(degX10 % 10));
}

int encoderAcceleration(unsigned long intervalMs) {
    // A gentle, wide gradient: even a "quick" turn in the 150-400ms/detent range (about
    // as fast as this encoder's detents register in practice) gets a small boost, while
    // a slow, deliberate single click stays at 1x for 0.1 precision. The top end is
    // intentionally modest (4x, not 10x+) so fast spins feel controllable.
    if (intervalMs <= 15)  return 8;
    if (intervalMs <= 50)  return 4;
    if (intervalMs <= 150) return 2;
    return 1;
}

void motorStep(int direction) {
    bool dirLevel = direction > 0;
    if (motorDirectionInverted) dirLevel = !dirLevel;
    digitalWrite(TMC_DIR, dirLevel ? HIGH : LOW);
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

void motorMoveToBlocking(int target, unsigned int stepDelayUs) {
    setDriverParkHold(false);
    motorSetEnable(true);
    while (motorPosition != target) {
        motorMoveTo(target);
        delayMicroseconds(stepDelayUs);
    }
}

// ============= OUTPUTS =============
void setLED(int ledPin, bool state) { digitalWrite(ledPin, state ? HIGH : LOW); }
void setFan(int speed) { analogWrite(FAN_PWM, constrain(speed, 0, 255)); }
void setUltrasonic(bool on) { digitalWrite(ULTRASONIC, on ? LOW : HIGH); ultrasonicActive = on; }

// True when the arm tip is over the wafer disk (within ±half-sweep of centre),
// i.e. |currentAngle - centre| <= half the calculated sweep.
bool armOverWafer() {
    int half = (calculatedSweepDegX10() + 1) / 2;
    return abs(stepsToDegX10(motorPosition) - CENTER_DEG_X10) <= half;
}

const char* stateLabel(SystemState state) {
    switch (state) {
        case STATE_IDLE:          return "IDLE";
        case STATE_HOMING:        return "HOME";
        case STATE_PARKED:        return "PARK";
        case STATE_WAITING_SPRAY: return "WAIT";
        case STATE_SPRAY_ACTIVE:  return "SPRAY";
        case STATE_OSCILLATING:   return "OSC";
        case STATE_ERROR:         return "ERROR";
    }
    return "----";
}

// Side status bar. Top half: live STATE + arm ANGLE. Middle: the SWEEP config summary
// (calculated sweep angle, time, wafer, type, profile). Bottom: spray/flow sensors.
void drawStatusColumn(SystemState state, int posDeg, bool forceRedraw) {
    static SystemState drawnState = (SystemState)-1;
    static int  drawnPosDeg = -9999;
    static bool drawnSpray = false;
    static bool drawnFlow  = false;
    static int  drawnSweepAng = -1, drawnTime = -1, drawnWafer = -1, drawnType = -1, drawnProfile = -1;
    static bool drawnSensorEnabled = true;

    int x = STATUS_X + 6;          // 238
    int valueW = STATUS_W - 8;     // 80

    bool sensorChanged = (SENSOR_INPUTS_ENABLED != drawnSensorEnabled);

    if (forceRedraw) {
        tft.fillRect(STATUS_X, 0, STATUS_W, tft.height(), ST77XX_BLACK);
        tft.drawFastVLine(STATUS_X, 0, tft.height(), tft.color565(70, 70, 70));

        tft.setTextWrap(false);
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
        tft.setCursor(x, 4);  tft.print("STATUS");
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.setCursor(x, 24); tft.print("STATE");
        tft.setCursor(x, 54); tft.print("ANGLE");
        tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
        tft.setCursor(x, 86); tft.print("SWEEP");
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.setCursor(x, 148); tft.print("SPRAY");
        tft.setCursor(x, 158); tft.print("FLOW");

        drawnState = (SystemState)-1;
        drawnPosDeg = -9999;
        drawnSpray = !sprayActive;
        drawnFlow  = !flowDetected;
        drawnSweepAng = drawnTime = drawnWafer = drawnType = drawnProfile = -1;
    }

    // DEBUG ON indicator: re-drawn whenever the sensor-bypass toggle changes,
    // not just on a full redraw, so flipping it in Setup updates live.
    if (forceRedraw || sensorChanged) {
        tft.fillRect(x, 14, valueW, 8, ST77XX_BLACK);
        if (!SENSOR_INPUTS_ENABLED) {
            tft.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
            tft.setCursor(x, 14); tft.print("DEBUG ON");
        }
        drawnSensorEnabled = SENSOR_INPUTS_ENABLED;
    }

    if (forceRedraw || state != drawnState) {
        tft.fillRect(x, 34, valueW, 16, ST77XX_BLACK);
        tft.setTextSize(2);
        tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
        tft.setCursor(x, 34); tft.print(stateLabel(state));
        drawnState = state;
    }

    if (forceRedraw || posDeg != drawnPosDeg) {
        tft.fillRect(x, 64, valueW, 16, ST77XX_BLACK);
        tft.setTextSize(2);
        tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
        tft.setCursor(x, 64); tft.print(posDeg);
        tft.setTextSize(1); tft.print(" deg");
        drawnPosDeg = posDeg;
    }

    // SWEEP config summary (updates live while editing on the Settings screen).
    int sweepAng = travelSweepDegX10();
    if (forceRedraw || sweepAng != drawnSweepAng) {
        tft.fillRect(x, 96, valueW, 8, ST77XX_BLACK);
        tft.setTextSize(1); tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
        tft.setCursor(x, 96); printDegX10(sweepAng); tft.print("deg");
        drawnSweepAng = sweepAng;
    }
    if (forceRedraw || (int)SWEEP_TIME_MS != drawnTime) {
        tft.fillRect(x, 106, valueW, 8, ST77XX_BLACK);
        tft.setTextSize(1); tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.setCursor(x, 106); tft.print((SWEEP_TIME_MS + 500) / 1000); tft.print("s");
        drawnTime = (int)SWEEP_TIME_MS;
    }
    if (forceRedraw || sampleIndex != drawnWafer) {
        tft.fillRect(x, 116, valueW, 8, ST77XX_BLACK);
        tft.setTextSize(1); tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.setCursor(x, 116); tft.print(waferDiameterMm()); tft.print("mm");
        drawnWafer = sampleIndex;
    }
    if (forceRedraw || sweepType != drawnType) {
        tft.fillRect(x, 126, valueW, 8, ST77XX_BLACK);
        tft.setTextSize(1); tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.setCursor(x, 126); tft.print(sweepTypeLabel());
        drawnType = sweepType;
    }
    if (forceRedraw || sweepProfile != drawnProfile) {
        tft.fillRect(x, 136, valueW, 8, ST77XX_BLACK);
        tft.setTextSize(1); tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.setCursor(x, 136); tft.print(sweepProfileLabel());
        drawnProfile = sweepProfile;
    }

    if (forceRedraw || sprayActive != drawnSpray || sensorChanged) {
        tft.fillRect(x + 38, 148, valueW - 38, 8, ST77XX_BLACK);
        tft.setTextSize(1);
        if (SENSOR_INPUTS_ENABLED) {
            tft.setTextColor(sprayActive ? ST77XX_GREEN : ST77XX_RED, ST77XX_BLACK);
            tft.setCursor(x + 38, 148); tft.print(sprayActive ? "ON" : "OFF");
        } else {
            tft.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
            tft.setCursor(x + 38, 148); tft.print("DIS");
        }
        drawnSpray = sprayActive;
    }

    if (forceRedraw || flowDetected != drawnFlow || sensorChanged) {
        tft.fillRect(x + 38, 158, valueW - 38, 8, ST77XX_BLACK);
        tft.setTextSize(1);
        if (SENSOR_INPUTS_ENABLED) {
            tft.setTextColor(flowDetected ? ST77XX_GREEN : ST77XX_RED, ST77XX_BLACK);
            tft.setCursor(x + 38, 158); tft.print(flowDetected ? "YES" : "NO");
        } else {
            tft.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
            tft.setCursor(x + 38, 158); tft.print("DIS");
        }
        drawnFlow = flowDetected;
    }
}

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

    tft.fillRect(6, y - 2, CONTENT_W - 12, 28, bg);

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

    int visibleCount = advancedMenuUnlocked ? MENU_COUNT : BASIC_MENU_COUNT;
    int mi = menuIndex;  // snapshot to avoid race with Core 0
    if (mi >= visibleCount) {
        mi = 0;
        menuIndex = 0;
    }

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
        tft.setCursor(28, 2);
        tft.print("MEGASONIC CLEANER");

        for (int i = 0; i < visibleCount; i++)
            drawMenuRow(i, 22 + i * 28, i == mi);

    } else if (menuDrawState != mi) {
        if (menuDrawState >= 0 && menuDrawState < visibleCount) {
            drawMenuRow(menuDrawState, 22 + menuDrawState * 28, false);
        }
        drawMenuRow(mi,            22 + mi             * 28, true);
    }

    menuDrawState = mi;
}

// Circle outline (Bresenham midpoint) clipped to a vertical [yTop, yBot] window,
// so an oversized wafer can be cropped top and bottom instead of overflowing.
void drawCircleClipped(int cx, int cy, int r, int yTop, int yBot, uint16_t color) {
    int x = 0, y = r, d = 3 - 2 * r;
    while (x <= y) {
        const int px[8] = { cx + x, cx - x, cx + x, cx - x, cx + y, cx - y, cx + y, cx - y };
        const int py[8] = { cy + y, cy + y, cy - y, cy - y, cy + x, cy + x, cy - x, cy - x };
        for (int i = 0; i < 8; i++)
            if (py[i] >= yTop && py[i] <= yBot && px[i] >= 0 && px[i] < 320)
                tft.drawPixel(px[i], py[i], color);
        if (d < 0) d += 4 * x + 6; else { d += 4 * (x - y) + 10; y--; }
        x++;
    }
}

// ============= ARM POSITION ANIMATION (basic menu) =============
// Top-down sketch under the START/Settings rows: the wafer as a circle (size ∝ wafer
// diameter), the park position as a green tick, and the live arm position as a red
// down-arrow. Geometry: arm-tip horizontal offset from wafer centre =
// ARM_LENGTH * sin(angle - centre), so the sweep extremes land on the wafer edges.
// The scale is fixed to the largest selectable wafer (150 mm) so the circle is drawn
// as large as possible; it is cropped top and bottom by the [yTop, yBot] window.
void drawArmAnim(bool fullRedraw, int posDegX10) {
    const int axisY   = ANIM_Y + ANIM_H / 2;        // 126: wafer centre / arrow-tip line
    const int yTop    = ANIM_Y;                      // crop window top
    const int yBot    = ANIM_Y + ANIM_H - 1;         // crop window bottom
    const int headH   = 7;                            // arrowhead height
    const int tipY    = axisY;                        // arrow points down onto the plane
    const uint16_t gray = tft.color565(60, 60, 60);

    static int   prevArmX    = -1;
    static int   lastAnimDeg = -9999;                // whole degrees, for the 1° flicker gate
    static bool  prevPulse   = false;                // last lightning-sign blink state
    static int   cPark = -1, cCenter = -1, cArm = -1, cWafer = -1;
    static int   waferCx = 0, radiusPx = 0, parkX = 0, startX = 0, drawnW = 0, axL = 0, axR = 0;
    static float sScale = 1.0f, sLo = 0.0f;

    const float toRad     = 3.14159265f / 180.0f;
    const float centerDeg = CENTER_DEG_X10 * 0.1f;

    // Static layout depends only on park / centre / arm length / wafer settings.
    bool redrawStatic = fullRedraw || cPark != PARK_DEG_X10 || cCenter != CENTER_DEG_X10 ||
                        cArm != ARM_LENGTH_MM || cWafer != sampleIndex;

    if (redrawStatic) {
        cPark = PARK_DEG_X10; cCenter = CENTER_DEG_X10;
        cArm = ARM_LENGTH_MM; cWafer = sampleIndex;

        float parkDeg   = PARK_DEG_X10 * 0.1f;
        float waferR    = waferDiameterMm() * 0.5f;
        float waferRmax = SAMPLE_TABLE[SAMPLE_COUNT - 1] * 0.5f;   // 150 mm wafer
        float parkOff   = ARM_LENGTH_MM * sinf((parkDeg - centerDeg) * toRad);

        // Fixed scale sized for the largest wafer so the circle fills the width;
        // taller-than-the-window circles are cropped top/bottom by drawCircleClipped.
        float lo = fminf(parkOff, -waferRmax);
        float hi = fmaxf(parkOff,  waferRmax);
        float spanMm = hi - lo; if (spanMm < 1.0f) spanMm = 1.0f;
        sScale = (ANIM_W - 16) / spanMm;
        sLo    = lo;

        drawnW   = (int)lroundf(spanMm * sScale);
        startX   = ANIM_X + (ANIM_W - drawnW) / 2;
        radiusPx = (int)lroundf(waferR * sScale);
        waferCx  = startX + (int)lroundf((0.0f    - lo) * sScale);
        parkX    = startX + (int)lroundf((parkOff - lo) * sScale);
        axL = max(ANIM_X,       min(parkX, waferCx - radiusPx) - 4);
        axR = min(STATUS_X - 2, waferCx + radiusPx + 4);

        // Clear region and paint the static scene.
        tft.fillRect(ANIM_X, ANIM_Y - 2, ANIM_W, ANIM_H + 2, ST77XX_BLACK);
        tft.drawFastHLine(ANIM_X, ANIM_Y - 4, ANIM_W, gray);            // separator
        tft.drawFastHLine(axL, axisY, axR - axL, gray);                 // sweep axis
        drawCircleClipped(waferCx, axisY, radiusPx, yTop, yBot, ST77XX_CYAN);
        tft.fillCircle(waferCx, axisY, 2, ST77XX_CYAN);                 // wafer centre dot
        tft.drawFastVLine(parkX, axisY - 6, 13, ST77XX_GREEN);          // park tick
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
        tft.setCursor(parkX - 2, axisY + 10);
        tft.print("P");

        prevArmX = -1; lastAnimDeg = -9999; prevPulse = false;  // force the arrow to repaint
    }

    // Live arm position.
    float armOff = ARM_LENGTH_MM * sinf((posDegX10 * 0.1f - centerDeg) * toRad);
    int   armX   = startX + (int)lroundf((armOff - sLo) * sScale);
    armX = constrain(armX, startX, startX + drawnW);

    // Generator lightning sign blinks ~2 Hz while the ultrasonic generator is on.
    bool pulseOn = ultrasonicActive && ((millis() / 250) & 1);

    // Redraw the arrow when it moved ≥1° (flicker gate) or when the pulse toggled.
    int  curDeg    = (posDegX10 + (posDegX10 >= 0 ? 5 : -5)) / 10;
    bool angleStep = redrawStatic || abs(curDeg - lastAnimDeg) >= 1;
    bool moved     = angleStep && (armX != prevArmX);
    if (!redrawStatic && !moved && pulseOn == prevPulse && prevArmX >= 0) return;
    if (angleStep) lastAnimDeg = curDeg;

    // Erase the old arrow + sign within the crop window, then restore the scene.
    if (prevArmX >= 0) {
        tft.fillRect(prevArmX - 5, yTop, 17, (tipY + 1) - yTop, ST77XX_BLACK);
        tft.drawFastHLine(axL, axisY, axR - axL, gray);
        drawCircleClipped(waferCx, axisY, radiusPx, yTop, yBot, ST77XX_CYAN);
        tft.fillCircle(waferCx, axisY, 2, ST77XX_CYAN);
        tft.drawFastVLine(parkX, axisY - 6, 13, ST77XX_GREEN);
    }

    // Draw the arrow (red): tall shaft from the crop top + downward head onto the plane.
    tft.fillRect(armX - 1, yTop, 2, (tipY - headH) - yTop, ST77XX_RED);
    tft.fillTriangle(armX - 4, tipY - headH, armX + 4, tipY - headH, armX, tipY, ST77XX_RED);

    // Pulsing lightning bolt next to the arrow while the generator is energised.
    if (pulseOn) {
        int bx = armX + 5, by = yTop + 1;
        tft.drawLine(bx + 3, by,     bx,     by + 5,  ST77XX_YELLOW);
        tft.drawLine(bx,     by + 5, bx + 4, by + 5,  ST77XX_YELLOW);
        tft.drawLine(bx + 4, by + 5, bx + 1, by + 11, ST77XX_YELLOW);
        tft.drawLine(bx + 2, by,     bx - 1, by + 5,  ST77XX_YELLOW);  // thicken upper stroke
    }
    prevPulse = pulseOn;
    prevArmX  = armX;
}

// ============= SETTINGS SCREEN (sweep params) =============
// Rows show "label:value" at textSize 2 (matching the larger UI), with the changeable
// value drawn in yellow so it stands out. The space after each title is dropped so the
// longest line ("Sweep type:Edge↔Edge") just fits the content width left of the status bar.
void drawSettingsRow(int i, int y, bool selected, bool editing) {
    const char* labels[] = { "Sweep time", "Wafer diam.", "Sweep type", "Profile" };
    uint16_t bg = editing  ? tft.color565(0, 140, 0) :
                  selected ? ST77XX_BLUE : ST77XX_BLACK;
    tft.fillRect(0, y - 2, CONTENT_W, 16, bg);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE, bg);
    tft.setCursor(2, y);
    tft.print(labels[i]);
    tft.print(":");
    tft.setTextColor(ST77XX_YELLOW, bg);   // highlight the changeable value
    switch (i) {
        case 0: tft.print((SWEEP_TIME_MS + 500) / 1000); tft.print(" s."); break;
        case 1: tft.print(SAMPLE_TABLE[sampleIndex]);  tft.print(" mm"); break;
        case 2: tft.print(sweepTypeLabel());    break;
        case 3: tft.print(sweepProfileLabel()); break;
    }
}

void drawSettings() {
    static int  lastIdx     = -1;
    static bool lastEdit    = false;
    static int  lastVals[4] = {};

    if (settingsNeedsRedraw) { settingsNeedsRedraw = false; lastIdx = -1; }

    int  si = settingsIndex;
    bool ed = editingSettings;
    int  vals[4] = { (int)(SWEEP_TIME_MS / 100), sampleIndex, sweepType, sweepProfile };

    bool changed = (lastIdx == -1) || (si != lastIdx) || (ed != lastEdit);
    for (int i = 0; i < 4 && !changed; i++) changed = (vals[i] != lastVals[i]);
    if (!changed) return;

    if (lastIdx == -1) {
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextWrap(false);
        tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
        tft.setTextSize(2);
        tft.setCursor(6, 3);
        tft.print("SWEEP SETTINGS");
    }

    for (int i = 0; i < SETTINGS_COUNT; i++)
        drawSettingsRow(i, 26 + i * 22, i == si, i == si && ed);   // spread over the taller field

    lastIdx = si; lastEdit = ed;
    for (int i = 0; i < 4; i++) lastVals[i] = vals[i];
}

// ============= SETUP SCREEN (hardware params) =============
void drawSetupRow(int i, int y, bool selected, bool editing) {
    const char* labels[] = {
        "Park   ", "Centre ", "Arm    ", "Cycles ",
        "Current", "Mstep  ", "Invert ", "Debug  "
    };
    uint16_t bg = editing  ? tft.color565(0, 140, 0) :
                  selected ? ST77XX_BLUE : ST77XX_BLACK;
    tft.fillRect(0, y - 2, CONTENT_W, 18, bg);
    tft.setTextColor(ST77XX_WHITE, bg);
    tft.setTextSize(2);
    tft.setCursor(6, y);
    tft.print(labels[i]);
    tft.print(": ");
    switch (i) {
        case 0: printDegX10(PARK_DEG_X10);              tft.print("deg ");  break;
        case 1: printDegX10(CENTER_DEG_X10);            tft.print("deg ");  break;
        case 2: tft.print(ARM_LENGTH_MM);               tft.print("mm  ");  break;
        case 3:
            if (OSCILLATION_CYCLES == 0) tft.print("inf   ");
            else { tft.print(OSCILLATION_CYCLES); tft.print("      "); }
            break;
        case 4: tft.print(driverCurrent);               tft.print("mA  ");  break;
        case 5: tft.print(driverMicrosteps);            tft.print("x     "); break;
        case 6: tft.print(motorDirectionInverted ? "ON    " : "OFF   "); break;
        case 7: tft.print(SENSOR_INPUTS_ENABLED ? "OFF   " : "ON    "); break;
    }
}

void drawSetup() {
    static int  lastIdx     = -1;
    static bool lastEdit    = false;
    static int  lastVals[8] = {};

    if (setupNeedsRedraw) { setupNeedsRedraw = false; lastIdx = -1; }

    int  si = setupIndex;
    bool ed = editingSetup;
    int  vals[8] = {
        PARK_DEG_X10, CENTER_DEG_X10, ARM_LENGTH_MM, (int)OSCILLATION_CYCLES,
        driverCurrent, driverMicrosteps, motorDirectionInverted ? 1 : 0,
        SENSOR_INPUTS_ENABLED ? 1 : 0
    };

    bool changed = (lastIdx == -1) || (si != lastIdx) || (ed != lastEdit);
    for (int i = 0; i < 8 && !changed; i++) changed = (vals[i] != lastVals[i]);
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
        drawSetupRow(i, 22 + i * 18, i == si, i == si && ed);

    lastIdx = si; lastEdit = ed;
    for (int i = 0; i < 8; i++) lastVals[i] = vals[i];
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
        tft.setCursor(8, 3);   tft.print("MEGASONIC v1.2");
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

    bool     stst   = (ds >> 31) & 1;
    bool     ot     = (ds >> 25) & 1;
    bool     otpw   = (ds >> 26) & 1;
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
    static int         lastPosDeg      = -9999;
    static int         lastMenu        = -1;
    static bool        lastRunning     = false;
    static bool        lastSpray       = false;
    static bool        lastFlow        = false;
    static bool        lastHold        = false;
    static DisplayMode lastDisplayMode = DISP_MENU;

    DisplayMode dm    = displayMode;
    int         mi    = menuIndex;
    SystemState state = currentState;
    int         pos   = motorPosition;
    int         posDegX10 = stepsToDegX10(pos);
    int         posDeg    = (posDegX10 + (posDegX10 >= 0 ? 5 : -5)) / 10;  // whole degrees for status bar
    bool statusChanged = (state != lastState || posDeg != lastPosDeg ||
                          sprayActive != lastSpray || flowDetected != lastFlow ||
                          driverParkHoldMode != lastHold);

    if (dm != DISP_MENU) {
        bool modeChanged = (dm != lastDisplayMode);
        mutex_enter_blocking(&spi_mutex);
        if (dm == DISP_SETTINGS) {
            // Same arm animation as the main menu sits under the rows; the status column
            // is refreshed every frame so its SWEEP summary tracks live edits (it
            // self-gates per field, so this does not flicker).
            drawSettings();
            drawArmAnim(modeChanged, posDegX10);
            drawStatusColumn(state, posDeg, modeChanged);
        } else if (dm == DISP_SETUP) {
            drawSetup();
            drawStatusColumn(state, posDeg, modeChanged);
        } else {
            drawAbout();
        }
        mutex_exit(&spi_mutex);
        if (modeChanged || statusChanged) {
            lastState = state; lastPosDeg = posDeg;
            lastSpray = sprayActive; lastFlow = flowDetected; lastHold = driverParkHoldMode;
        }
        lastDisplayMode = dm;
        return;
    }

    if (lastDisplayMode != DISP_MENU) {
        lastState   = (SystemState)-1;
        lastPosDeg  = -9999;
        lastMenu    = -1;
        lastRunning = false;
        lastSpray   = false;
        lastFlow    = false;
        lastHold    = false;
        lastDisplayMode = DISP_MENU;
    }

    bool running = (state != STATE_IDLE);

    // State changed from idle↔running: force icon on row 0 to repaint
    if (running != lastRunning) {
        menuDrawState = -1;
        lastRunning   = running;
    }

    bool menuChanged  = (mi    != lastMenu);
    bool forceStatusRedraw = (menuChanged && menuDrawState == -1);

    // Keep ticking while the generator is on so the lightning sign can blink.
    bool pulseTick = !advancedMenuUnlocked && ultrasonicActive;
    if (!menuChanged && !statusChanged && !forceStatusRedraw && !pulseTick) return;

    mutex_enter_blocking(&spi_mutex);

    bool animFull = false;
    if (menuChanged) {
        animFull = (menuDrawState == -1);   // full-screen repaint about to happen
        drawMenu();
        lastMenu = mi;
    }

    // Arm-position animation lives under the two basic rows (no room once advanced).
    if (!advancedMenuUnlocked) {
        drawArmAnim(animFull, posDegX10);
    }

    if (statusChanged || forceStatusRedraw) {
        drawStatusColumn(state, posDeg, forceStatusRedraw);

        // Only echo to serial on a real state/sensor change, not on every
        // arm-motion step (position alone changes constantly while sweeping).
        bool eventChanged = (state != lastState || sprayActive != lastSpray ||
                              flowDetected != lastFlow || driverParkHoldMode != lastHold);

        lastState = state; lastPosDeg = posDeg;
        lastSpray = sprayActive; lastFlow = flowDetected; lastHold = driverParkHoldMode;

        if (eventChanged) {
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
            Serial.printf(" | Pos:%d steps (%.1f deg) | Spray:%s | Flow:%s\n",
                pos, posDegX10 * 0.1f, sprayActive ? "ON" : "OFF", flowDetected ? "YES" : "NO");
        }
    }

    mutex_exit(&spi_mutex);
}
