#include <Arduino.h>
#include <SPI.h>

// ============= PIN DEFINITIONS =============
// TMC2130 Stepper Driver
const int TMC_CS = 17;       // Chip Select
const int TMC_MOSI = 19;     // SPI MOSI
const int TMC_MISO = 16;     // SPI MISO
const int TMC_SCK = 18;      // SPI Clock
const int TMC_STEP = 14;     // Step pin
const int TMC_DIR = 15;      // Direction pin
const int TMC_EN = 13;       // Enable pin

// SPI LCD (adjust pins based on your display)
const int LCD_CS = 9;        // Chip Select
const int LCD_DC = 10;       // Data/Command
const int LCD_RST = 11;      // Reset

// Rotary Encoder
const int ENC_A = 26;        // Encoder pin A
const int ENC_B = 27;        // Encoder pin B (optional button on SW)

// Sensors & Outputs
const int LIMIT_SWITCH = 28; // End position limit
const int SPRAY_VALVE = 2;   // Spray developer valve
const int FLOW_SENSOR = 3;   // Flow sensor input
const int LED_GREEN = 8;     // Status LED (green)
const int LED_YELLOW = 7;    // Status LED (yellow) - changed from 9 to avoid conflict
const int FAN_PWM = 12;      // Fan PWM control
const int ULTRASONIC = 4;    // Ultrasonic generator remote

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
const int STEPS_PER_REVOLUTION = 200;
const int MOTOR_SPEED = 50;           // RPM equivalent
const int PARK_POSITION = 7;
const int CENTER_POSITION = 26;       // Park + 34 mm
const int OSCILLATION_STEPS = 16;
const unsigned long OSCILLATION_DELAY = 10000; // ms

// ============= STATUS VARIABLES =============
int encoderValue = 0;
int motorPosition = 0;
bool limitSwitchPressed = false;
bool sprayActive = false;
bool flowDetected = false;
unsigned long oscillationCount = 0;
const unsigned long OSCILLATION_CYCLES = 20;

// ============= TIMING =============
unsigned long lastMotorUpdate = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastEncoderRead = 0;
const unsigned long MOTOR_UPDATE_INTERVAL = 50;
const unsigned long DISPLAY_UPDATE_INTERVAL = 100;
const unsigned long ENCODER_READ_INTERVAL = 20;

// ============= TMC2130 COMMUNICATION =============
void tmcWriteReg(uint8_t addr, uint32_t datagram) {
  digitalWrite(TMC_CS, LOW);
  delayMicroseconds(2);
  SPI.transfer((addr | 0x80)); // Write bit
  SPI.transfer((datagram >> 24) & 0xFF);
  SPI.transfer((datagram >> 16) & 0xFF);
  SPI.transfer((datagram >> 8) & 0xFF);
  SPI.transfer(datagram & 0xFF);
  delayMicroseconds(2);
  digitalWrite(TMC_CS, HIGH);
}

uint32_t tmcReadReg(uint8_t addr) {
  uint32_t datagram = 0;
  digitalWrite(TMC_CS, LOW);
  delayMicroseconds(2);
  SPI.transfer(addr & 0x7F); // Read bit
  delayMicroseconds(2);
  digitalWrite(TMC_CS, HIGH);
  delayMicroseconds(5);
  
  digitalWrite(TMC_CS, LOW);
  delayMicroseconds(2);
  SPI.transfer(addr & 0x7F);
  datagram |= ((uint32_t)SPI.transfer(0) << 24);
  datagram |= ((uint32_t)SPI.transfer(0) << 16);
  datagram |= ((uint32_t)SPI.transfer(0) << 8);
  datagram |= SPI.transfer(0);
  delayMicroseconds(2);
  digitalWrite(TMC_CS, HIGH);
  return datagram;
}

// ============= FUNCTION DECLARATIONS =============
void initHardware();
void initSPI();
void initEncoder();
void readEncoder();
void readSensors();
void updateStateMachine();
void handleState();
void motorStep(int direction);
void motorSetEnable(bool enable);
void updateDisplay();
void setLED(int ledPin, bool state);
void setFan(int speed);
void setUltrasonic(bool active);

// ============= SETUP =============
void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("=== Stepper Controller Initializing ===");
  
  initHardware();
  initSPI();
  initEncoder();
  initTMC2130();
  
  Serial.println("Initialization complete!");
  currentState = STATE_IDLE;
  lastStateChange = millis();
}

// ============= MAIN LOOP =============
void loop() {
  unsigned long now = millis();
  
  // Read inputs periodically
  if (now - lastEncoderRead >= ENCODER_READ_INTERVAL) {
    readEncoder();
    lastEncoderRead = now;
  }
  
  readSensors();
  
  // Update state machine
  updateStateMachine();
  
  // Process current state
  handleState();
  
  // Update display periodically
  if (now - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay();
    lastDisplayUpdate = now;
  }
}

// ============= HARDWARE INITIALIZATION =============
void initHardware() {
  // Motor control
  pinMode(TMC_STEP, OUTPUT);
  pinMode(TMC_DIR, OUTPUT);
  pinMode(TMC_EN, OUTPUT);
  digitalWrite(TMC_EN, HIGH); // Disable motor initially
  
  // Sensors
  pinMode(LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(SPRAY_VALVE, INPUT);
  pinMode(FLOW_SENSOR, INPUT);
  
  // Outputs
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(FAN_PWM, OUTPUT);
  pinMode(ULTRASONIC, OUTPUT);
  
  // Safe initial state
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  analogWrite(FAN_PWM, 0);
  digitalWrite(ULTRASONIC, HIGH); // Ultrasonic OFF (active low)
  
  Serial.println("Hardware initialized");
}

void initSPI() {
  // Configure SPI for TMC2130
  // RP2040 uses default SPI0: SCK=18, MOSI=19, MISO=16
  // Verify these pins match your TMC2130 connections
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32); // ~1 MHz for RP2040 @ 125 MHz
  SPI.setDataMode(SPI_MODE3); // TMC2130 requires SPI mode 3
  
  pinMode(TMC_CS, OUTPUT);
  digitalWrite(TMC_CS, HIGH);
  
  Serial.print("SPI initialized: SCK=");
  Serial.print(TMC_SCK);
  Serial.print(" MOSI=");
  Serial.print(TMC_MOSI);
  Serial.print(" MISO=");
  Serial.println(TMC_MISO);
}

void initEncoder() {
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  // TODO: Configure interrupt-driven encoder reading if needed
  Serial.print("Encoder initialized: A=");
  Serial.print(ENC_A);
  Serial.print(" B=");
  Serial.println(ENC_B);
}

void initTMC2130() {
  // TODO: Configure TMC2130 via SPI
  // Required registers:
  //  - GCONF: General config (spreadCycle, stealthChop, etc.)
  //  - IHOLD_IRUN: Current settings
  //  - CHOPCONF: Chopper config
  //  - PWMCONF: PWM config for stealthChop
  Serial.println("TMC2130 driver: Configure via SPI - TODO");
}

// ============= SENSOR READING =============
void readSensors() {
  limitSwitchPressed = (digitalRead(LIMIT_SWITCH) == LOW);
  sprayActive = (digitalRead(SPRAY_VALVE) == HIGH);
  flowDetected = (digitalRead(FLOW_SENSOR) == HIGH);
}

void readEncoder() {
  // TODO: Implement quadrature decoding
  // For now, placeholder
  encoderValue += 0; // Replace with actual delta from encoder
}

// ============= STATE MACHINE =============
void updateStateMachine() {
  unsigned long now = millis();
  unsigned long stateAge = now - lastStateChange;
  
  switch (currentState) {
    case STATE_IDLE:
      if (sprayActive) {
        currentState = STATE_HOMING;
        lastStateChange = now;
      }
      break;
      
    case STATE_HOMING:
      // Move to limit switch, then to park position
      if (limitSwitchPressed) {
        motorPosition = 0;
        currentState = STATE_PARKED;
        lastStateChange = now;
      }
      break;
      
    case STATE_PARKED:
      if (sprayActive && flowDetected) {
        currentState = STATE_SPRAY_ACTIVE;
        lastStateChange = now;
        oscillationCount = 0;
      }
      break;
      
    case STATE_SPRAY_ACTIVE:
      if (!sprayActive || !flowDetected) {
        currentState = STATE_IDLE;
        lastStateChange = now;
      }
      break;
      
    case STATE_OSCILLATING:
      if (oscillationCount >= OSCILLATION_CYCLES || !sprayActive) {
        currentState = STATE_IDLE;
        lastStateChange = now;
      }
      break;
      
    case STATE_ERROR:
      // TODO: Implement error recovery
      break;
  }
}

void handleState() {
  unsigned long now = millis();
  
  switch (currentState) {
    case STATE_IDLE:
      motorSetEnable(false);
      setLED(LED_GREEN, true);
      setLED(LED_YELLOW, false);
      setFan(0);
      setUltrasonic(true); // OFF
      break;
      
    case STATE_HOMING:
      motorSetEnable(true);
      setLED(LED_GREEN, false);
      setLED(LED_YELLOW, true);
      motorStep(-1); // Move toward limit
      break;
      
    case STATE_PARKED:
      motorSetEnable(true);
      motorStep(PARK_POSITION);
      setLED(LED_GREEN, true);
      break;
      
    case STATE_SPRAY_ACTIVE:
      setFan(255);
      setUltrasonic(false); // ON
      // Prepare for oscillation
      motorPosition = CENTER_POSITION;
      currentState = STATE_OSCILLATING;
      break;
      
    case STATE_OSCILLATING:
      // Oscillate motor back and forth
      if (now - lastMotorUpdate >= OSCILLATION_DELAY) {
        motorStep(-1);
        oscillationCount++;
        lastMotorUpdate = now;
      }
      break;
      
    case STATE_ERROR:
      motorSetEnable(false);
      setLED(LED_GREEN, false);
      setLED(LED_YELLOW, true);
      break;
  }
}

// ============= MOTOR CONTROL =============
void motorStep(int direction) {
  digitalWrite(TMC_DIR, direction > 0 ? HIGH : LOW);
  digitalWrite(TMC_STEP, HIGH);
  delayMicroseconds(10);
  digitalWrite(TMC_STEP, LOW);
  delayMicroseconds(10);
  
  motorPosition += (direction > 0) ? 1 : -1;
}

void motorSetEnable(bool enable) {
  digitalWrite(TMC_EN, enable ? LOW : HIGH);
}

// ============= OUTPUT CONTROL =============
void setLED(int ledPin, bool state) {
  digitalWrite(ledPin, state ? HIGH : LOW);
}

void setFan(int speed) {
  // speed: 0-255
  analogWrite(FAN_PWM, speed);
}

void setUltrasonic(bool active) {
  // active = true means OFF (active low logic)
  digitalWrite(ULTRASONIC, active ? HIGH : LOW);
}

// ============= DISPLAY UPDATE =============
void updateDisplay() {
  // TODO: Implement SPI LCD display update
  // For now, serial debug output
  
  Serial.print("State: ");
  switch (currentState) {
    case STATE_IDLE: Serial.print("IDLE"); break;
    case STATE_HOMING: Serial.print("HOMING"); break;
    case STATE_PARKED: Serial.print("PARKED"); break;
    case STATE_WAITING_SPRAY: Serial.print("WAITING"); break;
    case STATE_SPRAY_ACTIVE: Serial.print("SPRAY_ACTIVE"); break;
    case STATE_OSCILLATING: Serial.print("OSCILLATING"); break;
    case STATE_ERROR: Serial.print("ERROR"); break;
  }
  Serial.print(" | Pos: ");
  Serial.print(motorPosition);
  Serial.print(" | Spray: ");
  Serial.print(sprayActive ? "ON" : "OFF");
  Serial.print(" | Flow: ");
  Serial.print(flowDetected ? "YES" : "NO");
  Serial.println();
}
