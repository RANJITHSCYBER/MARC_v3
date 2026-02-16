/*
 * ESP32 5-DOF Robotic Arm Controller
 * Professional Industrial Grade
 * Version 3.0
 * 
 * Servos:
 * 1. Base Rotation (GPIO 25)
 * 2. Shoulder (GPIO 26)
 * 3. Elbow (GPIO 27)
 * 4. Wrist Pitch (GPIO 14)
 * 5. Gripper (GPIO 12)
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ESP32Servo.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include "position.h"

// ==================== CONFIGURATION ====================
#define FIRMWARE_VERSION "3.0"
#define SERIAL_BAUD 115200

// WiFi Credentials
const char* ssid = "Act boii";
const char* password = "roronova";

// AP Mode Credentials (fallback)
const char* ap_ssid = "RobotArm-Controller";
const char* ap_password = "admin123";

#define SERVO1_PIN 25    // Base rotation
#define SERVO2_PIN 26    // Shoulder
#define SERVO3_PIN 27    // Elbow
#define SERVO4_PIN 14    // Wrist pitch
#define SERVO5_PIN 12    // Gripper
#define STATUS_LED 2     // Built-in LED for status

// Gripper button (press=open, release=close)
#define GRIPPER_BUTTON_PIN 13
#define BUTTON_DEBOUNCE_MS 30

// Status LEDs
#define LED_GREEN_PIN 5
#define LED_BLUE_PIN 18
#define LED_RED_PIN 19
#define LED_ACTIVE_HIGH true
#define BLUE_BLINK_INTERVAL_MS 1000
#define GREEN_STEP_PULSE_MS 120

// Potentiometer ADC pins (base/shoulder/elbow/wrist)
#define POT_BASE_PIN 32
#define POT_SHOULDER_PIN 33
#define POT_ELBOW_PIN 34
#define POT_WRIST_PIN 35

// Potentiometer settings
#define POT_READ_INTERVAL 50   // ms
#define POT_DEADBAND 2         // degrees
#define POT_SAMPLES 5          // ADC samples per read
#define POT_FILTER_ALPHA 0.25f // EMA smoothing factor

// Potentiometer calibration (ADC raw range 0-4095)
#define POT_BASE_ADC_MIN 200
#define POT_BASE_ADC_MAX 3800
#define POT_SHOULDER_ADC_MIN 200
#define POT_SHOULDER_ADC_MAX 3800
#define POT_ELBOW_ADC_MIN 200
#define POT_ELBOW_ADC_MAX 3800
#define POT_WRIST_ADC_MIN 200
#define POT_WRIST_ADC_MAX 3800

// Servo Limits (hardware protection)
#define SERVO_MIN 0
#define SERVO_MAX 180

// Gripper specific limits
#define GRIPPER_MIN 0    // Fully open
#define GRIPPER_MAX 90   // Fully closed
// Swap these if your gripper direction is reversed.
#define GRIPPER_OPEN_ANGLE GRIPPER_MIN
#define GRIPPER_CLOSE_ANGLE GRIPPER_MAX

// Wrist specific limits
#define WRIST_MIN 0
#define WRIST_MAX 180

// EEPROM Configuration
#define EEPROM_SIZE 2048  // Increased for 5 servos
#define MAX_PRESETS 10
#define PRESET_SIZE 80    // Increased for 5 servos

// Recording Configuration
#define RECORD_MAX_STEPS 150
#define RECORD_META_SIZE 2
#define RECORD_STEP_SIZE 7
#define RECORD_EEPROM_ADDR (1 + (MAX_PRESETS * PRESET_SIZE))
#define RECORD_DATA_ADDR (RECORD_EEPROM_ADDR + RECORD_META_SIZE)

// Timing Constants
#define SERVO_UPDATE_INTERVAL 20  // ms
#define INACTIVITY_TIMEOUT 300000 // 5 minutes

// Motion Control
#define DEFAULT_SPEED 2  // 1-100%

// ==================== STRUCTURES ====================
struct PresetPosition {
  char name[32];
  int servo1;  // Base
  int servo2;  // Shoulder
  int servo3;  // Elbow
  int servo4;  // Wrist pitch
  int servo5;  // Gripper
  int speed;
};

struct RecordStep {
  uint8_t servo1;
  uint8_t servo2;
  uint8_t servo3;
  uint8_t servo4;
  uint8_t servo5;
  uint16_t delayMs;
};

// ==================== GLOBAL VARIABLES ====================
WebServer server(80);

// Servo Objects - 5 SERVOS
Servo servo1;  // Base
Servo servo2;  // Shoulder
Servo servo3;  // Elbow
Servo servo4;  // Wrist pitch
Servo servo5;  // Gripper

// Current Positions - 5 SERVOS
int currentPos1 = HOME_BASE;
int currentPos2 = HOME_SHOULDER;
int currentPos3 = HOME_ELBOW;
int currentPos4 = HOME_WRIST;
int currentPos5 = HOME_GRIPPER;

// Target Positions - 5 SERVOS
int targetPos1 = HOME_BASE;
int targetPos2 = HOME_SHOULDER;
int targetPos3 = HOME_ELBOW;
int targetPos4 = HOME_WRIST;
int targetPos5 = HOME_GRIPPER;

// Motion Control
int moveSpeed = DEFAULT_SPEED;
bool smoothMoveEnabled = true;
bool emergencyStop = false;
bool servosEnabled = true;
bool potControlEnabled = false;

// Potentiometer state
int lastPotBase = HOME_BASE;
int lastPotShoulder = HOME_SHOULDER;
int lastPotElbow = HOME_ELBOW;
int lastPotWrist = HOME_WRIST;
unsigned long lastPotRead = 0;
float filteredPotBase = HOME_BASE;
float filteredPotShoulder = HOME_SHOULDER;
float filteredPotElbow = HOME_ELBOW;
float filteredPotWrist = HOME_WRIST;
int lastGripperButtonState = HIGH;
int lastGripperButtonStable = HIGH;
unsigned long lastGripperButtonChange = 0;

// LED state
unsigned long lastBlueBlinkToggle = 0;
bool blueBlinkOn = false;
unsigned long greenPulseUntil = 0;

// System State
unsigned long lastActivityTime = 0;
unsigned long lastServoUpdate = 0;
unsigned long systemStartTime = 0;
bool wifiConnected = false;
bool apMode = false;
String systemLogs = "";

// Preset Storage
PresetPosition presets[MAX_PRESETS];
int nextPresetIndex = 0;

// Recording Storage
RecordStep recordSteps[RECORD_MAX_STEPS];
uint16_t recordCount = 0;
bool recordingActive = false;
bool playbackActive = false;
uint16_t playbackIndex = 0;
unsigned long lastRecordTime = 0;
unsigned long nextPlaybackTime = 0;

// ==================== FUNCTION DECLARATIONS ====================
// Utility Functions
void addLog(const String &message, const String &type = "");
String getTimestamp();
String getSystemStatus();
String generateHTML();

// ==================== WIFI FUNCTIONS ====================
bool connectWiFi() {
  addLog("Connecting to WiFi: " + String(ssid));
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    apMode = false;
    addLog("WiFi connected! IP: " + WiFi.localIP().toString());
    return true;
  } else {
    addLog("WiFi connection failed!", "error");
    return false;
  }
}

void startAPMode() {
  addLog("Starting AP mode with SSID: " + String(ap_ssid));
  WiFi.softAP(ap_ssid, ap_password);
  wifiConnected = true;
  apMode = true;
  addLog("AP mode active! IP: " + WiFi.softAPIP().toString());
}

void setupmDNS() {
  if (!MDNS.begin("robotarm")) {
    addLog("Error setting up mDNS!", "error");
  } else {
    MDNS.addService("http", "tcp", 80);
    addLog("mDNS responder started: robotarm.local");
  }
}

// ==================== SERVO FUNCTIONS ====================
void initServos() {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Set servo frequency (standard 50Hz for all servos)
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  servo4.setPeriodHertz(50);
  servo5.setPeriodHertz(50);
  
  attachServos();
  
  // Move to home positions
  servo1.write(HOME_BASE);
  servo2.write(HOME_SHOULDER);
  servo3.write(HOME_ELBOW);
  servo4.write(HOME_WRIST);
  servo5.write(HOME_GRIPPER);
  
  delay(500);
  addLog("All 5 servos initialized and homed");
}

void attachServos() {
  if (!servo1.attached()) servo1.attach(SERVO1_PIN, 500, 2400);
  if (!servo2.attached()) servo2.attach(SERVO2_PIN, 500, 2400);
  if (!servo3.attached()) servo3.attach(SERVO3_PIN, 500, 2400);
  if (!servo4.attached()) servo4.attach(SERVO4_PIN, 500, 2400);
  if (!servo5.attached()) servo5.attach(SERVO5_PIN, 500, 2400);
  servosEnabled = true;
  emergencyStop = false;
}

void detachServos() {
  servo1.detach();
  servo2.detach();
  servo3.detach();
  servo4.detach();
  servo5.detach();
  servosEnabled = false;
  emergencyStop = true;
}

void setServoAngle(Servo &servo, int angle, int servoNum = 0) {
  // Apply different limits based on servo number
  if (servoNum == 5) { // Gripper
    angle = constrain(angle, GRIPPER_MIN, GRIPPER_MAX);
  } else if (servoNum == 4) { // Wrist
    angle = constrain(angle, WRIST_MIN, WRIST_MAX);
  } else { // Base, Shoulder, Elbow
    angle = constrain(angle, SERVO_MIN, SERVO_MAX);
  }
  servo.write(angle);
}

void moveServo(int servoNum, int angle, bool immediate = false) {
  if (emergencyStop && !immediate) {
    return;
  }
  
  // Apply servo-specific limits
  int constrainedAngle = angle;
  if (servoNum == 5) { // Gripper
    constrainedAngle = constrain(angle, GRIPPER_MIN, GRIPPER_MAX);
  } else if (servoNum == 4) { // Wrist
    constrainedAngle = constrain(angle, WRIST_MIN, WRIST_MAX);
  } else { // Base, Shoulder, Elbow
    constrainedAngle = constrain(angle, SERVO_MIN, SERVO_MAX);
  }
  
  if (!immediate && smoothMoveEnabled) {
    // Set target for smooth movement
    switch(servoNum) {
      case 1: targetPos1 = constrainedAngle; break;
      case 2: targetPos2 = constrainedAngle; break;
      case 3: targetPos3 = constrainedAngle; break;
      case 4: targetPos4 = constrainedAngle; break;
      case 5: targetPos5 = constrainedAngle; break;
    }
  } else {
    // Immediate movement
    switch(servoNum) {
      case 1:
        setServoAngle(servo1, constrainedAngle, 1);
        currentPos1 = constrainedAngle;
        break;
      case 2:
        setServoAngle(servo2, constrainedAngle, 2);
        currentPos2 = constrainedAngle;
        break;
      case 3:
        setServoAngle(servo3, constrainedAngle, 3);
        currentPos3 = constrainedAngle;
        break;
      case 4:
        setServoAngle(servo4, constrainedAngle, 4);
        currentPos4 = constrainedAngle;
        break;
      case 5:
        setServoAngle(servo5, constrainedAngle, 5);
        currentPos5 = constrainedAngle;
        break;
    }
  }
  
  lastActivityTime = millis();
}

void updateServos() {
  unsigned long currentTime = millis();
  if (currentTime - lastServoUpdate < SERVO_UPDATE_INTERVAL) return;
  
  lastServoUpdate = currentTime;
  
  if (smoothMoveEnabled && !emergencyStop) {
    int step = map(moveSpeed, 1, 100, 1, 10);
    
    // Smooth movement for all 5 servos
    if (currentPos1 != targetPos1) {
      int dir = targetPos1 > currentPos1 ? 1 : -1;
      currentPos1 += dir * min(step, abs(targetPos1 - currentPos1));
      setServoAngle(servo1, currentPos1, 1);
    }
    
    if (currentPos2 != targetPos2) {
      int dir = targetPos2 > currentPos2 ? 1 : -1;
      currentPos2 += dir * min(step, abs(targetPos2 - currentPos2));
      setServoAngle(servo2, currentPos2, 2);
    }
    
    if (currentPos3 != targetPos3) {
      int dir = targetPos3 > currentPos3 ? 1 : -1;
      currentPos3 += dir * min(step, abs(targetPos3 - currentPos3));
      setServoAngle(servo3, currentPos3, 3);
    }
    
    if (currentPos4 != targetPos4) {
      int dir = targetPos4 > currentPos4 ? 1 : -1;
      currentPos4 += dir * min(step, abs(targetPos4 - currentPos4));
      setServoAngle(servo4, currentPos4, 4);
    }
    
    if (currentPos5 != targetPos5) {
      int dir = targetPos5 > currentPos5 ? 1 : -1;
      currentPos5 += dir * min(step, abs(targetPos5 - currentPos5));
      setServoAngle(servo5, currentPos5, 5);
    }
  }
}

void homeAllServos() {
  targetPos1 = HOME_BASE;
  targetPos2 = HOME_SHOULDER;
  targetPos3 = HOME_ELBOW;
  targetPos4 = HOME_WRIST;
  targetPos5 = HOME_GRIPPER;
  
  if (!smoothMoveEnabled) {
    setServoAngle(servo1, HOME_BASE, 1);
    setServoAngle(servo2, HOME_SHOULDER, 2);
    setServoAngle(servo3, HOME_ELBOW, 3);
    setServoAngle(servo4, HOME_WRIST, 4);
    setServoAngle(servo5, HOME_GRIPPER, 5);
    
    currentPos1 = HOME_BASE;
    currentPos2 = HOME_SHOULDER;
    currentPos3 = HOME_ELBOW;
    currentPos4 = HOME_WRIST;
    currentPos5 = HOME_GRIPPER;
  }
  
  addLog("All 5 servos moved to home position");
}

void emergencyStopAll() {
  detachServos();
  playbackActive = false;
  addLog("EMERGENCY STOP ACTIVATED!", "error");
}

// ==================== EEPROM FUNCTIONS ====================
void initEEPROM() {
  if (!EEPROM.begin(EEPROM_SIZE)) {
    addLog("Failed to initialize EEPROM!", "error");
    return;
  }

  int recordEnd = RECORD_DATA_ADDR + (RECORD_MAX_STEPS * RECORD_STEP_SIZE);
  if (recordEnd > EEPROM_SIZE) {
    addLog("Recording storage exceeds EEPROM size", "warning");
  }
  
  // Check if EEPROM is initialized
  byte initFlag = EEPROM.read(0);
  if (initFlag != 0xFF) {
    // Initialize EEPROM
    for (int i = 0; i < EEPROM_SIZE; i++) {
      EEPROM.write(i, 0);
    }
    EEPROM.write(0, 0xFF);
    EEPROM.commit();
    addLog("EEPROM initialized for 5 servos");
  } else {
    addLog("EEPROM already initialized");
  }
  
  loadAllPresets();
  loadRecordingFromEEPROM();
}

void savePreset(int index, const PresetPosition &preset) {
  if (index < 0 || index >= MAX_PRESETS) return;
  
  int addr = 1 + (index * PRESET_SIZE);
  
  // Write name (32 bytes)
  for (int i = 0; i < 32; i++) {
    EEPROM.write(addr + i, i < strlen(preset.name) ? preset.name[i] : 0);
  }
  
  // Write 5 servo angles and speed (6 bytes total)
  EEPROM.write(addr + 32, preset.servo1);
  EEPROM.write(addr + 33, preset.servo2);
  EEPROM.write(addr + 34, preset.servo3);
  EEPROM.write(addr + 35, preset.servo4);
  EEPROM.write(addr + 36, preset.servo5);
  EEPROM.write(addr + 37, preset.speed);
  
  EEPROM.commit();
  
  presets[index] = preset;
  if (index >= nextPresetIndex) {
    nextPresetIndex = index + 1;
  }
  
  addLog("Preset saved: " + String(preset.name) + " (5 servos)");
}

bool loadPreset(int index, PresetPosition &preset) {
  if (index < 0 || index >= MAX_PRESETS) return false;
  
  int addr = 1 + (index * PRESET_SIZE);
  
  // Read name
  char name[32];
  for (int i = 0; i < 32; i++) {
    name[i] = EEPROM.read(addr + i);
  }
  name[31] = '\0';
  
  if (strlen(name) == 0) return false;
  
  strcpy(preset.name, name);
  preset.servo1 = EEPROM.read(addr + 32);
  preset.servo2 = EEPROM.read(addr + 33);
  preset.servo3 = EEPROM.read(addr + 34);
  preset.servo4 = EEPROM.read(addr + 35);
  preset.servo5 = EEPROM.read(addr + 36);
  preset.speed = EEPROM.read(addr + 37);
  
  return true;
}

void loadAllPresets() {
  nextPresetIndex = 0;
  for (int i = 0; i < MAX_PRESETS; i++) {
    if (loadPreset(i, presets[i])) {
      nextPresetIndex = i + 1;
    }
  }
  addLog("Loaded " + String(nextPresetIndex) + " presets (5 servos) from EEPROM");
}

void clearAllPresets() {
  for (int i = 0; i < MAX_PRESETS; i++) {
    memset(presets[i].name, 0, 32);
    presets[i].servo1 = 0;
    presets[i].servo2 = 0;
    presets[i].servo3 = 0;
    presets[i].servo4 = 0;
    presets[i].servo5 = 0;
    presets[i].speed = DEFAULT_SPEED;
    
    int addr = 1 + (i * PRESET_SIZE);
    for (int j = 0; j < 38; j++) {
      EEPROM.write(addr + j, 0);
    }
  }
  EEPROM.commit();
  nextPresetIndex = 0;
  addLog("All presets cleared");
}

// ==================== RECORDING FUNCTIONS ====================
int getRecordedAngle(int servoNum) {
  if (smoothMoveEnabled) {
    switch (servoNum) {
      case 1: return targetPos1;
      case 2: return targetPos2;
      case 3: return targetPos3;
      case 4: return targetPos4;
      case 5: return targetPos5;
    }
  }
  switch (servoNum) {
    case 1: return currentPos1;
    case 2: return currentPos2;
    case 3: return currentPos3;
    case 4: return currentPos4;
    case 5: return currentPos5;
  }
  return HOME_BASE;
}

void recordSnapshot() {
  if (!recordingActive || playbackActive) return;
  if (recordCount >= RECORD_MAX_STEPS) {
    recordingActive = false;
    addLog("Recording full, stopped", "warning");
    return;
  }

  unsigned long now = millis();
  uint16_t delta = (recordCount == 0) ? 0 : (uint16_t)min((unsigned long)65535, now - lastRecordTime);
  lastRecordTime = now;

  RecordStep &step = recordSteps[recordCount++];
  step.servo1 = (uint8_t)constrain(getRecordedAngle(1), SERVO_MIN, SERVO_MAX);
  step.servo2 = (uint8_t)constrain(getRecordedAngle(2), SERVO_MIN, SERVO_MAX);
  step.servo3 = (uint8_t)constrain(getRecordedAngle(3), SERVO_MIN, SERVO_MAX);
  step.servo4 = (uint8_t)constrain(getRecordedAngle(4), WRIST_MIN, WRIST_MAX);
  step.servo5 = (uint8_t)constrain(getRecordedAngle(5), GRIPPER_MIN, GRIPPER_MAX);
  step.delayMs = delta;
}

void saveRecordingToEEPROM() {
  int endAddr = RECORD_DATA_ADDR + (RECORD_MAX_STEPS * RECORD_STEP_SIZE);
  if (endAddr > EEPROM_SIZE) {
    addLog("Recording EEPROM area exceeds EEPROM_SIZE", "error");
    return;
  }

  EEPROM.write(RECORD_EEPROM_ADDR, (uint8_t)(recordCount & 0xFF));
  EEPROM.write(RECORD_EEPROM_ADDR + 1, (uint8_t)((recordCount >> 8) & 0xFF));

  for (uint16_t i = 0; i < recordCount; i++) {
    int addr = RECORD_DATA_ADDR + (i * RECORD_STEP_SIZE);
    EEPROM.write(addr + 0, recordSteps[i].servo1);
    EEPROM.write(addr + 1, recordSteps[i].servo2);
    EEPROM.write(addr + 2, recordSteps[i].servo3);
    EEPROM.write(addr + 3, recordSteps[i].servo4);
    EEPROM.write(addr + 4, recordSteps[i].servo5);
    EEPROM.write(addr + 5, (uint8_t)(recordSteps[i].delayMs & 0xFF));
    EEPROM.write(addr + 6, (uint8_t)((recordSteps[i].delayMs >> 8) & 0xFF));
  }

  EEPROM.commit();
}

void loadRecordingFromEEPROM() {
  int endAddr = RECORD_DATA_ADDR + (RECORD_MAX_STEPS * RECORD_STEP_SIZE);
  if (endAddr > EEPROM_SIZE) {
    addLog("Recording EEPROM area exceeds EEPROM_SIZE", "error");
    return;
  }

  uint16_t storedCount = EEPROM.read(RECORD_EEPROM_ADDR) | (EEPROM.read(RECORD_EEPROM_ADDR + 1) << 8);
  if (storedCount > RECORD_MAX_STEPS) {
    recordCount = 0;
    return;
  }

  recordCount = storedCount;
  for (uint16_t i = 0; i < recordCount; i++) {
    int addr = RECORD_DATA_ADDR + (i * RECORD_STEP_SIZE);
    recordSteps[i].servo1 = EEPROM.read(addr + 0);
    recordSteps[i].servo2 = EEPROM.read(addr + 1);
    recordSteps[i].servo3 = EEPROM.read(addr + 2);
    recordSteps[i].servo4 = EEPROM.read(addr + 3);
    recordSteps[i].servo5 = EEPROM.read(addr + 4);
    recordSteps[i].delayMs = (uint16_t)EEPROM.read(addr + 5) | ((uint16_t)EEPROM.read(addr + 6) << 8);
  }
}

void clearRecording() {
  recordCount = 0;
  recordingActive = false;
  playbackActive = false;
  saveRecordingToEEPROM();
}

void startRecording() {
  if (playbackActive) return;
  if (!servosEnabled) {
    attachServos();
  }

  recordingActive = true;
  recordCount = 0;
  lastRecordTime = millis();
  recordSnapshot();
  addLog("Recording started");
}

void stopRecording() {
  if (!recordingActive) return;
  recordingActive = false;
  saveRecordingToEEPROM();
  addLog("Recording stopped and saved");
}

void startPlayback() {
  if (recordCount == 0) return;
  if (!servosEnabled) {
    attachServos();
  }

  playbackActive = true;
  playbackIndex = 0;
  nextPlaybackTime = millis();
  addLog("Playback started");
}

void stopPlayback() {
  if (!playbackActive) return;
  playbackActive = false;
  addLog("Playback stopped");
}

void updatePlayback() {
  if (!playbackActive) return;
  if (playbackIndex >= recordCount) {
    playbackActive = false;
    addLog("Playback complete");
    return;
  }

  unsigned long now = millis();
  if (now < nextPlaybackTime) return;

  RecordStep &step = recordSteps[playbackIndex];
  moveServo(1, step.servo1);
  moveServo(2, step.servo2);
  moveServo(3, step.servo3);
  moveServo(4, step.servo4);
  moveServo(5, step.servo5);

  greenPulseUntil = now + GREEN_STEP_PULSE_MS;

  nextPlaybackTime = now + step.delayMs;
  playbackIndex++;
}

// ==================== POTENTIOMETER CONTROL ====================
int readPotAngleFiltered(int pin, int rawMin, int rawMax, int minAngle, int maxAngle, float &filterState) {
  long total = 0;
  for (int i = 0; i < POT_SAMPLES; i++) {
    total += analogRead(pin);
  }
  int raw = (int)(total / POT_SAMPLES);
  int clampedRaw = constrain(raw, rawMin, rawMax);
  int angle = map(clampedRaw, rawMin, rawMax, minAngle, maxAngle);
  angle = constrain(angle, minAngle, maxAngle);

  filterState = filterState + (POT_FILTER_ALPHA * (angle - filterState));
  int filteredAngle = (int)round(filterState);
  return constrain(filteredAngle, minAngle, maxAngle);
}

void syncPotState() {
  lastPotBase = readPotAngleFiltered(POT_BASE_PIN, POT_BASE_ADC_MIN, POT_BASE_ADC_MAX, SERVO_MIN, SERVO_MAX, filteredPotBase);
  lastPotShoulder = readPotAngleFiltered(POT_SHOULDER_PIN, POT_SHOULDER_ADC_MIN, POT_SHOULDER_ADC_MAX, SERVO_MIN, SERVO_MAX, filteredPotShoulder);
  lastPotElbow = readPotAngleFiltered(POT_ELBOW_PIN, POT_ELBOW_ADC_MIN, POT_ELBOW_ADC_MAX, SERVO_MIN, SERVO_MAX, filteredPotElbow);
  lastPotWrist = readPotAngleFiltered(POT_WRIST_PIN, POT_WRIST_ADC_MIN, POT_WRIST_ADC_MAX, WRIST_MIN, WRIST_MAX, filteredPotWrist);
}

void updatePotControl() {
  if (!potControlEnabled || playbackActive || emergencyStop || !servosEnabled) return;

  unsigned long now = millis();
  if (now - lastPotRead < POT_READ_INTERVAL) return;
  lastPotRead = now;

  int baseAngle = readPotAngleFiltered(POT_BASE_PIN, POT_BASE_ADC_MIN, POT_BASE_ADC_MAX, SERVO_MIN, SERVO_MAX, filteredPotBase);
  int shoulderAngle = readPotAngleFiltered(POT_SHOULDER_PIN, POT_SHOULDER_ADC_MIN, POT_SHOULDER_ADC_MAX, SERVO_MIN, SERVO_MAX, filteredPotShoulder);
  int elbowAngle = readPotAngleFiltered(POT_ELBOW_PIN, POT_ELBOW_ADC_MIN, POT_ELBOW_ADC_MAX, SERVO_MIN, SERVO_MAX, filteredPotElbow);
  int wristAngle = readPotAngleFiltered(POT_WRIST_PIN, POT_WRIST_ADC_MIN, POT_WRIST_ADC_MAX, WRIST_MIN, WRIST_MAX, filteredPotWrist);

  bool moved = false;

  if (abs(baseAngle - lastPotBase) >= POT_DEADBAND) {
    moveServo(1, baseAngle);
    lastPotBase = baseAngle;
    moved = true;
  }

  if (abs(shoulderAngle - lastPotShoulder) >= POT_DEADBAND) {
    moveServo(2, shoulderAngle);
    lastPotShoulder = shoulderAngle;
    moved = true;
  }

  if (abs(elbowAngle - lastPotElbow) >= POT_DEADBAND) {
    moveServo(3, elbowAngle);
    lastPotElbow = elbowAngle;
    moved = true;
  }

  if (abs(wristAngle - lastPotWrist) >= POT_DEADBAND) {
    moveServo(4, wristAngle);
    lastPotWrist = wristAngle;
    moved = true;
  }

  if (moved) {
    recordSnapshot();
  }
}

void handlePotEnable() {
  potControlEnabled = true;
  syncPotState();
  server.send(200, "application/json", "{\"success\":true,\"pot_mode\":true,\"message\":\"Potentiometer control enabled\"}");
}

void handlePotDisable() {
  potControlEnabled = false;
  server.send(200, "application/json", "{\"success\":true,\"pot_mode\":false,\"message\":\"Potentiometer control disabled\"}");
}

void handlePotToggle() {
  potControlEnabled = !potControlEnabled;
  if (potControlEnabled) {
    syncPotState();
    server.send(200, "application/json", "{\"success\":true,\"pot_mode\":true,\"message\":\"Potentiometer control enabled\"}");
  } else {
    server.send(200, "application/json", "{\"success\":true,\"pot_mode\":false,\"message\":\"Potentiometer control disabled\"}");
  }
}

// ==================== UTILITY FUNCTIONS ====================
void addLog(const String &message, const String &type) {
  String timestamp = getTimestamp();
  String logEntry = "[" + timestamp + "] ";
  
  if (type == "error") logEntry += "ERROR: ";
  else if (type == "warning") logEntry += "WARNING: ";
  else if (type == "success") logEntry += "SUCCESS: ";
  
  logEntry += message;
  
  Serial.println(logEntry);
  
  // Keep last logs
  systemLogs = logEntry + "\n" + systemLogs;
  int maxLogs = 5000;
  if (systemLogs.length() > maxLogs) {
    systemLogs = systemLogs.substring(0, maxLogs);
  }
}

String getTimestamp() {
  unsigned long currentMillis = millis();
  unsigned long seconds = currentMillis / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  
  char buffer[12];
  snprintf(buffer, sizeof(buffer), "%02lu:%02lu:%02lu", hours % 24, minutes % 60, seconds % 60);
  return String(buffer);
}

String getSystemStatus() {
  DynamicJsonDocument doc(1024);
  doc["success"] = true;
  doc["firmware"] = FIRMWARE_VERSION;
  doc["servo1"] = currentPos1;
  doc["servo2"] = currentPos2;
  doc["servo3"] = currentPos3;
  doc["servo4"] = currentPos4;
  doc["servo5"] = currentPos5;
  doc["servos_enabled"] = servosEnabled;
  doc["emergency_stop"] = emergencyStop;
  doc["wifi_connected"] = wifiConnected;
  doc["ap_mode"] = apMode;
  doc["smooth_move"] = smoothMoveEnabled;
  doc["speed"] = moveSpeed;
  doc["recording_active"] = recordingActive;
  doc["playback_active"] = playbackActive;
  doc["record_count"] = recordCount;
  doc["pot_mode"] = potControlEnabled;
  
  if (apMode) {
    doc["ip"] = WiFi.softAPIP().toString();
  } else {
    doc["ip"] = WiFi.localIP().toString();
  }
  
  unsigned long uptimeSeconds = (millis() - systemStartTime) / 1000;
  char uptimeStr[20];
  snprintf(uptimeStr, sizeof(uptimeStr), "%02lu:%02lu:%02lu",
           uptimeSeconds / 3600, (uptimeSeconds % 3600) / 60, uptimeSeconds % 60);
  doc["uptime"] = uptimeStr;
  
  doc["free_memory"] = ESP.getFreeHeap();
  doc["last_activity"] = millis() - lastActivityTime;
  
  String response;
  serializeJson(doc, response);
  return response;
}

void checkInactivity() {
  unsigned long currentTime = millis();
  if (currentTime - lastActivityTime > INACTIVITY_TIMEOUT) {
    if (servosEnabled) {
      detachServos();
      addLog("Servos disabled due to inactivity", "warning");
    }
  }
}

void setLedState(int pin, bool on) {
  bool level = LED_ACTIVE_HIGH ? on : !on;
  digitalWrite(pin, level ? HIGH : LOW);
}

void initLeds() {
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  setLedState(LED_GREEN_PIN, false);
  setLedState(LED_BLUE_PIN, false);
  setLedState(LED_RED_PIN, false);
}

void updateStatusLeds() {
  unsigned long now = millis();

  if (emergencyStop) {
    setLedState(LED_RED_PIN, true);
    setLedState(LED_BLUE_PIN, false);
    setLedState(LED_GREEN_PIN, false);
    return;
  }

  if (recordingActive) {
    setLedState(LED_RED_PIN, false);
    setLedState(LED_BLUE_PIN, true);
    setLedState(LED_GREEN_PIN, false);
    return;
  }

  if (playbackActive) {
    if (now - lastBlueBlinkToggle >= BLUE_BLINK_INTERVAL_MS) {
      lastBlueBlinkToggle = now;
      blueBlinkOn = !blueBlinkOn;
    }
    setLedState(LED_RED_PIN, false);
    setLedState(LED_BLUE_PIN, blueBlinkOn);
    setLedState(LED_GREEN_PIN, now < greenPulseUntil);
    return;
  }

  bool ready = servosEnabled && !emergencyStop;
  setLedState(LED_RED_PIN, false);
  setLedState(LED_BLUE_PIN, false);
  setLedState(LED_GREEN_PIN, ready);
}

// ==================== WEB HANDLERS ====================
void handleRoot() {
  server.send(200, "text/html", generateHTML());
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  
  server.send(404, "text/plain", message);
}

void handleGetStatus() {
  server.send(200, "application/json", getSystemStatus());
}

void handleSetServo() {
  if (potControlEnabled) {
    server.send(409, "application/json", "{\"success\":false,\"error\":\"Potentiometer mode active. Disable pot mode to use web sliders.\"}");
    return;
  }

  if (!server.hasArg("angle")) {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"Missing angle parameter\"}");
    return;
  }

  if (playbackActive) {
    server.send(409, "application/json", "{\"success\":false,\"error\":\"Playback active. Stop playback to control servos.\"}");
    return;
  }
  
  String path = server.uri();
  int servoNum = 0;
  
  if (path.endsWith("/1")) servoNum = 1;
  else if (path.endsWith("/2")) servoNum = 2;
  else if (path.endsWith("/3")) servoNum = 3;
  else if (path.endsWith("/4")) servoNum = 4;
  else if (path.endsWith("/5")) servoNum = 5;
  else {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"Invalid servo number (1-5)\"}");
    return;
  }
  
  int angle = server.arg("angle").toInt();
  
  // Validate angle based on servo type
  if (servoNum == 5) { // Gripper
    if (angle < GRIPPER_MIN || angle > GRIPPER_MAX) {
      server.send(400, "application/json", "{\"success\":false,\"error\":\"Gripper angle out of range (0-90)\"}");
      return;
    }
  } else if (servoNum == 4) { // Wrist
    if (angle < WRIST_MIN || angle > WRIST_MAX) {
      server.send(400, "application/json", "{\"success\":false,\"error\":\"Wrist angle out of range (0-180)\"}");
      return;
    }
  } else { // Base, Shoulder, Elbow
    if (angle < SERVO_MIN || angle > SERVO_MAX) {
      server.send(400, "application/json", "{\"success\":false,\"error\":\"Angle out of range (0-180)\"}");
      return;
    }
  }
  
  moveServo(servoNum, angle);
  recordSnapshot();
  
  DynamicJsonDocument doc(256);
  doc["success"] = true;
  doc["servo"] = servoNum;
  doc["angle"] = angle;
  String servoName = "";
  switch(servoNum) {
    case 1: servoName = "Base"; break;
    case 2: servoName = "Shoulder"; break;
    case 3: servoName = "Elbow"; break;
    case 4: servoName = "Wrist"; break;
    case 5: servoName = "Gripper"; break;
  }
  doc["message"] = servoName + " moved to " + String(angle) + "°";
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleEmergencyStop() {
  emergencyStopAll();
  server.send(200, "application/json", "{\"success\":true,\"message\":\"Emergency stop activated\"}");
}

void handleEnableServos() {
  attachServos();
  server.send(200, "application/json", "{\"success\":true,\"message\":\"All 5 servos enabled\"}");
}

void handleHomePosition() {
  if (playbackActive) {
    server.send(409, "application/json", "{\"success\":false,\"error\":\"Playback active. Stop playback to control servos.\"}");
    return;
  }
  homeAllServos();
  recordSnapshot();
  server.send(200, "application/json", "{\"success\":true,\"message\":\"All 5 servos moved to home position\"}");
}

void handleSavePreset() {
  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"success\":false,\"error\":\"Method not allowed\"}");
    return;
  }
  
  String body = server.arg("plain");
  if (body.length() == 0) {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"Empty request body\"}");
    return;
  }
  
  DynamicJsonDocument doc(256);
  DeserializationError error = deserializeJson(doc, body);
  
  if (error) {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
    return;
  }
  
  if (!doc.containsKey("name") || nextPresetIndex >= MAX_PRESETS) {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"Missing name or no slots available\"}");
    return;
  }
  
  PresetPosition preset;
  strncpy(preset.name, doc["name"].as<const char*>(), 31);
  preset.servo1 = doc.containsKey("servo1") ? doc["servo1"].as<int>() : currentPos1;
  preset.servo2 = doc.containsKey("servo2") ? doc["servo2"].as<int>() : currentPos2;
  preset.servo3 = doc.containsKey("servo3") ? doc["servo3"].as<int>() : currentPos3;
  preset.servo4 = doc.containsKey("servo4") ? doc["servo4"].as<int>() : currentPos4;
  preset.servo5 = doc.containsKey("servo5") ? doc["servo5"].as<int>() : currentPos5;
  preset.speed = DEFAULT_SPEED;
  
  int index = nextPresetIndex;
  savePreset(index, preset);
  
  DynamicJsonDocument responseDoc(256);
  responseDoc["success"] = true;
  responseDoc["index"] = index;
  responseDoc["message"] = "Preset saved successfully (5 servos)";
  
  String response;
  serializeJson(responseDoc, response);
  server.send(200, "application/json", response);
}

void handleLoadPreset() {
  String path = server.uri();
  int lastSlash = path.lastIndexOf('/');
  if (lastSlash == -1) {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"Invalid URL\"}");
    return;
  }
  
  int index = path.substring(lastSlash + 1).toInt();
  
  if (index < 0 || index >= MAX_PRESETS) {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"Invalid preset index\"}");
    return;
  }
  
  PresetPosition preset;
  if (!loadPreset(index, preset)) {
    server.send(404, "application/json", "{\"success\":false,\"error\":\"Preset not found\"}");
    return;
  }
  
  // Move all 5 servos to preset position
  moveServo(1, preset.servo1);
  moveServo(2, preset.servo2);
  moveServo(3, preset.servo3);
  moveServo(4, preset.servo4);
  moveServo(5, preset.servo5);
  
  moveSpeed = DEFAULT_SPEED;
  
  DynamicJsonDocument doc(512);
  doc["success"] = true;
  doc["index"] = index;
  doc["position"]["name"] = preset.name;
  doc["position"]["servo1"] = preset.servo1;
  doc["position"]["servo2"] = preset.servo2;
  doc["position"]["servo3"] = preset.servo3;
  doc["position"]["servo4"] = preset.servo4;
  doc["position"]["servo5"] = preset.servo5;
  doc["position"]["speed"] = DEFAULT_SPEED;
  doc["message"] = "Preset loaded: " + String(preset.name) + " (5 servos)";
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleGetPresets() {
  DynamicJsonDocument doc(2048);
  JsonArray presetsArray = doc.createNestedArray("presets");
  
  for (int i = 0; i < MAX_PRESETS; i++) {
    JsonObject presetObj = presetsArray.createNestedObject();
    if (strlen(presets[i].name) > 0) {
      presetObj["name"] = presets[i].name;
      presetObj["servo1"] = presets[i].servo1;
      presetObj["servo2"] = presets[i].servo2;
      presetObj["servo3"] = presets[i].servo3;
      presetObj["servo4"] = presets[i].servo4;
      presetObj["servo5"] = presets[i].servo5;
      presetObj["speed"] = DEFAULT_SPEED;
      presetObj["index"] = i;
    } else {
      presetObj["name"] = "";
      presetObj["servo1"] = 0;
      presetObj["servo2"] = 0;
      presetObj["servo3"] = 0;
      presetObj["servo4"] = 0;
      presetObj["servo5"] = 0;
      presetObj["speed"] = DEFAULT_SPEED;
      presetObj["index"] = i;
    }
  }
  
  doc["success"] = true;
  doc["count"] = nextPresetIndex;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleClearPresets() {
  clearAllPresets();
  server.send(200, "application/json", "{\"success\":true,\"message\":\"All presets cleared\"}");
}

void handleSetSpeed() {
  if (!server.hasArg("speed")) {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"Missing speed parameter\"}");
    return;
  }
  
  moveSpeed = DEFAULT_SPEED;
  
  DynamicJsonDocument doc(256);
  doc["success"] = true;
  doc["speed"] = moveSpeed;
  doc["message"] = "Speed fixed at " + String(moveSpeed) + "%";
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleSetAllServos() {
  if (potControlEnabled) {
    server.send(409, "application/json", "{\"success\":false,\"error\":\"Potentiometer mode active. Disable pot mode to use web sliders.\"}");
    return;
  }

  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"success\":false,\"error\":\"Method not allowed\"}");
    return;
  }

  if (playbackActive) {
    server.send(409, "application/json", "{\"success\":false,\"error\":\"Playback active. Stop playback to control servos.\"}");
    return;
  }

  String body = server.arg("plain");
  if (body.length() == 0) {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"Empty request body\"}");
    return;
  }

  DynamicJsonDocument doc(256);
  DeserializationError error = deserializeJson(doc, body);
  if (error) {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
    return;
  }

  if (doc.containsKey("servo1")) moveServo(1, doc["servo1"].as<int>());
  if (doc.containsKey("servo2")) moveServo(2, doc["servo2"].as<int>());
  if (doc.containsKey("servo3")) moveServo(3, doc["servo3"].as<int>());
  if (doc.containsKey("servo4")) moveServo(4, doc["servo4"].as<int>());
  if (doc.containsKey("servo5")) moveServo(5, doc["servo5"].as<int>());

  moveSpeed = DEFAULT_SPEED;

  recordSnapshot();

  DynamicJsonDocument responseDoc(256);
  responseDoc["success"] = true;
  responseDoc["message"] = "Servos updated";
  String response;
  serializeJson(responseDoc, response);
  server.send(200, "application/json", response);
}

void handleRecordStart() {
  if (playbackActive) {
    server.send(409, "application/json", "{\"success\":false,\"error\":\"Playback active. Stop playback first.\"}");
    return;
  }

  startRecording();

  DynamicJsonDocument doc(256);
  doc["success"] = true;
  doc["recording"] = recordingActive;
  doc["count"] = recordCount;
  doc["message"] = "Recording started";
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleRecordStop() {
  stopRecording();

  DynamicJsonDocument doc(256);
  doc["success"] = true;
  doc["recording"] = recordingActive;
  doc["count"] = recordCount;
  doc["message"] = "Recording stopped and saved";
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleRecordPlay() {
  if (recordingActive) {
    server.send(409, "application/json", "{\"success\":false,\"error\":\"Recording active. Stop recording first.\"}");
    return;
  }

  if (recordCount == 0) {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"No recorded steps to play\"}");
    return;
  }

  startPlayback();

  DynamicJsonDocument doc(256);
  doc["success"] = true;
  doc["playback"] = playbackActive;
  doc["count"] = recordCount;
  doc["message"] = "Playback started";
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleRecordStopPlayback() {
  stopPlayback();

  DynamicJsonDocument doc(256);
  doc["success"] = true;
  doc["playback"] = playbackActive;
  doc["message"] = "Playback stopped";
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleRecordClear() {
  clearRecording();

  DynamicJsonDocument doc(256);
  doc["success"] = true;
  doc["count"] = recordCount;
  doc["message"] = "Recording cleared";
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleGetLogs() {
  DynamicJsonDocument doc(4096);
  doc["success"] = true;
  
  // Parse logs into array
  JsonArray logsArray = doc.createNestedArray("logs");
  String logCopy = systemLogs;
  int startPos = 0;
  int logCount = 0;
  
  while (startPos < logCopy.length() && logCount < 50) {
    int endPos = logCopy.indexOf('\n', startPos);
    if (endPos == -1) endPos = logCopy.length();
    
    String line = logCopy.substring(startPos, endPos);
    startPos = endPos + 1;
    
    if (line.length() > 0) {
      JsonObject logEntry = logsArray.createNestedObject();
      
      // Extract timestamp
      int timestampEnd = line.indexOf(']');
      if (timestampEnd != -1) {
        logEntry["timestamp"] = line.substring(1, timestampEnd);
        String message = line.substring(timestampEnd + 2);
        
        // Determine log type
        if (message.startsWith("ERROR:")) {
          logEntry["type"] = "error";
          logEntry["message"] = message.substring(7);
        } else if (message.startsWith("WARNING:")) {
          logEntry["type"] = "warning";
          logEntry["message"] = message.substring(9);
        } else if (message.startsWith("SUCCESS:")) {
          logEntry["type"] = "success";
          logEntry["message"] = message.substring(9);
        } else {
          logEntry["type"] = "info";
          logEntry["message"] = message;
        }
        logCount++;
      }
    }
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

// Special gripper controls
void handleGripperOpen() {
  if (playbackActive) {
    server.send(409, "application/json", "{\"success\":false,\"error\":\"Playback active. Stop playback to control servos.\"}");
    return;
  }
  moveServo(5, GRIPPER_OPEN_ANGLE);
  recordSnapshot();
  server.send(200, "application/json", "{\"success\":true,\"message\":\"Gripper fully opened\"}");
}

void handleGripperClose() {
  if (playbackActive) {
    server.send(409, "application/json", "{\"success\":false,\"error\":\"Playback active. Stop playback to control servos.\"}");
    return;
  }
  moveServo(5, GRIPPER_CLOSE_ANGLE);
  recordSnapshot();
  server.send(200, "application/json", "{\"success\":true,\"message\":\"Gripper fully closed\"}");
}

void handleGripperHome() {
  if (playbackActive) {
    server.send(409, "application/json", "{\"success\":false,\"error\":\"Playback active. Stop playback to control servos.\"}");
    return;
  }
  moveServo(5, HOME_GRIPPER);
  recordSnapshot();
  server.send(200, "application/json", "{\"success\":true,\"message\":\"Gripper moved to home position\"}");
}

// Wrist special controls
void handleWristUp() {
  if (playbackActive) {
    server.send(409, "application/json", "{\"success\":false,\"error\":\"Playback active. Stop playback to control servos.\"}");
    return;
  }
  moveServo(4, WRIST_MAX);
  recordSnapshot();
  server.send(200, "application/json", "{\"success\":true,\"message\":\"Wrist pitched up\"}");
}

void handleWristDown() {
  if (playbackActive) {
    server.send(409, "application/json", "{\"success\":false,\"error\":\"Playback active. Stop playback to control servos.\"}");
    return;
  }
  moveServo(4, WRIST_MIN);
  recordSnapshot();
  server.send(200, "application/json", "{\"success\":true,\"message\":\"Wrist pitched down\"}");
}

void handleWristHome() {
  if (playbackActive) {
    server.send(409, "application/json", "{\"success\":false,\"error\":\"Playback active. Stop playback to control servos.\"}");
    return;
  }
  moveServo(4, HOME_WRIST);
  recordSnapshot();
  server.send(200, "application/json", "{\"success\":true,\"message\":\"Wrist moved to home position\"}");
}

// ==================== HTML GENERATION ====================
String generateHTML() {
  String html = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>5-DOF Robotic Arm Controller</title>
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
    <style>
        :root {
            --primary: #2c3e50;
            --secondary: #3498db;
            --success: #27ae60;
            --warning: #f39c12;
            --danger: #e74c3c;
            --dark: #1a252f;
            --light: #ecf0f1;
            --gray: #95a5a6;
        }
        
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
        }
        
        body {
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            color: var(--light);
            min-height: 100vh;
            padding: 20px;
        }
        
        .container {
            max-width: 1600px;
            margin: 0 auto;
        }
        
        header {
            background: linear-gradient(90deg, var(--primary) 0%, var(--dark) 100%);
            padding: 20px;
            border-radius: 10px;
            margin-bottom: 20px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        
        .logo {
            display: flex;
            align-items: center;
            gap: 15px;
        }
        
        .logo i {
            font-size: 2.5rem;
            color: var(--secondary);
        }
        
        .logo h1 {
            font-size: 1.8rem;
            font-weight: 600;
        }
        
        .status-indicators {
            display: flex;
            gap: 20px;
            flex-wrap: wrap;
        }
        
        .status-item {
            display: flex;
            align-items: center;
            gap: 8px;
            padding: 8px 15px;
            background: rgba(255,255,255,0.1);
            border-radius: 20px;
        }
        
        .status-dot {
            width: 12px;
            height: 12px;
            border-radius: 50%;
        }
        
        .online { background: var(--success); box-shadow: 0 0 10px var(--success); }
        .warning { background: var(--warning); box-shadow: 0 0 10px var(--warning); }
        .error { background: var(--danger); box-shadow: 0 0 10px var(--danger); }
        .offline { background: var(--gray); }
        
        .main-content {
            display: grid;
            grid-template-columns: 3fr 1fr;
            gap: 20px;
        }
        
        @media (max-width: 1200px) {
            .main-content {
                grid-template-columns: 1fr;
            }
        }
        
        .control-panel {
            background: rgba(255,255,255,0.05);
            backdrop-filter: blur(10px);
            border-radius: 15px;
            padding: 25px;
            box-shadow: 0 8px 32px rgba(0,0,0,0.2);
            border: 1px solid rgba(255,255,255,0.1);
        }
        
        .panel-title {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 25px;
            padding-bottom: 15px;
            border-bottom: 2px solid rgba(255,255,255,0.1);
        }
        
        .panel-title h2 {
            font-size: 1.4rem;
            display: flex;
            align-items: center;
            gap: 10px;
        }
        
        .servo-control {
            background: rgba(0,0,0,0.3);
            padding: 20px;
            border-radius: 10px;
            margin-bottom: 15px;
            transition: transform 0.3s;
        }
        
        .servo-control:hover {
            transform: translateY(-2px);
        }
        
        .servo-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 15px;
        }
        
        .position-display {
            font-size: 2rem;
            font-weight: bold;
            color: var(--secondary);
            text-align: center;
            padding: 10px;
            background: rgba(0,0,0,0.5);
            border-radius: 8px;
            margin: 10px 0;
        }
        
        .slider-container {
            margin: 15px 0;
        }
        
        input[type="range"] {
            width: 100%;
            height: 12px;
            -webkit-appearance: none;
            background: linear-gradient(90deg, var(--danger) 0%, var(--warning) 50%, var(--success) 100%);
            border-radius: 10px;
            outline: none;
        }
        
        input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 28px;
            height: 28px;
            background: white;
            border-radius: 50%;
            cursor: pointer;
            box-shadow: 0 2px 10px rgba(0,0,0,0.3);
            border: 3px solid var(--secondary);
        }
        
        .quick-buttons {
            display: flex;
            justify-content: space-between;
            gap: 8px;
            margin-top: 10px;
        }
        
        .quick-buttons button {
            flex: 1;
            padding: 8px;
            border: none;
            border-radius: 6px;
            background: var(--primary);
            color: white;
            cursor: pointer;
            transition: all 0.3s;
            font-weight: 600;
            font-size: 0.9rem;
        }
        
        .quick-buttons button:hover {
            background: var(--secondary);
            transform: scale(1.05);
        }
        
        .action-buttons {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 12px;
            margin-top: 20px;
        }

        .recording-panel {
          margin-top: 18px;
          padding: 15px;
          background: rgba(0,0,0,0.35);
          border-radius: 10px;
          border: 1px solid rgba(255,255,255,0.1);
        }

        .recording-status {
          display: flex;
          justify-content: space-between;
          align-items: center;
          margin-bottom: 12px;
          color: var(--gray);
          font-size: 0.9rem;
        }

        .recording-buttons {
          display: grid;
          grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
          gap: 10px;
        }

        .btn-record {
          background: linear-gradient(45deg, #16a085, #1abc9c);
          color: white;
        }

        .btn-play {
          background: linear-gradient(45deg, #f39c12, #f1c40f);
          color: #1a1a1a;
        }

        .btn-stop {
          background: linear-gradient(45deg, #e74c3c, #c0392b);
          color: white;
        }

        .btn-clear {
          background: linear-gradient(45deg, #7f8c8d, #95a5a6);
          color: white;
        }
        
        .btn {
            padding: 12px;
            border: none;
            border-radius: 8px;
            font-weight: 600;
            font-size: 0.95rem;
            cursor: pointer;
            transition: all 0.3s;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 8px;
        }
        
        .btn-emergency {
            background: linear-gradient(45deg, var(--danger), #c0392b);
            color: white;
            animation: pulse 2s infinite;
        }
        
        @keyframes pulse {
            0%, 100% { box-shadow: 0 0 0 0 rgba(231, 76, 60, 0.7); }
            50% { box-shadow: 0 0 0 10px rgba(231, 76, 60, 0); }
        }
        
        .btn-home {
            background: linear-gradient(45deg, var(--success), #219653);
            color: white;
        }
        
        .btn-enable {
            background: linear-gradient(45deg, var(--secondary), #2980b9);
            color: white;
        }

        .btn-pot {
          background: linear-gradient(45deg, #8e44ad, #c0392b);
          color: white;
        }
        
        .btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(0,0,0,0.3);
        }
        
        .special-controls {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 12px;
            margin-top: 20px;
        }
        
        .special-control {
            background: rgba(255,255,255,0.1);
            padding: 15px;
            border-radius: 10px;
            text-align: center;
        }
        
        .special-control h4 {
            margin-bottom: 10px;
            color: var(--secondary);
        }
        
        .special-buttons {
            display: flex;
            gap: 8px;
            justify-content: center;
        }
        
        .special-buttons button {
            padding: 8px 12px;
            border: none;
            border-radius: 6px;
            background: var(--primary);
            color: white;
            cursor: pointer;
            transition: all 0.3s;
        }
        
        .special-buttons button:hover {
            background: var(--secondary);
        }
        
        .presets-panel {
            background: rgba(255,255,255,0.05);
            backdrop-filter: blur(10px);
            border-radius: 15px;
            padding: 25px;
            box-shadow: 0 8px 32px rgba(0,0,0,0.2);
            border: 1px solid rgba(255,255,255,0.1);
        }
        
        .preset-grid {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(180px, 1fr));
            gap: 12px;
            margin-top: 20px;
        }
        
        .preset-card {
            background: rgba(255,255,255,0.1);
            padding: 12px;
            border-radius: 10px;
            cursor: pointer;
            transition: all 0.3s;
            border: 2px solid transparent;
        }
        
        .preset-card:hover {
            border-color: var(--secondary);
            transform: translateY(-3px);
            background: rgba(52, 152, 219, 0.1);
        }
        
        .preset-name {
            font-weight: 600;
            margin-bottom: 8px;
            color: var(--secondary);
            font-size: 0.95rem;
        }
        
        .preset-angles {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 5px;
            font-size: 0.8rem;
            color: var(--gray);
        }
        
        .modal {
            display: none;
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: rgba(0,0,0,0.8);
            z-index: 1000;
            align-items: center;
            justify-content: center;
        }
        
        .modal-content {
            background: var(--primary);
            padding: 25px;
            border-radius: 15px;
            width: 90%;
            max-width: 500px;
        }
        
        .form-group {
            margin-bottom: 15px;
        }
        
        .form-group label {
            display: block;
            margin-bottom: 8px;
            font-weight: 600;
        }
        
        .form-group input {
            width: 100%;
            padding: 10px;
            border-radius: 6px;
            border: 2px solid rgba(255,255,255,0.1);
            background: rgba(0,0,0,0.3);
            color: white;
            font-size: 1rem;
        }
        
        .logs-panel {
            margin-top: 20px;
            background: rgba(0,0,0,0.3);
            padding: 15px;
            border-radius: 10px;
            max-height: 250px;
            overflow-y: auto;
        }
        
        .log-entry {
            padding: 6px 0;
            border-bottom: 1px solid rgba(255,255,255,0.1);
            font-family: 'Courier New', monospace;
            font-size: 0.85rem;
        }
        
        .log-timestamp {
            color: var(--gray);
            margin-right: 10px;
        }
        
        .log-message {
            color: var(--light);
        }
        
        .log-error { color: var(--danger); }
        .log-warning { color: var(--warning); }
        .log-success { color: var(--success); }
        
        footer {
            text-align: center;
            margin-top: 25px;
            padding-top: 15px;
            border-top: 1px solid rgba(255,255,255,0.1);
            color: var(--gray);
            font-size: 0.85rem;
        }
        
        .notification {
            position: fixed;
            top: 20px;
            right: 20px;
            padding: 12px 20px;
            border-radius: 8px;
            color: white;
            font-weight: 600;
            z-index: 1001;
            animation: slideIn 0.3s, fadeOut 0.3s 2.7s;
            display: none;
        }
        
        .notification-success { background: var(--success); }
        .notification-error { background: var(--danger); }
        .notification-warning { background: var(--warning); }
        
        @keyframes slideIn {
            from { transform: translateX(100%); opacity: 0; }
            to { transform: translateX(0); opacity: 1; }
        }
        
        @keyframes fadeOut {
            from { opacity: 1; }
            to { opacity: 0; }
        }
        
        .servo-grid {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
            gap: 15px;
        }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <div class="logo">
                <i class="fas fa-robot"></i>
                <div>
                    <h1>5-DOF Robotic Arm Controller</h1>
                    <p>Firmware v)=====";
  html += FIRMWARE_VERSION;
  html += R"=====( | ESP32 5-DOF System</p>
                </div>
            </div>
            <div class="status-indicators">
                <div class="status-item">
                    <div class="status-dot" id="wifi-status"></div>
                    <span id="wifi-text">WiFi</span>
                </div>
                <div class="status-item">
                    <div class="status-dot" id="servo-status"></div>
                    <span id="servo-text">Servos</span>
                </div>
                <div class="status-item">
                    <div class="status-dot" id="emergency-status"></div>
                    <span>Emergency</span>
                </div>
                <div class="status-item">
                    <span>Servos: 5 Active</span>
                </div>
            </div>
        </header>
        
        <div class="main-content">
            <div class="control-panel">
                <div class="panel-title">
                    <h2><i class="fas fa-gamepad"></i> Manual Control - 5 DOF</h2>
                    <div class="speed-control">
                        <label>Speed (Fixed): <span id="speed-value">)=====";
  html += String(moveSpeed);
  html += R"=====(</span>%</label>
                        <input type="range" id="speed-slider" min="1" max="100" value=")=====";
  html += String(moveSpeed);
  html += R"=====(" disabled>
                    </div>
                </div>
                
                <div class="servo-grid">
                    <!-- Base Rotation -->
                    <div class="servo-control" id="servo1-control">
                        <div class="servo-header">
                            <h3><i class="fas fa-sync-alt"></i> Base Rotation</h3>
                            <span class="servo-limit">0° - 180°</span>
                        </div>
                        <div class="position-display" id="servo1-display">)=====";
  html += String(currentPos1);
  html += R"=====(°</div>
                        <div class="slider-container">
                            <input type="range" class="servo-slider" data-servo="1" min="0" max="180" value=")=====";
  html += String(currentPos1);
  html += R"=====(">
                        </div>
                        <div class="quick-buttons">
                            <button onclick="moveTo(1, 0)">0° (Min)</button>
                            <button onclick="moveTo(1, 90)">90° (Center)</button>
                            <button onclick="moveTo(1, 180)">180° (Max)</button>
                        </div>
                    </div>
                    
                    <!-- Shoulder -->
                    <div class="servo-control" id="servo2-control">
                        <div class="servo-header">
                            <h3><i class="fas fa-arrow-up"></i> Shoulder</h3>
                            <span class="servo-limit">0° - 180°</span>
                        </div>
                        <div class="position-display" id="servo2-display">)=====";
  html += String(currentPos2);
  html += R"=====(°</div>
                        <div class="slider-container">
                            <input type="range" class="servo-slider" data-servo="2" min="0" max="180" value=")=====";
  html += String(currentPos2);
  html += R"=====(">
                        </div>
                        <div class="quick-buttons">
                            <button onclick="moveTo(2, 0)">0° (Min)</button>
                            <button onclick="moveTo(2, 90)">90° (Center)</button>
                            <button onclick="moveTo(2, 180)">180° (Max)</button>
                        </div>
                    </div>
                    
                    <!-- Elbow -->
                    <div class="servo-control" id="servo3-control">
                        <div class="servo-header">
                            <h3><i class="fas fa-angle-right"></i> Elbow</h3>
                            <span class="servo-limit">0° - 180°</span>
                        </div>
                        <div class="position-display" id="servo3-display">)=====";
  html += String(currentPos3);
  html += R"=====(°</div>
                        <div class="slider-container">
                            <input type="range" class="servo-slider" data-servo="3" min="0" max="180" value=")=====";
  html += String(currentPos3);
  html += R"=====(">
                        </div>
                        <div class="quick-buttons">
                            <button onclick="moveTo(3, 0)">0° (Min)</button>
                            <button onclick="moveTo(3, 90)">90° (Center)</button>
                            <button onclick="moveTo(3, 180)">180° (Max)</button>
                        </div>
                    </div>
                    
                    <!-- Wrist Pitch -->
                    <div class="servo-control" id="servo4-control">
                        <div class="servo-header">
                            <h3><i class="fas fa-hand-point-up"></i> Wrist Pitch</h3>
                            <span class="servo-limit">0° - 180°</span>
                        </div>
                        <div class="position-display" id="servo4-display">)=====";
  html += String(currentPos4);
  html += R"=====(°</div>
                        <div class="slider-container">
                            <input type="range" class="servo-slider" data-servo="4" min="0" max="180" value=")=====";
  html += String(currentPos4);
  html += R"=====(">
                        </div>
                        <div class="quick-buttons">
                            <button onclick="moveTo(4, 0)">0° (Down)</button>
                            <button onclick="moveTo(4, 90)">90° (Level)</button>
                            <button onclick="moveTo(4, 180)">180° (Up)</button>
                        </div>
                    </div>
                    
                    <!-- Gripper -->
                    <div class="servo-control" id="servo5-control">
                        <div class="servo-header">
                            <h3><i class="fas fa-hand-rock"></i> Gripper</h3>
                            <span class="servo-limit">0° - 90°</span>
                        </div>
                        <div class="position-display" id="servo5-display">)=====";
  html += String(currentPos5);
  html += R"=====(°</div>
                        <div class="slider-container">
                            <input type="range" class="servo-slider" data-servo="5" min="0" max="90" value=")=====";
  html += String(currentPos5);
  html += R"=====(">
                        </div>
                        <div class="quick-buttons">
                            <button onclick="gripperOpen()">Open</button>
                            <button onclick="gripperHome()">Half</button>
                            <button onclick="gripperClose()">Close</button>
                        </div>
                    </div>
                </div>
                
                <div class="special-controls">
                    <div class="special-control">
                        <h4><i class="fas fa-hand-point-up"></i> Wrist Quick Actions</h4>
                        <div class="special-buttons">
                            <button onclick="wristUp()">↑ Up (180°)</button>
                            <button onclick="wristHome()">⏏ Home (90°)</button>
                            <button onclick="wristDown()">↓ Down (0°)</button>
                        </div>
                    </div>
                    
                    <div class="special-control">
                        <h4><i class="fas fa-hand-rock"></i> Gripper Quick Actions</h4>
                        <div class="special-buttons">
                            <button onclick="gripperOpen()">Open (0°)</button>
                            <button onclick="gripperHome()">Half (45°)</button>
                            <button onclick="gripperClose()">Close (90°)</button>
                        </div>
                    </div>
                </div>
                
                <div class="action-buttons">
                    <button class="btn btn-emergency" onclick="emergencyStop()">
                        <i class="fas fa-stop-circle"></i> EMERGENCY STOP
                    </button>
                    <button class="btn btn-home" onclick="goHome()">
                        <i class="fas fa-home"></i> HOME ALL SERVOS
                    </button>
                    <button class="btn btn-enable" onclick="toggleServos()" id="enable-btn">
                        <i class="fas fa-power-off"></i> ENABLE SERVOS
                    </button>
                  <button class="btn btn-pot" onclick="togglePotMode()" id="pot-btn">
                    <i class="fas fa-sliders"></i> POT MODE OFF
                  </button>
                    <button class="btn" onclick="openSaveModal()" style="background: linear-gradient(45deg, #9b59b6, #8e44ad);">
                        <i class="fas fa-save"></i> SAVE POSITION
                    </button>
                </div>

                <div class="recording-panel">
                  <div class="recording-status">
                    <span id="recording-state">Recording: Off</span>
                    <span id="recording-count">Steps: 0</span>
                  </div>
                  <div class="recording-buttons">
                    <button class="btn btn-record" onclick="startRecording()" id="record-start-btn">
                      <i class="fas fa-circle"></i> START RECORD
                    </button>
                    <button class="btn btn-stop" onclick="stopRecordOrPlayback()" id="record-stop-btn">
                      <i class="fas fa-stop"></i> STOP
                    </button>
                    <button class="btn btn-play" onclick="playRecording()" id="record-play-btn">
                      <i class="fas fa-play"></i> PLAY
                    </button>
                    <button class="btn btn-clear" onclick="clearRecording()" id="record-clear-btn">
                      <i class="fas fa-trash"></i> CLEAR
                    </button>
                  </div>
                </div>
            </div>
            
            <div class="presets-panel">
                <div class="panel-title">
                    <h2><i class="fas fa-memory"></i> Position Memory</h2>
                    <button class="btn" onclick="clearPresets()" style="background: var(--danger); padding: 6px 12px; font-size: 0.85rem;">
                        <i class="fas fa-trash"></i> Clear All
                    </button>
                </div>
                
                <div class="preset-grid" id="preset-grid">
                    <!-- Preset cards will be loaded here -->
                </div>
                
                <div class="logs-panel">
                    <h3><i class="fas fa-terminal"></i> System Logs</h3>
                    <div id="system-logs">
                        <!-- Logs will be loaded here -->
                    </div>
                </div>
            </div>
        </div>
        
        <div class="modal" id="save-modal">
            <div class="modal-content">
                <h2><i class="fas fa-save"></i> Save Current Position</h2>
                <div class="form-group">
                    <label>Preset Name</label>
                    <input type="text" id="preset-name" placeholder="e.g., Pick Position, Drop Position" maxlength="31">
                </div>
                <div class="form-group">
                    <label>Current Angles (All 5 Servos)</label>
                    <div style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; color: var(--gray); margin-top: 10px;">
                        <div>Base: <span id="current-angle-1">)=====";
  html += String(currentPos1);
  html += R"=====(</span>°</div>
                        <div>Shoulder: <span id="current-angle-2">)=====";
  html += String(currentPos2);
  html += R"=====(</span>°</div>
                        <div>Elbow: <span id="current-angle-3">)=====";
  html += String(currentPos3);
  html += R"=====(</span>°</div>
                        <div>Wrist: <span id="current-angle-4">)=====";
  html += String(currentPos4);
  html += R"=====(</span>°</div>
                        <div>Gripper: <span id="current-angle-5">)=====";
  html += String(currentPos5);
  html += R"=====(</span>°</div>
                        <div>Speed: <span id="current-speed">)=====";
  html += String(moveSpeed);
  html += R"=====(</span>%</div>
                    </div>
                </div>
                <div style="display: flex; gap: 10px; margin-top: 20px;">
                    <button class="btn" onclick="savePreset()" style="flex: 1; background: var(--success);">
                        <i class="fas fa-check"></i> Save
                    </button>
                    <button class="btn" onclick="closeSaveModal()" style="flex: 1; background: var(--danger);">
                        <i class="fas fa-times"></i> Cancel
                    </button>
                </div>
            </div>
        </div>
        
        <div class="notification" id="notification"></div>
        
        <footer>
            <p>© 2024 5-DOF Robotic Arm Controller | ESP32 System | Connection: <span id="ip-address">)=====";
  html += WiFi.localIP().toString();
  html += R"=====(</span></p>
            <p>Uptime: <span id="uptime">00:00:00</span> | Free Memory: <span id="free-mem">0</span> bytes | 5 Servos Active</p>
        </footer>
    </div>
    
    <script>
        // Global variables for 5 servos
        let currentServo1 = )=====";
  html += String(currentPos1);
  html += R"=====(;
        let currentServo2 = )=====";
  html += String(currentPos2);
  html += R"=====(;
        let currentServo3 = )=====";
  html += String(currentPos3);
  html += R"=====(;
        let currentServo4 = )=====";
  html += String(currentPos4);
  html += R"=====(;
        let currentServo5 = )=====";
  html += String(currentPos5);
  html += R"=====(;
        let servosEnabled = )=====";
  html += servosEnabled ? "true" : "false";
  html += R"=====(;
        let emergencyActive = )=====";
  html += emergencyStop ? "true" : "false";
  html += R"=====(;
          let potMode = )=====";
        html += potControlEnabled ? "true" : "false";
        html += R"=====(;
          const gripperOpenAngle = )=====";
        html += String(GRIPPER_OPEN_ANGLE);
        html += R"=====(;
          const gripperCloseAngle = )=====";
        html += String(GRIPPER_CLOSE_ANGLE);
        html += R"=====(;
          const gripperHomeAngle = )=====";
        html += String(HOME_GRIPPER);
        html += R"=====(;
          let recordingActive = false;
          let playbackActive = false;
          let recordCount = 0;
        
        // Update position displays for all 5 servos
        function updatePositionDisplays() {
            document.getElementById('servo1-display').textContent = currentServo1 + '°';
            document.getElementById('servo2-display').textContent = currentServo2 + '°';
            document.getElementById('servo3-display').textContent = currentServo3 + '°';
            document.getElementById('servo4-display').textContent = currentServo4 + '°';
            document.getElementById('servo5-display').textContent = currentServo5 + '°';
            
            document.querySelectorAll('.servo-slider').forEach(slider => {
                const servoNum = slider.dataset.servo;
                if(servoNum == '1') slider.value = currentServo1;
                if(servoNum == '2') slider.value = currentServo2;
                if(servoNum == '3') slider.value = currentServo3;
                if(servoNum == '4') slider.value = currentServo4;
                if(servoNum == '5') slider.value = currentServo5;
            });
        }
        
        // Move servo to specific angle
        function moveTo(servoNum, angle) {
            if(emergencyActive) {
                showNotification('Emergency stop active! Enable servos first.', 'error');
                return;
            }

            if(potMode) {
                showNotification('Pot mode active! Disable pot mode to use sliders.', 'warning');
                return;
            }

          if(playbackActive) {
            showNotification('Playback active! Stop playback first.', 'warning');
            return;
          }
            
            fetch('/api/servo/' + servoNum + '?angle=' + angle)
                .then(response => response.json())
                .then(data => {
                    if(data.success) {
                        // Update the correct servo variable
                        if(servoNum == 1) currentServo1 = angle;
                        if(servoNum == 2) currentServo2 = angle;
                        if(servoNum == 3) currentServo3 = angle;
                        if(servoNum == 4) currentServo4 = angle;
                        if(servoNum == 5) currentServo5 = angle;
                        
                        updatePositionDisplays();
                        showNotification(data.message, 'success');
                    } else {
                        showNotification(data.message, 'error');
                    }
                })
                .catch(error => {
                    showNotification('Network error: ' + error, 'error');
                });
        }
        
        // Special wrist functions
        function wristUp() {
            moveTo(4, 180);
        }
        
        function wristDown() {
            moveTo(4, 0);
        }
        
        function wristHome() {
            moveTo(4, 90);
        }
        
        // Special gripper functions
        function gripperOpen() {
          moveTo(5, gripperOpenAngle);
        }
        
        function gripperClose() {
          moveTo(5, gripperCloseAngle);
        }
        
        function gripperHome() {
          moveTo(5, gripperHomeAngle);
        }
        
        // Emergency stop
        function emergencyStop() {
            fetch('/api/emergency')
                .then(response => response.json())
                .then(data => {
                    if(data.success) {
                        emergencyActive = true;
                        document.getElementById('emergency-status').className = 'status-dot error';
                        showNotification('EMERGENCY STOP ACTIVATED!', 'error');
                        updateEnableButton();
                    }
                });
        }
        
        // Go to home position for all 5 servos
        function goHome() {
            if(emergencyActive) {
                showNotification('Emergency stop active! Enable servos first.', 'error');
                return;
            }
            
            fetch('/api/home')
                .then(response => response.json())
                .then(data => {
                    if(data.success) {
                        currentServo1 = )=====";
  html += String(HOME_BASE);
  html += R"=====(;
                        currentServo2 = )=====";
  html += String(HOME_SHOULDER);
  html += R"=====(;
                        currentServo3 = )=====";
  html += String(HOME_ELBOW);
  html += R"=====(;
                        currentServo4 = )=====";
  html += String(HOME_WRIST);
  html += R"=====(;
                        currentServo5 = )=====";
  html += String(HOME_GRIPPER);
  html += R"=====(;
                        updatePositionDisplays();
                        showNotification('All 5 servos moved to home position', 'success');
                    }
                });
        }
        
        // Toggle servos enabled/disabled
        function toggleServos() {
            if(emergencyActive) {
                fetch('/api/enable')
                    .then(response => response.json())
                    .then(data => {
                        if(data.success) {
                            emergencyActive = false;
                            servosEnabled = true;
                            document.getElementById('emergency-status').className = 'status-dot online';
                            showNotification('All 5 servos enabled', 'success');
                            updateEnableButton();
                        }
                    });
            } else {
                fetch('/api/emergency')
                    .then(response => response.json())
                    .then(data => {
                        if(data.success) {
                            emergencyActive = true;
                            servosEnabled = false;
                            showNotification('All servos disabled', 'warning');
                            updateEnableButton();
                        }
                    });
            }
        }
        
        // Update enable button text
        function updateEnableButton() {
            const btn = document.getElementById('enable-btn');
            if(emergencyActive) {
                btn.innerHTML = '<i class="fas fa-power-off"></i> ENABLE SERVOS';
                btn.className = 'btn btn-enable';
            } else {
                btn.innerHTML = '<i class="fas fa-ban"></i> DISABLE SERVOS';
                btn.className = 'btn btn-emergency';
            }
        }

        function updatePotButton() {
          const btn = document.getElementById('pot-btn');
          if(potMode) {
            btn.innerHTML = '<i class="fas fa-sliders"></i> POT MODE ON';
            btn.className = 'btn btn-pot';
          } else {
            btn.innerHTML = '<i class="fas fa-sliders"></i> POT MODE OFF';
            btn.className = 'btn btn-pot';
          }
        }

        function togglePotMode() {
          fetch('/api/pot/toggle')
            .then(response => response.json())
            .then(data => {
              if(data.success) {
                potMode = data.pot_mode;
                updatePotButton();
                showNotification(data.message || 'Pot mode updated', 'success');
              } else {
                showNotification(data.error || data.message || 'Failed to toggle pot mode', 'error');
              }
            });
        }

        function updateRecordingUI() {
          const stateEl = document.getElementById('recording-state');
          const countEl = document.getElementById('recording-count');
          const startBtn = document.getElementById('record-start-btn');
          const stopBtn = document.getElementById('record-stop-btn');
          const playBtn = document.getElementById('record-play-btn');
          const clearBtn = document.getElementById('record-clear-btn');

          if(playbackActive) {
            stateEl.textContent = 'Playback: ON';
          } else if(recordingActive) {
            stateEl.textContent = 'Recording: ON';
          } else {
            stateEl.textContent = 'Recording: Off';
          }

          countEl.textContent = 'Steps: ' + recordCount;

          startBtn.disabled = recordingActive || playbackActive;
          playBtn.disabled = recordingActive || playbackActive || recordCount === 0;
          stopBtn.disabled = !(recordingActive || playbackActive);
          clearBtn.disabled = recordingActive || playbackActive;
        }

        function startRecording() {
          if(emergencyActive) {
            showNotification('Enable servos before recording.', 'warning');
            return;
          }

          fetch('/api/record/start')
            .then(response => response.json())
            .then(data => {
              if(data.success) {
                recordingActive = true;
                playbackActive = false;
                recordCount = data.count || 0;
                updateRecordingUI();
                showNotification(data.message || 'Recording started', 'success');
              } else {
                showNotification(data.error || data.message || 'Failed to start recording', 'error');
              }
            });
        }

        function stopRecordOrPlayback() {
          const url = playbackActive ? '/api/record/stop_playback' : '/api/record/stop';
          fetch(url)
            .then(response => response.json())
            .then(data => {
              if(data.success) {
                recordingActive = false;
                playbackActive = false;
                recordCount = data.count !== undefined ? data.count : recordCount;
                updateRecordingUI();
                showNotification(data.message || 'Stopped', 'success');
              } else {
                showNotification(data.error || data.message || 'Failed to stop', 'error');
              }
            });
        }

        function playRecording() {
          if(recordCount === 0) {
            showNotification('No recorded steps to play.', 'warning');
            return;
          }

          fetch('/api/record/play')
            .then(response => response.json())
            .then(data => {
              if(data.success) {
                playbackActive = true;
                recordingActive = false;
                updateRecordingUI();
                showNotification(data.message || 'Playback started', 'success');
              } else {
                showNotification(data.error || data.message || 'Failed to start playback', 'error');
              }
            });
        }

        function clearRecording() {
          if(!confirm('Clear the recorded sequence?')) {
            return;
          }

          fetch('/api/record/clear')
            .then(response => response.json())
            .then(data => {
              if(data.success) {
                recordCount = 0;
                recordingActive = false;
                playbackActive = false;
                updateRecordingUI();
                showNotification(data.message || 'Recording cleared', 'success');
              } else {
                showNotification(data.error || data.message || 'Failed to clear recording', 'error');
              }
            });
        }
        
        // Save preset modal
        function openSaveModal() {
            document.getElementById('current-angle-1').textContent = currentServo1;
            document.getElementById('current-angle-2').textContent = currentServo2;
            document.getElementById('current-angle-3').textContent = currentServo3;
            document.getElementById('current-angle-4').textContent = currentServo4;
            document.getElementById('current-angle-5').textContent = currentServo5;
            document.getElementById('current-speed').textContent = document.getElementById('speed-value').textContent;
            document.getElementById('save-modal').style.display = 'flex';
        }
        
        function closeSaveModal() {
            document.getElementById('save-modal').style.display = 'none';
            document.getElementById('preset-name').value = '';
        }
        
        // Save preset with 5 servo values
        function savePreset() {
            const name = document.getElementById('preset-name').value.trim();
            if(!name) {
                showNotification('Please enter a preset name', 'warning');
                return;
            }
            
            fetch('/api/save_preset', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    name: name,
                    servo1: currentServo1,
                    servo2: currentServo2,
                    servo3: currentServo3,
                    servo4: currentServo4,
                    servo5: currentServo5,
                    speed: parseInt(document.getElementById('speed-value').textContent)
                })
            })
            .then(response => response.json())
            .then(data => {
                if(data.success) {
                    showNotification('Preset "' + name + '" saved successfully (5 servos)', 'success');
                    closeSaveModal();
                    loadPresets();
                } else {
                    showNotification(data.message, 'error');
                }
            });
        }
        
        // Load preset
        function loadPreset(index) {
            fetch('/api/load_preset/' + index)
                .then(response => response.json())
                .then(data => {
                    if(data.success) {
                        // Update all 5 servo positions
                        currentServo1 = data.position.servo1;
                        currentServo2 = data.position.servo2;
                        currentServo3 = data.position.servo3;
                        currentServo4 = data.position.servo4;
                        currentServo5 = data.position.servo5;
                        
                        // Update speed if present
                        if(data.position.speed) {
                            document.getElementById('speed-value').textContent = data.position.speed;
                            document.getElementById('speed-slider').value = data.position.speed;
                        }
                        
                        updatePositionDisplays();
                        showNotification('Loaded preset: ' + data.position.name + ' (5 servos)', 'success');
                    } else {
                        showNotification(data.message, 'error');
                    }
                });
        }
        
        // Load all presets
        function loadPresets() {
            fetch('/api/presets')
                .then(response => response.json())
                .then(data => {
                    const grid = document.getElementById('preset-grid');
                    grid.innerHTML = '';
                    
                    data.presets.forEach((preset, index) => {
                        if(preset.name) {
                            const card = document.createElement('div');
                            card.className = 'preset-card';
                            card.onclick = () => loadPreset(index);
                            card.innerHTML = `
                                <div class="preset-name">${preset.name}</div>
                                <div class="preset-angles">
                                    <span>B: ${preset.servo1}°</span>
                                    <span>S: ${preset.servo2}°</span>
                                    <span>E: ${preset.servo3}°</span>
                                    <span>W: ${preset.servo4}°</span>
                                    <span>G: ${preset.servo5}°</span>
                                    <span>⚡${preset.speed || 50}%</span>
                                </div>
                            `;
                            grid.appendChild(card);
                        }
                    });
                    
                    // Add empty slots
                    for(let i = data.presets.length; i < 10; i++) {
                        const card = document.createElement('div');
                        card.className = 'preset-card';
                        card.style.opacity = '0.5';
                        card.innerHTML = `
                            <div class="preset-name">Empty Slot ${i + 1}</div>
                            <div class="preset-angles">
                                <span>--°</span>
                                <span>--°</span>
                                <span>--°</span>
                                <span>--°</span>
                                <span>--°</span>
                                <span>--%</span>
                            </div>
                        `;
                        grid.appendChild(card);
                    }
                });
        }
        
        // Clear all presets
        function clearPresets() {
            if(confirm('Are you sure you want to clear ALL saved positions?')) {
                fetch('/api/clear_presets')
                    .then(response => response.json())
                    .then(data => {
                        if(data.success) {
                            showNotification('All presets cleared', 'success');
                            loadPresets();
                        }
                    });
            }
        }
        
        // Set speed
        document.getElementById('speed-slider').addEventListener('input', function(e) {
            const speed = e.target.value;
            document.getElementById('speed-value').textContent = speed;
            
            fetch('/api/set_speed?speed=' + speed)
                .then(response => response.json())
                .then(data => {
                    if(data.success) {
                        showNotification('Speed set to ' + speed + '%', 'success');
                    }
                });
        });
        
        // Slider event listeners for all 5 servos
        document.querySelectorAll('.servo-slider').forEach(slider => {
            slider.addEventListener('input', function(e) {
                const servoNum = this.dataset.servo;
                const angle = parseInt(this.value);
                
                // Update display for the specific servo
                document.getElementById('servo' + servoNum + '-display').textContent = angle + '°';
            });
            
            slider.addEventListener('change', function(e) {
                const servoNum = this.dataset.servo;
                const angle = parseInt(this.value);
              if(potMode) {
                showNotification('Pot mode active! Disable pot mode to use sliders.', 'warning');
                return;
              }
              moveTo(servoNum, angle);
            });
        });
        
        // Show notification
        function showNotification(message, type) {
            const notification = document.getElementById('notification');
            notification.textContent = message;
            notification.className = 'notification notification-' + type;
            notification.style.display = 'block';
            
            setTimeout(() => {
                notification.style.display = 'none';
            }, 3000);
        }
        
        // Update system status
        function updateSystemStatus() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    // Update status indicators
                    document.getElementById('wifi-status').className = 
                        data.wifi_connected ? 'status-dot online' : 'status-dot error';
                    document.getElementById('wifi-text').textContent = 
                        data.wifi_connected ? ('WiFi (' + data.ip + ')') : 'WiFi (AP)';
                    
                    document.getElementById('servo-status').className = 
                        data.servos_enabled ? 'status-dot online' : 'status-dot warning';
                    document.getElementById('servo-text').textContent = 
                        data.servos_enabled ? 'Servos ON' : 'Servos OFF';
                    
                    document.getElementById('emergency-status').className = 
                        data.emergency_stop ? 'status-dot error' : 'status-dot online';
                    
                    // Update servos enabled state
                    emergencyActive = data.emergency_stop;
                    servosEnabled = data.servos_enabled;
                    updateEnableButton();

                    recordingActive = data.recording_active;
                    playbackActive = data.playback_active;
                    recordCount = data.record_count;
                    updateRecordingUI();

                    potMode = data.pot_mode;
                    updatePotButton();
                    
                    // Update uptime
                    document.getElementById('uptime').textContent = data.uptime;
                    
                    // Update free memory
                    document.getElementById('free-mem').textContent = data.free_memory;
                    
                    // Update IP address
                    document.getElementById('ip-address').textContent = data.ip;
                    
                    // Update current positions if changed (all 5 servos)
                    if(currentServo1 != data.servo1) {
                        currentServo1 = data.servo1;
                        updatePositionDisplays();
                    }
                    if(currentServo2 != data.servo2) {
                        currentServo2 = data.servo2;
                        updatePositionDisplays();
                    }
                    if(currentServo3 != data.servo3) {
                        currentServo3 = data.servo3;
                        updatePositionDisplays();
                    }
                    if(currentServo4 != data.servo4) {
                        currentServo4 = data.servo4;
                        updatePositionDisplays();
                    }
                    if(currentServo5 != data.servo5) {
                        currentServo5 = data.servo5;
                        updatePositionDisplays();
                    }
                })
                .catch(error => {
                    console.error('Status update failed:', error);
                });
        }
        
        // Load logs
        function loadLogs() {
            fetch('/api/logs')
                .then(response => response.json())
                .then(data => {
                    const logsDiv = document.getElementById('system-logs');
                    logsDiv.innerHTML = '';
                    
                    data.logs.forEach(log => {
                        const logEntry = document.createElement('div');
                        logEntry.className = 'log-entry';
                        logEntry.innerHTML = `
                            <span class="log-timestamp">[${log.timestamp}]</span>
                            <span class="log-message ${log.type ? 'log-' + log.type : ''}">${log.message}</span>
                        `;
                        logsDiv.appendChild(logEntry);
                    });
                });
        }
        
        // Keyboard shortcuts for all 5 servos
        document.addEventListener('keydown', function(e) {
            switch(e.key) {
                case 'Escape':
                case ' ':
                    emergencyStop();
                    break;
                case 'h':
                case 'H':
                    if(!e.ctrlKey) goHome();
                    break;
                case '1':
                    if(e.ctrlKey) moveTo(1, 0);
                    break;
                case '2':
                    if(e.ctrlKey) moveTo(2, 90);
                    break;
                case '3':
                    if(e.ctrlKey) moveTo(3, 180);
                    break;
                case '4':
                    if(e.ctrlKey) wristUp();
                    break;
                case '5':
                    if(e.ctrlKey) gripperClose();
                    break;
            }
        });
        
        // Initialize
        document.addEventListener('DOMContentLoaded', function() {
            updatePositionDisplays();
            updateEnableButton();
            updatePotButton();
          updateRecordingUI();
            loadPresets();
            updateSystemStatus();
            loadLogs();
            
            // Auto-refresh
            setInterval(updateSystemStatus, 2000);
            setInterval(loadLogs, 5000);
        });
    </script>
</body>
</html>
)=====";
  
  return html;
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  
  systemStartTime = millis();
  lastActivityTime = systemStartTime;
  
  Serial.println("\n========================================");
  Serial.println("5-DOF Robotic Arm Controller");
  Serial.println("Firmware Version: " + String(FIRMWARE_VERSION));
  Serial.println("ESP32 Chip ID: " + String((uint32_t)ESP.getEfuseMac(), HEX));
  Serial.println("Servos: Base(25), Shoulder(26), Elbow(27), Wrist(14), Gripper(12)");
  Serial.println("========================================\n");
  
  addLog("System starting up with 5 servos...");

  // ADC setup for potentiometers (0-3.3V range)
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(GRIPPER_BUTTON_PIN, INPUT_PULLUP);
  initLeds();
  
  // Initialize EEPROM
  initEEPROM();
  
  // Initialize servos
  initServos();
  
  // Initialize WiFi
  addLog("Initializing network...");
  
  if (!connectWiFi()) {
    addLog("WiFi connection failed, starting AP mode...", "warning");
    startAPMode();
  }
  
  // Setup mDNS
  setupmDNS();
  
  // Setup web server routes for 5 servos
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleGetStatus);
  server.on("/api/servo/1", HTTP_GET, handleSetServo);
  server.on("/api/servo/2", HTTP_GET, handleSetServo);
  server.on("/api/servo/3", HTTP_GET, handleSetServo);
  server.on("/api/servo/4", HTTP_GET, handleSetServo);
  server.on("/api/servo/5", HTTP_GET, handleSetServo);
  server.on("/api/emergency", HTTP_GET, handleEmergencyStop);
  server.on("/api/enable", HTTP_GET, handleEnableServos);
  server.on("/api/home", HTTP_GET, handleHomePosition);
  server.on("/api/save_preset", HTTP_POST, handleSavePreset);
  
  // Load preset routes for all 10 slots
  for (int i = 0; i < MAX_PRESETS; i++) {
    server.on("/api/load_preset/" + String(i), HTTP_GET, handleLoadPreset);
  }
  
  server.on("/api/presets", HTTP_GET, handleGetPresets);
  server.on("/api/clear_presets", HTTP_GET, handleClearPresets);
  server.on("/api/set_speed", HTTP_GET, handleSetSpeed);
  server.on("/api/logs", HTTP_GET, handleGetLogs);
  server.on("/api/servos", HTTP_POST, handleSetAllServos);

  // Potentiometer control routes
  server.on("/api/pot/enable", HTTP_GET, handlePotEnable);
  server.on("/api/pot/disable", HTTP_GET, handlePotDisable);
  server.on("/api/pot/toggle", HTTP_GET, handlePotToggle);

  // Recording routes
  server.on("/api/record/start", HTTP_GET, handleRecordStart);
  server.on("/api/record/stop", HTTP_GET, handleRecordStop);
  server.on("/api/record/play", HTTP_GET, handleRecordPlay);
  server.on("/api/record/stop_playback", HTTP_GET, handleRecordStopPlayback);
  server.on("/api/record/clear", HTTP_GET, handleRecordClear);
  
  // Special control routes
  server.on("/api/gripper/open", HTTP_GET, handleGripperOpen);
  server.on("/api/gripper/close", HTTP_GET, handleGripperClose);
  server.on("/api/gripper/home", HTTP_GET, handleGripperHome);
  server.on("/api/wrist/up", HTTP_GET, handleWristUp);
  server.on("/api/wrist/down", HTTP_GET, handleWristDown);
  server.on("/api/wrist/home", HTTP_GET, handleWristHome);
  
  server.onNotFound(handleNotFound);
  
  // Start server
  server.begin();
  addLog("HTTP server started on port 80 (5-DOF system)");
  
  // Move to home position
  homeAllServos();
  
  addLog("5-DOF system initialization complete!");
  addLog("Connect to: http://" + (apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) + 
         " or http://robotarm.local");
}

void updateGripperButton() {
  if (emergencyStop || !servosEnabled || playbackActive) return;

  int reading = digitalRead(GRIPPER_BUTTON_PIN);
  unsigned long now = millis();
  if (reading != lastGripperButtonState) {
    lastGripperButtonChange = now;
    lastGripperButtonState = reading;
  }

  if (now - lastGripperButtonChange < BUTTON_DEBOUNCE_MS) return;

  if (reading != lastGripperButtonStable) {
    lastGripperButtonStable = reading;
    if (reading == LOW) {
      moveServo(5, GRIPPER_OPEN_ANGLE);
    } else {
      moveServo(5, GRIPPER_CLOSE_ANGLE);
    }
  }
}

// ==================== MAIN LOOP ====================
void loop() {
  server.handleClient();
  updatePlayback();
  updateServos();
  updatePotControl();
  updateGripperButton();
  checkInactivity();
  updateStatusLeds();
  
  // Periodic status logging
  static unsigned long lastStatusUpdate = 0;
  if (millis() - lastStatusUpdate > 10000) {
    lastStatusUpdate = millis();
    static int loopCount = 0;
    loopCount++;
    if (loopCount % 6 == 0) { // Every minute
      addLog("System heartbeat - Uptime: " + String((millis() - systemStartTime) / 1000) + 
             "s, Free mem: " + String(ESP.getFreeHeap()) + " bytes, 5 servos active");
    }
  }
}