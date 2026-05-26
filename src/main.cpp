#include <Arduino.h>
#include <SPI.h>
#include <PID_v1.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <RBDdimmer.h>
#include <avr/sleep.h>
#include <avr/power.h>

// === Configuration ==========================================================
#define ZC_TRIGGER_EDGE CHANGE  
#ifndef ALL_DIMMERS
#define ALL_DIMMERS 13          
#endif
#define PUMP_KILL_DURATION_MS 2000 

// I2C -----------------------------------------------------------------------
#define I2C_SLAVE_ADDRESS 0x08
#define CMD_SET_PID_TARGETS      0x10
#define CMD_PRESS_POWER_BUTTON   0x20
#define CMD_SET_MANUAL_PREINFUSION 0x30
#define I2C_CMD_DEBUG_MESSAGE    0x90
#define I2C_CMD_ERROR_MESSAGE    0x91
#define I2C_CMD_STATUS_UPDATE    0x92
#define ENABLE_ARDUINO_IDLE_MODE  true
#define TB_IDLE_THRESHOLD_C       25.0f

volatile bool   newI2CCommand      = false;
volatile uint8_t receivedI2CCommand= 0;
volatile bool   i2cRequestOccurred = false;
volatile uint8_t i2cCommandData[8];
volatile uint8_t i2cDataLength     = 0;
bool arduino_in_idle_mode = false;

struct I2CDebugMessage { uint8_t messageType; uint8_t messageLength; char messageData[30]; } __attribute__((packed));
static I2CDebugMessage receivedDebugMsg; 
static bool newDebugMessage = false;

// Pins ----------------------------------------------------------------------
const int PRESSURE_PIN           = A0;
const int GROUPHEAD_TEMP_PIN     = A1;
const int THERMOBLOCK_TEMP_PIN   = A2;
const int MANUAL_POT_PIN         = A3;
const int I2C_SDA_PIN            = A4;
const int I2C_SCL_PIN            = A5;
const int ZERO_CROSS_PIN         = 2;  
const int AC_DIMMER_GATE_PIN     = 3;  
const int MCP23S08_CS_PIN        = 4;
const int MCP23S08_MOSI_PIN      = 5;
const int MCP23S08_MISO_PIN      = 6;
const int MCP23S08_SCK_PIN       = 7;
const int MODE_SWITCH_PIN        = 8;   
const int POT_SWITCH_PIN         = 10;  
const int POWER_BUTTON_PIN       = 12;  
const int POWER_BUTTON_CTRL_PIN  = 13;   

static_assert(AC_DIMMER_GATE_PIN == 3,  "ERROR: Gate pin must be D3");
static_assert(ZERO_CROSS_PIN     == 2,  "ERROR: ZC pin must be D2");

dimmerLamp acDimmer(AC_DIMMER_GATE_PIN); 
void setPumpPower(uint8_t powerPercent);
void forcePumpOff();
void returnToMonitor();

int freeRam() { extern int __heap_start, *__brkval; int v; return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval); }

// Data structures ------------------------------------------------------------
struct MachineState {
  float pressure;
  float groupHeadTemp;
  float thermoblockTemp;
  float targetPressure;
  float targetTemp;
  uint8_t pumpPower;
  uint8_t selectedProgram;
  uint8_t operatingMode;
  uint8_t stateFlags;
  uint8_t heartbeat;
  uint8_t padding;
} __attribute__((packed));

struct SystemState {
  int   pressureRaw, groupHeadRaw, thermoblockRaw, manualPotRaw;
  float pressure, groupHeadTemp, thermoblockTemp;
  bool  isManualMode, potSwitchOff, isBrewing, sensorError;
  int32_t selectedProgram; 
  uint8_t heartbeat;
  bool  powerButtonPressed;
  bool  hasCriticalError;
  unsigned long lastErrorTime;
  char  lastErrorMessage[16];
  bool shotIsActive;      
} systemState;

struct ControlTargets { float targetPressure; float targetTemp; } __attribute__((packed));
struct ManualPreinfusionConfig { float pressure; uint32_t duration_ms; } __attribute__((packed));

// Resistor Logic
float calculateEstimatedTempOffset(uint8_t mcpValue) {
  const float parallelEffects[4] = {-0.5, -1.0, -2.0, -4.0};  
  const float seriesEffects[4] = {+0.5, +1.0, +2.0, +4.0};    
  float totalOffset = 0.0;
  for (uint8_t i = 0; i < 4; i++) { if (mcpValue & (1 << i)) totalOffset += parallelEffects[i]; }
  for (uint8_t i = 0; i < 4; i++) { if (!(mcpValue & (1 << (i + 4)))) totalOffset += seriesEffects[i]; }
  return totalOffset;
}

String getActiveResistors(uint8_t mcpValue) {
  String result = "";
  const char* parallelResistors[4] = {"2.2MΩ", "1.0MΩ", "470kΩ", "220kΩ"};
  const char* seriesResistors[4] = {"470kΩ", "220kΩ", "100kΩ", "47kΩ"};
  for (uint8_t i = 0; i < 4; i++) { if (mcpValue & (1 << i)) { if (result.length() > 0) result += ", "; result += parallelResistors[i]; result += "(||)"; } }
  for (uint8_t i = 0; i < 4; i++) { if (!(mcpValue & (1 << (i + 4)))) { if (result.length() > 0) result += ", "; result += seriesResistors[i]; result += "(ser)"; } }
  if (result.length() == 0) result = "None (bypass/baseline)";
  return result;
}

// MCP23S08 ---------------------------------------------------
struct TemperatureControl { uint8_t gpio_code; float target_temp; const char* description; };
const TemperatureControl temperatureSettings[15] PROGMEM = {
  {0b00100011, 80.0, "Extreme Low"}, {0b00100010, 82.0, "Very Low"}, {0b01000110, 85.0, "Dark Roast Low"},
  {0b01000100, 88.0, "Dark Roast"}, {0b10010000, 89.0, "Transition"}, {0b00110000, 90.0, "Medium-Dark"},
  {0b00100000, 91.0, "Medium Prog"}, {0b01000000, 92.0, "Medium Roast"}, {0b00010000, 93.0, "Medium High"},
  {0b01000000, 94.0, "Medium-Light"}, {0b00110000, 95.0, "Light Roast"}, {0b00100000, 96.0, "Light High"},
  {0b00010000, 98.0, "Nordic Light"}, {0b00000000, 99.0, "Maximum"}, {0b11111111,100.0, "Override"}
};

class MCP23S08Controller {
  uint8_t currentState;
  void spiTransfer(uint8_t cs_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t sck_pin, uint8_t data) {
    for (int i = 7; i >= 0; i--) { digitalWrite(sck_pin, LOW); digitalWrite(mosi_pin, (data >> i) & 0x01); digitalWrite(sck_pin, HIGH); }
  }
  uint8_t spiReceive(uint8_t cs_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t sck_pin) {
    uint8_t result = 0;
    for (int i = 7; i >= 0; i--) { digitalWrite(sck_pin, LOW); digitalWrite(sck_pin, HIGH); result |= (digitalRead(miso_pin) ? 1 : 0) << i; }
    return result;
  }
public:
  void init() {
    pinMode(MCP23S08_CS_PIN,   OUTPUT); pinMode(MCP23S08_MOSI_PIN, OUTPUT);
    pinMode(MCP23S08_MISO_PIN, INPUT);  pinMode(MCP23S08_SCK_PIN,  OUTPUT);
    digitalWrite(MCP23S08_CS_PIN, HIGH); digitalWrite(MCP23S08_SCK_PIN, LOW);
    delay(10);
    writeRegister(0x00, 0x00); 
    writeRegister(0x0A, 0x00);  
    Serial.println(F("MCP23S08 initialized to CLOSED (baseline)"));
  }
  void writeRegister(uint8_t reg, uint8_t value) {
    digitalWrite(MCP23S08_CS_PIN, LOW); delayMicroseconds(1);
    spiTransfer(MCP23S08_CS_PIN, MCP23S08_MOSI_PIN, MCP23S08_MISO_PIN, MCP23S08_SCK_PIN, 0x40);
    spiTransfer(MCP23S08_CS_PIN, MCP23S08_MOSI_PIN, MCP23S08_MISO_PIN, MCP23S08_SCK_PIN, reg);
    spiTransfer(MCP23S08_CS_PIN, MCP23S08_MOSI_PIN, MCP23S08_MISO_PIN, MCP23S08_SCK_PIN, value);
    delayMicroseconds(1); digitalWrite(MCP23S08_CS_PIN, HIGH);
  }
  uint8_t readRegister(uint8_t reg) {
    digitalWrite(MCP23S08_CS_PIN, LOW); delayMicroseconds(1);
    spiTransfer(MCP23S08_CS_PIN, MCP23S08_MOSI_PIN, MCP23S08_MISO_PIN, MCP23S08_SCK_PIN, 0x41);
    spiTransfer(MCP23S08_CS_PIN, MCP23S08_MOSI_PIN, MCP23S08_MISO_PIN, MCP23S08_SCK_PIN, reg);
    uint8_t result = spiReceive(MCP23S08_CS_PIN, MCP23S08_MOSI_PIN, MCP23S08_MISO_PIN, MCP23S08_SCK_PIN);
    delayMicroseconds(1); digitalWrite(MCP23S08_CS_PIN, HIGH);
    return result;
  }
  void setTemperature(float targetTemp) {
    static float lastTargetTemp = -999.0f;
    uint8_t bestIndex = 7; float bestDiff = 999.0f;
    for (uint8_t i = 0; i < 15; i++) {
      TemperatureControl setting; memcpy_P(&setting, &temperatureSettings[i], sizeof(TemperatureControl));
      float diff = fabsf(setting.target_temp - targetTemp);
      if (diff < bestDiff) { bestDiff = diff; bestIndex = i; }
    }
    TemperatureControl selected; memcpy_P(&selected, &temperatureSettings[bestIndex], sizeof(TemperatureControl));
    writeRegister(0x0A, selected.gpio_code); 
    currentState = selected.gpio_code;
    if (fabsf(targetTemp - lastTargetTemp) > 0.1f) {
      lastTargetTemp = targetTemp;
    }
  }
  uint8_t getCurrentState() { return currentState; }
};
MCP23S08Controller tempControl;

uint8_t TEST_TEMP_OVERRIDE = 0x80;  
bool ENABLE_TEST_OVERRIDE = false;   

// Operating modes & states ---------------------------------------------------
void onPumpActivated();
void onPumpDeactivated();
enum OperatingMode   { MODE_BYPASS, MODE_MANUAL, MODE_AUTO };
enum MachineStateEnum{ MACHINE_OFF, MACHINE_WARMING, MACHINE_READY, MACHINE_ERROR };
enum BrewState       { BREW_IDLE, BREW_STARTING, BREW_PREINFUSION, BREW_EXTRACTION, BREW_ENDING };
OperatingMode    currentOperatingMode = MODE_BYPASS;
MachineStateEnum currentMachineState  = MACHINE_OFF;
BrewState        currentBrewState     = BREW_IDLE;
bool pumpActive=false, wasPumpActive=false;

static const uint8_t  READY_STREAK_MIN = 3;
static const uint32_t READY_LATCH_MS   = 60000UL;
volatile bool controlSessionActive     = false;
static unsigned long controlSessionStartMs = 0;
static uint32_t CONTROL_SESSION_TIMEOUT_MS = 60000UL; 
static unsigned long pumpKillUntil = 0; // Time until pump kill can be released

volatile uint16_t      zcEdgesPerSecond = 0;
volatile unsigned long lastZcCountReset = 0;
uint8_t dirtyCount = 0;
uint16_t zcFreqHistory[5] = {0};
uint8_t  zcFreqIndex = 0;
uint16_t avgZcFrequency = 0;
#define ZC_PUMP_ON_THRESHOLD 100

struct PumpDetectionConfig { uint16_t offPulseThreshold_us; uint16_t cleanPulseThreshold_us; uint16_t symmetryTolerance_us; uint8_t stableReadingsRequired; };
PumpDetectionConfig pumpDetectConfig = { 5500, 2000, 1500, 5 };
enum PumpState { PUMP_OFF, PUMP_BREVILLE_CONTROL, PUMP_READY_FOR_CONTROL };
struct PumpDetection {
  PumpState currentState;
  PumpState detectedState;
  uint8_t   stableCount;
  uint16_t  currentRisingPulse_us;
  uint16_t  previousRisingPulse_us;
  unsigned long lastStateChange;
  uint32_t  stateChangeCount;
  uint32_t  offCount, brevilleCount, readyCount;
};
PumpDetection pumpDetect = { PUMP_OFF, PUMP_OFF, 0, 0, 0, 0, 0, 0, 0, 0 };

volatile uint32_t zcRisingEdgeTime = 0;
volatile uint32_t zcFallingEdgeTime = 0;
volatile bool     newRisingPulsePair = false;
volatile uint8_t  lastZCState = LOW;

// PID Variables
double pressureSetpoint=0.0, currentPressure=0.0, pressureOutput=0.0;
double tempSetpoint=92.0,   currentTemp=92.0,     tempOutput=128.0;
const double Kp_pressure=6.0, Ki_pressure=2.5, Kd_pressure=0.0; 
const double Kp_temp=40.0,    Ki_temp=3.0,     Kd_temp=0.8;
PID pressurePID(&currentPressure,&pressureOutput,&pressureSetpoint,Kp_pressure,Ki_pressure,Kd_pressure,DIRECT);
PID tempPID(&currentTemp,&tempOutput,&tempSetpoint,Kp_temp,Ki_temp,Kd_temp,DIRECT);

enum ShotState { IDLE, PREINFUSION, BREWING, STOPPING, FAULT };
ShotState currentShotState = IDLE;
unsigned long shotStartTime = 0;
ControlTargets currentTargets = {0.0f, 92.0f};
ManualPreinfusionConfig preinfusionConfig = {3.0f, 4000};

const uint32_t HALF_CYCLE_US = 8333; 
uint8_t  pumpPowerPercent = 0;        

enum GateMode: uint8_t { GATE_MONITOR_HIGH=0, GATE_CONTROLLED=1 };
volatile GateMode gateMode = GATE_MONITOR_HIGH;

inline void gateSetMonitor() {
  if (gateMode != GATE_MONITOR_HIGH) gateMode = GATE_MONITOR_HIGH;
  if (controlSessionActive) { acDimmer.setPower(0); acDimmer.setState(ON); } else { digitalWrite(AC_DIMMER_GATE_PIN, HIGH); }
}
inline void gateSetControlled() {
  if (gateMode != GATE_CONTROLLED) gateMode = GATE_CONTROLLED;
  if (!controlSessionActive) digitalWrite(AC_DIMMER_GATE_PIN, LOW);
}

static const uint16_t POT_ADC_MIN = 0;       
static const uint16_t POT_ADC_MAX = 1023;    
static const uint8_t  POT_SMOOTH_ALPHA = 3;  
static uint16_t       potFilt = 0;
static inline uint8_t mapPotToPercent_fixed(uint16_t raw) {
  if (POT_ADC_MAX <= POT_ADC_MIN + 1) return 0;    
  long pct = (long)(raw - POT_ADC_MIN) * 100L / (POT_ADC_MAX - POT_ADC_MIN);
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  return (uint8_t)pct;
}

static void logPotSnapshot_1Hz() {
  static unsigned long last = 0; 
  if (millis() - last < 1000) return; 
  last = millis();
  
  uint16_t raw = analogRead(MANUAL_POT_PIN);
  
  Serial.print(F("[POT] Raw: ")); Serial.print(raw);
  Serial.print(F(" | Setpoint Var: ")); Serial.println(pressureSetpoint);
}

// Pump state detection -------------------------------------------------------
PumpState analyzePumpState(uint16_t currentPulse, uint16_t previousPulse) {
  uint16_t avgWidth = (currentPulse + previousPulse) / 2;
  uint16_t widthDifference = abs((int16_t)currentPulse - (int16_t)previousPulse);
  bool isSymmetric = (widthDifference <= pumpDetectConfig.symmetryTolerance_us);
  if (isSymmetric && avgWidth > pumpDetectConfig.offPulseThreshold_us) return PUMP_OFF;
  else if (!isSymmetric) return PUMP_BREVILLE_CONTROL;
  else if (isSymmetric && avgWidth < pumpDetectConfig.cleanPulseThreshold_us) return PUMP_READY_FOR_CONTROL;
  else return pumpDetect.currentState;
}
void updatePumpState() {
  if (gateMode == GATE_CONTROLLED && controlSessionActive) return;
  if (!newRisingPulsePair) return;
  noInterrupts(); uint16_t current = pumpDetect.currentRisingPulse_us; uint16_t previous = pumpDetect.previousRisingPulse_us; newRisingPulsePair=false; interrupts();
  PumpState detectedState = analyzePumpState(current, previous);
  if (detectedState == pumpDetect.detectedState) pumpDetect.stableCount++; else { pumpDetect.detectedState = detectedState; pumpDetect.stableCount = 1; }
  if (pumpDetect.stableCount >= pumpDetectConfig.stableReadingsRequired) {
    if (detectedState != pumpDetect.currentState) { pumpDetect.currentState = detectedState; pumpDetect.lastStateChange = millis(); pumpDetect.stateChangeCount++; }
    switch (pumpDetect.currentState) { case PUMP_OFF: pumpDetect.offCount++; break; case PUMP_BREVILLE_CONTROL: pumpDetect.brevilleCount++; break; case PUMP_READY_FOR_CONTROL: pumpDetect.readyCount++; break; }
  }
}

void zeroCrossISR();
ISR(PCINT2_vect) { zeroCrossISR(); }

volatile uint16_t zcCountForFreq = 0;
void zeroCrossISR() {
  uint32_t now = micros();
  uint8_t currentState = digitalRead(ZERO_CROSS_PIN);
  zcEdgesPerSecond++;
  if (now - lastZcCountReset > 1000000UL) { lastZcCountReset = now; zcEdgesPerSecond = 0; }
  static uint32_t lastEdgeTime = 0; if (now - lastEdgeTime < 500) return; lastEdgeTime = now;
  if (currentState == HIGH && lastZCState == LOW) {
    zcRisingEdgeTime = now;
    zcCountForFreq++;
  } else if (currentState == LOW && lastZCState == HIGH) {
    if (zcRisingEdgeTime > 0) {
      uint32_t pulseWidth = now - zcRisingEdgeTime;
      if (pulseWidth < 20000) {
        pumpDetect.previousRisingPulse_us = pumpDetect.currentRisingPulse_us;
        pumpDetect.currentRisingPulse_us  = pulseWidth;
        if (pumpDetect.previousRisingPulse_us > 0) newRisingPulsePair = true;
      }
    }
    zcFallingEdgeTime = now; zcCountForFreq++;
  }
  lastZCState = currentState;
}

void monitorZeroCross() {
  static unsigned long lastZCReport = 0;
  if (millis() - lastZCReport >= 1000) {
    zcFreqHistory[zcFreqIndex] = zcCountForFreq; zcFreqIndex = (zcFreqIndex + 1) % 5;
    avgZcFrequency = 0; for (int i=0;i<5;i++) avgZcFrequency += zcFreqHistory[i]; avgZcFrequency/=5;
    zcCountForFreq = 0; lastZCReport = millis();
  }
}

// Sensors & conversions ------------------------------------------------------
#define MAX_SAFE_PRESSURE 12.0f
#define MAX_SAFE_TEMP     125.0f
#define MIN_SENSOR_VALUE  0
#define MAX_SENSOR_VALUE  1023
#define DEBOUNCE_DELAY    50
#define WDT_TIMEOUT       WDTO_4S
#define NTC_B_VALUE           3950.0f
#define NTC_REF_RESISTANCE    10000.0f
#define NTC_NOMINAL_TEMP_C    25.0f

float adcToCelsius(int adcValue) {
  if (adcValue <= MIN_SENSOR_VALUE || adcValue >= MAX_SENSOR_VALUE) return -100.0f;
  float voltage = (adcValue / 1023.0f) * 5.0f;
  float resistance = NTC_REF_RESISTANCE * (voltage / (5.0f - voltage));
  float steinhart = resistance / NTC_REF_RESISTANCE; steinhart = log(steinhart);
  steinhart /= NTC_B_VALUE; steinhart += 1.0f / (NTC_NOMINAL_TEMP_C + 273.15f);
  steinhart = 1.0f / steinhart; steinhart -= 273.15f; return steinhart;
}
float adcToPressure(int adcValue) {
  if (adcValue <= MIN_SENSOR_VALUE || adcValue >= MAX_SENSOR_VALUE) return -1.0f;
  float voltage = (adcValue / 1023.0f) * 5.0f;
  float pressure = (voltage - 0.5f) * (12.0f / 4.0f);
  return constrain(pressure, 0.0f, 15.0f);
}
bool validateSensorReading(int adcValue, const char* sensorName) {
  if (adcValue < MIN_SENSOR_VALUE || adcValue > MAX_SENSOR_VALUE) { Serial.print(F("Sensor error: ")); Serial.print(sensorName); Serial.print(F(" = ")); Serial.println(adcValue); return false; } return true; }

void updateSensorReadings() {
  systemState.pressureRaw     = analogRead(PRESSURE_PIN);
  systemState.groupHeadRaw    = analogRead(GROUPHEAD_TEMP_PIN);
  systemState.thermoblockRaw  = analogRead(THERMOBLOCK_TEMP_PIN);
  systemState.manualPotRaw    = analogRead(MANUAL_POT_PIN);
  
  bool sensorsValid = true;
  sensorsValid &= validateSensorReading(systemState.pressureRaw,    "Pressure");
  sensorsValid &= validateSensorReading(systemState.groupHeadRaw,   "GroupHead");
  sensorsValid &= validateSensorReading(systemState.thermoblockRaw, "Thermoblock");
  
  if (!sensorsValid) { systemState.sensorError = true; systemState.hasCriticalError = true; return; }
  
  // === DUAL-STREAM FILTERING ===
  static float pressureSlow = 0.0f; 
  static float pressureFast = 0.0f; 
  static bool firstReading = true;
  
  float instantPressure = adcToPressure(systemState.pressureRaw);
  
  if (firstReading) {
      pressureSlow = instantPressure;
      pressureFast = instantPressure;
      firstReading = false;
  } else {
      pressureSlow = (pressureSlow * 0.95f) + (instantPressure * 0.05f);
      pressureFast = (pressureFast * 0.75f) + (instantPressure * 0.25f);
  }
  
  systemState.pressure = pressureSlow; 
  currentPressure = pressureFast;      

  systemState.groupHeadTemp   = adcToCelsius(systemState.groupHeadRaw);
  systemState.thermoblockTemp = adcToCelsius(systemState.thermoblockRaw);
  
  if (systemState.pressure > MAX_SAFE_PRESSURE || systemState.groupHeadTemp > MAX_SAFE_TEMP || systemState.thermoblockTemp > MAX_SAFE_TEMP) { Serial.println(F("Safety limits exceeded")); }
  
  systemState.isManualMode = (digitalRead(MODE_SWITCH_PIN) == LOW);
  
  // === FIX: REMOVED LEGACY LINE ===
  // The line below was resetting selectedProgram to -1, fighting with runManualMode.
  // We now handle selectedProgram assignment exclusively in runManualMode (Manual) 
  // or map logic in Auto (added below for completeness).
  
  if (!systemState.isManualMode) {
      // In Auto Mode, map pot to 0, 1, 2
      systemState.selectedProgram = map(systemState.manualPotRaw, 0, 1023, 0, 2);
  }
  // In Manual Mode, do nothing here. runManualMode will update it.
  
  systemState.sensorError = false;
}

void updateButtonStates() {
  static unsigned long lastDebounceTime[3] = {0};
  static bool lastButtonState[3]   = {HIGH, HIGH, HIGH};
  static bool stableButtonState[3] = {HIGH, HIGH, HIGH};
  bool currentButtonState[3];
  currentButtonState[0] = digitalRead(MODE_SWITCH_PIN);
  currentButtonState[1] = digitalRead(POT_SWITCH_PIN);
  currentButtonState[2] = digitalRead(POWER_BUTTON_PIN);
  for (int i=0;i<3;i++) {
    if (currentButtonState[i] != lastButtonState[i]) lastDebounceTime[i] = millis();
    if ((millis() - lastDebounceTime[i]) > DEBOUNCE_DELAY) {
      if (currentButtonState[i] != stableButtonState[i]) stableButtonState[i] = currentButtonState[i];
    }
    lastButtonState[i] = currentButtonState[i];
  }
  systemState.powerButtonPressed = !stableButtonState[2];
}

void updateOperatingMode() {
  bool potSwitchOff = (digitalRead(POT_SWITCH_PIN) == HIGH);
  bool modeIsManual = (digitalRead(MODE_SWITCH_PIN) == LOW);
  OperatingMode previousMode = currentOperatingMode;
  
  if (potSwitchOff) currentOperatingMode = MODE_BYPASS;
  else if (modeIsManual) currentOperatingMode = MODE_MANUAL;
  else currentOperatingMode = MODE_AUTO;
  
  if (currentOperatingMode != previousMode) {
      // === FIX: HARD RESET STATE ON MODE CHANGE ===
      // Kill any active session immediately
      if (controlSessionActive) {
          returnToMonitor();
      }
      // Force shot state inactive to prevent "Zombie" Auto shots
      systemState.shotIsActive = false;
      pressureSetpoint = 0;
      setPumpPower(0);
  }
  
  systemState.potSwitchOff = potSwitchOff; 
  systemState.isManualMode = modeIsManual;
}

void updateMachineState() {
  MachineStateEnum previousState = currentMachineState;
  if (systemState.sensorError) currentMachineState = MACHINE_ERROR;
  else if (systemState.thermoblockTemp < 30.0f) currentMachineState = MACHINE_OFF;
  else {
    switch (currentMachineState) {
      case MACHINE_OFF:
      case MACHINE_WARMING:
        if (systemState.thermoblockTemp >= 81.0f) currentMachineState = MACHINE_READY;
        else if (systemState.thermoblockTemp >= 30.0f) currentMachineState = MACHINE_WARMING;
        break;
      case MACHINE_READY:
        if (systemState.thermoblockTemp < 79.0f) currentMachineState = MACHINE_WARMING;
        break;
      case MACHINE_ERROR: break;
    }
  }
}

void handlePumpDetection() {
  if (pumpActive && !wasPumpActive) onPumpActivated();
  if (!pumpActive && wasPumpActive) onPumpDeactivated();
  wasPumpActive = pumpActive;
}

void onPumpActivated() {
  Serial.println(); Serial.println(F("=== PUMP ACTIVATED ==="));
  switch (currentOperatingMode) {
    case MODE_BYPASS: Serial.println(F("BYPASS mode")); break;
    
    case MODE_MANUAL: 
      Serial.println(F("MANUAL mode")); 
      systemState.shotIsActive = true; 
      break;

    case MODE_AUTO:
      Serial.println(F("AUTO mode - Waiting for Target..."));
      break;
  }
}
void onPumpDeactivated() {
  Serial.println(); Serial.println(F("=== PUMP DEACTIVATED ==="));
  if (systemState.shotIsActive) { 
      if (currentOperatingMode == MODE_MANUAL && controlSessionActive) {
          Serial.println(F("Ignoring deactivation (Manual Mode)"));
      } else {
          Serial.println(F("Shot ended")); 
          systemState.shotIsActive = false; 
          currentBrewState = BREW_IDLE; 
          
          pressureSetpoint = 0;
          forcePumpOff(); // Hard Kill
      }
  }
}

void runBypassMode() { 
  if (controlSessionActive) { returnToMonitor(); }
  digitalWrite(AC_DIMMER_GATE_PIN, HIGH);
  logPotSnapshot_1Hz();  
}

void runManualMode() {
  logPotSnapshot_1Hz();
  uint16_t raw = analogRead(MANUAL_POT_PIN);
  potFilt = ((potFilt * (8 - POT_SMOOTH_ALPHA)) + (raw * POT_SMOOTH_ALPHA)) >> 3;
  
  float rawTarget = map(potFilt, 0, 1023, 0, 90) / 10.0f;
  if(rawTarget < 0) rawTarget = 0;
  
  static float currentSmoothedTarget = 0.0f;
  const float MAX_RISE_PER_LOOP = 0.04f; 
  const float MAX_FALL_PER_LOOP = 0.20f; 
  
  if (rawTarget > currentSmoothedTarget) {
      currentSmoothedTarget += MAX_RISE_PER_LOOP;
      if (currentSmoothedTarget > rawTarget) currentSmoothedTarget = rawTarget; 
  } 
  else if (rawTarget < currentSmoothedTarget) {
      currentSmoothedTarget -= MAX_FALL_PER_LOOP;
      if (currentSmoothedTarget < rawTarget) currentSmoothedTarget = rawTarget; 
  }
  
  pressureSetpoint = (double)currentSmoothedTarget;
  systemState.selectedProgram = (int32_t)(rawTarget * 10); 
  
  if (controlSessionActive) {
      systemState.shotIsActive = true;
      controlSessionStartMs = millis(); 
  }
}

void runAutoMode() {
  // === FIX: REMOVED THE FORCE 99% LINE ===
  // If shot inactive, just leave. Let Cleanup handle the pump.
  if (!systemState.shotIsActive) { 
      return; 
  }
  
  pressureSetpoint = currentTargets.targetPressure;
  tempSetpoint     = currentTargets.targetTemp;
}

// === FORCE KILL ===
void forcePumpOff() {
    pumpPowerPercent = 0;
    acDimmer.setState(OFF); 
    digitalWrite(AC_DIMMER_GATE_PIN, LOW); 
}

void updatePIDControllers() {
  currentTemp = (double)systemState.thermoblockTemp;
  bool runPID = false;
  
  if (currentOperatingMode != MODE_BYPASS) {
      tempControl.setTemperature(currentTargets.targetTemp);
      // === FIX: Update the Feedback Variable ===
      // Previously this was only updating inside the 'if shotActive' block.
      // We move it here so the UI sees the correct "Set:" temp during Idle/Preheat.
      tempSetpoint = currentTargets.targetTemp;
  }
  else {
      // Bypass Mode: We MUST release control to the Breville
      tempControl.writeRegister(0x0A, 0x00); // <--- ENSURE THIS EXISTS FOR BYPASS
  }

  if (systemState.shotIsActive && !systemState.hasCriticalError) {
      if (currentOperatingMode == MODE_AUTO) {
          runPID = true;
      }
      else if (currentOperatingMode == MODE_MANUAL) {
          runPID = true;
      }
  }
  else {
      if (pressurePID.GetMode() != MANUAL) { 
          pressurePID.SetMode(MANUAL); 
          tempPID.SetMode(MANUAL); 
          pressureOutput=0; 
          tempOutput=128; 
      }
      
      // Only clear if NOT manual (Persist manual target for UI/Handoff)
      if (currentOperatingMode != MODE_MANUAL) {
          pressureSetpoint = 0;
      }
      //tempControl.writeRegister(0x0A, 0x00); 
  }
  
  // Kill Window Check
  if (millis() < pumpKillUntil) {
      runPID = false;
      if (controlSessionActive) forcePumpOff();
  }

  if (runPID) {
      if (pressurePID.GetMode() != AUTOMATIC) {
          pressurePID.SetMode(AUTOMATIC);
          tempPID.SetMode(AUTOMATIC);
      }
      pressurePID.Compute();
      tempPID.Compute();
      
      if (controlSessionActive) {
          setPumpPower((uint8_t)pressureOutput);
          
          // === RESTORED DEBUG LOGGING ===
          static unsigned long lastPidPrint = 0;
          if (millis() - lastPidPrint > 500) {
              lastPidPrint = millis();
              Serial.print(F("[PID] Tgt: ")); Serial.print(pressureSetpoint);
              Serial.print(F(" | Act: ")); Serial.print(currentPressure);
              Serial.print(F(" | Out: ")); Serial.println(pressureOutput);
          }
      }
  }
  else if (controlSessionActive && millis() >= pumpKillUntil) {
      forcePumpOff();
  }
}

void onI2CRequest() {
  MachineState s; 
  s.pressure = systemState.pressure; 
  s.groupHeadTemp = systemState.groupHeadTemp; 
  s.thermoblockTemp = systemState.thermoblockTemp;
  s.targetPressure = (float)pressureSetpoint; 
  s.targetTemp = (float)tempSetpoint;         
  s.pumpPower = pumpPowerPercent;
  s.selectedProgram = (uint8_t)systemState.selectedProgram;
  s.operatingMode = (uint8_t)currentOperatingMode;
  
  s.stateFlags = 0;
  if (systemState.shotIsActive)    s.stateFlags |= (1 << 0);
  if (systemState.isManualMode)    s.stateFlags |= (1 << 1);
  if (systemState.sensorError)     s.stateFlags |= (1 << 2);
  if (systemState.powerButtonPressed) s.stateFlags |= (1 << 3);
  
  s.heartbeat = systemState.heartbeat;
  s.padding = 0;
  
  Wire.write((uint8_t*)&s, sizeof(MachineState)); 
  i2cRequestOccurred=true;
}

void onI2CReceive(int numBytes) {
  if (numBytes > 0 && Wire.available()) {
    uint8_t firstByte = Wire.peek();
    if (firstByte <= 3 && numBytes > 1) { int bytesRead = Wire.readBytes((uint8_t*)&receivedDebugMsg, numBytes); if (bytesRead == numBytes) newDebugMessage = true; }
    else { receivedI2CCommand = Wire.read(); i2cDataLength = 0; while (Wire.available() && i2cDataLength < sizeof(i2cCommandData)) { i2cCommandData[i2cDataLength++] = Wire.read(); } newI2CCommand = true; }
  }
}

void handleI2CCommands() {
  if (i2cRequestOccurred) { noInterrupts(); i2cRequestOccurred=false; interrupts(); }
  if (newI2CCommand) {
    noInterrupts(); uint8_t command=receivedI2CCommand; uint8_t dataLen=i2cDataLength; uint8_t commandData[16]; memcpy(commandData,(void*)i2cCommandData,dataLen); newI2CCommand=false; interrupts();
    switch (command) {
      case CMD_SET_PID_TARGETS:
        if (dataLen >= sizeof(ControlTargets)) { 
            memcpy(&currentTargets, commandData, sizeof(ControlTargets)); 
            if (currentOperatingMode == MODE_AUTO) {
                if (currentTargets.targetPressure > 0.1) {
                    if (!systemState.shotIsActive) {
                        Serial.println(F("Target Received -> Starting Auto Shot"));
                        systemState.shotIsActive = true;
                    }
                }
                else {
                    if (systemState.shotIsActive) {
                        Serial.println(F("Zero Target -> Stopping Auto Shot"));
                        systemState.shotIsActive = false;
                        currentBrewState = BREW_IDLE;
                        pressureSetpoint = 0;
                        
                        // === SET KILL TIMER ===
                        pumpKillUntil = millis() + PUMP_KILL_DURATION_MS;
                        forcePumpOff(); 
                        
                        controlSessionStartMs = millis();
                    }
                }
            }
        }
        break;
      case CMD_PRESS_POWER_BUTTON:
        digitalWrite(POWER_BUTTON_CTRL_PIN, HIGH); delay(250); digitalWrite(POWER_BUTTON_CTRL_PIN, LOW);
        Serial.println(F("Power Button Pressed via I2C Command"));
        break;
      case CMD_SET_MANUAL_PREINFUSION:
        if (dataLen >= sizeof(ManualPreinfusionConfig)) { memcpy(&preinfusionConfig, commandData, sizeof(ManualPreinfusionConfig)); }
        break;
      default: break;
    }
  }
}

void emergencyACShutdown() {
  pumpPowerPercent = 0;
  if (controlSessionActive) { acDimmer.setPower(0); acDimmer.setState(ON); } else { digitalWrite(AC_DIMMER_GATE_PIN, LOW); }
  Serial.println(F("[DIMMER] EMERGENCY SHUTDOWN"));
}
void emergencyStop(const char* reason) {
  currentShotState = FAULT; systemState.shotIsActive = false; systemState.hasCriticalError = true;
  emergencyACShutdown(); tempControl.setTemperature(85.0f);
  pressurePID.SetMode(MANUAL); tempPID.SetMode(MANUAL); pressureOutput=0; tempOutput=128;
  Serial.print(F("EMERGENCY STOP: ")); Serial.println(reason);
  strncpy(systemState.lastErrorMessage, reason, sizeof(systemState.lastErrorMessage)-1); systemState.lastErrorTime = millis();
}

void initACDimmer() {
  Serial.println(F("[DIMMER] Initializing RBDdimmer library..."));
  acDimmer.begin(NORMAL_MODE, ON);
  acDimmer.setPower(0);
  Serial.println(F("[DIMMER] RBDdimmer initialized"));
}
void returnToMonitor() {
  if (!controlSessionActive) return;
  acDimmer.setPower(0); acDimmer.setState(OFF);
  noInterrupts(); EIMSK &= ~(1 << INT0); TIMSK2 = 0; interrupts();
  pinMode(AC_DIMMER_GATE_PIN, OUTPUT); digitalWrite(AC_DIMMER_GATE_PIN, HIGH);
  noInterrupts(); PCICR |= (1 << PCIE2); PCMSK2 |= (1 << PCINT18); interrupts();
  gateMode = GATE_MONITOR_HIGH; controlSessionActive = false;
  Serial.println(F("[DIMMER] Returned to MONITOR (library disabled; D3=HIGH)"));
}
void handoffToRBDDimmer() {
  if (controlSessionActive) return;
  noInterrupts(); PCMSK2 &= ~(1 << PCINT18); PCICR &= ~(1 << PCIE2); interrupts();
  pinMode(AC_DIMMER_GATE_PIN, OUTPUT); digitalWrite(AC_DIMMER_GATE_PIN, LOW);
  initACDimmer();
  
  uint8_t startPower = (uint8_t)constrain(pressureOutput, 0, 99);
  acDimmer.setPower(startPower);
  pumpPowerPercent = startPower;
  
  controlSessionActive   = true;
  controlSessionStartMs  = millis();
  Serial.println(F("[DIMMER] Handoff complete → RBDdimmer now owns D2/D3/timers"));
}

void setPumpPower(uint8_t powerPercent) {
  // === FIX: CHECK KILL TIMER FIRST ===
  if (millis() < pumpKillUntil) {
      acDimmer.setPower(0);
      return;
  }

  // === FIX: LINEARIZE PUMP OUTPUT ===
  // Instead of a hard clamp (1-45 -> 45), we map the PID's 0-99 range
  // to the pump's effective 45-99 range. This gives the PID fine control
  // and immediate response, preventing integral windup on start.
  
  uint8_t effectivePower = 0;
  
  if (powerPercent > 0) {
      // Map 1..99 input to 45..99 output
      // This means PID output 1 instantly gets the pump to its starting threshold
      effectivePower = map(powerPercent, 1, 99, 45, 99);
  } else {
      effectivePower = 0;
  }
  
  // Update Global
  pumpPowerPercent = effectivePower;
  
  if (!controlSessionActive) return; 
  
  acDimmer.setState(ON);
  acDimmer.setPower(effectivePower);
}

void enter_arduino_idle_mode(void)
{
    if (arduino_in_idle_mode) return;
    Serial.println(F("Entering Arduino idle mode (TB < 25°C)"));
    digitalWrite(AC_DIMMER_GATE_PIN, HIGH);
    set_sleep_mode(SLEEP_MODE_IDLE);
    arduino_in_idle_mode = true;
    delay(10); 
}

void wake_from_arduino_idle_mode(void)
{
    if (!arduino_in_idle_mode) return;
    arduino_in_idle_mode = false;
    Serial.println(F("Arduino waking from idle mode (TB >= 25°C)"));
}

void monitor_arduino_power_mode(void)
{
    static unsigned long last_check = 0;
    if (!arduino_in_idle_mode && (millis() - last_check < 1000)) return; 
    last_check = millis();
    if (!ENABLE_ARDUINO_IDLE_MODE) return;
    static uint8_t temp_stable_count = 0;
    const uint8_t STABLE_READINGS_REQUIRED = 5;
    float tb_temp = systemState.thermoblockTemp;
    
    if (!arduino_in_idle_mode) {
        if (tb_temp < TB_IDLE_THRESHOLD_C && !systemState.shotIsActive) {
            temp_stable_count++;
            if (temp_stable_count >= STABLE_READINGS_REQUIRED) {
                enter_arduino_idle_mode();
                temp_stable_count = 0;
            }
        } else {
            temp_stable_count = 0;
        }
    }
    else {
        if (tb_temp >= (TB_IDLE_THRESHOLD_C + 1.0f) || systemState.shotIsActive) {
            temp_stable_count++;
            if (temp_stable_count >= STABLE_READINGS_REQUIRED) {
                wake_from_arduino_idle_mode();
                temp_stable_count = 0;
            }
        } else {
            temp_stable_count = 0;
        }
    }
}

void setup() {
  Serial.begin(115200); delay(50);
  analogReference(DEFAULT);
  Serial.println(F("=== Arduino V44 - Final Clean Auto Fix ==="));
  
  pinMode(AC_DIMMER_GATE_PIN, OUTPUT); digitalWrite(AC_DIMMER_GATE_PIN, HIGH); gateMode = GATE_MONITOR_HIGH;
  pinMode(PRESSURE_PIN, INPUT); pinMode(GROUPHEAD_TEMP_PIN, INPUT); pinMode(THERMOBLOCK_TEMP_PIN, INPUT); pinMode(MANUAL_POT_PIN, INPUT);
  pinMode(POWER_BUTTON_PIN, INPUT_PULLUP); pinMode(POWER_BUTTON_CTRL_PIN, OUTPUT);
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP); pinMode(POT_SWITCH_PIN,  INPUT_PULLUP);
  digitalWrite(POWER_BUTTON_CTRL_PIN, LOW);
  
  tempControl.init();   
  pinMode(ZERO_CROSS_PIN, INPUT);
  noInterrupts(); PCICR |= (1 << PCIE2); PCMSK2 |= (1 << PCINT18); interrupts();
  
  pressurePID.SetOutputLimits(0, 99); 
  tempPID.SetOutputLimits(0, 255); 
  pressurePID.SetTunings(6.0, 2.5, 0.0); 
  pressurePID.SetSampleTime(20); 
  tempPID.SetSampleTime(20);
  pressurePID.SetMode(MANUAL); 
  tempPID.SetMode(MANUAL);
  
  Wire.begin(I2C_SLAVE_ADDRESS); Wire.onRequest(onI2CRequest); Wire.onReceive(onI2CReceive);
  wdt_enable(WDT_TIMEOUT);
}

void loop() {
  wdt_reset();
  monitor_arduino_power_mode();
  if (arduino_in_idle_mode) { sleep_enable(); sleep_cpu(); sleep_disable(); }
  
  monitorZeroCross();
  updatePumpState();
  systemState.heartbeat++;

  static unsigned long lastControlUpdate = 0;
  if (millis() - lastControlUpdate >= 20) {
    lastControlUpdate = millis();
    updateSensorReadings();
    updateButtonStates();
    updateOperatingMode();
    updateMachineState();
    handlePumpDetection();
    switch (currentOperatingMode) { 
        case MODE_BYPASS: runBypassMode(); break; 
        case MODE_MANUAL: runManualMode(); break; 
        case MODE_AUTO:   runAutoMode(); break; 
    }
    updatePIDControllers();
  }

  bool isWarmEnoughForControl = (systemState.thermoblockTemp >= 50.0f);
  bool wantControl = false;

  if (currentOperatingMode == MODE_MANUAL && pressureSetpoint > 0.5) {
      wantControl = true; 
  } 
  else if (currentOperatingMode == MODE_AUTO && pressureSetpoint > 0.5) {
      wantControl = true; 
  }

  if (!controlSessionActive && 
      isWarmEnoughForControl && 
      pumpDetect.currentState == PUMP_READY_FOR_CONTROL &&
      wantControl &&
      currentOperatingMode != MODE_BYPASS) {
      
      handoffToRBDDimmer();
  }

  if (controlSessionActive) {
    const bool idle = !systemState.shotIsActive && (currentBrewState == BREW_IDLE);
    const bool timedOut = (millis() - controlSessionStartMs) >= CONTROL_SESSION_TIMEOUT_MS;
    if (idle && timedOut) returnToMonitor();
  }

  handleI2CCommands();
  delay(1);
}