#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Preferences.h>
#include <cstdint>

// OLED
constexpr uint8_t OLED_ADDR = 0x3C;
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// IO Pins
constexpr uint8_t PIN_MODE_A = 32;   // left -> GND = OFF
constexpr uint8_t PIN_MODE_B = 33;   // right -> GND = ON
constexpr uint8_t PIN_BTN_UP = 25;   // to GND (active LOW)
constexpr uint8_t PIN_BTN_DOWN = 26; // to GND (active LOW)

// Sensor & Relay
constexpr uint8_t PIN_ONEWIRE = 4; // DS18B20 Data
constexpr uint8_t PIN_RELAY = 23;  // Relay IN
constexpr bool RELAY_ACTIVE_HIGH =
    true; // true if relay is active-HIGH (many are active-LOW)

OneWire oneWire(PIN_ONEWIRE);
DallasTemperature sensors(&oneWire);
DeviceAddress dsAddr;

// Settings
// setpoint step (°C)
constexpr float SET_STEP = 0.1f;
constexpr float SET_MIN = -20.0f;
constexpr float SET_MAX = 80.0f;
// hysteresis (°C)
constexpr float HYST = 0.2f;

enum class Mode { OFF, AUTO, ON };
// raw
float setpoint = 18.0f;
float lastTempRaw = NAN;
// filtered
float lastTemp = NAN;
bool relayState = false;

// Mode
Mode lastMode = Mode::AUTO;

// Smoothing
// 0..1 (higher = less smoothing)
float emaTemp = NAN;
constexpr float EMA_ALPHA = 0.25f;
inline float smooth(float prev, float now) {
  if (isnan(prev))
    return now;
  return prev + EMA_ALPHA * (now - prev);
}

// AUTO timing
constexpr unsigned long OPEN_DURATION_MS = 10UL * 1000UL;
constexpr unsigned long COOLDOWN_MS = 120UL * 1000UL;

enum class AutoState { OPEN, CLOSED };
AutoState autoState = AutoState::CLOSED;
unsigned long stateStartedAt = 0;
unsigned long lastOpenAt = 0;

// Display power
constexpr unsigned long DISPLAY_ON_TIMEOUT_MS = 120UL * 1000UL;
bool displayIsOn = true;
unsigned long lastInteractionAt = 0;

// Persistence
Preferences prefs;
constexpr const char *NVS_NS = "winectrl";
constexpr const char *NVS_KEY_SETPOINT = "sp";
bool setpointDirty = false;
unsigned long lastSetpointChangeAt = 0;
constexpr unsigned long SETPOINT_SAVE_DELAY_MS = 1500;

constexpr float SP_SAVE_EPS = 0.05f;
float lastSavedSetpoint = NAN;

void saveSetpoint() {
  if (!isnan(setpoint)) {
    if (isnan(lastSavedSetpoint) ||
        fabsf(setpoint - lastSavedSetpoint) >= SP_SAVE_EPS) {
      prefs.begin(NVS_NS, false);
      prefs.putFloat(NVS_KEY_SETPOINT, setpoint);
      prefs.end();
      lastSavedSetpoint = setpoint;
      setpointDirty = false;
    }
  }
}
void loadSetpoint() {
  prefs.begin(NVS_NS, true);
  float v = prefs.getFloat(NVS_KEY_SETPOINT, NAN);
  prefs.end();
  if (!isnan(v)) {
    setpoint = v;
    if (setpoint < SET_MIN)
      setpoint = SET_MIN;
    if (setpoint > SET_MAX)
      setpoint = SET_MAX;
    lastSavedSetpoint = setpoint;
  }
}
void maybePersistSetpoint(unsigned long now) {
  if (setpointDirty && (now - lastSetpointChangeAt) >= SETPOINT_SAVE_DELAY_MS) {
    saveSetpoint();
  }
}

// Calibration
// piecewise linear mapping
static const int CAL_COUNT = 4;
static const float CAL_RAW[] = {9.0f, 24.0f, 41.0f, 95.5f};
static const float CAL_REF[] = {11.0f, 25.4f, 43.0f, 92.0f};

float applyCalibration(float raw) {
  if (isnan(raw) || CAL_COUNT <= 0)
    return raw;
  if (CAL_COUNT == 1)
    return raw + (CAL_REF[0] - CAL_RAW[0]);

  if (raw <= CAL_RAW[0]) {
    float x1 = CAL_RAW[0], x2 = CAL_RAW[1];
    float y1 = CAL_REF[0], y2 = CAL_REF[1];
    return y1 + (raw - x1) * (y2 - y1) / (x2 - x1);
  }
  if (raw >= CAL_RAW[CAL_COUNT - 1]) {
    float x1 = CAL_RAW[CAL_COUNT - 2], x2 = CAL_RAW[CAL_COUNT - 1];
    float y1 = CAL_REF[CAL_COUNT - 2], y2 = CAL_REF[CAL_COUNT - 1];
    return y1 + (raw - x1) * (y2 - y1) / (x2 - x1);
  }
  for (int i = 0; i < CAL_COUNT - 1; ++i) {
    if (raw >= CAL_RAW[i] && raw <= CAL_RAW[i + 1]) {
      float x1 = CAL_RAW[i], x2 = CAL_RAW[i + 1];
      float y1 = CAL_REF[i], y2 = CAL_REF[i + 1];
      return y1 + (raw - x1) * (y2 - y1) / (x2 - x1);
    }
  }
  return raw;
}

// on
void displayWake() {
  if (!displayIsOn) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    displayIsOn = true;
  }
  lastInteractionAt = millis();
}

// off
void displaySleep() {
  if (displayIsOn) {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    displayIsOn = false;
  }
}

void updateDisplayPower(unsigned long now) {
  if (displayIsOn) {
    if (now - lastInteractionAt >= DISPLAY_ON_TIMEOUT_MS) {
      displaySleep();
    }
  }
}

// both LOW -> keep last
Mode readModeSwitch(Mode lastKnown) {
  bool a = digitalRead(PIN_MODE_A);
  bool b = digitalRead(PIN_MODE_B);
  if (!a && b)
    return Mode::OFF;
  if (a && !b)
    return Mode::ON;
  if (a && b)
    return Mode::AUTO; // middle
  // both LOW -> keep last
  return lastKnown;
}

void relayWrite(bool on) {
  relayState = on;
  if (RELAY_ACTIVE_HIGH)
    digitalWrite(PIN_RELAY, on ? HIGH : LOW);
  else
    digitalWrite(PIN_RELAY, on ? LOW : HIGH);
}

void clampSetpoint() {
  if (setpoint < SET_MIN)
    setpoint = SET_MIN;
  if (setpoint > SET_MAX)
    setpoint = SET_MAX;
}

// read latest (non-blocking)
void readTemperatureRaw() {
  sensors.requestTemperatures();
  float t = NAN;
  if (sensors.getAddress(dsAddr, 0))
    t = sensors.getTempC(dsAddr);
  lastTempRaw = (t > -127.0f && t < 125.0f) ? t : NAN;
}

// AUTO helpers
// cooldown done?
inline bool cooldownOver(unsigned long now) {
  return (now - lastOpenAt) >= COOLDOWN_MS;
}
// Open valve when temperature exceeds setpoint + half hysteresis
inline bool shouldOpenNow() {
  if (isnan(lastTemp))
    return false;
  return lastTemp > (setpoint + HYST * 0.5f);
}
// OPEN + start cooldown
void startOpen(unsigned long now) {
  autoState = AutoState::OPEN;
  stateStartedAt = now;
  lastOpenAt = now;
  relayWrite(true);
}
// CLOSED
void startClosed(unsigned long now) {
  autoState = AutoState::CLOSED;
  stateStartedAt = now;
  relayWrite(false);
}

// Control
// AUTO state machine
void controlLogicTimedAuto(bool modeJustEntered) {
  unsigned long now = millis();

  if (modeJustEntered) {
    lastOpenAt = (now >= COOLDOWN_MS) ? (now - COOLDOWN_MS) : 0;
    startClosed(now);
    if (shouldOpenNow() && cooldownOver(now)) {
      startOpen(now);
    }
    return;
  }

  switch (autoState) {
  case AutoState::CLOSED: {
    if (cooldownOver(now) && shouldOpenNow())
      startOpen(now);
    break;
  }
  case AutoState::OPEN: {
    bool timeUp = (now - stateStartedAt) >= OPEN_DURATION_MS;
    if (timeUp || isnan(lastTemp))
      startClosed(now);
    break;
  }
  }
}

void controlLogic(Mode mode, bool modeJustEntered) {
  switch (mode) {
  case Mode::OFF:
    autoState = AutoState::CLOSED;
    stateStartedAt = millis();
    relayWrite(false);
    break;
  case Mode::ON:
    autoState = AutoState::CLOSED;
    stateStartedAt = millis();
    relayWrite(true);
    break;
  case Mode::AUTO:
    controlLogicTimedAuto(modeJustEntered);
    break;
  }
}

// UI icon
void drawGrapeOutline(int16_t x, int16_t y) {
  display.drawCircle(x + 6, y + 10, 4, SSD1306_WHITE);
  display.drawCircle(x + 12, y + 10, 4, SSD1306_WHITE);
  display.drawCircle(x + 9, y + 16, 4, SSD1306_WHITE);
  display.drawCircle(x + 4, y + 18, 4, SSD1306_WHITE);
  display.drawCircle(x + 14, y + 18, 4, SSD1306_WHITE);
  display.drawCircle(x + 9, y + 22, 4, SSD1306_WHITE);
  display.drawLine(x + 9, y + 4, x + 9, y + 10, SSD1306_WHITE);
  display.drawLine(x + 9, y + 5, x + 14, y + 2, SSD1306_WHITE);
  display.drawLine(x + 14, y + 2, x + 16, y + 5, SSD1306_WHITE);
  display.drawLine(x + 16, y + 5, x + 12, y + 6, SSD1306_WHITE);
  display.drawLine(x + 12, y + 6, x + 9, y + 5, SSD1306_WHITE);
}

void drawSplash() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("Wine Cooling System"));
  display.setCursor(0, 12);
  display.println(F("designed by"));
  display.setCursor(0, 22);
  display.println(F("Simon Riedinger"));
  display.setCursor(0, 36);
  display.println(F("for"));
  display.setCursor(0, 46);
  display.println(F("Hannes Schock"));
  drawGrapeOutline(92, 8);
  display.display();
}

// UI
void drawMain(Mode m, float sp, float t, bool relay) {
  if (!displayIsOn)
    return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(F("Controller"));

  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print(F("Mode: "));
  switch (m) {
  case Mode::OFF:
    display.print(F("OFF"));
    break;
  case Mode::AUTO:
    display.print(F("AUTO"));
    break;
  case Mode::ON:
    display.print(F("ON"));
    break;
  }

  display.setCursor(0, 32);
  display.print(F("Set:  "));
  display.printf("%.1f", sp);
  display.print((char)247);
  display.print(F("C"));

  display.setCursor(0, 44);
  display.print(F("Temp: "));
  if (isnan(t)) {
    display.print(F("--.-"));
    display.setCursor(80, 44);
    display.print(F("ERR"));
  } else {
    display.printf("%.1f", t);
  }
  display.print((char)247);
  display.print(F("C"));

  display.setCursor(0, 56);
  display.print(F("Valve: "));
  display.print(relay ? "OPEN" : "CLOSED");

  display.display();
}

// Buttons
struct Btn {
  uint8_t pin;
  bool lastLevel = true;
  unsigned long pressedAt = 0;
  unsigned long lastRepeat = 0;
  bool handled = false;
  Btn(uint8_t p) : pin(p) {}
};

Btn btnUp(PIN_BTN_UP);
Btn btnDown(PIN_BTN_DOWN);

bool handleButton(Btn &b, float delta) {
  bool level = digitalRead(b.pin); // pull-up: LOW = pressed
  unsigned long now = millis();
  unsigned long held = now - b.pressedAt;
  float step = delta; // step
  if (held > 1200) {
    step = (delta > 0 ? +1.0f : -1.0f); // fast
  } else if (held > 600) {
    step = (delta > 0 ? +0.5f : -0.5f); // medium
  }
  bool changed = false;

  if (!level && b.lastLevel) {
    // wake only
    displayWake();
    b.lastLevel = level;
    if (!displayIsOn)
      return false;
    b.pressedAt = now;
    b.lastRepeat = now;
    b.handled = false;
  }

  if (!level && displayIsOn) { // only when display on
    if (!b.handled && now - b.pressedAt > 30) {
      setpoint += step;
      clampSetpoint();
      b.handled = true;
      changed = true;
    }
    if (now - b.pressedAt > 600 && now - b.lastRepeat > 120) {
      setpoint += step;
      clampSetpoint();
      b.lastRepeat = now;
      changed = true;
    }
  }

  b.lastLevel = level;

  // mark for save
  if (changed) {
    setpointDirty = true;
    lastSetpointChangeAt = now;
  }
  return changed;
}

// Setup
void setup() {
  Wire.begin();

  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  pinMode(PIN_MODE_A, INPUT_PULLUP);
  pinMode(PIN_MODE_B, INPUT_PULLUP);
  pinMode(PIN_BTN_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
  pinMode(PIN_RELAY, OUTPUT);
  relayWrite(false);

  sensors.begin();
  bool hasAddr = sensors.getAddress(dsAddr, 0);
  if (hasAddr) {
    sensors.setResolution(dsAddr, 12);
  }
  sensors.setWaitForConversion(false);

  // Load stored setpoint
  loadSetpoint();

  drawSplash();
  displayIsOn = true;
  lastInteractionAt = millis();
  delay(5000);
}

// Loop
void loop() {
  static unsigned long lastTempPoll = 0;
  static unsigned long lastDraw = 0;

  static float lastDrawnTemp = NAN, lastDrawnSp = NAN;
  static bool lastDrawnRelay = false;

  unsigned long now = millis();
  updateDisplayPower(now);

  // Buttons (setpoint)
  bool changed = false;
  changed |= handleButton(btnUp, +SET_STEP);
  changed |= handleButton(btnDown, -SET_STEP);

  // save (debounced)
  maybePersistSetpoint(now);

  // Temperature measurement asynchronous and smoothing
  static bool convRunning = false;
  if (!convRunning && (now - lastTempPoll > 1000)) {
    // start conversion ~1s
    sensors.requestTemperatures();
    convRunning = true;
    lastTempPoll = now;
  } else if (convRunning && (now - lastTempPoll >= 800)) {
    // small margin
    float t = NAN;
    if (sensors.getAddress(dsAddr, 0))
      t = sensors.getTempC(dsAddr);
    lastTempRaw = (t > -127.0f && t < 125.0f) ? t : NAN;
    float calibrated = applyCalibration(lastTempRaw);
    emaTemp = smooth(emaTemp, calibrated);
    lastTemp = emaTemp;
    convRunning = false;
  }

  // Mode & control
  Mode mode = readModeSwitch(lastMode);
  bool modeJustEntered = (mode != lastMode);
  if (modeJustEntered) {
    displayWake();
  }
  controlLogic(mode, modeJustEntered);
  lastMode = mode;

  bool needRedraw =
      changed || relayState != lastDrawnRelay ||
      (displayIsOn && (now - lastDraw > 250)) ||
      (!isnan(lastTemp) && fabsf(lastTemp - lastDrawnTemp) >= 0.1f) ||
      (fabsf(setpoint - lastDrawnSp) >= 0.1f) || (now - lastInteractionAt) < 50;

  if (displayIsOn && needRedraw) {
    drawMain(mode, setpoint, lastTemp, relayState);
    lastDrawnTemp = lastTemp;
    lastDrawnSp = setpoint;
    lastDrawnRelay = relayState;
    lastDraw = now;
  }

  delay(5);
}