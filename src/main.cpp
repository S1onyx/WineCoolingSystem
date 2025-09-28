#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Preferences.h>
#include <Wire.h>
#include <cstdint>

#if defined(ESP32)
#include <esp_task_wdt.h>
#endif

// ---------- OLED / I2C ----------
constexpr uint8_t OLED_ADDR = 0x3C;
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------- IO Pins ----------
constexpr uint8_t PIN_MODE_A = 32;   // left -> GND = OFF
constexpr uint8_t PIN_MODE_B = 33;   // right -> GND = ON
constexpr uint8_t PIN_BTN_UP = 25;   // to GND (active LOW)
constexpr uint8_t PIN_BTN_DOWN = 26; // to GND (active LOW)

// ---------- Sensor & Relay ----------
constexpr uint8_t PIN_ONEWIRE = 4; // DS18B20 Data
constexpr uint8_t PIN_RELAY = 23;  // Relay IN
constexpr bool RELAY_ACTIVE_HIGH = true;

OneWire oneWire(PIN_ONEWIRE);
DallasTemperature sensors(&oneWire);
DeviceAddress dsAddr;

// ---------- Settings ----------
constexpr float SET_STEP = 0.1f;
constexpr float SET_MIN = -20.0f;
constexpr float SET_MAX = 80.0f;
constexpr float HYST = 0.2f;

enum class Mode { OFF, AUTO, ON };
float setpoint = 18.0f;
float lastTempRaw = NAN;
float lastTemp = NAN;
bool relayState = false;
Mode lastMode = Mode::AUTO;

// ---------- Smoothing ----------
float emaTemp = NAN;
constexpr float EMA_ALPHA = 0.25f;
inline float smooth(float prev, float now) {
  if (isnan(prev))
    return now;
  return prev + EMA_ALPHA * (now - prev);
}

// ---------- AUTO timing ----------
constexpr uint32_t OPEN_DURATION_MS = 10UL * 1000UL;
constexpr uint32_t COOLDOWN_MS = 120UL * 1000UL;

enum class AutoState { OPEN, CLOSED };
AutoState autoState = AutoState::CLOSED;
uint32_t stateStartedAt = 0;
uint32_t lastOpenAt = 0;

// ---------- Display power ----------
constexpr uint32_t DISPLAY_ON_TIMEOUT_MS = 120UL * 1000UL;
bool displayIsOn = true;
uint32_t lastInteractionAt = 0;

// ---------- Persistence ----------
Preferences prefs;
constexpr const char *NVS_NS = "winectrl";
constexpr const char *NVS_KEY_SETPOINT = "sp";
bool setpointDirty = false;
uint32_t lastSetpointChangeAt = 0;
constexpr uint32_t SETPOINT_SAVE_DELAY_MS = 1500;
constexpr float SP_SAVE_EPS = 0.05f;
float lastSavedSetpoint = NAN;

// ---------- Robustness Counters ----------
static uint32_t i2cErrorCount = 0;
static uint32_t dsNaNStreak = 0;
constexpr uint32_t DS_NAN_STREAK_RESET = 8; // n*~1s → ~8 Sekunden
constexpr uint32_t I2C_ERROR_RESET = 5;     // nach 5 Fehlern neu initialisieren

// ---------- Forward declarations ----------
void recoverI2CAndDisplay();
void initDisplay(bool showSplash);
void displayWake();
void displaySleep();

// ---------- Persistence helpers ----------
void saveSetpoint() {
  if (isnan(setpoint))
    return;
  if (isnan(lastSavedSetpoint) ||
      fabsf(setpoint - lastSavedSetpoint) >= SP_SAVE_EPS) {
    prefs.putFloat(NVS_KEY_SETPOINT, setpoint);
    lastSavedSetpoint = setpoint;
    setpointDirty = false;
  }
}
void loadSetpoint() {
  float v = prefs.getFloat(NVS_KEY_SETPOINT, NAN);
  if (!isnan(v)) {
    setpoint = constrain(v, SET_MIN, SET_MAX);
    lastSavedSetpoint = setpoint;
  }
}
void maybePersistSetpoint(uint32_t now) {
  if (setpointDirty &&
      (uint32_t)(now - lastSetpointChangeAt) >= SETPOINT_SAVE_DELAY_MS) {
    saveSetpoint();
  }
}

// ---------- Calibration (piecewise linear) ----------
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

// ---------- Small I2C sanity check (optional) ----------
static bool i2cPing(uint8_t addr = OLED_ADDR) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0; // true = OK
}

// ---------- Display power ----------
void displayWake() {
  if (!displayIsOn) {
    display.ssd1306_command(SSD1306_DISPLAYON); // void API, no return value
    if (!i2cPing())
      i2cErrorCount++;
    displayIsOn = true;
  }
  lastInteractionAt = millis();
}
void displaySleep() {
  if (displayIsOn) {
    display.ssd1306_command(SSD1306_DISPLAYOFF); // void API, no return value
    if (!i2cPing())
      i2cErrorCount++;
    displayIsOn = false;
  }
}
void updateDisplayPower(uint32_t now) {
  if (displayIsOn &&
      (uint32_t)(now - lastInteractionAt) >= DISPLAY_ON_TIMEOUT_MS) {
    displaySleep();
  }
}

// ---------- Mode switch ----------
Mode readModeSwitch(Mode lastKnown) {
  bool a = digitalRead(PIN_MODE_A);
  bool b = digitalRead(PIN_MODE_B);
  if (!a && b)
    return Mode::OFF;
  if (a && !b)
    return Mode::ON;
  if (a && b)
    return Mode::AUTO; // middle
  return lastKnown;    // both LOW -> keep last
}

// ---------- Relay ----------
void relayWrite(bool on) {
  relayState = on;
  if (RELAY_ACTIVE_HIGH)
    digitalWrite(PIN_RELAY, on ? HIGH : LOW);
  else
    digitalWrite(PIN_RELAY, on ? LOW : HIGH);
}
void clampSetpoint() { setpoint = constrain(setpoint, SET_MIN, SET_MAX); }

// ---------- AUTO helpers ----------
inline bool cooldownOver(uint32_t now) {
  return (uint32_t)(now - lastOpenAt) >= COOLDOWN_MS;
}
inline bool shouldOpenNow() {
  if (isnan(lastTemp))
    return false;
  return lastTemp > (setpoint + HYST * 0.5f);
}
void startOpen(uint32_t now) {
  autoState = AutoState::OPEN;
  stateStartedAt = now;
  lastOpenAt = now;
  relayWrite(true);
}
void startClosed(uint32_t now) {
  autoState = AutoState::CLOSED;
  stateStartedAt = now;
  relayWrite(false);
}

void controlLogicTimedAuto(bool modeJustEntered) {
  uint32_t now = millis();
  if (modeJustEntered) {
    lastOpenAt = (now >= COOLDOWN_MS) ? (now - COOLDOWN_MS) : 0;
    startClosed(now);
    if (shouldOpenNow() && cooldownOver(now))
      startOpen(now);
    return;
  }
  switch (autoState) {
  case AutoState::CLOSED:
    if (cooldownOver(now) && shouldOpenNow())
      startOpen(now);
    break;
  case AutoState::OPEN: {
    bool timeUp = (uint32_t)(now - stateStartedAt) >= OPEN_DURATION_MS;
    if (timeUp || isnan(lastTemp))
      startClosed(now);
    break;
  }
  }
}
void controlLogic(Mode mode, bool modeJustEntered) {
  switch (mode) {
  case Mode::OFF:
    startClosed(millis());
    break;
  case Mode::ON:
    relayWrite(true);
    autoState = AutoState::CLOSED;
    stateStartedAt = millis();
    break;
  case Mode::AUTO:
    controlLogicTimedAuto(modeJustEntered);
    break;
  }
}

// ---------- UI ----------
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
  display.display(); // void API
  if (!i2cPing())
    i2cErrorCount++; // optional: count I2C issue
}
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

  display.display(); // void API
  if (!i2cPing())
    i2cErrorCount++; // optional: count I2C issue
}

// ---------- Buttons ----------
struct Btn {
  uint8_t pin;
  bool lastLevel = true;
  uint32_t pressedAt = 0;
  uint32_t lastRepeat = 0;
  bool handled = false;
  Btn(uint8_t p) : pin(p) {}
};
Btn btnUp(PIN_BTN_UP);
Btn btnDown(PIN_BTN_DOWN);

bool handleButton(Btn &b, float delta) {
  bool level = digitalRead(b.pin); // pull-up: LOW = pressed
  uint32_t now = millis();
  uint32_t held = now - b.pressedAt;
  float step = delta;
  if (held > 1200)
    step = (delta > 0 ? +1.0f : -1.0f);
  else if (held > 600)
    step = (delta > 0 ? +0.5f : -0.5f);

  bool changed = false;

  if (!level && b.lastLevel) {
    displayWake();
    b.lastLevel = level;
    if (!displayIsOn)
      return false;
    b.pressedAt = now;
    b.lastRepeat = now;
    b.handled = false;
  }

  if (!level && displayIsOn) {
    if (!b.handled && (uint32_t)(now - b.pressedAt) > 30) {
      setpoint += step;
      clampSetpoint();
      b.handled = true;
      changed = true;
    }
    if ((uint32_t)(now - b.pressedAt) > 600 &&
        (uint32_t)(now - b.lastRepeat) > 120) {
      setpoint += step;
      clampSetpoint();
      b.lastRepeat = now;
      changed = true;
    }
  }
  b.lastLevel = level;

  if (changed) {
    setpointDirty = true;
    lastSetpointChangeAt = now;
  }
  return changed;
}

// ---------- Robustness helpers ----------
void initDisplay(bool showSplash) {
  // Safe I2C defaults (slower clock, timeout to avoid hard hang)
  Wire.begin();
  Wire.setClock(100000); // 100 kHz for stability
  Wire.setTimeOut(50);   // abort I2C ops after 50ms to avoid deadlocks

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    // If we fail here, count an error; try later again
    i2cErrorCount++;
    displayIsOn = false;
    return;
  }
  displayIsOn = true;
  if (showSplash)
    drawSplash();
}
void recoverI2CAndDisplay() {
  // Simple recovery: re-init I2C & display
  initDisplay(false);
  // reset error counter after attempt
  i2cErrorCount = 0;
}

// ---------- Setup ----------
void setup() {
  // I/O
  pinMode(PIN_MODE_A, INPUT_PULLUP);
  pinMode(PIN_MODE_B, INPUT_PULLUP);
  pinMode(PIN_BTN_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
  pinMode(PIN_RELAY, OUTPUT);
  relayWrite(false);

  // I2C + OLED
  initDisplay(true);

  // DS18B20
  sensors.begin();
  bool hasAddr = sensors.getAddress(dsAddr, 0);
  if (hasAddr)
    sensors.setResolution(dsAddr, 12);
  sensors.setWaitForConversion(false);

  // Persistence
  prefs.begin(NVS_NS, false);
  loadSetpoint();

  displayIsOn = true;
  lastInteractionAt = millis();

#if defined(ESP32)
  // Watchdog: 5s window, panic on trigger → reboot
  esp_task_wdt_init(5, true);
  esp_task_wdt_add(NULL);
#endif

  // Short splash without blocking WDT
  uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < 1500) {
    delay(10);
#if defined(ESP32)
    esp_task_wdt_reset();
#endif
  }
}

// ---------- Loop ----------
void loop() {
  static uint32_t lastTempPoll = 0;
  static uint32_t lastDraw = 0;
  static float lastDrawnTemp = NAN, lastDrawnSp = NAN;
  static bool lastDrawnRelay = false;

  uint32_t now = millis();

#if defined(ESP32)
  esp_task_wdt_reset();
#endif

  updateDisplayPower(now);

  // Buttons
  bool changed = false;
  changed |= handleButton(btnUp, +SET_STEP);
  changed |= handleButton(btnDown, -SET_STEP);

  // Save setpoint (debounced)
  maybePersistSetpoint(now);

  // Temperature: asynchronous cycle ~1s
  static bool convRunning = false;
  if (!convRunning && (uint32_t)(now - lastTempPoll) > 1000) {
    sensors.requestTemperaturesByAddress(dsAddr); // target the known sensor
    convRunning = true;
    lastTempPoll = now;
  } else if (convRunning && (uint32_t)(now - lastTempPoll) >= 800) {
    float t = sensors.getTempC(dsAddr);
    lastTempRaw = (t > -127.0f && t < 125.0f) ? t : NAN;
    if (isnan(lastTempRaw)) {
      dsNaNStreak++;
      if (dsNaNStreak >= DS_NAN_STREAK_RESET) {
        // try to recover OneWire/DS
        sensors.begin();
        sensors.setWaitForConversion(false);
        dsNaNStreak = 0;
      }
    } else {
      dsNaNStreak = 0;
      float calibrated = applyCalibration(lastTempRaw);
      emaTemp = smooth(emaTemp, calibrated);
      lastTemp = emaTemp;
    }
    convRunning = false;
  }

  // Mode & control
  Mode mode = readModeSwitch(lastMode);
  bool modeJustEntered = (mode != lastMode);
  if (modeJustEntered)
    displayWake();
  controlLogic(mode, modeJustEntered);
  lastMode = mode;

  // Need redraw?
  bool needRedraw = changed || relayState != lastDrawnRelay ||
                    (displayIsOn && (uint32_t)(now - lastDraw) > 250) ||
                    (!isnan(lastTemp) && !isnan(lastDrawnTemp) &&
                     fabsf(lastTemp - lastDrawnTemp) >= 0.1f) ||
                    (fabsf(setpoint - lastDrawnSp) >= 0.1f) ||
                    (uint32_t)(now - lastInteractionAt) < 50;

  if (displayIsOn && needRedraw) {
    drawMain(mode, setpoint, lastTemp, relayState);
    lastDrawnTemp = lastTemp;
    lastDrawnSp = setpoint;
    lastDrawnRelay = relayState;
    lastDraw = now;
  }

  // I2C/OLED recovery if errors piled up
  if (i2cErrorCount >= I2C_ERROR_RESET) {
    recoverI2CAndDisplay();
  }

  delay(5); // yields to RTOS
}