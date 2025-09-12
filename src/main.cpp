#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <cstdint>

// --------- OLED ---------
constexpr uint8_t OLED_ADDR = 0x3C;
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --------- Schalter / Buttons ---------
constexpr uint8_t PIN_MODE_A = 32;   // links -> GND = OFF
constexpr uint8_t PIN_MODE_B = 33;   // rechts -> GND = ON
constexpr uint8_t PIN_BTN_UP = 25;   // gegen GND
constexpr uint8_t PIN_BTN_DOWN = 26; // gegen GND

// --------- Sensor / Relais ---------
constexpr uint8_t PIN_ONEWIRE = 4;       // DS18B20 Data
constexpr uint8_t PIN_RELAY = 23;        // Relais IN
constexpr bool RELAY_ACTIVE_HIGH = true; // viele 1-Kanal-Module sind active-LOW

OneWire oneWire(PIN_ONEWIRE);
DallasTemperature sensors(&oneWire);
DeviceAddress dsAddr;

// --------- Regelung / Anzeige ---------
constexpr float SET_STEP = 0.1f;
constexpr float SET_MIN = -20.0f;
constexpr float SET_MAX = 80.0f;
constexpr float HYST = 0.2f;

enum class Mode { OFF, AUTO, ON };
float setpoint = 18.0f;
float lastTempRaw = NAN; // unkalibriert
float lastTemp = NAN;    // kalibriert
bool relayState = false;

// --------- AUTO: Zeit-Pulsbetrieb mit Cooldown (10 s offen, mind. 120 s zu)
// ---------
constexpr unsigned long OPEN_DURATION_MS = 10UL * 1000UL; // 10 s
constexpr unsigned long COOLDOWN_MS = 120UL * 1000UL;     // 2 min

enum class AutoState { OPEN, CLOSED };
AutoState autoState = AutoState::CLOSED;
unsigned long stateStartedAt = 0; // Zeitstempel Start des aktuellen Zustands
unsigned long lastOpenAt = 0;     // Zeitstempel Beginn der letzten Öffnung

// =====================================================================
//                          KALIBRIERUNG IM CODE
// =====================================================================
// Default: keine Kalibrierung (Identität)
static const int CAL_COUNT = 0;
static const float CAL_RAW[] = {};
static const float CAL_REF[] = {};

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

// =====================================================================

Mode readModeSwitch() {
  bool a = digitalRead(PIN_MODE_A);
  bool b = digitalRead(PIN_MODE_B);
  if (!a && b)
    return Mode::OFF;
  if (a && !b)
    return Mode::ON;
  return Mode::AUTO;
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

void readTemperatureRaw() {
  sensors.requestTemperatures();
  float t = NAN;
  if (sensors.getAddress(dsAddr, 0))
    t = sensors.getTempC(dsAddr);
  lastTempRaw = (t > -127.0f && t < 125.0f) ? t : NAN;
}

// --------- AUTO-Helfer ---------
inline bool cooldownOver(unsigned long now) {
  return (now - lastOpenAt) >= COOLDOWN_MS;
}
inline bool shouldOpenNow() {
  if (isnan(lastTemp))
    return false;
  return lastTemp < (setpoint - HYST * 0.5f);
}
void startOpen(unsigned long now) {
  autoState = AutoState::OPEN;
  stateStartedAt = now;
  lastOpenAt = now; // Cooldown ab Start der Öffnung messen
  relayWrite(true);
}
void startClosed(unsigned long now) {
  autoState = AutoState::CLOSED;
  stateStartedAt = now;
  relayWrite(false);
}

// --------- Steuerlogik ---------
void controlLogicTimedAuto(bool modeJustEntered) {
  unsigned long now = millis();

  if (modeJustEntered) {
    // Beim Eintritt in AUTO sofortiges Öffnen erlauben, wenn es zu kalt ist:
    // Cooldown künstlich abgelaufen setzen
    lastOpenAt = (now >= COOLDOWN_MS) ? (now - COOLDOWN_MS) : 0;
    startClosed(now);

    // Direkt prüfen: zu kalt? -> sofort öffnen
    if (shouldOpenNow() && cooldownOver(now)) {
      startOpen(now);
    }
    return;
  }

  switch (autoState) {
  case AutoState::CLOSED: {
    // Dauerhaft messen: sobald zu kalt UND Cooldown vorbei -> sofort öffnen
    if (cooldownOver(now) && shouldOpenNow()) {
      startOpen(now);
    }
    // sonst geschlossen lassen
    break;
  }

  case AutoState::OPEN: {
    // Immer mindestens 10 s offen bleiben (nie früher schließen)
    bool timeUp = (now - stateStartedAt) >= OPEN_DURATION_MS;
    if (timeUp) {
      startClosed(now);
    }
    break;
  }
  }
}

void controlLogic(Mode mode, bool modeJustEntered) {
  switch (mode) {
  case Mode::OFF:
    relayWrite(false);
    break;
  case Mode::ON:
    relayWrite(true);
    break;
  case Mode::AUTO:
    controlLogicTimedAuto(modeJustEntered);
    break;
  }
}

// Traube (nur Konturen)
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
  display.println("Wine Cooling System");
  display.setCursor(0, 12);
  display.println("designed by");
  display.setCursor(0, 22);
  display.println("Simon Riedinger");
  display.setCursor(0, 36);
  display.println("for");
  display.setCursor(0, 46);
  display.println("Hannes Schock");
  drawGrapeOutline(92, 8);
  display.display();
}

void drawMain(Mode m, float sp, float t, bool relay) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("Controller");

  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print("Mode: ");
  switch (m) {
  case Mode::OFF:
    display.print("OFF");
    break;
  case Mode::AUTO:
    display.print("AUTO");
    break;
  case Mode::ON:
    display.print("ON");
    break;
  }

  display.setCursor(0, 32);
  display.print("Set:  ");
  display.printf("%.1f", sp);
  display.print((char)247);
  display.print("C");

  display.setCursor(0, 44);
  display.print("Temp: ");
  if (isnan(t))
    display.print("--.-");
  else
    display.printf("%.1f", t);
  display.print((char)247);
  display.print("C");

  display.setCursor(0, 56);
  display.print("Valve: ");
  display.print(relay ? "OPEN" : "CLOSED");

  display.display();
}

// --------- Buttons für Setpoint (+/-) ---------
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
  bool level = digitalRead(b.pin); // Pullup: LOW = gedrückt
  unsigned long now = millis();
  bool changed = false;

  if (!level && b.lastLevel) {
    b.pressedAt = now;
    b.lastRepeat = now;
    b.handled = false;
  }
  if (!level) {
    if (!b.handled && now - b.pressedAt > 30) {
      setpoint += delta;
      clampSetpoint();
      b.handled = true;
      changed = true;
    }
    if (now - b.pressedAt > 600 && now - b.lastRepeat > 120) {
      setpoint += delta;
      clampSetpoint();
      b.lastRepeat = now;
      changed = true;
    }
  }
  b.lastLevel = level;
  return changed;
}

// --------- Setup / Loop ---------
void setup() {
  Wire.begin(); // I2C (SDA=21, SCL=22)

  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  pinMode(PIN_MODE_A, INPUT_PULLUP);
  pinMode(PIN_MODE_B, INPUT_PULLUP);
  pinMode(PIN_BTN_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
  pinMode(PIN_RELAY, OUTPUT);
  relayWrite(false);

  sensors.begin();
  sensors.getAddress(dsAddr, 0);
  sensors.setResolution(dsAddr, 12);
  sensors.setWaitForConversion(true);

  drawSplash();
  delay(5000);
}

void loop() {
  static unsigned long lastTempPoll = 0;
  static unsigned long lastDraw = 0;
  static Mode lastMode = Mode::AUTO;

  unsigned long now = millis();

  // Buttons (Setpoint)
  bool changed = false;
  changed |= handleButton(btnUp, +SET_STEP);
  changed |= handleButton(btnDown, -SET_STEP);

  // Temperatur 1x/s lesen und kalibrieren (dauerhaft messen)
  if (now - lastTempPoll > 1000) {
    readTemperatureRaw();
    lastTemp = applyCalibration(lastTempRaw);
    lastTempPoll = now;
  }

  // Modus & Regelung
  Mode mode = readModeSwitch();
  bool modeJustEntered = (mode != lastMode);
  controlLogic(mode, modeJustEntered);
  lastMode = mode;

  // Anzeige
  if (changed || now - lastDraw > 200) {
    drawMain(mode, setpoint, lastTemp, relayState);
    lastDraw = now;
  }

  delay(5);
}