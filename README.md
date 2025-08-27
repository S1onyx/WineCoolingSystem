# 🍷 Wine Cooling System

Ein selbstgebautes Kühlsystem für Wein, gesteuert über einen ESP32.  
Entwickelt von **Simon Riedinger** für **Hannes Schock**.

---

## ✨ Features
- Anzeige von **Temperatur**, **Sollwert**, **Modus** und **Ventilstatus** auf einem OLED-Display.
- **3 Modi** über 3-Wege-Schalter:
  - **OFF**: Ventil geschlossen
  - **ON**: Ventil offen
  - **AUTO**: Ventil abhängig von Temperatur und Sollwert
- **Sollwert einstellbar** mit Tastern in 0,1 °C Schritten.
- **Mehrpunkt-Kalibrierung** im Code möglich (Offset oder stückweise linear).
- **Hysterese** verhindert Flattern um den Sollwert.

---

## 🔧 Hardware
- ESP32 DevKit (ELEGOO, USB-C, CP2102)
- DS18B20 Temperatursensor (wasserdicht, Edelstahl)
- OLED-Display SSD1306 (0,96", 128×64 px, I²C, Adresse 0x3C)
- 1-Kanal-Relaismodul (ESP32 → Ventil)
- 3-Wege-Schalter
- 2 Taster
- 230V Magnetventil

📌 **Widerstand**: 4,7 kΩ Pull-Up zwischen DATA des DS18B20 und 3,3 V.

---

## 📌 Pinbelegung (ESP32)

| Komponente         | Pin am ESP32 | Bemerkung                  |
|--------------------|--------------|----------------------------|
| DS18B20 DATA       | GPIO4        | Pull-Up 4,7 kΩ nach 3,3 V |
| OLED SDA           | GPIO21       | I²C SDA                   |
| OLED SCL           | GPIO22       | I²C SCL                   |
| Relais             | GPIO23       | active-LOW (Standard)     |
| Schalter OFF-Kontakt | GPIO32     | gegen GND                 |
| Schalter ON-Kontakt  | GPIO33     | gegen GND                 |
| Taster UP          | GPIO25       | gegen GND                 |
| Taster DOWN        | GPIO26       | gegen GND                 |

---

## 💻 Software
- Framework: Arduino (ESP32)
- IDE: PlatformIO + VS Code
- Libraries:
  - [Adafruit GFX Library](https://github.com/adafruit/Adafruit-GFX-Library)
  - [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306)
  - [DallasTemperature](https://github.com/milesburton/Arduino-Temperature-Control-Library)
  - [OneWire](https://github.com/PaulStoffregen/OneWire)

---

## ⚙️ Kalibrierung
Die Kalibrierung erfolgt **direkt im Code**:

```cpp
// Beispiel für 2-Punkt-Kalibrierung
static const int   CAL_COUNT = 2;
static const float CAL_RAW[] = {  0.0f, 30.0f };   // Rohwerte
static const float CAL_REF[] = {  0.0f, 30.2f };   // Referenzwerte
```

- `CAL_RAW` → gemessene Werte des DS18B20  
- `CAL_REF` → echte Referenzwerte (z. B. Thermometer)  
- `CAL_COUNT` → Anzahl der verwendeten Punkte (1–4)

📌 Wenn keine Kalibrierung gewünscht ist → `CAL_COUNT = 0`.

---

## 🚀 Getting Started
1. Projekt in VS Code mit PlatformIO öffnen.  
2. Bibliotheken installieren (werden automatisch durch `platformio.ini` nachgeladen).  
3. `src/main.cpp` anpassen (Kalibrierpunkte setzen).  
4. ESP32 per USB-C verbinden.  
5. Upload starten:  
   ```bash
   pio run --target upload
   ```
6. Fertig!  

---

## 📸 OLED-UI
- **Startscreen**: "Wine Cooling System – designed by Simon Riedinger for Hannes Schock" mit Trauben-Symbol.
- **Hauptscreen**:
  - Zeile 1: "Controller"
  - Zeile 2: aktueller Modus (OFF/AUTO/ON)
  - Zeile 3: Sollwert
  - Zeile 4: Temperatur
  - Zeile 5: Ventilstatus

---

## ❤️ Credits
Mit ❤️ entwickelt von **Simon Riedinger** für **Hannes Schock**.
