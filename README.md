# 🍷 Wine Cooling System

Ein selbstgebautes Kühlsystem für Wein, gesteuert über einen **ESP32**.  
Entwickelt von **Simon Riedinger** für **Hannes Schock**.

---

## ✨ Features
- OLED-Display mit Anzeige von:
  - **Temperatur**
  - **Sollwert**
  - **Modus**
  - **Ventilstatus**
- **3 Betriebsmodi** per 3-Wege-Schalter:
  - **OFF** → Ventil dauerhaft geschlossen
  - **ON** → Ventil dauerhaft geöffnet
  - **AUTO** → Ventil temperaturgesteuert mit Pulsbetrieb
    - öffnet für **10 s**
    - danach mindestens **120 s geschlossen** (Cooldown)
    - erneutes Öffnen nur bei **Temperatur unter Sollwert – Hysterese berücksichtigt**
- **Sollwert** einstellbar über Taster in 0,1 °C-Schritten
- Speicherung des Sollwerts im **NVS** → bleibt auch nach Stromausfall erhalten
- **Automatische Display-Abschaltung** nach 2 min Inaktivität
- **Mehrpunkt-Kalibrierung** im Code möglich (Offset oder stückweise linear)
- **Hysterese** verhindert ständiges Flattern um den Sollwert

---

## 🔧 Hardware

- ESP32 DevKit (ELEGOO, USB-C, CP2102)
- DS18B20 Temperatursensor (wasserdicht, Edelstahl)
- OLED-Display SSD1306 (0,96", 128×64 px, I²C, Adresse 0x3C)
- 1-Kanal-Relaismodul (ESP32 → Magnetventil)
- 3-Wege-Schalter
- 2 Taster
- 230 V Magnetventil

📌 **Hinweis:** DS18B20 benötigt einen **4,7 kΩ Pull-Up-Widerstand** zwischen DATA und 3,3 V.

---

## 📌 Pinbelegung (ESP32)

| Komponente            | Pin am ESP32 | Bemerkung                  |
|-----------------------|--------------|----------------------------|
| DS18B20 DATA          | GPIO4        | Pull-Up 4,7 kΩ nach 3,3 V |
| OLED SDA              | GPIO21       | I²C SDA                   |
| OLED SCL              | GPIO22       | I²C SCL                   |
| Relais                | GPIO23       | **active-HIGH** (im Code konfigurierbar) |
| Schalter OFF-Kontakt  | GPIO32       | gegen GND                 |
| Schalter ON-Kontakt   | GPIO33       | gegen GND                 |
| Taster UP             | GPIO25       | gegen GND                 |
| Taster DOWN           | GPIO26       | gegen GND                 |

---

## 💻 Software

- Framework: **Arduino (ESP32)**
- IDE: **PlatformIO** + VS Code
- Libraries:
  - [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library)
  - [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306)
  - [DallasTemperature](https://github.com/milesburton/Arduino-Temperature-Control-Library)
  - [OneWire](https://github.com/PaulStoffregen/OneWire)

---

## ⚙️ Kalibrierung

Die Kalibrierung erfolgt **im Code** über Mehrpunkt-Lookup:  

```cpp
// Beispiel: 2-Punkt-Kalibrierung
static const int   CAL_COUNT = 2;
static const float CAL_RAW[] = {  0.0f, 30.0f };   // Sensor-Rohwerte
static const float CAL_REF[] = {  0.0f, 30.2f };   // Referenzwerte
```

- `CAL_RAW[]` → Messwerte des DS18B20  
- `CAL_REF[]` → echte Referenzwerte (z. B. Labor-Thermometer)  
- `CAL_COUNT` → Anzahl der Punkte (1–4 empfohlen)

➡️ Keine Kalibrierung? → `CAL_COUNT = 0`.

---

## 🚀 Getting Started

1. Projekt in VS Code mit PlatformIO öffnen  
2. Bibliotheken werden automatisch installiert (`platformio.ini`)  
3. Falls nötig: Kalibrierwerte in `src/main.cpp` eintragen  
4. ESP32 per USB-C verbinden  
5. Upload starten:  

   ```bash
   pio run --target upload
   ```

6. Fertig! 🎉  

---

## 📸 OLED-UI

- **Splashscreen**  
  „Wine Cooling System – designed by Simon Riedinger for Hannes Schock“  
  mit Trauben-Symbol

- **Hauptanzeige**  
  - Titel: *Controller*  
  - Modus: OFF / AUTO / ON  
  - Sollwert (°C)  
  - Ist-Temperatur (°C)  
  - Ventilstatus (OPEN / CLOSED)

---

## ❤️ Credits

Mit ❤️ entwickelt von **Simon Riedinger** für **Hannes Schock** 🍇  
