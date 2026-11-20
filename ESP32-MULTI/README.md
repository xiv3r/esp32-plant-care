## 1. Hardware You Need

| Component | Connection |
|-----------|-----------|
| ESP32 Dev Module (WROOM-32) | — |
| SSD1306 OLED 128×64 (I2C) | SDA→GPIO21, SCL→GPIO22 |
| BME280 (I2C) | SDA→21, SCL→22, addr 0x76 or 0x77 |
| BH1750 (I2C) | SDA→21, SCL→22, addr 0x23 |
| MQ-2 gas sensor (analog) | AOUT→GPIO32 |
| Soil moisture sensor (analog) | AOUT→GPIO34 |
| Rain sensor (analog) | AOUT→GPIO35 |
| Buzzer | GPIO26 |
| Status LED (anode) | GPIO33 |
| BOOT button | GPIO0 (built-in) |
| 6-Channel relay module | GPIO25, 23, 19, 18, 17, 16 |
| Power | 5V for relays/sensors, 3.3V for I2C devices |

⚠️ **GPIO34/35 are input-only** — correct for analog sensors. Do **not** power 3.3V-only sensors (BME280, BH1750) from 5V.

---

## 2. Software Prerequisites (Arduino IDE 2.x)

Install these libraries via **Library Manager**:

1. **ESPAsyncWebServer** (by ESP32Async / me-no-dev)
2. **AsyncTCP** (dependency for the above)
3. **Adafruit GFX Library**
4. **Adafruit SSD1306**
5. **Adafruit BME280 Library** (also installs Adafruit Unified Sensor)
6. **BH1750** (by Christopher Laws)
7. **ArduinoJson** (v7.x)

Board manager: **esp32 by Espressif** (v2.x or v3.x both supported).

---

## 3. Upload Steps

1. Open `ESP32_Multi_Smart_Garden.ino` in Arduino IDE.
2. **Tools → Board → ESP32 Arduino → ESP32 Dev Module**
3. **Tools → Port** → select your COM port.
4. **Tools → Partition Scheme → Default 4MB with spiffs** (or larger, since the web UI is embedded).
5. Click **Upload**.
6. If upload fails, hold **BOOT (GPIO0)** while it says "Connecting…", then release.

---

## 4. First Boot & WiFi Connection

1. After flashing, the ESP32 creates a WiFi access point:
   - **SSID:** `ESP32_Multi_Smart_Garden`
   - **Password:** `12345678`
2. Connect your phone/PC to that WiFi.
3. Open a browser → **http://192.168.4.1**
4. The dashboard loads. A captive portal may pop up automatically.

> If the portal doesn't appear, manually navigate to `192.168.4.1`.

---

## 5. Using the Web Dashboard

The single-page app has these tabs (left sidebar):

| Tab | Purpose |
|-----|---------|
| **Dashboard** | Live sensor readings, relay states, alert banner |
| **BME280 / BH1750 / MQ-2 / Soil / Rain** | Per-sensor settings: enable, interval, thresholds, hysteresis, relay/buzzer/LED mapping |
| **OLED** | Enable, contrast, flip, page cycling, timeout |
| **Relay Matrix** | Per-relay mode (auto/always/off), active-low, source mapping |
| **Buzzer / LED** | Pattern, volume, blink mode |
| **System Status** | Heap, uptime, config integrity, factory reset |
| **WiFi AP Settings** | Change SSID/password/channel |

**Save** buttons only write to NVS when values actually change (A/B slot with CRC + sequence number — safe against power loss).

---

## 6. Factory Reset (No PC Needed)

- **Hold the BOOT (GPIO0) button for 5 seconds.**
- OLED shows a progress bar ("FACTORY RESET").
- On release after 5s: NVS is wiped, buzzer sounds, device reboots to defaults.

---

## 7. Configuration Persistence

- Settings are stored in **NVS** in two slots (`cfg_a`, `cfg_b`) with:
  - Magic `0x31455247`, version, CRC-32, sequence counter.
  - On boot, the newest valid slot wins; corrupt slots fall back to the other.
- **No config is lost** on power cuts mid-write.

---

## 8. Default Thresholds (out of the box)

| Sensor | Default trigger |
|--------|----------------|
| BME280 | Temp > 35 °C, hysteresis 1 °C |
| BH1750 | Lux < 400, hysteresis 50 |
| MQ-2 | Gas > 40 %, hysteresis 5 % |
| Soil | Moisture < 30 %, hysteresis 5 % |
| Rain | Precip > 60 %, hysteresis 8 % |

Relay 0 (GPIO25) is the **unified relay** — fires when any sensor with "Unified" checked triggers. Relays 1–5 are per-sensor dedicated relays.

---

## 9. Calibration Tips

- **Soil sensor:** In the Soil tab, set `Dry Raw` (e.g., 3100 in air) and `Wet Raw` (e.g., 1400 in water). The firmware converts raw ADC → % moisture.
- **Rain sensor:** Same idea — `Dry Raw` 3500, `Wet Raw` 900.
- **BME280 offsets:** Use the temperature/humidity/pressure offset fields to calibrate against a known reference.

---

## 10. Troubleshooting

| Problem | Fix |
|---------|-----|
| No WiFi AP | Check serial monitor @ 115200 baud; ensure `apSsid`/`apPass` defaults weren't corrupted |
| OLED blank | Verify I2C address (0x3C typical); check SDA/SCL wiring |
| BME280 not detected | Try addr 0x77; check 4.7kΩ pull-ups on SDA/SCL |
| Relay always on/off | Toggle **Active Low** in Relay Matrix (most modules are active-low) |
| Can't upload | Hold BOOT during "Connecting…"; try a different USB cable |
| Web UI won't save | Check serial for CRC/NVS errors; do a factory reset |
