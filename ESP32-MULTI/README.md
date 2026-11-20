## 1. Hardware Wiring Checklist
| Component | ESP32 Pin | Notes |
|-----------|-----------|-------|
| OLED SSD1306 | SDA 21, SCL 22 | 3.3V, addr 0x3C |
| BME280 | SDA 21, SCL 22 | 3.3V, addr 0x76/0x77 (auto-probe) |
| BH1750 | SDA 21, SCL 22 | 3.3V, addr 0x23 |
| MQ-2 AOUT | GPIO 32 | ADC1, 12-bit |
| Soil AOUT | GPIO 34 | Input-only |
| Rain AOUT | GPIO 35 | Input-only |
| Buzzer | GPIO 26 | PWM (LEDC) |
| Status LED | GPIO 33 | PWM anode |
| BOOT button | GPIO 0 | Factory reset (5 s hold) |
| Unified relay | GPIO 25 | Active-low default |
| BME relay | GPIO 15 | Active-low default |
| BH relay | GPIO 2 | Active-low default (boot strap pin) |
| MQ relay | GPIO 4 | Active-low default |
| Soil relay | GPIO 5 | Active-low default (boot strap pin) |
| Rain relay | GPIO 18 | Active-low default |

> ⚠️ **Critical:** GPIO 2 and GPIO 5 are ESP32 boot-strap pins. If relays pull them LOW at boot, the board may fail to enter flash mode. Keep relays de-energized (HIGH) during boot — the code writes inactive levels first in `relayWritePinsInactiveFirst()`, so this is handled — but verify with a multimeter that inactive = HIGH on those pins.

## 2. Arduino IDE Setup
1. **Board:** ESP32 Dev Module (WROOM-32)
2. **Core:** Arduino-ESP32 2.x **or** 3.x (code has `#if` guards for both)
3. **Libraries to install:**
   - `ESPAsyncWebServer` + `AsyncTCP` (by me-no-dev / ESP32Async fork)
   - `Adafruit GFX Library`
   - `Adafruit SSD1306`
   - `Adafruit BME280 Library`
   - `BH1750` (by Christopher Laws)
   - `ArduinoJson` **7.x** (critical — 6.x will not compile)
4. **Partition scheme:** any with ≥1.5 MB APP (default 4MB with SPIFFS is fine — no filesystem used)
5. **Upload speed:** 921600 or 115200

## 3. First Boot
1. Power on → OLED shows "SMART GARDEN" overview page.
2. Connect to WiFi AP: **`ESP32_Multi_Smart_Garden`** / password **`12345678`**
3. Open browser → **`http://192.168.4.1`**
4. Dashboard should populate sensor cards within ~2–5 s.

## 4. Pre-Flight Verification (quadruple-check)
Run these checks **before** relying on automation:

### A. Compile check
- [ ] No errors with ArduinoJson 7.x (if you see `DynamicJsonDocument` deprecation warnings, that's expected/normal — code targets 7.x API)
- [ ] No errors with core 3.x (`ledcAttach` path) **and** core 2.x (`ledcSetup` path)
- [ ] Sketch size fits partition (should be well under 1.5 MB)

### B. Sensor presence
- [ ] Dashboard badges show **OK** (teal) for BME280, BH1750, MQ-2, Soil, Rain
- [ ] If BME280 badge shows **ERR** → check I2C address (try 0x77) or wiring
- [ ] If BH1750 shows **ERR** → check ADDR pin (0x23 vs 0x5C)

### C. Relay safety
- [ ] Before connecting mains loads, test each relay in **Manual ON/OFF** mode from Relay Matrix page
- [ ] Confirm inactive state = relay OFF (measure continuity)
- [ ] Confirm GPIO 2 / GPIO 5 relays don't prevent boot (power-cycle test 3×)

### D. Threshold / trigger logic
- [ ] Open each sensor page → set a threshold you can cross manually (e.g., BME temp threshold 25 °C, breathe on sensor)
- [ ] Confirm badge flips to **TRIG** (amber, blinking) and OLED alerts page shows the condition
- [ ] Confirm buzzer sounds (if enabled) and stops after `BUZZ_MAX_ALERT_S`
- [ ] Confirm hysteresis prevents chatter (value must recover past `threshold ± hysteresis` to clear)

### E. Config persistence
- [ ] Change a setting → **Save changes**
- [ ] Power-cycle ESP32
- [ ] Confirm setting persisted (check `Config integrity` shows **OK** and slot A/B, not **DEFAULTS**)
- [ ] Save again → confirm slot alternates A↔B (A/B wear-leveling working)

### F. Factory reset
- [ ] Hold **BOOT (GPIO0)** for 5 s → OLED shows progress bar
- [ ] Device beeps and restarts
- [ ] All settings return to defaults, AP SSID back to `ESP32_Multi_Smart_Garden`

### G. Stability soak (30 min minimum)
- [ ] `Free heap` stays > 100 KB (no leak)
- [ ] `Heap fragmentation` stays < 30 %
- [ ] `Uptime` counter increments correctly
- [ ] No watchdog resets in serial monitor (115200 baud)

## 5. Known Gotchas
- **ArduinoJson 7.x is mandatory** — 6.x uses removed APIs.
- **GPIO 2 & 5 boot straps** — relays on these pins must be inactive at boot (code handles it, but hardware pull-ups/downs matter).
- **GPIO 34/35 are input-only** — no internal pull-ups; soil/rain boards must provide their own.
- **OLED `dim(true)`** is used for sleep (not `displayOff`) — this keeps I2C traffic low; if your SSD1306 clone doesn't support dim, swap to `ssd1306_command(SSD1306_DISPLAYOFF)`.
- **Async web server** requires `AsyncTCP` matching your core version — mismatches cause boot loops.
- **`strlcpy`** is used — available on ESP32 core, fine.
- **Config struct is `packed`** — do not add fields without bumping `CFG_VERSION`, or old NVS blobs will fail CRC and fall back to defaults (safe, but settings lost).

## 6. Serial Monitor Quick Reference
Open at **115200 baud**. Expected boot sequence:
```
[boot] FW 2.4.1 · boots=N
[cfg] slot A CRC OK seq=N
[i2c] BME280 @0x76 · BH1750 @0x23 · OLED OK
[wifi] AP ESP32_Multi_Smart_Garden · 192.168.4.1
[web] server started
```
If any line is missing → that subsystem failed; check wiring/library.

## 7. If Something Is Wrong
| Symptom | Likely cause |
|---------|--------------|
| Boot loop | AsyncTCP/core version mismatch, or GPIO2/5 relay pulling low |
| Web UI blank / 404 | `INDEX_HTML` truncated — re-upload full sketch |
| Sensors all ERR | I2C bus not initialized — check SDA/SCL swap, 3.3V, pull-ups |
| Relay stuck ON | `relayActiveLow[]` set wrong for your module (some are active-high) |
| Buzzer silent | `buzzVolume` too low, or LEDC channel conflict on core 2.x |
| Config resets every boot | NVS partition full or CRC failing — check serial for `[cfg] DEFAULTS` |
| OLED blank | Address 0x3C vs 0x3D — code hardcodes `Adafruit_SSD1306` default 0x3C |

**Bottom line:** Wire per the pin map, install ArduinoJson **7.x** + Async web server stack, flash, connect to the AP, and verify the four checklist groups (B: sensors, C: relays, D: triggers, E: persistence) before trusting it with live loads.
