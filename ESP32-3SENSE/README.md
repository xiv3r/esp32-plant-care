| Qty | Component | Notes |
|-----|-----------|-------|
| 1 | ESP32 DevKit (30/38-pin) | Any ESP32-WROOM-32 board |
| 1 | Capacitive Soil Moisture Sensor v2.0 | Analog output |
| 1 | SHT31 temperature/humidity sensor | I²C, address 0x44 or 0x45 |
| 1 | BH1750 light sensor | I²C, address 0x23 or 0x5C |
| 1 | Relay module (1-channel) | 3.3 V logic-compatible preferred |
| 1 | Push button | For factory reset |
| — | Jumper wires, breadboard/PCB | — |

---

## ESP32 Pin Map

| GPIO | Function | Firmware define |
|------|----------|-----------------|
| GPIO 34 | Soil moisture analog input | `SOIL_PIN` |
| GPIO 21 | I²C SDA | `I2C_SDA` |
| GPIO 22 | I²C SCL | `I2C_SCL` |
| GPIO 25 | Relay control output | `RELAY_PIN` |
| GPIO 0 | Boot / factory-reset button | `BOOT_BTN_PIN` |

---

## 1. Capacitive Soil Moisture Sensor v2.0

| Sensor pin | ESP32 pin | Notes |
|------------|-----------|-------|
| VCC | 3V3 | **Do not use 5 V** — sensor output would exceed ADC range |
| GND | GND | Common ground |
| AOUT | GPIO 34 | ADC1 channel, input-only pin |

> **Important:** GPIO 34 is input-only and belongs to ADC1. ADC2 pins cannot be used while WiFi is active, so GPIO 34 is the correct choice. The firmware sets `ADC_11db` attenuation for ~0–3.3 V range.

---

## 2. SHT31 Temperature / Humidity Sensor (I²C)

| SHT31 pin | ESP32 pin | Notes |
|-----------|-----------|-------|
| VIN / VCC | 3V3 | 3.3 V |
| GND | GND | Common ground |
| SDA | GPIO 21 | Shared I²C bus |
| SCL | GPIO 22 | Shared I²C bus |
| ADDR | GND (or floating) | GND → 0x44 (default), VCC → 0x45 |

The firmware tries **0x44 first, then 0x45**, so either address works.

---

## 3. BH1750 Light Sensor (I²C)

| BH1750 pin | ESP32 pin | Notes |
|------------|-----------|-------|
| VCC | 3V3 | 3.3 V |
| GND | GND | Common ground |
| SDA | GPIO 21 | Shared with SHT31 |
| SCL | GPIO 22 | Shared with SHT31 |
| ADDR | GND (or floating) | GND → 0x23 (default), VCC → 0x5C |

The firmware tries **0x23 first, then 0x5C**, so either address works.

### I²C Bus Summary

```
ESP32 GPIO 21 (SDA) ──┬── SHT31 SDA
                      └── BH1750 SDA

ESP32 GPIO 22 (SCL) ──┬── SHT31 SCL
                      └── BH1750 SCL

ESP32 3V3 ────────────┬── SHT31 VCC
                      └── BH1750 VCC

ESP32 GND ────────────┴── SHT31 GND, BH1750 GND, Soil GND, Relay GND
```

> Both sensors share the same bus at 100 kHz (`I2C_FREQ = 100000`). Pull-up resistors are usually already present on the breakout boards. If not, add 4.7 kΩ from SDA→3V3 and SCL→3V3.

---

## 4. Relay Module

| Relay module pin | ESP32 pin | Notes |
|------------------|-----------|-------|
| VCC | 5V (VIN) or 3V3 | Depends on module — see below |
| GND | GND | Common ground |
| IN | GPIO 25 | Control signal |

### Relay logic

The firmware uses **active-LOW logic by default**:

```cpp
void setRelayOutput(bool on) {
  bool highLevel;
  if (cfg.relayInvert) {
    highLevel = on;        // invert OFF → active-HIGH module
  } else {
    highLevel = !on;       // default → active-LOW module
  }
  digitalWrite(RELAY_PIN, highLevel ? HIGH : LOW);
  r.relayState = on;
}
```

| Module type | `relayInvert` setting | GPIO 25 = LOW | GPIO 25 = HIGH |
|-------------|----------------------|---------------|----------------|
| Active-LOW (most common) | `false` (default) | Relay **ON** | Relay **OFF** |
| Active-HIGH | `true` | Relay **OFF** | Relay **ON** |

> **Boot safety:** In `setup()`, the pin is first driven to the "off" level (`cfg.relayInvert ? LOW : HIGH`) before `setRelayOutput()` is called, preventing a momentary relay click on power-up.

### Powering the relay

- **3.3 V-compatible relay modules** (opto-isolated, with own transistor): power VCC from **3V3**, signal from GPIO 25.
- **5 V relay modules**: power VCC from **5 V / VIN**, but **check that the IN pin accepts 3.3 V logic**. Many 5 V modules have a threshold near 3 V and work; if not, use a level shifter or a transistor driver.
- If the relay coil draws significant current, power it from a **separate 5 V supply** with a common ground to the ESP32.

---

## 5. Boot / Factory-Reset Button

| Button | ESP32 pin | Notes |
|--------|-----------|-------|
| One side | GPIO 0 | Internal pull-up enabled (`INPUT_PULLUP`) |
| Other side | GND | |

- The **on-board BOOT button** already connects GPIO 0 to GND — no external button required.
- Hold for **5 seconds** (`BOOT_BTN_HOLD_MS`) to trigger a factory reset.
- The firmware arms the button only after it has seen a released state, so it won't trigger if held during power-up.

---
