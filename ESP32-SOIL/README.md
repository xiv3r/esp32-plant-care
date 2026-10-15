# 🔌 Complete Wiring Connection Guide

Here's the **complete wiring** for the ESP32 Soil Moisture Monitoring + Relay Control + Web Dashboard system.

<img src="https://github.com/xiv3r/esp32-plant-care/blob/main/ESP32-SOIL/esp32-soil-sensor.png">
---

## 📦 REQUIRED COMPONENTS

| # | Component | Quantity |
|---|---|---|
| 1 | ESP32 DevKit (30-pin or 38-pin) | 1 |
| 2 | Soil Moisture Sensor (capacitive or resistive) | 1 |
| 3 | Relay Module (5V, 1-channel, opto-isolated recommended) | 1 |
| 4 | Water pump OR solenoid valve (load for relay) | 1 |
| 5 | External 5V power supply (for relay + pump) | 1 |
| 6 | Jumper wires (female-female, male-female) | ~10 |
| 7 | USB cable (for powering ESP32 + programming) | 1 |
| 8 | Optional: DC barrel jack adapter for pump | 1 |

---

---

## 🔧 DETAILED WIRING

### 1️⃣ Soil Moisture Sensor → ESP32

| Sensor Pin | Wire Color (typical) | ESP32 Pin | Notes |
|---|---|---|---|
| **VCC / +** | Red | **3V3** | Do NOT use 5V — ESP32 ADC max is 3.3V |
| **GND / −** | Black | **GND** | Common ground |
| **AOUT / SIG** | Yellow/Blue | **GPIO34** | ADC1_CH6 (WiFi-safe) |

**⚠️ Important:** GPIO34 is **input-only** and belongs to **ADC1**, which is safe to use when WiFi is active. Do NOT move to GPIO32/33/35 — those are also OK, but avoid GPIO0/2/4/12–15/25–27 (ADC2) which conflict with WiFi.

**Capacitive sensor** (recommended):
- Output 0–3.0V — perfectly safe
- Doesn't corrode (no exposed electrodes)
- More expensive (~$3–5)

**Resistive sensor** (cheap):
- Output 0–3.3V — safe but borderline
- Corrodes over time (electrolysis)
- Cheap (~$1)

---

### 2️⃣ Relay Module → ESP32

| Relay Pin | Wire Color | ESP32 Pin | Notes |
|---|---|---|---|
| **VCC** | Red | **VIN (5V)** | Most relay modules need 5V. Do NOT use 3V3 |
| **GND** | Black | **GND** | Must be common with ESP32 GND |
| **IN / SIG** | Yellow | **GPIO25** | Digital output |

**⚠️ Important notes:**

- **5V relay modules** must be powered from **VIN (5V)**, NOT 3V3 — 3.3V may not reliably trigger them.
- **3.3V-compatible relay modules** exist — those can use 3V3.
- **Opto-isolated modules** (recommended) protect the ESP32 from back-EMF.
- **Active-LOW modules** are the most common. The code already accounts for this:
  ```cpp
  #define RELAY_ACTIVE_LOW true
  ```
  If your relay behaves inverted (ON when should be OFF), change this to `false`.

**Relay module pin labels vary:**
- `VCC` / `+` / `5V`
- `GND` / `−`
- `IN` / `SIG` / `S` / `CH1`

---

### 3️⃣ Relay Output Side → Pump/Valve

**Wire description:**
- **COM** (Common) → to **Pump negative (-)**
- **NO** (Normally Open) → to **Power supply positive (+)**
- **NC** (Normally Closed) → unused
- **Pump positive (+)** → to **Power supply positive (+)** directly

**Result:** When relay closes (ESP32 drives GPIO25 LOW for active-low), current flows: Power Supply → NO → COM → Pump → Power Supply GND. Pump runs.

**⚠️ Do NOT power the pump from the ESP32's VIN.** Use a **separate 5V/12V supply** rated for the pump's current.

**⚠️ If using mains AC (110/220V pump):** Use a relay rated for AC, and **never** touch the relay's AC side while powered. Consider using a certified AC relay module with proper terminal blocks and fusing.

---

---

**Recommended power setup:**
- **ESP32** powered via USB (5V, 500mA)
- **Relay + Pump** powered via **external 5V or 12V** supply (rated ≥ pump current + 100mA)
- **Common GND** between ESP32 and external supply

**If you don't have an external supply:**
- ESP32's VIN pin can supply **~500mA max** (limited by USB)
- A small 5V pump (≤ 200mA) might work
- A 12V pump or larger — **must** use separate supply

---

## 📋 COMPLETE WIRING TABLE

| Component | Pin | Connects To | Wire |
|---|---|---|---|
| Soil Sensor | VCC | ESP32 3V3 | Red |
| Soil Sensor | GND | ESP32 GND | Black |
| Soil Sensor | AOUT | ESP32 GPIO34 | Yellow |
| Relay | VCC | ESP32 VIN (5V) | Red |
| Relay | GND | ESP32 GND | Black |
| Relay | IN | ESP32 GPIO25 | Yellow |
| Relay | COM | Pump (−) | Any |
| Relay | NO | PSU (+) | Any |
| Pump | (+) | PSU (+) | Any |
| Pump | (−) | Relay COM | Any |
| PSU | (−) | ESP32 GND (common) | Black |

---

---

## ⚙️ CONFIGURATION AFTER WIRING

### 1. Set relay polarity (if needed)

If the relay turns ON when the dashboard says OFF (and vice versa), edit the top of the `.ino`:

```cpp
#define RELAY_ACTIVE_LOW    false   // ← change from true to false
```
---

### 3. Calibrate the sensor

After first boot, open the dashboard and:
1. Hold sensor **in dry air** → note the **Raw ADC** value → enter as **Dry ADC**
2. Dip sensor **in a glass of water** → note the **Raw ADC** value → enter as **Wet ADC**
3. Set **Threshold** (30% is a good start)
4. Set **Read Interval** (5s is a good start)
5. Click **Save**

---

## 🧪 TESTING CHECKLIST

| # | Test | Expected Result |
|---|---|---|
| 1 | Power on ESP32 | Serial at 115200 (if enabled) shows IP or AP start |
| 2 | Open `http://<IP>/` in browser | Dashboard loads |
| 3 | Sensor in air | Moisture reads ~0–10% |
| 4 | Sensor in water | Moisture reads ~90–100% |
| 5 | Click **ON** (manual) | Relay clicks, pump starts |
| 6 | Click **OFF** | Relay clicks, pump stops |
| 7 | Toggle **AUTO** mode | Relay follows threshold |
| 8 | Save settings | Values persist after power cycle |
| 9 | Open `/wifi` page | WiFi settings load |
| 10 | Scan networks | List of nearby SSIDs appears |

---

## ⚠️ SAFETY NOTES

1. **Never** touch the relay's high-voltage side while powered (if using AC).
2. **Always** use a **common ground** between ESP32 and any external supply.
3. **Do not** power a pump directly from ESP32's 3V3 or VIN — use external supply.
4. **Add a flyback diode** across DC pump terminals (1N4007 or similar) to suppress back-EMF spikes.
5. **Add a 100µF capacitor** across relay VCC/GND if relay causes ESP32 brownouts.
6. **Use a fuse** on the pump's positive supply for overcurrent protection.
7. **Keep** the soil sensor's exposed wires away from mains wiring.
8. **Waterproof** the sensor's connection to the ESP32 if used outdoors.

---
