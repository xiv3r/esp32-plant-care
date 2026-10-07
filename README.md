## Components Required

| Qty | Component |
|-----|-----------|
| 1 | ESP32 DevKit (e.g. ESP32-WROOM-32) |
| 1 | DHT22 temperature/humidity sensor |
| 1 | Capacitive or resistive soil moisture sensor (analog) |
| 1 | 5V relay module (opto-isolated preferred) |
| 1 | Small water pump (5V/12V depending on relay) |
| 1 | 10kΩ resistor (DHT22 pull-up) |
| 1 | External 5V/12V power supply for pump |
| — | Jumper wires, breadboard/PCB |

---

## ESP32 Pinout Used by Firmware

| GPIO | Function | Direction |
|------|----------|-----------|
| GPIO 34 | Soil moisture analog input | Input (ADC1_CH6) |
| GPIO 4 | DHT22 data | Input/Output |
| GPIO 26 | Relay control | Output |
| GND | Common ground | — |
| 3.3V | DHT22 power | — |
| 5V / VIN | Relay module VCC (if 5V coil) | — |

---

## Detailed Wiring

### 1. DHT22 → ESP32

```
DHT22 Pin 1 (VCC)  ──────► ESP32 3.3V
DHT22 Pin 2 (DATA) ──────► ESP32 GPIO 4
                          │
                          └── 10kΩ resistor ──► ESP32 3.3V  (pull-up)
DHT22 Pin 3 (NC)   ──────  (not connected)
DHT22 Pin 4 (GND)  ──────► ESP32 GND
```

> **Note:** DHT22 can run on 3.3V or 5V. Using 3.3V keeps the data line logic-level safe for the ESP32 without a level shifter. The 10kΩ pull-up between DATA and VCC is **required**.

---

### 2. Soil Moisture Sensor → ESP32

**Capacitive type (recommended):**
```
Sensor VCC  ──────► ESP32 3.3V
Sensor GND  ──────► ESP32 GND
Sensor AOUT ──────► ESP32 GPIO 34
```

**Resistive type (2-pin probe):**
```
One probe leg ─────► ESP32 3.3V
Other probe leg ───► ESP32 GPIO 34
                    │
                    └── 10kΩ resistor ──► ESP32 GND  (voltage divider)
```

> **Important:** GPIO 34 is **input-only** and belongs to ADC1. Do **not** use ADC2 pins (GPIO 0, 2, 4, 12–15, 25–27) for analog reads while WiFi is active — ADC2 is shared with the WiFi radio and will give garbage readings.

> The firmware reads raw 12-bit values (0–4095) and treats **higher = drier**. If your sensor reads inverted (higher = wetter), swap the logic or use the calibration thresholds accordingly.

---

### 3. Relay Module → ESP32

```
Relay VCC   ──────► ESP32 5V / VIN   (or external 5V — see note)
Relay GND   ──────► ESP32 GND
Relay IN    ──────► ESP32 GPIO 26
```

> **Note:** Most 5V relay modules draw more current than the ESP32's onboard regulator can comfortably supply. If you see brownouts/reboots when the relay switches, power the relay from a **separate 5V supply** and connect only GND + IN to the ESP32. The firmware uses `RELAY_ON = HIGH` (active-high), which matches most opto-isolated relay boards.

---

### 4. Pump → Relay → Power

```
External 5V/12V PSU (+) ──────► Relay COM
Relay NO (Normally Open) ─────► Pump (+)
Pump (–) ─────────────────────► External PSU (–)
```

> The relay acts as a switch on the **positive** pump lead. Use **NO** (Normally Open) so the pump is OFF by default when the ESP32 is off or resetting. Match the PSU voltage to your pump rating.

---

## Full Wiring Diagram (ASCII)

```
                    ┌─────────────────────┐
                    │      ESP32          │
                    │                     │
   DHT22            │  3.3V ●─────────────┼──── DHT22 VCC
   ┌─────┐          │  GND  ●─────────────┼──── DHT22 GND
   │     │          │  GPIO4●─────────────┼──── DHT22 DATA
   │     │          │                     │        │
   └─────┘          │                     │       10kΩ
      │             │                     │        │
      └─────────────┼─────────────────────┼────────┘
                    │                     │      3.3V
   Soil Sensor      │  3.3V ●─────────────┼──── Soil VCC
   ┌─────┐          │  GND  ●─────────────┼──── Soil GND
   │     │          │  GPIO34●────────────┼──── Soil AOUT
   └─────┘          │                     │
                    │                     │
   Relay Module     │  5V/VIN●────────────┼──── Relay VCC
   ┌─────┐          │  GND  ●─────────────┼──── Relay GND
   │     │          │  GPIO26●────────────┼──── Relay IN
   └─────┘          │                     │
      │             └─────────────────────┘
      │
   ┌──┴──┐
   │ COM │◄──── PSU (+)
   │ NO  │─────► Pump (+)
   │ NC  │      Pump (–) ──► PSU (–)
   └─────┘
```

---

## Power Summary

| Rail | Source | Feeds |
|------|--------|-------|
| 3.3V | ESP32 onboard regulator | DHT22, soil sensor |
| 5V / VIN | USB or external 5V | Relay coil (if 5V) |
| External PSU | Separate 5V/12V | Pump only |

**Common GND** must tie together: ESP32 GND, DHT22 GND, soil sensor GND, relay GND, and external PSU GND.

---

## Safety Notes

1. **Never** drive the pump directly from an ESP32 GPIO — it will fry the pin (max ~12mA safe, ~40mA absolute).
2. Use a **flyback diode** (1N4007) across the pump terminals if it's a DC motor, to suppress inductive kickback.
3. Keep the pump's high-current wiring physically separated from the sensor wiring to avoid ADC noise.
4. If using a 12V pump, the relay coil should still be 5V (or use a 12V-coil relay with a transistor driver). Do not feed 12V into the ESP32.
5. The soil sensor's analog output must never exceed 3.3V. If your sensor outputs 0–5V, add a voltage divider before GPIO 34.
