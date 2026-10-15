
---

## 1 Components Needed

| # | Component | Notes |
|---|---|---|
| 1 | ESP32 DevKit (30 or 38 pin) | Any standard ESP32 with WiFi |
| 2 | Capacitive Soil Moisture Sensor v2.0 | 3-pin analog output (AOUT) |
| 3 | SHT31 temperature + humidity sensor | I2C breakout (Adafruit or generic) |
| 4 | BH1750 light sensor (GY-302 or similar) | I2C breakout |
| 5 | Jumper wires | Female-to-female, ~10 cm |
| 6 | USB cable | Micro-USB or USB-C depending on board |
| 7 | (Optional) 100 kΩ resistor | Pull-down for soil pin if sensor disconnected |
| 8 | (Optional) Breadboard | For stable connections |

**Note on voltage:** All three sensors are 3.3 V compatible. **Do NOT power them from 5 V** — GPIO34 is not 5 V tolerant.

---

## 2 Pin Map (ESP32 side)

| ESP32 Pin | GPIO | Function | Connects To |
|---|---|---|---|
| 3V3 | — | 3.3 V power | SHT31 VIN, BH1750 VCC, Soil VCC |
| GND | — | Ground | SHT31 GND, BH1750 GND, Soil GND |
| D34 | GPIO34 | ADC1_CH6 (analog in) | Soil sensor AOUT |
| D21 | GPIO21 | I2C SDA | SHT31 SDA, BH1750 SDA |
| D22 | GPIO22 | I2C SCL | SHT31 SCL, BH1750 SCL |
| D0 / BOOT | GPIO0 | Factory reset button | Onboard BOOT button (no wiring needed) |

**Why GPIO34?** Input-only pin, part of **ADC1**. Wi-Fi uses ADC2 on ESP32, so ADC1 is required for reliable readings while WiFi is active.

---

## 3 Sensor Pin-outs

### 3.1 Capacitive Soil Moisture Sensor v2.0

| Pin (silkscreen) | Meaning | Wire to |
|---|---|---|
| VCC | Power (3.3–5 V; use 3.3 V) | ESP32 3V3 |
| GND | Ground | ESP32 GND |
| AOUT | Analog output (0–3.0 V) | ESP32 GPIO34 |

**Important:** This sensor has a **3-pin JST or 3-pin header**. Older resistive sensors have 4 pins — this guide assumes the 3-pin capacitive v2.0.

### 3.2 SHT31 (I2C Temperature + Humidity)

| Pin (silkscreen) | Meaning | Wire to |
|---|---|---|
| VIN / VCC | Power (3.3 V) | ESP32 3V3 |
| GND | Ground | ESP32 GND |
| SDA | I2C data | ESP32 GPIO21 |
| SCL | I2C clock | ESP32 GPIO22 |
| ADDR | Address select (leave floating = 0x44; tie to GND = 0x44; tie to VIN = 0x45) | Leave unconnected |

**Note:** Some SHT31 breakouts label pins as `VCC/VIN` or `3V3`. **Do not connect to 5 V.**

### 3.3 BH1750 (I2C Light Sensor)

| Pin (silkscreen) | Meaning | Wire to |
|---|---|---|
| VCC | Power (3.3 V) | ESP32 3V3 |
| GND | Ground | ESP32 GND |
| SDA | I2C data | ESP32 GPIO21 |
| SCL | I2C clock | ESP32 GPIO22 |
| ADDR | Address select (leave floating = 0x23; tie to VCC = 0x5C) | Leave unconnected |

**Note:** BH1750 breakout pin order may vary by manufacturer. Verify silkscreen labels before connecting.

---

## 4 Simplified Wiring Table

Just connect each sensor to the ESP32 directly:

### Soil Sensor → ESP32

| Soil Pin | ESP32 Pin |
|---|---|
| VCC | 3V3 |
| GND | GND |
| AOUT | GPIO34 |

### SHT31 → ESP32

| SHT31 Pin | ESP32 Pin |
|---|---|
| VIN | 3V3 |
| GND | GND |
| SDA | GPIO21 |
| SCL | GPIO22 |
| ADDR | (leave unconnected) |

### BH1750 → ESP32

| BH1750 Pin | ESP32 Pin |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO21 (shared with SHT31) |
| SCL | GPIO22 (shared with SHT31) |
| ADDR | (leave unconnected) |

**I2C bus is shared:** SHT31 and BH1750 both connect to GPIO21 (SDA) and GPIO22 (SCL). They have different I2C addresses (0x44/0x45 vs 0x23/0x5C), so no conflict.

---

## 5 Connection Summary (all in one table)

| ESP32 Pin | Soil Sensor | SHT31 | BH1750 |
|---|---|---|---|
| 3V3 | VCC | VIN | VCC |
| GND | GND | GND | GND |
| GPIO34 | AOUT | — | — |
| GPIO21 | — | SDA | SDA |
| GPIO22 | — | SCL | SCL |
| GPIO0 (BOOT) | — | — | — |

---

## 6 Assembly Steps

1. **Power off ESP32** (unplug USB).
2. **Connect soil sensor:**
   - VCC → ESP32 3V3
   - GND → ESP32 GND
   - AOUT → ESP32 GPIO34
3. **Connect SHT31:**
   - VIN → ESP32 3V3
   - GND → ESP32 GND
   - SDA → ESP32 GPIO21
   - SCL → ESP32 GPIO22
   - Leave ADDR floating
4. **Connect BH1750:**
   - VCC → ESP32 3V3
   - GND → ESP32 GND
   - SDA → ESP32 GPIO21 (same node as SHT31 SDA)
   - SCL → ESP32 GPIO22 (same node as SHT31 SCL)
   - Leave ADDR floating
5. **Double-check all GND connections** are common.
6. **Power on** via USB.

---

### 7 Live readings

Every sample interval, you should see:
```
soil=2450mV (3.8%) temp=22.5C hum=48.3% lux=340
```

If soil shows `nan` → sensor uncalibrated (expected on first boot)
If temp/hum/lux show `nan` → sensor not detected

### 7.1 Dashboard

1. Connect phone/PC to `PlantMonitor-XXXX` AP
2. Open `http://192.168.4.1/`
3. Dashboard should show 4 cards with live values

---

## 8 Safety Notes

| Warning | Reason |
|---|---|
| **Never feed 5 V to any sensor** | ESP32 GPIOs are 3.3 V; 5 V will damage the ADC pin |
| **Do not connect 5 V to SHT31/BH1750 VCC** | They may tolerate 5 V (check datasheet) but 3.3 V is required for I2C logic levels |
| **Keep soil sensor cable dry** | Water on the connector can short pins |
| **Do not insert soil sensor into liquid above the marked line** | The v2.0 sensor has a "water line" marking; only the probe tip should be submerged |
| **Ensure common ground** | All sensors must share GND with ESP32 |
| **Check I2C pull-ups** | Most breakouts have built-in 4.7 kΩ pull-ups. If chaining multiple I2C boards, total pull-up = parallel — usually fine at 100 kHz |

---

## 9 Optional Additions

### 9.1 External BOOT button (if your board doesn't have one)

If your ESP32 board lacks an onboard BOOT button:

```
GPIO0 ──────[ Button ]────── GND
```

The internal pull-up keeps GPIO0 HIGH when not pressed. **No external resistor needed.**

### 9.2 Soil pin pull-down (recommended for stability)

If the soil sensor is disconnected, GPIO34 floats. Adding a pull-down gives a defined 0 mV reading:

```
GPIO34 ──────┬────── Soil AOUT
             │
           [100kΩ]
             │
            GND
```

### 9.3 Decoupling capacitors (optional)

For noisy environments, add a 100 nF ceramic capacitor between 3V3 and GND near each sensor.

### 9.4 External antenna (if board supports it)

Not needed for typical indoor plant monitoring range.

---

## 10 Final Checklist

Before powering on, verify:

- [ ] Soil VCC → 3V3 (not 5 V)
- [ ] Soil GND → GND
- [ ] Soil AOUT → GPIO34
- [ ] SHT31 VIN → 3V3
- [ ] SHT31 GND → GND
- [ ] SHT31 SDA → GPIO21
- [ ] SHT31 SCL → GPIO22
- [ ] SHT31 ADDR → not connected
- [ ] BH1750 VCC → 3V3
- [ ] BH1750 GND → GND
- [ ] BH1750 SDA → GPIO21 (same node as SHT31 SDA)
- [ ] BH1750 SCL → GPIO22 (same node as SHT31 SCL)
- [ ] BH1750 ADDR → not connected
- [ ] All GND connections common with ESP32
- [ ] No 5 V connections anywhere
- [ ] Wires firmly seated

Power on and check Serial Monitor for the boot messages.

---

## 11 Troubleshooting Quick Reference

| Symptom | Likely Cause | Fix |
|---|---|---|
| `SHT31 NOT FOUND` | SDA/SCL swapped or loose | Verify GPIO21=SDA, GPIO22=SCL |
| `BH1750 NOT FOUND` | Same as above, or I2C address conflict | Check ADDR pin |
| `soil=0mV` always | Sensor unplugged or AOUT wire loose | Verify GPIO34 connection |
| `soil=3300mV` always | AOUT shorted to VCC | Inspect wiring |
| No serial output | Wrong baud or wrong USB driver | Set 115200, install CP210x/CH340 driver |
| Can't connect to AP | Board in STA mode | Check Serial for `STA IP:` message |
| Dashboard shows `—` for values | Sensor not detected | Check serial for `NOT FOUND` |
| Values wildly fluctuate | Loose connections or noise | Solder connections, add pull-down |

---
