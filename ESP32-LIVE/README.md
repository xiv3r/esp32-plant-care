
## Components Required

| Component | Qty | Notes |
|---|---|---|
| ESP32 dev board (38-pin or 30-pin) | 1 | Any ESP32 with GPIO25 & GPIO0 exposed |
| Relay module (5V, 1-channel) | 1 | Opto-isolated recommended |
| Boot/Flash button | — | Built into most dev boards (GPIO0) |
| Jumper wires | ~6 | Dupont |
| 5V power supply | 1 | USB or external 5V (≥ 500 mA) |

---

## Pin Map

| ESP32 Pin | Function | Firmware constant |
|---|---|---|
| **GPIO 25** | Relay control output | `RELAY_PIN` |
| **GPIO 0** | Boot button (factory reset, hold 5 s) | `BOOT_BUTTON_PIN` |
| 3V3 | Relay module VCC (if 3.3 V logic relay) | — |
| 5V / VIN | Relay module VCC (if 5 V relay coil) | — |
| GND | Common ground | — |

---

## Connection Table

### 1. Relay → ESP32

| Relay module pin | ESP32 pin | Notes |
|---|---|---|
| **VCC** | **5V (VIN)** | Most relay modules need 5 V for the coil. Use **3V3** only if your module is explicitly 3.3 V-compatible. |
| **GND** | **GND** | Must share ground with ESP32. |
| **IN** (or IN1) | **GPIO 25** | Control signal. |
| **COM** | — | Load side (see below). |
| **NO** (Normally Open) | — | Load side. |
| **NC** (Normally Closed) | — | Load side (optional). |

> ⚠️ **Active-low note:** `cfg.relayActiveLow` defaults to `true`. Most cheap relay modules are **active-LOW** (IN pulled LOW = relay energised), so the default works. If your module is active-HIGH, change "Relay active LOW" to **No** in the Settings page.

---

### 2. Load → Relay (mains / low-voltage)

**For a mains AC load (lamp, fan, pump):**

```
Live (L) ──────► Relay COM
Relay NO ──────► Load Live terminal
Load Neutral ◄─┴─ Mains Neutral
Earth/Ground ────► Load Earth (if applicable)
```

> ⚠️ **DANGER:** Mains wiring must be done by a qualified person. Keep mains traces physically separated from the ESP32 side. Use a properly rated relay and enclosure. For safety, prefer a low-voltage DC load when prototyping.

**For a low-voltage DC load (12 V fan, LED strip):**

```
+12V ──────► Relay COM
Relay NO ──► Load (+)
Load (−) ──► GND (common with supply −)
```

Use the **NO** contact so the load is off by default when the relay is de-energised.

---

### 3. Boot Button (Factory Reset)

| Connection | Notes |
|---|---|
| **GPIO 0 → GND** (momentary push-button) | Most ESP32 dev boards already have a "BOOT" button wired this way. No external button needed. |

Hold **BOOT for 5 seconds** while powered → factory reset (`FACTORY_RESET_MS = 5000`).

> Firmware uses `INPUT_PULLUP` on GPIO 0, so pressing the button pulls it LOW. No external resistor required.

---

### 4. Power

| Source | Connection |
|---|---|
| USB (5 V) | ESP32 USB port — powers board + relay VCC if relay is 5 V and current is low |
| External 5 V | 5V/VIN pin + GND (common with ESP32 GND) — recommended if relay coil draws > 100 mA |

> If using an external 5 V supply for the relay **and** USB for the ESP32, **tie the grounds together**.

---

## Safety Notes

1. **GPIO25 is a 3.3 V logic output.** It cannot drive a bare relay coil directly — always use a relay **module** with a driver transistor/optocoupler.
2. **Do not power a 5 V relay coil from the 3V3 pin** — it will brown out the ESP32.
3. If the relay module has a **JD-VCC / VCC jumper**, keep it installed for simple wiring; remove it only if you want full opto-isolation (then power JD-VCC from a separate 5 V supply and do **not** connect GND between sides).
4. Add a **flyback diode** (e.g. 1N4007) across the relay coil if your module lacks one — most modules include it.
5. For AC loads, use a relay rated ≥ 2× your load current and an appropriately rated fuse on the live line.
