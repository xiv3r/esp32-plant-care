
## 1. Bill of Materials

| Component | Spec / Notes |
|---|---|
| ESP32 Dev Module | 30-pin or 38-pin, 3.3V logic, ADC1 on GPIO32–39 |
| Capacitive soil moisture sensor | v1.2 or v2.0, 3.3V-tolerant (most are 3.3–5.5V) |
| SHT31 breakout | Adafruit #2857 or generic GY-SHT31, I2C, 3.3V |
| Relay module | 1-channel, **opto-isolated preferred**, 5V coil, active-LOW typical |
| Boot button | Onboard `BOOT` (GPIO0) — no external button needed |
| Power supply | 5V ≥ 1A (relay coil + ESP32), or USB 5V |
| Optional | 2× 4.7kΩ pull-ups for I2C (often onboard SHT31 breakout) |

---

## 2. Pin Assignment

| Signal | GPIO | Direction | Notes |
|---|---|---|---|
| Soil analog | **GPIO34** | Input only | ADC1_CH6 — WiFi-safe |
| Relay control | **GPIO25** | Output | Drives relay IN |
| Boot button | **GPIO0** | Input (pullup) | Onboard BOOT button |
| I2C SDA | **GPIO21** | Bidir | Default ESP32 SDA |
| I2C SCL | **GPIO22** | Bidir | Default ESP32 SCL |
| 3V3 | 3V3 pin | Power | Sensors |
| GND | GND pins | Ground | Common ground |
| 5V (VIN) | 5V pin | Power | Relay module VCC |

---

## 3. Connection Tables

### 3.1 Capacitive Soil Moisture Sensor

| Sensor pin | Connect to | Notes |
|---|---|---|
| VCC | ESP32 **3V3** | Do **not** use 5V — ESP32 ADC max is 3.3V |
| GND | ESP32 **GND** | Common ground |
| AOUT (signal) | ESP32 **GPIO34** | Input-only pin, ADC1 — no pull-up needed |

**Notes:**
- GPIO34 is **input-only** (no internal pull-up, no output) — correct for analog input.
- **Never** power the soil sensor from 5V while its signal feeds a 3.3V ADC pin. The AOUT would swing to ~5V, damaging GPIO34.
- Some v1.2 modules are 3.3–5.5V tolerant; v2.0 is 3.3V-only. When in doubt, use 3V3.
- Long cable runs (>2m) pick up noise — twist signal + GND, or add a 100nF cap from AOUT to GND near the ESP32.

### 3.2 SHT31 (I2C Temperature/Humidity)

| SHT31 pin | Connect to | Notes |
|---|---|---|
| VCC / VIN | ESP32 **3V3** | 3.3V only |
| GND | ESP32 **GND** | Common ground |
| SDA | ESP32 **GPIO21** | I2C data |
| SCL | ESP32 **GPIO22** | I2C clock |
| ADDR (if present) | **GND** (default) | 0x44; float/pull-up → 0x45 |

**Notes:**
- Firmware tries **0x44 first, then 0x45** — either address works, no config needed.
- If SHT31 breakout lacks onboard pull-ups, add **4.7kΩ from SDA→3V3** and **4.7kΩ from SCL→3V3**.
- Most GY-SHT31 modules have onboard 10kΩ pull-ups — fine, do not add more.
- Keep SDA/SCL wires short (<30cm) or use shielded cable.
- Only **one** device should be at each address on the bus; if you add more I2C devices, ensure no address clash.

### 3.3 Relay Module (1-channel)

| Relay pin | Connect to | Notes |
|---|---|---|
| VCC | ESP32 **5V (VIN)** | Most modules need 5V coil — see note |
| GND | ESP32 **GND** | **Must** be common with ESP32 GND |
| IN | ESP32 **GPIO25** | Control signal (3.3V logic) |
| COM | Load supply (live/hot) | Relay common |
| NO | Load (pump/valve) | Normally-open — load OFF when relay unpowered |
| NC | unused | Leave disconnected |

**Notes:**

- **Opto-isolated modules:** Most 5V relay boards have an opto-coupler input. They accept 3.3V logic on IN reliably. VCC **must** be 5V for the coil. GND **must** be shared with ESP32 GND even on opto-isolated boards (the opto LED return is to GND).
- **Non-isolated modules (SRD-05VDC-SL-C type):** IN threshold ~2.5–3.0V. GPIO25 at 3.3V is marginal but usually works. If the relay chatters or fails to switch, add a 1kΩ pull-up from IN to 5V — **or better, use a logic-level transistor (2N2222 + 1kΩ base resistor) between GPIO25 and IN** for guaranteed switching.
- **Active level:** The firmware defaults to **active LOW** (most modules: IN=LOW → relay ON). If your module is active HIGH, change "Relay Active Level" to "Active HIGH" in the dashboard Relay tab.
- **Boot glitch:** The firmware deliberately leaves GPIO25 high-Z until after `loadSettings()`, then sets it OUTPUT + OFF. However, during the ~50ms boot window, GPIO25 floats. If your relay module has **no input pull-up/down**, add a **10kΩ resistor from IN to VCC** (active-LOW modules) or **IN to GND** (active-HIGH modules) to guarantee the relay stays OFF during boot. This is a **hardware** safeguard the firmware cannot provide.
- **Mains safety:** If switching AC mains (pump/valve), use a properly rated relay (10A @ 250VAC) and enclose it. **Never** work on live mains. Consider a pre-made "IoT relay" or a solid-state relay for pumps with high inrush.

### 3.4 Boot Button

No wiring — the onboard `BOOT` button is tied to **GPIO0** with a 10kΩ pull-up on the DevKit. Firmware uses it for 5-second-hold factory reset.

**Important:** Do **not** hold BOOT during power-on — that enters the ROM download mode and the firmware won't run. The factory reset only works **after** boot (hold ≥5s while running).

### 3.5 Power

| Source | Connect to | Notes |
|---|---|---|
| USB 5V | ESP32 micro-USB / USB-C | Simplest — powers ESP32 + 5V pin → relay |
| External 5V ≥ 1A | ESP32 **5V (VIN)** + GND | Preferred for unattended deployments |
| 3.3V rail | ESP32 **3V3** | Sensors only — do **not** power relay from 3V3 |

**Power budget:**
- ESP32 WiFi active: ~240mA peak
- Relay coil: ~70mA (5V)
- SHT31: ~1mA
- Soil sensor: ~5mA
- **Total: ~320mA peak** — a 5V/1A supply is comfortable, 5V/2A gives headroom.

**Do not** power the relay from the ESP32's 3.3V regulator — it will brown out the ESP32 on relay switching.
