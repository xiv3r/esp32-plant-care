// ======================================================
//
// ESP32 Multi Smart Garden - Unified Production Firmware
// Board: ESP32 Dev Module (WROOM-32) - Arduino IDE 2.x
// Core: Arduino-ESP32 2.x or 3.x, ArduinoJson 7.x
// One unified sketch: sensors, web UI, captive portal, NVS
// ======================================================


#include <WiFi.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <esp_system.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>
#include <esp_wifi.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

// ======================================================
//
// FIRMWARE METADATA
// ======================================================
#define FW_VERSION "2.4.1"
#define FW_NAME "ESP32 Multi Smart Garden"
#define CFG_MAGIC 0x31455247UL
#define CFG_VERSION 1u
#define NVS_NAMESPACE "sgdnvs"
#define NVS_SLOT_A "cfg_a"
#define NVS_SLOT_B "cfg_b"
#define NVS_KEY_SEQ "seq"
#define NVS_KEY_BOOTS "boots"

// ======================================================
//
// PIN MAP (GPIO assignments - fixed hardware contract)
// ======================================================
static const uint8_t PIN_I2C_SDA = 21;
static const uint8_t PIN_I2C_SCL = 22;
static const uint8_t PIN_MQ2_AOUT = 32;
static const uint8_t PIN_SOIL_AOUT = 34;
static const uint8_t PIN_RAIN_AOUT = 35;
static const uint8_t PIN_BUZZER = 26;
static const uint8_t PIN_LED_ANODE = 33;
static const uint8_t PIN_BUTTON_GPIO0 = 0;
static const uint8_t PIN_RELAY_UNIFIED = 25;
static const uint8_t PIN_RELAY_BME = 23;
static const uint8_t PIN_RELAY_BH = 19;
static const uint8_t PIN_RELAY_MQ = 18;
static const uint8_t PIN_RELAY_SOIL = 17;
static const uint8_t PIN_RELAY_RAIN = 16;
static const uint8_t RELAY_PIN[6] = {PIN_RELAY_UNIFIED, PIN_RELAY_BME, PIN_RELAY_BH, PIN_RELAY_MQ, PIN_RELAY_SOIL, PIN_RELAY_RAIN};
static const uint8_t RELAY_COUNT = 6;

static const uint8_t SRC_BIT_BME = 0;
static const uint8_t SRC_BIT_BH = 1;
static const uint8_t SRC_BIT_MQ = 2;
static const uint8_t SRC_BIT_SOIL = 3;
static const uint8_t SRC_BIT_RAIN = 4;

// ======================================================
//
// TUNING TABLES
// ======================================================
static const uint16_t INTERVAL_S[6] = {1, 2, 5, 10, 30, 60};
static const uint16_t OLED_TIMEOUT_S[5] = {0, 30, 60, 300, 600};
static const uint8_t OLED_PAGE_CYCLE_S[4] = {2, 4, 6, 8};
static const uint16_t BUZZ_MAX_ALERT_S[5] = {5, 15, 30, 60, 0};
static const uint8_t MQ_SAMPLE_COUNT[4] = {4, 8, 16, 32};
static const uint32_t AP_IP_OCTETS[4] = {192, 168, 4, 1};

// ======================================================
//
// PERSISTENT CONFIGURATION (packed, CRC-32 guarded, A/B)
// ======================================================
typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t seq;
  uint8_t oledEnabled;
  uint8_t oledFlip;
  uint8_t oledContrast;
  uint8_t oledTimeoutIdx;
  uint8_t oledCycleIdx;
  uint8_t oledPageOverview;
  uint8_t oledPageStatus;
  uint8_t oledPageAlerts;
  uint8_t oledPad0;
  uint8_t bmeEnabled;
  uint8_t bmeIntervalIdx;
  uint8_t bmeMetric;
  uint8_t bmeCondition;
  uint8_t bmeTrigger;
  uint8_t bmeRelay;
  uint8_t bmeUnified;
  uint8_t bmeBuzzer;
  uint8_t bmeLed;
  uint8_t bmePad0;
  int16_t bmeTempOffsetX10;
  int16_t bmeHumOffsetX10;
  int16_t bmePresOffsetX10;
  uint16_t bmeCooldownSec;
  float bmeThreshold;
  float bmeHysteresis;
  uint8_t bhEnabled;
  uint8_t bhIntervalIdx;
  uint8_t bhCondition;
  uint8_t bhTrigger;
  uint8_t bhRelay;
  uint8_t bhUnified;
  uint8_t bhBuzzer;
  uint8_t bhLed;
  uint16_t bhCooldownSec;
  float bhThresholdLux;
  float bhHysteresisLux;
  uint8_t mqEnabled;
  uint8_t mqIntervalIdx;
  uint8_t mqSamplesIdx;
  uint8_t mqCondition;
  uint8_t mqTrigger;
  uint8_t mqRelay;
  uint8_t mqUnified;
  uint8_t mqBuzzer;
  uint8_t mqLed;
  uint16_t mqCooldownSec;
  float mqThresholdPct;
  float mqHysteresisPct;
  uint8_t soilEnabled;
  uint8_t soilIntervalIdx;
  uint8_t soilCondition;
  uint8_t soilTrigger;
  uint8_t soilRelay;
  uint8_t soilUnified;
  uint8_t soilBuzzer;
  uint8_t soilLed;
  uint16_t soilDryRaw;
  uint16_t soilWetRaw;
  uint16_t soilCooldownSec;
  uint16_t soilPad0;
  float soilThresholdPct;
  float soilHysteresisPct;
  uint8_t rainEnabled;
  uint8_t rainIntervalIdx;
  uint8_t rainCondition;
  uint8_t rainTrigger;
  uint8_t rainRelay;
  uint8_t rainUnified;
  uint8_t rainBuzzer;
  uint8_t rainLed;
  uint16_t rainDryRaw;
  uint16_t rainWetRaw;
  uint16_t rainCooldownSec;
  uint16_t rainPad0;
  float rainThresholdPct;
  float rainHysteresisPct;
  uint8_t buzzEnabled;
  uint8_t buzzPattern;
  uint8_t buzzVolume;
  uint8_t buzzMaxAlertIdx;
  uint8_t ledEnabled;
  uint8_t ledMode;
  uint8_t ledBrightness;
  uint8_t ledBlink;
  uint8_t relayEnabled[6];
  uint8_t relayMode[6];
  uint8_t relayActiveLow[6];
  uint8_t relaySources[6];
  char apSsid[33];
  char apPass[64];
  uint8_t apChannel;
  uint8_t apHidden;
  uint8_t apMaxConn;
  uint8_t cfgPad0;
  uint32_t crc;
} Config;

// ======================================================
//
// CRC-32 (IEEE 802.3, bitwise, table-free)
// ======================================================
static uint32_t crc32Ieee(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
    }
  }
  return ~crc;
}

// ======================================================
//
// CONFIG DEFAULTS
// ======================================================
static void configDefaults(Config& c) {
  memset(&c, 0, sizeof(Config));
  c.magic = CFG_MAGIC;
  c.version = CFG_VERSION;
  c.size = (uint16_t)sizeof(Config);
  c.seq = 0;
  c.oledEnabled = 1;
  c.oledFlip = 0;
  c.oledContrast = 160;
  c.oledTimeoutIdx = 0;
  c.oledCycleIdx = 1;
  c.oledPageOverview = 1;
  c.oledPageStatus = 1;
  c.oledPageAlerts = 1;
  c.bmeEnabled = 1;
  c.bmeIntervalIdx = 1;
  c.bmeMetric = 0;
  c.bmeCondition = 0;
  c.bmeTrigger = 0;
  c.bmeRelay = 1;
  c.bmeUnified = 0;
  c.bmeBuzzer = 1;
  c.bmeLed = 1;
  c.bmeTempOffsetX10 = 0;
  c.bmeHumOffsetX10 = 0;
  c.bmePresOffsetX10 = 0;
  c.bmeCooldownSec = 30;
  c.bmeThreshold = 35.0f;
  c.bmeHysteresis = 1.0f;
  c.bhEnabled = 1;
  c.bhIntervalIdx = 1;
  c.bhCondition = 1;
  c.bhTrigger = 0;
  c.bhRelay = 0;
  c.bhUnified = 0;
  c.bhBuzzer = 0;
  c.bhLed = 0;
  c.bhCooldownSec = 30;
  c.bhThresholdLux = 400.0f;
  c.bhHysteresisLux = 50.0f;
  c.mqEnabled = 1;
  c.mqIntervalIdx = 2;
  c.mqSamplesIdx = 1;
  c.mqCondition = 0;
  c.mqTrigger = 0;
  c.mqRelay = 1;
  c.mqUnified = 1;
  c.mqBuzzer = 1;
  c.mqLed = 1;
  c.mqCooldownSec = 60;
  c.mqThresholdPct = 40.0f;
  c.mqHysteresisPct = 5.0f;
  c.soilEnabled = 1;
  c.soilIntervalIdx = 1;
  c.soilCondition = 1;
  c.soilTrigger = 1;
  c.soilRelay = 1;
  c.soilUnified = 1;
  c.soilBuzzer = 1;
  c.soilLed = 1;
  c.soilDryRaw = 3100;
  c.soilWetRaw = 1400;
  c.soilCooldownSec = 60;
  c.soilThresholdPct = 30.0f;
  c.soilHysteresisPct = 5.0f;
  c.rainEnabled = 1;
  c.rainIntervalIdx = 1;
  c.rainCondition = 0;
  c.rainTrigger = 0;
  c.rainRelay = 1;
  c.rainUnified = 1;
  c.rainBuzzer = 1;
  c.rainLed = 1;
  c.rainDryRaw = 3500;
  c.rainWetRaw = 900;
  c.rainCooldownSec = 60;
  c.rainThresholdPct = 60.0f;
  c.rainHysteresisPct = 8.0f;
  c.buzzEnabled = 1;
  c.buzzPattern = 1;
  c.buzzVolume = 160;
  c.buzzMaxAlertIdx = 1;
  c.ledEnabled = 1;
  c.ledMode = 0;
  c.ledBrightness = 128;
  c.ledBlink = 1;
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    c.relayEnabled[i] = 1;
    c.relayMode[i] = 0;
    c.relayActiveLow[i] = 1;
  }
  c.relaySources[0] = 31;
  c.relaySources[1] = (uint8_t)(1u << SRC_BIT_BME);
  c.relaySources[2] = (uint8_t)(1u << SRC_BIT_BH);
  c.relaySources[3] = (uint8_t)(1u << SRC_BIT_MQ);
  c.relaySources[4] = (uint8_t)(1u << SRC_BIT_SOIL);
  c.relaySources[5] = (uint8_t)(1u << SRC_BIT_RAIN);
  strlcpy(c.apSsid, "ESP32_Multi_Smart_Garden", sizeof(c.apSsid));
  strlcpy(c.apPass, "12345678", sizeof(c.apPass));
  c.apChannel = 6;
  c.apHidden = 0;
  c.apMaxConn = 4;
  c.crc = 0;
}

static void configStamp(Config& c, uint32_t seq) {
  c.magic = CFG_MAGIC;
  c.version = CFG_VERSION;
  c.size = (uint16_t)sizeof(Config);
  c.seq = seq;
  c.crc = 0;
  c.crc = crc32Ieee((const uint8_t*)&c, offsetof(Config, crc));
}

static bool configCheck(const Config& c) {
  if (c.magic != CFG_MAGIC) return false;
  if (c.version != CFG_VERSION) return false;
  if (c.size != (uint16_t)sizeof(Config)) return false;
  Config tmp;
  memcpy(&tmp, &c, sizeof(Config));
  tmp.crc = 0;
  return crc32Ieee((const uint8_t*)&tmp, offsetof(Config, crc)) == c.crc;
}

static bool configContentEqual(const Config& a, const Config& b) {
  const uint8_t* pa = (const uint8_t*)&a;
  const uint8_t* pb = (const uint8_t*)&b;
  const size_t headLen = offsetof(Config, seq);
  const size_t tailOff = offsetof(Config, seq) + sizeof(uint32_t);
  const size_t tailLen = offsetof(Config, crc) - tailOff;
  if (memcmp(pa, pb, headLen) != 0) return false;
  return memcmp(pa + tailOff, pb + tailOff, tailLen) == 0;
}

// ======================================================
//
// RUNTIME STATE (64-bit counters where it matters)
// ======================================================
static Config g_cfg;
static uint32_t g_cfgSeq = 0;
static uint8_t g_cfgSource = 0;
static uint32_t g_nvsWrites = 0;
static uint32_t g_bootCount = 0;
static uint64_t g_uptimeMs = 0;
static uint32_t g_lastUptimeTick = 0;
static uint32_t g_loopIterations = 0;
static uint32_t g_loopHzWindowStart = 0;
static uint32_t g_loopHz = 0;
static uint32_t g_pendingRestartMs = 0;

static bool g_bmePresent = false;
static uint8_t g_bmeAddr = 0x76;
static bool g_bmeValid = false;
static float g_bmeT = 0.0f;
static float g_bmeH = 0.0f;
static float g_bmeP = 0.0f;
static bool g_bmeTrig = false;
static uint32_t g_bmeLastMs = 0;

static bool g_bhPresent = false;
static uint8_t g_bhAddr = 0x23;
static bool g_bhValid = false;
static float g_bhLux = 0.0f;
static bool g_bhTrig = false;
static uint32_t g_bhLastMs = 0;

static bool g_mqValid = false;
static uint16_t g_mqRaw = 0;
static float g_mqPct = 0.0f;
static float g_mqEma = 0.0f;
static bool g_mqEmaInit = false;
static bool g_mqTrig = false;
static uint32_t g_mqLastMs = 0;

static bool g_soilValid = false;
static uint16_t g_soilRaw = 0;
static float g_soilPct = 0.0f;
static float g_soilEma = 0.0f;
static bool g_soilEmaInit = false;
static bool g_soilTrig = false;
static uint32_t g_soilLastMs = 0;

static bool g_rainValid = false;
static uint16_t g_rainRaw = 0;
static float g_rainPct = 0.0f;
static float g_rainEma = 0.0f;
static bool g_rainEmaInit = false;
static bool g_rainTrig = false;
static uint32_t g_rainLastMs = 0;

static uint32_t g_lastAlertMs[5] = {0, 0, 0, 0, 0};

static bool g_relayState[RELAY_COUNT] = {false, false, false, false, false, false};
static uint8_t g_relayWritten[RELAY_COUNT] = {255, 255, 255, 255, 255, 255};

static bool g_buzzActive = false;
static bool g_buzzMuted = false;
static bool g_buzzPlaying = false;
static uint8_t g_buzzPulseIndex = 0;
static uint8_t g_buzzPhase = 0;
static uint32_t g_buzzPhaseStart = 0;
static uint32_t g_buzzAlertSince = 0;
static uint32_t g_buzzTestUntil = 0;
static uint8_t g_buzzTestPattern = 1;

static bool g_ledOn = false;
static uint32_t g_ledTestUntil = 0;

static bool g_oledReady = false;
static bool g_oledSleeping = false;
static uint32_t g_oledLastActivityMs = 0;
static uint32_t g_oledPageSwitchMs = 0;
static uint8_t g_oledPageIndex = 0;
static uint32_t g_oledLastDrawMs = 0;
static bool g_oledDirty = true;

static uint32_t g_btnLastLowMs = 0;
static uint32_t g_btnDownSince = 0;
static bool g_factoryWipeArmed = false;
static bool g_factoryHolding = false;
static uint32_t g_pendingApRestartMs = 0;
static portMUX_TYPE g_cfgMux = portMUX_INITIALIZER_UNLOCKED;

static Adafruit_SSD1306 g_oled(128, 64, &Wire, -1);
static Adafruit_BME280 g_bme;
static BH1750 g_bh;

// ======================================================
//
// NVS ACCESS - A/B SLOT STRATEGY
// ======================================================
static bool nvsReadSlot(const char* key, Config* out) {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, true)) return false;
  Config staged;
  memset(&staged, 0, sizeof(Config));
  size_t got = prefs.getBytes(key, &staged, sizeof(Config));
  prefs.end();
  if (got != sizeof(Config)) return false;
  if (!configCheck(staged)) return false;
  memcpy(out, &staged, sizeof(Config));
  return true;
}

static bool nvsWriteSlot(const char* key, const Config& c) {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, false)) return false;
  size_t written = prefs.putBytes(key, &c, sizeof(Config));
  prefs.end();
  return written == sizeof(Config);
}

static uint8_t configArbitrate(bool validA, uint32_t seqA, bool validB, uint32_t seqB) {
  if (validA && (!validB || seqA >= seqB)) return 1;
  if (validB) return 2;
  return 0;
}

static void configLoad() {
  Config def;
  configDefaults(def);
  uint32_t seq = 0;
  {
    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, true)) {
      seq = prefs.getUInt(NVS_KEY_SEQ, 0);
      prefs.end();
    }
  }
  Config a;
  Config b;
  const bool va = nvsReadSlot(NVS_SLOT_A, &a);
  const bool vb = nvsReadSlot(NVS_SLOT_B, &b);
  const uint8_t chosen = configArbitrate(va, va ? a.seq : 0u, vb, vb ? b.seq : 0u);
  if (chosen == 1) {
    memcpy(&g_cfg, &a, sizeof(Config));
    g_cfgSource = 1;
  } else if (chosen == 2) {
    memcpy(&g_cfg, &b, sizeof(Config));
    g_cfgSource = 2;
  } else {
    configStamp(def, seq + 1u);
    memcpy(&g_cfg, &def, sizeof(Config));
    g_cfgSource = 0;
  }
  g_cfgSeq = g_cfg.seq;
}

static bool configSave() {
  Config out;
  portENTER_CRITICAL(&g_cfgMux);
  memcpy(&out, &g_cfg, sizeof(Config));
  portEXIT_CRITICAL(&g_cfgMux);
  uint32_t seq = g_cfgSeq + 1u;
  configStamp(out, seq);
  const char* slot = (seq & 1u) ? NVS_SLOT_A : NVS_SLOT_B;
  if (!nvsWriteSlot(slot, out)) return false;
  {
    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, false)) {
      prefs.putUInt(NVS_KEY_SEQ, seq);
      prefs.end();
    }
  }
  portENTER_CRITICAL(&g_cfgMux);
  memcpy(&g_cfg, &out, sizeof(Config));
  portEXIT_CRITICAL(&g_cfgMux);
  g_cfgSeq = seq;
  g_nvsWrites++;
  return true;
}

static void bootCountIncrement() {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, false)) return;
  uint32_t boots = prefs.getUInt(NVS_KEY_BOOTS, 0) + 1u;
  prefs.putUInt(NVS_KEY_BOOTS, boots);
  prefs.end();
  g_bootCount = boots;
  g_nvsWrites++;
}
// ======================================================
//
// HELPERS - clamp, schmitt trigger, EMA, rollover-safe checks
// ======================================================
static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static uint8_t clampu8(int v, int lo, int hi) {
  return (uint8_t)(v < lo ? lo : (v > hi ? hi : v));
}

static bool schmitt(bool state, float value, float threshold, float hysteresis, uint8_t condition) {
  const float hyst = fabsf(hysteresis);
  if (condition == 0) {
    if (!state && value >= threshold) return true;
    if (state && value <= threshold - hyst) return false;
    return state;
  }
  if (!state && value <= threshold) return true;
  if (state && value >= threshold + hyst) return false;
  return state;
}

static float emaStep(float previous, bool initialized, float sample, float alpha) {
  return initialized ? (previous + alpha * (sample - previous)) : sample;
}

static uint16_t analogAverage64(uint8_t pin, uint8_t samples) {
  uint64_t accumulator = 0;
  const uint8_t n = samples == 0 ? 1 : samples;
  for (uint8_t i = 0; i < n; i++) {
    accumulator += (uint32_t)(uint16_t)analogRead(pin);
  }
  return (uint16_t)(accumulator / (uint64_t)n);
}

static uint64_t uptimeSeconds() {
  return g_uptimeMs / 1000ULL;
}

// ======================================================
//
// OUTPUT DRIVERS - LEDC PWM (ABSTRACTED FOR CORE 2.x / 3.x)
// ======================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
static void buzzerAttach() { ledcAttach(PIN_BUZZER, 2093, 8); ledcWrite(PIN_BUZZER, 0); }
static void buzzerDuty(uint8_t duty) { ledcWrite(PIN_BUZZER, duty); }
static void ledAttach() { ledcAttach(PIN_LED_ANODE, 5000, 8); ledcWrite(PIN_LED_ANODE, 0); }
static void ledDuty(uint8_t duty) { ledcWrite(PIN_LED_ANODE, duty); }
#else
#define LEDC_CH_BUZZER 0
#define LEDC_CH_LED 1
static void buzzerAttach() { ledcSetup(LEDC_CH_BUZZER, 2093, 8); ledcAttachPin(PIN_BUZZER, LEDC_CH_BUZZER); ledcWrite(LEDC_CH_BUZZER, 0); }
static void buzzerDuty(uint8_t duty) { ledcWrite(LEDC_CH_BUZZER, duty); }
static void ledAttach() { ledcSetup(LEDC_CH_LED, 5000, 8); ledcAttachPin(PIN_LED_ANODE, LEDC_CH_LED); ledcWrite(LEDC_CH_LED, 0); }
static void ledDuty(uint8_t duty) { ledcWrite(LEDC_CH_LED, duty); }
#endif

static void relayWritePinsInactiveFirst() {
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    pinMode(RELAY_PIN[i], OUTPUT);
    const bool activeLow = g_cfg.relayActiveLow[i] != 0;
    digitalWrite(RELAY_PIN[i], activeLow ? HIGH : LOW);
    g_relayState[i] = false;
    g_relayWritten[i] = activeLow ? HIGH : LOW;
  }
}

static void relayApplyLevel(uint8_t index, bool energized) {
  const bool activeLow = g_cfg.relayActiveLow[index] != 0;
  const uint8_t level = (energized != activeLow) ? HIGH : LOW;
  if (g_relayWritten[index] != level) {
    digitalWrite(RELAY_PIN[index], level);
    g_relayWritten[index] = level;
  }
  g_relayState[index] = energized;
}

// ======================================================
//
// SENSOR SAMPLING - NON-BLOCKING STATE MACHINES
// ======================================================
static void bmeTick(uint32_t now) {
  if (!g_cfg.bmeEnabled || !g_bmePresent) return;
  const uint32_t intervalMs = (uint32_t)INTERVAL_S[clampu8(g_cfg.bmeIntervalIdx, 0, 5)] * 1000UL;
  if ((uint32_t)(now - g_bmeLastMs) < intervalMs) return;
  g_bmeLastMs = now;
  const float t = g_bme.readTemperature() + ((float)g_cfg.bmeTempOffsetX10 / 10.0f);
  const float h = clampf(g_bme.readHumidity() + ((float)g_cfg.bmeHumOffsetX10 / 10.0f), 0.0f, 100.0f);
  const float p = (g_bme.readPressure() / 100.0f) + ((float)g_cfg.bmePresOffsetX10 / 10.0f);
  if (isnan(t) || isnan(h) || isnan(p)) {
    g_bmeValid = false;
    return;
  }
  g_bmeT = t;
  g_bmeH = h;
  g_bmeP = p;
  g_bmeValid = true;
  float metric = g_bmeT;
  if (g_cfg.bmeMetric == 1) metric = g_bmeH;
  else if (g_cfg.bmeMetric == 2) metric = g_bmeP;
  const bool trig = schmitt(g_bmeTrig, metric, g_cfg.bmeThreshold, g_cfg.bmeHysteresis, g_cfg.bmeCondition);
  if (trig != g_bmeTrig) {
    g_bmeTrig = trig;
    if (trig) g_lastAlertMs[SRC_BIT_BME] = now;
    g_oledDirty = true;
  }
}

static void bhTick(uint32_t now) {
  if (!g_cfg.bhEnabled || !g_bhPresent) return;
  const uint32_t intervalMs = (uint32_t)INTERVAL_S[clampu8(g_cfg.bhIntervalIdx, 0, 5)] * 1000UL;
  if ((uint32_t)(now - g_bhLastMs) < intervalMs) return;
  g_bhLastMs = now;
  const float lux = g_bh.readLightLevel();
  if (isnan(lux) || lux < 0.0f) {
    g_bhValid = false;
    return;
  }
  g_bhLux = lux;
  g_bhValid = true;
  const bool trig = schmitt(g_bhTrig, g_bhLux, g_cfg.bhThresholdLux, g_cfg.bhHysteresisLux, g_cfg.bhCondition);
  if (trig != g_bhTrig) {
    g_bhTrig = trig;
    if (trig) g_lastAlertMs[SRC_BIT_BH] = now;
    g_oledDirty = true;
  }
}

static void mqTick(uint32_t now) {
  if (!g_cfg.mqEnabled) return;
  const uint32_t intervalMs = (uint32_t)INTERVAL_S[clampu8(g_cfg.mqIntervalIdx, 0, 5)] * 1000UL;
  if ((uint32_t)(now - g_mqLastMs) < intervalMs) return;
  g_mqLastMs = now;
  const uint16_t raw = analogAverage64(PIN_MQ2_AOUT, MQ_SAMPLE_COUNT[clampu8(g_cfg.mqSamplesIdx, 0, 3)]);
  const float sample = ((float)raw * 100.0f) / 4095.0f;
  g_mqEma = emaStep(g_mqEma, g_mqEmaInit, sample, 0.25f);
  g_mqEmaInit = true;
  g_mqRaw = raw;
  g_mqPct = g_mqEma;
  g_mqValid = true;
  const bool trig = schmitt(g_mqTrig, g_mqPct, g_cfg.mqThresholdPct, g_cfg.mqHysteresisPct, g_cfg.mqCondition);
  if (trig != g_mqTrig) {
    g_mqTrig = trig;
    if (trig) g_lastAlertMs[SRC_BIT_MQ] = now;
    g_oledDirty = true;
  }
}

static float analogToPercent(uint16_t raw, uint16_t dryRaw, uint16_t wetRaw) {
  const int32_t dry = (int32_t)dryRaw;
  const int32_t wet = (int32_t)wetRaw;
  const int32_t span = dry - wet;
  if (span == 0) return 0.0f;
  float pct = ((float)(dry - (int32_t)raw) * 100.0f) / (float)span;
  return clampf(pct, 0.0f, 100.0f);
}

static void soilTick(uint32_t now) {
  if (!g_cfg.soilEnabled) return;
  const uint32_t intervalMs = (uint32_t)INTERVAL_S[clampu8(g_cfg.soilIntervalIdx, 0, 5)] * 1000UL;
  if ((uint32_t)(now - g_soilLastMs) < intervalMs) return;
  g_soilLastMs = now;
  const uint16_t raw = analogAverage64(PIN_SOIL_AOUT, 8);
  const float sample = analogToPercent(raw, g_cfg.soilDryRaw, g_cfg.soilWetRaw);
  g_soilEma = emaStep(g_soilEma, g_soilEmaInit, sample, 0.25f);
  g_soilEmaInit = true;
  g_soilRaw = raw;
  g_soilPct = g_soilEma;
  g_soilValid = true;
  const bool trig = schmitt(g_soilTrig, g_soilPct, g_cfg.soilThresholdPct, g_cfg.soilHysteresisPct, g_cfg.soilCondition);
  if (trig != g_soilTrig) {
    g_soilTrig = trig;
    if (trig) g_lastAlertMs[SRC_BIT_SOIL] = now;
    g_oledDirty = true;
  }
}

static void rainTick(uint32_t now) {
  if (!g_cfg.rainEnabled) return;
  const uint32_t intervalMs = (uint32_t)INTERVAL_S[clampu8(g_cfg.rainIntervalIdx, 0, 5)] * 1000UL;
  if ((uint32_t)(now - g_rainLastMs) < intervalMs) return;
  g_rainLastMs = now;
  const uint16_t raw = analogAverage64(PIN_RAIN_AOUT, 8);
  const float sample = analogToPercent(raw, g_cfg.rainDryRaw, g_cfg.rainWetRaw);
  g_rainEma = emaStep(g_rainEma, g_rainEmaInit, sample, 0.25f);
  g_rainEmaInit = true;
  g_rainRaw = raw;
  g_rainPct = g_rainEma;
  g_rainValid = true;
  const bool trig = schmitt(g_rainTrig, g_rainPct, g_cfg.rainThresholdPct, g_cfg.rainHysteresisPct, g_cfg.rainCondition);
  if (trig != g_rainTrig) {
    g_rainTrig = trig;
    if (trig) g_lastAlertMs[SRC_BIT_RAIN] = now;
    g_oledDirty = true;
  }
}

// ======================================================
//
// ALERT AGGREGATION
// ======================================================
static bool alertBitActive(const Config& c, uint8_t bit) {
  switch (bit) {
    case SRC_BIT_BME: return g_bmeTrig && c.bmeTrigger != 0 && c.bmeEnabled != 0;
    case SRC_BIT_BH: return g_bhTrig && c.bhTrigger != 0 && c.bhEnabled != 0;
    case SRC_BIT_MQ: return g_mqTrig && c.mqTrigger != 0 && c.mqEnabled != 0;
    case SRC_BIT_SOIL: return g_soilTrig && c.soilTrigger != 0 && c.soilEnabled != 0;
    case SRC_BIT_RAIN: return g_rainTrig && c.rainTrigger != 0 && c.rainEnabled != 0;
    default: return false;
  }
}

static uint8_t alertBitsEffective(const Config& c) {
  uint8_t bits = 0;
  for (uint8_t bit = 0; bit < 5; bit++) {
    if (alertBitActive(c, bit)) bits |= (uint8_t)(1u << bit);
  }
  return bits;
}

static bool buzzerRequested(const Config& c, uint8_t bit) {
  switch (bit) {
    case SRC_BIT_BME: return c.bmeBuzzer != 0;
    case SRC_BIT_BH: return c.bhBuzzer != 0;
    case SRC_BIT_MQ: return c.mqBuzzer != 0;
    case SRC_BIT_SOIL: return c.soilBuzzer != 0;
    case SRC_BIT_RAIN: return c.rainBuzzer != 0;
    default: return false;
  }
}

static bool ledRequested(const Config& c, uint8_t bit) {
  switch (bit) {
    case SRC_BIT_BME: return c.bmeLed != 0;
    case SRC_BIT_BH: return c.bhLed != 0;
    case SRC_BIT_MQ: return c.mqLed != 0;
    case SRC_BIT_SOIL: return c.soilLed != 0;
    case SRC_BIT_RAIN: return c.rainLed != 0;
    default: return false;
  }
}

static bool relayAutoRequested(const Config& c, uint8_t bit) {
  switch (bit) {
    case SRC_BIT_BME: return c.bmeRelay != 0;
    case SRC_BIT_BH: return c.bhRelay != 0;
    case SRC_BIT_MQ: return c.mqRelay != 0;
    case SRC_BIT_SOIL: return c.soilRelay != 0;
    case SRC_BIT_RAIN: return c.rainRelay != 0;
    default: return false;
  }
}

static bool unifiedRequested(const Config& c, uint8_t bit) {
  switch (bit) {
    case SRC_BIT_BME: return c.bmeUnified != 0;
    case SRC_BIT_BH: return c.bhUnified != 0;
    case SRC_BIT_MQ: return c.mqUnified != 0;
    case SRC_BIT_SOIL: return c.soilUnified != 0;
    case SRC_BIT_RAIN: return c.rainUnified != 0;
    default: return false;
  }
}

static bool anyBuzzerAlert(const Config& c) {
  for (uint8_t bit = 0; bit < 5; bit++) {
    if (alertBitActive(c, bit) && buzzerRequested(c, bit)) return true;
  }
  return false;
}

static bool anyLedAlert(const Config& c) {
  for (uint8_t bit = 0; bit < 5; bit++) {
    if (alertBitActive(c, bit) && ledRequested(c, bit)) return true;
  }
  return false;
}

static uint8_t relayAutoBits(const Config& c, uint8_t relayIndex) {
  uint8_t mask = 0;
  for (uint8_t bit = 0; bit < 5; bit++) {
    if (!alertBitActive(c, bit)) continue;
    if (relayIndex == 0) {
      if (!unifiedRequested(c, bit)) continue;
    } else {
      if (!relayAutoRequested(c, bit)) continue;
    }
    if ((c.relaySources[relayIndex] & (uint8_t)(1u << bit)) != 0) {
      mask |= (uint8_t)(1u << bit);
    }
  }
  return mask;
}

// ======================================================
//
// RELAY MATRIX ENGINE (dedicated + unified GPIO25)
// ======================================================
static bool relayChannelEnergized(uint8_t relayEnabled, uint8_t mode, uint8_t autoBits) {
  if (relayEnabled == 0) return false;
  if (mode == 1) return true;
  if (mode == 2) return false;
  return autoBits != 0;
}

static void relayTick(uint32_t now) {
  (void)now;
  Config snap;
  portENTER_CRITICAL(&g_cfgMux);
  memcpy(&snap, &g_cfg, sizeof(Config));
  portEXIT_CRITICAL(&g_cfgMux);
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    relayApplyLevel(i, relayChannelEnergized(snap.relayEnabled[i], snap.relayMode[i], relayAutoBits(snap, i)));
  }
}

// ======================================================
//
// BUZZER ENGINE - PATTERN PLAYER, AUTO-MUTE
// ======================================================
static const uint8_t BUZZ_PULSE_COUNT[5] = {1, 2, 3, 1, 5};
static const uint16_t BUZZ_PULSE_ON_MS[5][5] = {
  {170, 0, 0, 0, 0},
  {140, 140, 0, 0, 0},
  {120, 120, 120, 0, 0},
  {1200, 0, 0, 0, 0},
  {80, 80, 80, 80, 80}
};
static const uint16_t BUZZ_PULSE_GAP_MS[5][5] = {
  {0, 0, 0, 0, 0},
  {90, 0, 0, 0, 0},
  {70, 70, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {70, 70, 70, 70, 0}
};
static const uint16_t BUZZ_CYCLE_GAP_MS = 700;

static void buzzerService(uint32_t now, bool demand, uint8_t pattern) {
  const uint8_t pat = pattern > 4 ? 4 : pattern;
  if (!demand) {
    if (g_buzzPlaying) buzzerDuty(0);
    g_buzzPlaying = false;
    g_buzzPhase = 0;
    g_buzzPulseIndex = 0;
    return;
  }
  const uint32_t limitMs = (uint32_t)BUZZ_MAX_ALERT_S[clampu8(g_cfg.buzzMaxAlertIdx, 0, 4)] * 1000UL;
  if (limitMs > 0 && (uint32_t)(now - g_buzzAlertSince) > limitMs) {
    if (g_buzzPlaying) buzzerDuty(0);
    g_buzzPlaying = false;
    return;
  }
  g_buzzPlaying = true;
  if (g_buzzPhase == 0) {
    g_buzzPulseIndex = 0;
    g_buzzPhase = 1;
    g_buzzPhaseStart = now;
    buzzerDuty(g_cfg.buzzVolume);
    return;
  }
  const uint8_t pulseCount = BUZZ_PULSE_COUNT[pat];
  const uint16_t onMs = BUZZ_PULSE_ON_MS[pat][g_buzzPulseIndex];
  const uint16_t gapMs = BUZZ_PULSE_GAP_MS[pat][g_buzzPulseIndex];
  if (g_buzzPhase == 1) {
    if ((uint32_t)(now - g_buzzPhaseStart) >= onMs) {
      buzzerDuty(0);
      if (gapMs > 0) {
        g_buzzPhase = 2;
        g_buzzPhaseStart = now;
      } else {
        g_buzzPulseIndex++;
        if (g_buzzPulseIndex >= pulseCount) {
          g_buzzPhase = 3;
          g_buzzPhaseStart = now;
        } else {
          g_buzzPhase = 1;
          g_buzzPhaseStart = now;
          buzzerDuty(g_cfg.buzzVolume);
        }
      }
    }
    return;
  }
  if (g_buzzPhase == 2) {
    if ((uint32_t)(now - g_buzzPhaseStart) >= gapMs) {
      g_buzzPulseIndex++;
      if (g_buzzPulseIndex >= pulseCount) {
        g_buzzPhase = 3;
        g_buzzPhaseStart = now;
      } else {
        g_buzzPhase = 1;
        g_buzzPhaseStart = now;
        buzzerDuty(g_cfg.buzzVolume);
      }
    }
    return;
  }
  if (g_buzzPhase == 3) {
    if ((uint32_t)(now - g_buzzPhaseStart) >= BUZZ_CYCLE_GAP_MS) {
      g_buzzPulseIndex = 0;
      g_buzzPhase = 1;
      g_buzzPhaseStart = now;
      buzzerDuty(g_cfg.buzzVolume);
    }
    return;
  }
  g_buzzPhase = 1;
  g_buzzPhaseStart = now;
  buzzerDuty(g_cfg.buzzVolume);
}

static void buzzerTick(uint32_t now) {
  const bool alertDemand = (g_cfg.buzzEnabled != 0) && anyBuzzerAlert(g_cfg);
  const bool testActive = (g_buzzTestUntil != 0) && ((int32_t)(now - g_buzzTestUntil) < 0);
  if (g_buzzTestUntil != 0 && !testActive) {
    g_buzzTestUntil = 0;
    buzzerDuty(0);
    g_buzzPhase = 0;
    g_buzzPulseIndex = 0;
    g_buzzPlaying = false;
  }
  if (testActive) {
    if (!g_buzzActive) {
      g_buzzActive = true;
      g_buzzAlertSince = now;
      g_buzzPhase = 0;
      g_buzzPulseIndex = 0;
    }
    buzzerService(now, true, g_buzzTestPattern);
    return;
  }
  if (alertDemand) {
    if (!g_buzzActive) {
      g_buzzActive = true;
      g_buzzAlertSince = now;
      g_buzzMuted = false;
      g_buzzPhase = 0;
      g_buzzPulseIndex = 0;
    }
    buzzerService(now, true, g_cfg.buzzPattern);
  } else {
    g_buzzActive = false;
    g_buzzMuted = false;
    buzzerService(now, false, g_cfg.buzzPattern);
  }
}

// ======================================================
//
// LED ENGINE - AUTO / ALWAYS / OFF WITH 2 HZ ALERT BLINK
// ======================================================
static void ledTick(uint32_t now) {
  const bool testActive = (g_ledTestUntil != 0) && ((int32_t)(now - g_ledTestUntil) < 0);
  uint8_t duty = 0;
  if (testActive) {
    duty = ((now / 250u) % 2u) ? g_cfg.ledBrightness : 0u;
  } else if (g_cfg.ledEnabled == 0 || g_cfg.ledMode == 2) {
    duty = 0;
  } else if (g_cfg.ledMode == 1) {
    duty = g_cfg.ledBrightness;
  } else {
    const bool alert = anyLedAlert(g_cfg);
    if (alert) {
      duty = (g_cfg.ledBlink != 0) ? (((now / 250u) % 2u) ? g_cfg.ledBrightness : 0u) : g_cfg.ledBrightness;
    } else {
      duty = 0;
    }
  }
  ledDuty(duty);
  g_ledOn = duty > 0;
}

// ======================================================
//
// OLED ENGINE - PAGES, IDLE SLEEP, NON-BLOCKING REDRAW
// ======================================================
static void oledWriteContrast() {
  if (g_oledSleeping || g_cfg.oledEnabled == 0) return;
  g_oled.ssd1306_command(SSD1306_SETCONTRAST);
  g_oled.ssd1306_command(g_cfg.oledContrast);
}

static void oledWake(uint32_t now) {
  g_oledLastActivityMs = now;
  if (g_oledSleeping) {
    g_oledSleeping = false;
    g_oled.dim(false);
    g_oledDirty = true;
  }
}

static uint8_t oledEnabledPageCount() {
  uint8_t count = 0;
  if (g_cfg.oledPageOverview) count++;
  if (g_cfg.oledPageStatus) count++;
  if (g_cfg.oledPageAlerts) count++;
  return count;
}

static uint8_t oledPageAt(uint8_t index) {
  uint8_t seen = 0;
  if (g_cfg.oledPageOverview) { if (seen == index) return 0; seen++; }
  if (g_cfg.oledPageStatus) { if (seen == index) return 1; seen++; }
  if (g_cfg.oledPageAlerts) { if (seen == index) return 2; seen++; }
  return 0;
}

static void oledDrawFrameHeader(const char* title, const char* right) {
  g_oled.fillRect(0, 0, 128, 11, SSD1306_WHITE);
  g_oled.setTextColor(SSD1306_BLACK);
  g_oled.setTextSize(1);
  g_oled.setCursor(2, 2);
  g_oled.print(title);
  const int16_t width = (int16_t)(strlen(right) * 6);
  g_oled.setCursor((int16_t)(126 - width), 2);
  g_oled.print(right);
  g_oled.setTextColor(SSD1306_WHITE);
}

static void oledDrawRelayDots(uint8_t y) {
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    const int16_t x = (int16_t)(2 + i * 14);
    if (g_relayState[i]) {
      g_oled.fillRect(x, y, 10, 6, SSD1306_WHITE);
    } else {
      g_oled.drawRect(x, y, 10, 6, SSD1306_WHITE);
    }
  }
}

static void oledRenderOverview() {
  oledDrawFrameHeader("SMART GARDEN", g_oledSleeping ? "" : "AP");
  g_oled.setTextSize(1);
  g_oled.setCursor(2, 15);
  if (g_bmeValid) {
    g_oled.printf("T:%.1fC H:%.0f%%", g_bmeT, g_bmeH);
  } else if (g_cfg.bmeEnabled) {
    g_oled.print("T:--- H:---");
  } else {
    g_oled.print("BME280 off");
  }
  g_oled.setCursor(2, 25);
  if (g_bmeValid) {
    g_oled.printf("P:%.0fhPa", g_bmeP);
  } else {
    g_oled.print("P:----");
  }
  g_oled.setCursor(70, 25);
  if (g_bhValid) {
    g_oled.printf("Lx:%.0f", g_bhLux);
  } else if (g_cfg.bhEnabled) {
    g_oled.print("Lx:---");
  } else {
    g_oled.print("Lx off");
  }
  g_oled.setCursor(2, 35);
  g_oled.printf("Gas:%.0f%% Soil:%.0f%%", g_mqValid ? g_mqPct : 0.0f, g_soilValid ? g_soilPct : 0.0f);
  g_oled.setCursor(2, 45);
  g_oled.printf("Rain:%.0f%%", g_rainValid ? g_rainPct : 0.0f);
  oledDrawRelayDots(55);
}

static void oledRenderStatus() {
  oledDrawFrameHeader("SYSTEM STATUS", "FW" FW_VERSION);
  g_oled.setTextSize(1);
  g_oled.setCursor(2, 15);
  g_oled.printf("Heap free: %u KB", (unsigned)(ESP.getFreeHeap() / 1024UL));
  g_oled.setCursor(2, 25);
  const size_t freeHeap = ESP.getFreeHeap();
  const size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  const uint32_t frag = freeHeap > 0 ? (uint32_t)(100u - (largest * 100u) / freeHeap) : 0u;
  g_oled.printf("Heap frag: %u%%", (unsigned)frag);
  g_oled.setCursor(2, 35);
  g_oled.printf("Uptime: %llud %02llu:%02llu", (unsigned long long)(uptimeSeconds() / 86400ULL), (unsigned long long)((uptimeSeconds() % 86400ULL) / 3600ULL), (unsigned long long)((uptimeSeconds() % 3600ULL) / 60ULL));
  g_oled.setCursor(2, 45);
  g_oled.printf("AP clients: %u", (unsigned)WiFi.softAPgetStationNum());
  g_oled.setCursor(2, 55);
  g_oled.printf("Cfg slot: %c CRC %s", g_cfgSource == 1 ? 'A' : (g_cfgSource == 2 ? 'B' : '-'), g_cfgSource != 0 ? "OK" : "DEFAULTS");
}

static void oledRenderAlerts() {
  oledDrawFrameHeader("ALERTS", alertBitsEffective(g_cfg) ? "!!" : "OK");
  g_oled.setTextSize(1);
  const uint8_t bits = alertBitsEffective(g_cfg);
  uint8_t y = 15;
  if (bits == 0) {
    g_oled.setCursor(2, 28);
    g_oled.print("ALL NOMINAL");
    return;
  }
  if ((bits & (1u << SRC_BIT_BME)) && y < 60) {
    g_oled.setCursor(2, y);
    if (g_cfg.bmeMetric == 1) g_oled.printf("BME hum %.0f%%", g_bmeH);
    else if (g_cfg.bmeMetric == 2) g_oled.printf("BME pres %.0f hPa", g_bmeP);
    else g_oled.printf("BME temp %.1fC", g_bmeT);
    y = (uint8_t)(y + 10);
  }
  if ((bits & (1u << SRC_BIT_BH)) && y < 60) {
    g_oled.setCursor(2, y);
    g_oled.printf("LUX %.0f lx", g_bhLux);
    y = (uint8_t)(y + 10);
  }
  if ((bits & (1u << SRC_BIT_MQ)) && y < 60) {
    g_oled.setCursor(2, y);
    g_oled.printf("GAS %.0f%%", g_mqPct);
    y = (uint8_t)(y + 10);
  }
  if ((bits & (1u << SRC_BIT_SOIL)) && y < 60) {
    g_oled.setCursor(2, y);
    g_oled.printf("SOIL %.0f%%", g_soilPct);
    y = (uint8_t)(y + 10);
  }
  if ((bits & (1u << SRC_BIT_RAIN)) && y < 60) {
    g_oled.setCursor(2, y);
    g_oled.printf("RAIN %.0f%%", g_rainPct);
    y = (uint8_t)(y + 10);
  }
}

static void oledRender(uint8_t page) {
  g_oled.clearDisplay();
  if (page == 0) oledRenderOverview();
  else if (page == 1) oledRenderStatus();
  else oledRenderAlerts();
  g_oled.display();
  g_oledLastDrawMs = millis();
  g_oledDirty = false;
}

static void oledTick(uint32_t now) {
  if (!g_oledReady) return;
  if (g_cfg.oledEnabled == 0) {
    if (!g_oledSleeping) {
      g_oled.clearDisplay();
      g_oled.display();
      g_oled.dim(true);
      g_oledSleeping = true;
    }
    return;
  }
  if (g_factoryHolding) return;
  const uint8_t alertBits = alertBitsEffective(g_cfg);
  if (alertBits != 0) oledWake(now);
  uint32_t timeoutS = OLED_TIMEOUT_S[clampu8(g_cfg.oledTimeoutIdx, 0, 4)];
  if (alertBits != 0) timeoutS = 0;
  if (timeoutS > 0 && (uint32_t)(now - g_oledLastActivityMs) > timeoutS * 1000UL) {
    if (!g_oledSleeping) {
      g_oledSleeping = true;
      g_oled.clearDisplay();
      g_oled.display();
      g_oled.dim(true);
    }
    return;
  }
  if (g_oledSleeping) return;
  const uint8_t pageCount = oledEnabledPageCount();
  if (pageCount == 0) {
    g_oled.clearDisplay();
    g_oled.setTextSize(1);
    g_oled.setCursor(2, 26);
    g_oled.print("No OLED pages enabled");
    g_oled.display();
    g_oledLastDrawMs = now;
    return;
  }
  const uint8_t pageIndex = (uint8_t)(g_oledPageIndex % pageCount);
  const uint8_t page = oledPageAt(pageIndex);
  const uint32_t cycleMs = (uint32_t)OLED_PAGE_CYCLE_S[clampu8(g_cfg.oledCycleIdx, 0, 3)] * 1000UL;
  const bool pageDue = (uint32_t)(now - g_oledPageSwitchMs) >= cycleMs;
  const bool secondTick = (uint32_t)(now - g_oledLastDrawMs) >= 1000UL;
  if (pageDue) {
    g_oledPageSwitchMs = now;
    g_oledPageIndex = (uint8_t)((g_oledPageIndex + 1u) % pageCount);
    oledRender(oledPageAt((uint8_t)(g_oledPageIndex % pageCount)));
    return;
  }
  if (g_oledDirty || secondTick) {
    oledRender(page);
  }
}

// ======================================================
//
// GPIO0 BOOT BUTTON - 5 SECOND FACTORY RESET HOLD
// ======================================================
static void factoryWipeAndRestart() {
  Preferences prefs;
  if (prefs.begin(NVS_NAMESPACE, false)) {
    prefs.clear();
    prefs.end();
  }
  buzzerAttach();
  buzzerDuty(g_cfg.buzzVolume);
  g_pendingRestartMs = millis() + 1500UL;
}

static void drawFactoryHoldProgress(uint32_t now, uint32_t heldMs) {
  const uint32_t capped = heldMs > 5000u ? 5000u : heldMs;
  const uint8_t pct = (uint8_t)((capped * 100UL) / 5000UL);
  g_oled.clearDisplay();
  g_oled.setTextColor(SSD1306_WHITE);
  g_oled.setTextSize(1);
  g_oled.setCursor(2, 8);
  g_oled.print("FACTORY RESET");
  g_oled.setCursor(2, 24);
  g_oled.print("Keep holding BOOT...");
  g_oled.drawRect(2, 40, 124, 12, SSD1306_WHITE);
  g_oled.fillRect(4, 42, (int16_t)(120UL * pct / 100UL), 8, SSD1306_WHITE);
  g_oled.display();
  g_oledLastDrawMs = now;
}

static void buttonTick(uint32_t now) {
  const bool low = digitalRead(PIN_BUTTON_GPIO0) == LOW;
  if (low) {
    g_btnLastLowMs = now;
    g_factoryHolding = true;
    if (g_btnDownSince == 0) g_btnDownSince = now;
    const uint32_t held = (uint32_t)(now - g_btnDownSince);
    if (g_oledReady && g_cfg.oledEnabled != 0) {
      oledWake(now);
      if ((now / 120u) != ((now - 33u) / 120u)) {
        drawFactoryHoldProgress(now, held);
      }
    }
    if (held >= 5000UL && !g_factoryWipeArmed) {
      g_factoryWipeArmed = true;
      factoryWipeAndRestart();
    }
    return;
  }
  if ((uint32_t)(now - g_btnLastLowMs) < 160UL && g_btnDownSince != 0) {
    return;
  }
  if (g_btnDownSince != 0 || g_factoryHolding) {
    g_btnDownSince = 0;
    g_factoryHolding = false;
    g_oledDirty = true;
  }
}

// ======================================================
//
// EMBEDDED WEB APPLICATION (PROGMEM, SINGLE PAGE APP)
// ======================================================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<meta name="theme-color" content="#070d0b">
<meta name="description" content="ESP32 Multi Smart Garden — unified production firmware dashboard and settings for SSD1306 OLED, BME280, BH1750, MQ-2, soil moisture and rain sensors with relay matrix, buzzer, LED and captive portal.">
<title>ESP32 Multi Smart Garden</title>
<link rel="icon" href="data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 64 64'%3E%3Crect width='64' height='64' rx='15' fill='%23071009'/%3E%3Cpath d='M49 15C31 17 19 28 17 45c17 2 30-9 32-30Z' fill='%2334d399'/%3E%3Cpath d='M21 43c8-10 16-17 25-22' stroke='%23052e1f' stroke-width='2.6' fill='none' stroke-linecap='round'/%3E%3C/svg%3E">
<style>
:root{
  --bg:#070d0b;
  --bg-2:#0b1310;
  --panel:#0e1815;
  --panel-2:#101d19;
  --panel-3:#132019;
  --card-border:rgba(148,233,190,.10);
  --card-border-strong:rgba(148,233,190,.20);
  --text:#e7f5ec;
  --text-dim:#8fb3a1;
  --text-faint:#5b7a6b;
  --accent:#34d399;
  --accent-2:#22d3ee;
  --accent-deep:#0f3d2b;
  --amber:#fbbf24;
  --red:#f87171;
  --red-deep:#451515;
  --green:#34d399;
  --mono:"JetBrains Mono","SFMono-Regular",ui-monospace,Menlo,Consolas,monospace;
  --sans:"Inter",-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif;
  --radius:16px;
  --radius-sm:10px;
  --shadow:0 18px 50px rgba(0,0,0,.45);
  --sidebar-w:264px;
}
*{margin:0;padding:0;box-sizing:border-box}
html{scroll-behavior:smooth}
body{
  background:var(--bg);
  color:var(--text);
  font-family:var(--sans);
  font-size:14px;
  line-height:1.5;
  min-height:100vh;
  overflow-x:hidden;
  -webkit-font-smoothing:antialiased;
}
.bg-fx{position:fixed;inset:0;z-index:0;pointer-events:none;
  background:
    radial-gradient(1000px 600px at 80% -10%, rgba(52,211,153,.10), transparent 60%),
    radial-gradient(900px 700px at -20% 110%, rgba(34,211,238,.08), transparent 60%),
    radial-gradient(600px 500px at 55% 50%, rgba(52,211,153,.04), transparent 70%);
}
.bg-fx::after{content:"";position:absolute;inset:0;opacity:.35;
  background-image:
    linear-gradient(rgba(148,233,190,.028) 1px, transparent 1px),
    linear-gradient(90deg, rgba(148,233,190,.028) 1px, transparent 1px);
  background-size:44px 44px;
  mask-image:radial-gradient(ellipse 90% 80% at 50% 0%, black 40%, transparent 75%);
  -webkit-mask-image:radial-gradient(ellipse 90% 80% at 50% 0%, black 40%, transparent 75%);
}
:focus-visible{outline:2px solid var(--accent-2);outline-offset:2px;border-radius:4px}
::selection{background:rgba(52,211,153,.28)}
svg{display:block}
button{font-family:var(--sans);cursor:pointer}
.mono{font-family:var(--mono)}

/* ============ SIDEBAR ============ */
.sidebar{
  position:fixed;left:0;top:0;bottom:0;width:var(--sidebar-w);z-index:40;
  background:linear-gradient(180deg, rgba(13,23,19,.92), rgba(9,16,13,.94));
  backdrop-filter:blur(18px);-webkit-backdrop-filter:blur(18px);
  border-right:1px solid var(--card-border);
  display:flex;flex-direction:column;
  transform:translateX(0);transition:transform .32s cubic-bezier(.32,.72,.28,1);
}
.brand{display:flex;align-items:center;gap:12px;padding:20px 20px 18px;border-bottom:1px solid var(--card-border)}
.brand-mark{width:44px;height:44px;flex:0 0 44px;border-radius:13px;background:linear-gradient(145deg,#10b981,#0d9488 60%,#155e4b);display:flex;align-items:center;justify-content:center;box-shadow:0 6px 18px rgba(16,185,129,.35), inset 0 1px 0 rgba(255,255,255,.35)}
.brand-mark svg{width:24px;height:24px}
.brand-text{display:flex;flex-direction:column;line-height:1.15;min-width:0}
.brand-text strong{font-size:15px;letter-spacing:.06em;font-weight:800;background:linear-gradient(90deg,#a7f3d0,#5eead4);-webkit-background-clip:text;background-clip:text;color:transparent}
.brand-text span{font-size:9.5px;letter-spacing:.22em;text-transform:uppercase;color:var(--text-dim);font-weight:600;margin-top:3px}
.nav{flex:1;overflow-y:auto;padding:14px 12px 10px;scrollbar-width:thin;scrollbar-color:#1f3a2e transparent}
.nav::-webkit-scrollbar{width:5px}
.nav::-webkit-scrollbar-thumb{background:#1f3a2e;border-radius:99px}
.nav-label{font-size:9.5px;letter-spacing:.24em;text-transform:uppercase;color:var(--text-faint);font-weight:700;padding:14px 10px 6px}
.nav-item{
  display:flex;align-items:center;gap:11px;width:100%;padding:9px 11px;margin:2px 0;
  background:transparent;border:1px solid transparent;border-radius:var(--radius-sm);
  color:var(--text-dim);font-size:13px;font-weight:600;text-align:left;position:relative;
  transition:background .18s,color .18s,border-color .18s;
}
.nav-item svg{width:18px;height:18px;flex:0 0 18px;stroke:currentColor;fill:none;stroke-width:1.7;stroke-linecap:round;stroke-linejoin:round;opacity:.85}
.nav-item:hover{background:rgba(52,211,153,.06);color:var(--text)}
.nav-item.active{background:linear-gradient(90deg,rgba(52,211,153,.14),rgba(34,211,238,.07));border-color:var(--card-border-strong);color:#a7f3d0}
.nav-item.active::before{content:"";position:absolute;left:-12px;top:20%;bottom:20%;width:3px;border-radius:0 3px 3px 0;background:linear-gradient(180deg,var(--accent),var(--accent-2))}
.nav-item .nav-pin{margin-left:auto;font-family:var(--mono);font-size:9px;color:var(--text-faint);background:rgba(148,233,190,.06);border:1px solid var(--card-border);padding:2px 6px;border-radius:6px}
.nav-item.active .nav-pin{color:#6ee7b7;border-color:rgba(52,211,153,.35)}
.side-foot{border-top:1px solid var(--card-border);padding:13px 18px;display:flex;align-items:center;gap:9px;font-family:var(--mono);font-size:10px;color:var(--text-faint)}
.pulse-dot{width:8px;height:8px;border-radius:99px;background:var(--green);box-shadow:0 0 0 0 rgba(52,211,153,.6);animation:pulse 2.2s infinite}
@keyframes pulse{0%{box-shadow:0 0 0 0 rgba(52,211,153,.55)}70%{box-shadow:0 0 0 8px rgba(52,211,153,0)}100%{box-shadow:0 0 0 0 rgba(52,211,153,0)}}

.scrim{position:fixed;inset:0;z-index:35;background:rgba(3,8,6,.66);backdrop-filter:blur(3px);-webkit-backdrop-filter:blur(3px);opacity:0;pointer-events:none;transition:opacity .28s}
.scrim.show{opacity:1;pointer-events:auto}

/* ============ MAIN / TOPBAR ============ */
.content{position:relative;z-index:1;margin-left:var(--sidebar-w);min-height:100vh;display:flex;flex-direction:column}
.topbar{
  position:sticky;top:0;z-index:30;display:flex;align-items:center;gap:14px;
  padding:12px 26px;border-bottom:1px solid var(--card-border);
  background:rgba(9,15,12,.78);backdrop-filter:blur(16px);-webkit-backdrop-filter:blur(16px);
}
.icon-btn{width:38px;height:38px;border-radius:11px;border:1px solid var(--card-border);background:var(--panel-2);color:var(--text-dim);display:flex;align-items:center;justify-content:center;transition:color .2s,border-color .2s}
.icon-btn:hover{color:var(--accent);border-color:var(--card-border-strong)}
.hamburger{display:none}
.crumb{display:flex;align-items:center;gap:10px;min-width:0;font-size:12px}
.crumb-root{font-family:var(--mono);font-size:10px;letter-spacing:.2em;color:var(--text-faint);font-weight:700}
.crumb-sep{color:var(--text-faint)}
#crumbPage{font-weight:800;color:var(--text);letter-spacing:.02em;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.top-right{margin-left:auto;display:flex;align-items:center;gap:10px}
.chip{display:inline-flex;align-items:center;gap:8px;height:32px;padding:0 13px;border-radius:99px;border:1px solid var(--card-border);background:var(--panel-2);font-size:11px;font-weight:600;color:var(--text-dim);white-space:nowrap}
.chip-live{color:#7ff0bb;border-color:rgba(52,211,153,.3)}
.live-dot{width:7px;height:7px;border-radius:99px;background:var(--green);animation:pulse 2s infinite}
.chip-live.lost{color:#fda4a4;border-color:rgba(248,113,113,.35)}
.chip-live.lost .live-dot{background:var(--red);animation:none}
.chip-mono{font-family:var(--mono);font-size:11px;letter-spacing:.05em}
.page-wrap{flex:1;padding:22px 26px 30px;max-width:1560px;width:100%;margin:0 auto}
.foot{border-top:1px solid var(--card-border);padding:14px 26px;font-family:var(--mono);font-size:9.5px;letter-spacing:.14em;color:var(--text-faint);text-transform:uppercase;display:flex;justify-content:space-between;gap:10px;flex-wrap:wrap}

/* ============ PANELS ============ */
.panel{display:none;animation:panelIn .38s ease}
.panel.active{display:block}
@keyframes panelIn{from{opacity:0;transform:translateY(10px)}to{opacity:1;transform:none}}
.page-head{display:flex;align-items:flex-end;justify-content:space-between;gap:18px;margin:4px 2px 18px;flex-wrap:wrap}
.page-head h2{font-size:clamp(19px,2.4vw,25px);font-weight:800;letter-spacing:-.01em;background:linear-gradient(180deg,#f2fdf7,#b9e6cd);-webkit-background-clip:text;background-clip:text;color:transparent}
.page-head p{color:var(--text-dim);font-size:12px;margin-top:5px;max-width:640px}
.page-head p code{font-family:var(--mono);color:#6ee7b7;background:rgba(52,211,153,.08);border:1px solid rgba(52,211,153,.18);border-radius:6px;padding:1px 6px;font-size:10.5px}

/* ============ CARDS ============ */
.card{
  background:linear-gradient(180deg,var(--panel-2),var(--panel));
  border:1px solid var(--card-border);border-radius:var(--radius);
  box-shadow:var(--shadow), inset 0 1px 0 rgba(255,255,255,.03);
  padding:18px;position:relative;overflow:hidden;
}
.card::before{content:"";position:absolute;top:0;left:0;right:0;height:1px;background:linear-gradient(90deg,transparent,rgba(167,243,208,.35),transparent)}
.card-title{font-size:11px;letter-spacing:.2em;text-transform:uppercase;font-weight:800;color:#8ff0bb;display:flex;align-items:center;justify-content:space-between;gap:10px;margin-bottom:14px;padding-bottom:10px;border-bottom:1px dashed rgba(148,233,190,.14)}
.card-title small{font-size:9px;letter-spacing:.08em;color:var(--text-faint);text-transform:none;font-weight:600}

.grid{display:grid;gap:16px}
.grid-sensors{grid-template-columns:repeat(auto-fill,minmax(255px,1fr))}
.grid-2{grid-template-columns:minmax(0,1.6fr) minmax(0,1fr)}
.grid-3{grid-template-columns:repeat(3,minmax(0,1fr))}

/* ============ BADGES ============ */
.badge{display:inline-flex;align-items:center;gap:6px;height:24px;padding:0 11px;border-radius:99px;font-size:9.5px;letter-spacing:.14em;font-weight:800;text-transform:uppercase;border:1px solid;white-space:nowrap;transition:all .25s}
.badge::before{content:"";width:6px;height:6px;border-radius:99px;background:currentColor}
.badge.ok{color:#5eead4;border-color:rgba(52,211,153,.4);background:rgba(52,211,153,.09)}
.badge.trig{color:#fcd34d;border-color:rgba(251,191,36,.45);background:rgba(251,191,36,.10);animation:badgeBlink 1s steps(2) infinite}
.badge.err{color:#fda4a4;border-color:rgba(248,113,113,.45);background:rgba(248,113,113,.10)}
.badge.off{color:#94a3b8;border-color:rgba(148,163,184,.3);background:rgba(148,163,184,.07)}
.badge.off::before{background:#64748b}
@keyframes badgeBlink{50%{box-shadow:0 0 12px rgba(251,191,36,.35)}}
.badge.big{height:30px;font-size:10.5px;padding:0 16px}

/* ============ SENSOR CARDS ============ */
.sensor-card{display:flex;flex-direction:column;gap:12px}
.card-head{display:flex;align-items:center;gap:11px}
.sensor-ico{width:42px;height:42px;flex:0 0 42px;border-radius:12px;display:flex;align-items:center;justify-content:center;border:1px solid}
.sensor-ico svg{width:21px;height:21px;stroke-width:1.7;fill:none;stroke-linecap:round;stroke-linejoin:round}
.sensor-ico.bme{background:rgba(56,189,248,.09);border-color:rgba(56,189,248,.3)}.sensor-ico.bme svg{stroke:#7dd3fc}
.sensor-ico.bh{background:rgba(253,224,71,.08);border-color:rgba(253,224,71,.3)}.sensor-ico.bh svg{stroke:#fde047}
.sensor-ico.mq{background:rgba(251,146,60,.09);border-color:rgba(251,146,60,.3)}.sensor-ico.mq svg{stroke:#fdba74}
.sensor-ico.soil{background:rgba(74,222,128,.08);border-color:rgba(74,222,128,.3)}.sensor-ico.soil svg{stroke:#86efac}
.sensor-ico.rain{background:rgba(147,197,253,.09);border-color:rgba(147,197,253,.3)}.sensor-ico.rain svg{stroke:#93c5fd}
.card-head h3{font-size:13px;font-weight:800;letter-spacing:.04em}
.card-head p{font-size:9.5px;color:var(--text-faint);letter-spacing:.09em;text-transform:uppercase;font-weight:600;margin-top:2px}
.card-head .badge{margin-left:auto}
.big-read{display:flex;align-items:baseline;gap:8px;padding:2px 0 0}
.big-val{font-family:var(--mono);font-size:clamp(30px,3vw,36px);font-weight:700;letter-spacing:-.02em;color:#ecfdf5;text-shadow:0 0 26px rgba(52,211,153,.25)}
.big-unit{font-family:var(--mono);font-size:13px;color:var(--accent);font-weight:600}
.mini-stats{display:flex;gap:8px}
.mini-stats>div{flex:1;background:rgba(9,16,13,.55);border:1px solid var(--card-border);border-radius:var(--radius-sm);padding:7px 10px;min-width:0}
.mini-stats label{display:block;font-size:8.5px;letter-spacing:.16em;text-transform:uppercase;color:var(--text-faint);font-weight:700;margin-bottom:3px}
.mini-stats b{font-family:var(--mono);font-size:13px;font-weight:600;color:#d1fae5;white-space:nowrap}
.meter{height:6px;border-radius:99px;background:#122019;overflow:hidden;border:1px solid rgba(148,233,190,.08)}
.meter>i{display:block;height:100%;border-radius:99px;background:linear-gradient(90deg,#10b981,#22d3ee);width:0%;transition:width .6s ease}
.meter.warn>i{background:linear-gradient(90deg,#f59e0b,#fbbf24)}
.meter.danger>i{background:linear-gradient(90deg,#ef4444,#f87171)}
.spark{width:100%;height:46px}
.card-foot{display:flex;justify-content:space-between;gap:8px;font-family:var(--mono);font-size:9px;color:var(--text-faint);border-top:1px dashed rgba(148,233,190,.12);padding-top:10px;letter-spacing:.06em}

/* ============ ALERT BANNER ============ */
.alert-banner{display:none;align-items:center;gap:13px;margin-bottom:16px;padding:12px 16px;border-radius:var(--radius);border:1px solid rgba(251,191,36,.4);background:linear-gradient(90deg,rgba(251,191,36,.13),rgba(248,113,113,.08));box-shadow:0 0 34px rgba(251,191,36,.09)}
.alert-banner.show{display:flex;animation:panelIn .3s ease}
.alert-banner svg{width:26px;height:26px;flex:0 0 26px;stroke:#fcd34d;fill:none;stroke-width:1.8;stroke-linecap:round;stroke-linejoin:round}
.alert-banner strong{display:block;font-size:12px;letter-spacing:.12em;text-transform:uppercase;color:#fde68a}
.alert-banner span{font-size:12px;color:#fef3c7;font-family:var(--mono)}

/* ============ OUTPUTS MATRIX ============ */
.relay-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:10px}
.relay-chip{background:rgba(9,16,13,.55);border:1px solid var(--card-border);border-radius:var(--radius-sm);padding:11px 12px;transition:border-color .25s, box-shadow .25s}
.relay-chip .rc-top{display:flex;justify-content:space-between;align-items:baseline;gap:6px}
.rc-name{font-size:10px;font-weight:800;letter-spacing:.1em;color:var(--text-dim)}
.rc-pin{font-family:var(--mono);font-size:8.5px;color:var(--text-faint)}
.rc-bottom{display:flex;justify-content:space-between;align-items:center;margin-top:9px}
.rc-mode{font-family:var(--mono);font-size:8.5px;letter-spacing:.12em;color:var(--text-faint)}
.rc-state{font-family:var(--mono);font-size:10.5px;font-weight:700;letter-spacing:.1em;color:#64748b;padding:2px 9px;border-radius:6px;border:1px solid rgba(148,163,184,.2)}
.relay-chip.on{border-color:rgba(52,211,153,.45);box-shadow:0 0 22px rgba(52,211,153,.15), inset 0 0 18px rgba(52,211,153,.06)}
.relay-chip.on .rc-state{color:#04150c;background:var(--green);border-color:var(--green);box-shadow:0 0 14px rgba(52,211,153,.5)}
.relay-chip.on .rc-name{color:#a7f3d0}
.output-duo{display:flex;gap:10px;margin-top:12px;flex-wrap:wrap}
.out-pill{display:flex;align-items:center;gap:10px;flex:1;min-width:150px;background:rgba(9,16,13,.55);border:1px solid var(--card-border);border-radius:var(--radius-sm);padding:10px 13px}
.out-pill .out-ico{width:34px;height:34px;border-radius:10px;display:flex;align-items:center;justify-content:center;background:rgba(52,211,153,.08);border:1px solid rgba(52,211,153,.22)}
.out-pill .out-ico svg{width:17px;height:17px;stroke:#6ee7b7;fill:none;stroke-width:1.8;stroke-linecap:round;stroke-linejoin:round}
.out-pill small{display:block;font-size:8.5px;letter-spacing:.18em;text-transform:uppercase;color:var(--text-faint);font-weight:700}
.out-pill b{font-family:var(--mono);font-size:12px;color:#d1fae5}

/* ============ SYSTEM SNAPSHOT ============ */
.kv{display:flex;flex-direction:column}
.kv-row{display:flex;justify-content:space-between;align-items:center;gap:14px;padding:9px 2px;border-bottom:1px dashed rgba(148,233,190,.1)}
.kv-row:last-child{border-bottom:none}
.kv-row span{font-size:11px;color:var(--text-dim);letter-spacing:.05em}
.kv-row b{font-family:var(--mono);font-size:12px;color:#ecfdf5;font-weight:600;text-align:right}
.good{color:#5eead4!important}

/* ============ SETTINGS FORMS ============ */
.settings-grid{display:grid;grid-template-columns:minmax(0,1.75fr) minmax(280px,1fr);gap:16px;align-items:start}
.form-card{display:flex;flex-direction:column}
.field-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:13px}
.field{display:flex;flex-direction:column;gap:6px;min-width:0}
.field.full{grid-column:1/-1}
.field label{font-size:10px;letter-spacing:.16em;text-transform:uppercase;color:var(--text-dim);font-weight:700}
.field .hint{font-size:10px;color:var(--text-faint)}
.text-input, select.text-input{
  height:40px;padding:0 13px;border-radius:var(--radius-sm);border:1px solid var(--card-border);
  background:#0a120e;color:var(--text);font-family:var(--mono);font-size:13px;width:100%;
  transition:border-color .2s, box-shadow .2s;appearance:none;
}
select.text-input{background-image:url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='12' height='8' viewBox='0 0 12 8'%3E%3Cpath d='M1 1.5 6 6.5 11 1.5' stroke='%238fb3a1' stroke-width='1.8' fill='none' stroke-linecap='round'/%3E%3C/svg%3E");background-repeat:no-repeat;background-position:right 13px center;padding-right:36px}
select.text-input option{background:#0e1815;color:var(--text)}
.text-input:focus{outline:none;border-color:rgba(52,211,153,.5);box-shadow:0 0 0 3px rgba(52,211,153,.12)}
.input-unit{position:relative}
.input-unit .text-input{padding-right:52px}
.input-unit em{position:absolute;right:14px;top:50%;transform:translateY(-50%);font-style:normal;font-family:var(--mono);font-size:11px;color:var(--text-faint);pointer-events:none}
.section-sep{display:flex;align-items:center;gap:10px;margin:18px 0 13px}
.section-sep span{font-size:10px;letter-spacing:.22em;text-transform:uppercase;color:#6ee7b7;font-weight:800;white-space:nowrap}
.section-sep::after{content:"";flex:1;height:1px;background:linear-gradient(90deg,rgba(110,231,183,.35),transparent)}
.toggle-row{display:flex;align-items:center;justify-content:space-between;gap:14px;padding:11px 13px;border:1px solid var(--card-border);background:rgba(9,16,13,.4);border-radius:var(--radius-sm)}
.toggle-row .tr-text{min-width:0}
.toggle-row .tr-text b{font-size:12.5px;font-weight:700;display:block}
.toggle-row .tr-text small{font-size:10.5px;color:var(--text-faint);display:block;margin-top:2px}
.switch{position:relative;display:inline-block;width:46px;height:25px;flex:0 0 46px}
.switch input{opacity:0;width:0;height:0}
.slider-ui{position:absolute;cursor:pointer;inset:0;border-radius:99px;background:#16241d;border:1px solid var(--card-border-strong);transition:.25s}
.slider-ui::before{content:"";position:absolute;height:17px;width:17px;left:3px;top:3px;border-radius:99px;background:#7d938a;transition:.25s}
.switch input:checked + .slider-ui{background:linear-gradient(90deg,#059669,#0d9488);border-color:rgba(52,211,153,.6);box-shadow:0 0 14px rgba(52,211,153,.35)}
.switch input:checked + .slider-ui::before{transform:translateX(21px);background:#ecfdf5}
.check-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:9px}
.check-pill{position:relative}
.check-pill input{position:absolute;opacity:0;pointer-events:none}
.check-pill label{display:flex;align-items:center;gap:9px;padding:10px 12px;border-radius:var(--radius-sm);border:1px solid var(--card-border);background:rgba(9,16,13,.4);font-size:11.5px;font-weight:600;color:var(--text-dim);cursor:pointer;transition:all .2s;user-select:none}
.check-pill label .box{width:17px;height:17px;flex:0 0 17px;border-radius:5px;border:1.5px solid #2b463a;display:flex;align-items:center;justify-content:center;transition:all .2s;background:#0a120e}
.check-pill label .box svg{width:11px;height:11px;stroke:#04150c;stroke-width:3;fill:none;opacity:0;transform:scale(.4);transition:all .2s}
.check-pill input:checked + label{border-color:rgba(52,211,153,.5);background:rgba(52,211,153,.09);color:#a7f3d0}
.check-pill input:checked + label .box{background:var(--green);border-color:var(--green)}
.check-pill input:checked + label .box svg{opacity:1;transform:scale(1)}
.form-actions{display:flex;align-items:center;gap:14px;margin-top:18px;padding-top:16px;border-top:1px solid var(--card-border);flex-wrap:wrap}
.btn{display:inline-flex;align-items:center;gap:9px;height:42px;padding:0 20px;border-radius:var(--radius-sm);border:1px solid transparent;font-size:12px;font-weight:800;letter-spacing:.09em;text-transform:uppercase;transition:transform .15s, filter .2s, background .2s}
.btn:active{transform:scale(.97)}
.btn:disabled{opacity:.55;cursor:not-allowed}
.btn-primary{background:linear-gradient(135deg,#10b981,#0d9488);color:#03120b;box-shadow:0 8px 24px rgba(16,185,129,.35), inset 0 1px 0 rgba(255,255,255,.4)}
.btn-primary:hover{filter:brightness(1.1)}
.btn-ghost{background:transparent;border-color:var(--card-border-strong);color:#9fe8c3}
.btn-ghost:hover{background:rgba(52,211,153,.08)}
.btn-danger{background:linear-gradient(135deg,#ef4444,#b91c1c);color:#2a0505;box-shadow:0 8px 24px rgba(239,68,68,.3)}
.btn-danger:hover{filter:brightness(1.12)}
.btn-danger-ghost{background:transparent;border-color:rgba(248,113,113,.4);color:#fca5a5}
.btn-danger-ghost:hover{background:rgba(248,113,113,.09)}
.btn-sm{height:34px;padding:0 14px;font-size:10.5px}
.save-hint{font-size:10.5px;color:var(--text-faint);font-family:var(--mono)}
.test-out{font-family:var(--mono);font-size:11px;color:#6ee7b7;min-height:16px;margin-top:8px}
.test-out.bad{color:#fda4a4}

/* range sliders */
.range-wrap{display:flex;align-items:center;gap:12px}
.range-wrap input[type=range]{flex:1;appearance:none;-webkit-appearance:none;height:6px;border-radius:99px;background:#16241d;outline-offset:4px}
.range-wrap input[type=range]::-webkit-slider-thumb{appearance:none;-webkit-appearance:none;width:19px;height:19px;border-radius:99px;background:#ecfdf5;border:2px solid #10b981;box-shadow:0 0 10px rgba(52,211,153,.5);cursor:pointer}
.range-wrap input[type=range]::-moz-range-thumb{width:19px;height:19px;border-radius:99px;background:#ecfdf5;border:2px solid #10b981;cursor:pointer}
.range-val{font-family:var(--mono);font-size:12px;color:#a7f3d0;min-width:38px;text-align:right}

/* live side column */
.side-col{display:flex;flex-direction:column;gap:16px}
.live-card .big-val{font-size:clamp(34px,4vw,44px)}
.live-rows{display:flex;flex-direction:column;gap:0;margin-top:10px}
.live-row{display:flex;justify-content:space-between;align-items:center;padding:8px 2px;border-bottom:1px dashed rgba(148,233,190,.1);font-size:12px}
.live-row:last-child{border:none}
.live-row span{color:var(--text-dim)}
.live-row b{font-family:var(--mono);color:#ecfdf5;font-weight:600}
.note-card{border-style:dashed;border-color:rgba(148,233,190,.18);background:rgba(15,32,25,.5)}
.note-card h4{font-size:10px;letter-spacing:.2em;text-transform:uppercase;color:#fcd34d;margin-bottom:8px;display:flex;gap:8px;align-items:center}
.note-card p{font-size:11.5px;color:var(--text-dim);line-height:1.6}
.note-card code{font-family:var(--mono);color:#6ee7b7;font-size:10.5px}

/* ============ RELAY MATRIX TABLE ============ */
.table-wrap{overflow-x:auto;margin:0 -6px;padding:0 6px}
table.matrix{width:100%;border-collapse:collapse;min-width:980px}
.matrix th{font-size:9px;letter-spacing:.2em;text-transform:uppercase;color:var(--text-faint);text-align:left;padding:8px 10px;border-bottom:2px solid var(--card-border-strong)}
.matrix td{padding:10px;border-bottom:1px solid var(--card-border);vertical-align:middle;font-size:12px}
.matrix tr:last-child td{border-bottom:none}
.matrix tr.unified td{background:linear-gradient(90deg,rgba(52,211,153,.07),transparent)}
.relay-name-cell b{display:block;font-size:12px;letter-spacing:.03em}
.relay-name-cell small{font-family:var(--mono);font-size:9px;color:var(--text-faint)}
select.mini{height:32px;font-size:11.5px}
.src-chips{display:flex;gap:4px;flex-wrap:wrap}
.src-chip{position:relative}
.src-chip input{position:absolute;opacity:0;pointer-events:none}
.src-chip label{font-family:var(--mono);font-size:8.5px;font-weight:700;letter-spacing:.08em;padding:4px 8px;border-radius:6px;border:1px solid #234033;color:#55705f;background:#0c1511;cursor:pointer;transition:all .18s;user-select:none}
.src-chip input:checked + label{color:#03150c;background:var(--accent);border-color:var(--accent);box-shadow:0 0 10px rgba(52,211,153,.4)}
.state-pill{display:inline-flex;min-width:52px;justify-content:center;font-family:var(--mono);font-size:10px;font-weight:700;letter-spacing:.12em;padding:4px 10px;border-radius:6px;border:1px solid rgba(148,163,184,.25);color:#7286a0}
.state-pill.on{color:#04150c;background:var(--green);border-color:var(--green);box-shadow:0 0 12px rgba(52,211,153,.45)}
.matrix-actions{display:flex;justify-content:space-between;align-items:center;gap:12px;margin-top:16px;flex-wrap:wrap}
.legend{display:flex;gap:14px;flex-wrap:wrap;margin-top:14px;padding-top:12px;border-top:1px dashed rgba(148,233,190,.12);font-size:9.5px;color:var(--text-faint);font-family:var(--mono)}

/* ============ SYSTEM STATUS ============ */
.status-grid{display:grid;grid-template-columns:minmax(0,1.5fr) minmax(280px,1fr);gap:16px;align-items:start}
.health-list{display:flex;flex-direction:column;gap:9px}
.health-item{display:flex;align-items:center;gap:11px;padding:9px 12px;border:1px solid var(--card-border);border-radius:var(--radius-sm);background:rgba(9,16,13,.4);font-size:12px}
.health-item svg{width:17px;height:17px;flex:0 0 17px}
.health-item.ok svg{stroke:#34d399;fill:none;stroke-width:2.4;stroke-linecap:round;stroke-linejoin:round}
.health-item.bad svg{stroke:#f87171;fill:none;stroke-width:2.4;stroke-linecap:round;stroke-linejoin:round}
.health-item.warn svg{stroke:#fbbf24;fill:none;stroke-width:2.4;stroke-linecap:round;stroke-linejoin:round}
.health-item span{color:var(--text-dim)}
.health-item b{margin-left:auto;font-family:var(--mono);font-size:10.5px;color:var(--text)}
.factory-box{border:1px solid rgba(248,113,113,.35);background:var(--red-deep);border-radius:var(--radius);padding:18px}
.factory-box h4{color:#fecaca;font-size:11px;letter-spacing:.2em;text-transform:uppercase;margin-bottom:8px}
.factory-box p{color:#fca5a5;font-size:11.5px;line-height:1.6;margin-bottom:12px}

/* ============ WIFI AP PAGE ============ */
.wifi-grid{display:grid;grid-template-columns:minmax(0,1.5fr) minmax(280px,1fr);gap:16px;align-items:start}
.pass-wrap{position:relative}
.pass-wrap .text-input{padding-right:84px}
.pass-toggle{position:absolute;right:8px;top:50%;transform:translateY(-50%);height:26px;padding:0 12px;border-radius:7px;border:1px solid var(--card-border-strong);background:transparent;color:#9fe8c3;font-family:var(--mono);font-size:9.5px;letter-spacing:.1em;font-weight:700}
.strength{height:5px;border-radius:99px;background:#16241d;overflow:hidden;margin-top:8px}
.strength>i{display:block;height:100%;width:0%;background:var(--red);transition:all .3s}
.inline-err{font-size:10.5px;color:#fda4a4;min-height:15px;margin-top:5px;font-family:var(--mono)}

/* ============ OLED PREVIEW ============ */
.oled-bezel{background:#02070a;border:1px solid #1e3a34;border-radius:14px;padding:12px;display:flex;flex-direction:column;align-items:center;gap:10px;box-shadow:inset 0 0 30px #000}
.oled-bezel canvas{width:100%;max-width:296px;aspect-ratio:2/1;image-rendering:pixelated;background:#010409;border-radius:4px}
.oled-bezel small{font-family:var(--mono);font-size:8.5px;letter-spacing:.2em;color:#3f5a4f;text-transform:uppercase}

/* ============ MODALS & TOAST ============ */
.modal-overlay{position:fixed;inset:0;z-index:60;background:rgba(2,6,5,.72);backdrop-filter:blur(6px);-webkit-backdrop-filter:blur(6px);display:flex;align-items:center;justify-content:center;padding:20px;opacity:0;pointer-events:none;transition:opacity .25s}
.modal-overlay.show{opacity:1;pointer-events:auto}
.modal{width:min(480px,100%);background:linear-gradient(180deg,#141f1a,#0d1512);border:1px solid var(--card-border-strong);border-radius:var(--radius);padding:22px;box-shadow:0 30px 80px rgba(0,0,0,.6);transform:translateY(14px) scale(.97);transition:transform .25s}
.modal-overlay.show .modal{transform:none}
.modal h3{font-size:15px;letter-spacing:.05em;margin-bottom:8px;display:flex;align-items:center;gap:10px}
.modal p{font-size:12px;color:var(--text-dim);line-height:1.6}
.modal-actions{display:flex;justify-content:flex-end;gap:10px;margin-top:18px}
.toast{
  position:fixed;bottom:26px;left:50%;transform:translate(-50%,18px);z-index:70;
  background:#101d18;border:1px solid rgba(52,211,153,.45);color:#a7f3d0;
  font-family:var(--mono);font-size:11.5px;padding:11px 20px;border-radius:12px;
  box-shadow:0 14px 44px rgba(0,0,0,.5),0 0 24px rgba(52,211,153,.14);
  opacity:0;pointer-events:none;transition:all .3s;max-width:min(480px,92vw);text-align:center}
.toast.show{opacity:1;transform:translate(-50%,0)}
.toast.bad{border-color:rgba(248,113,113,.5);color:#fca5a5}
.spin{animation:spin 1s linear infinite}@keyframes spin{to{transform:rotate(360deg)}}

/* ============ RESPONSIVE ============ */
@media (max-width:1180px){
  .grid-2{grid-template-columns:1fr}
  .settings-grid{grid-template-columns:1fr}
  .status-grid,.wifi-grid{grid-template-columns:1fr}
  .grid-3{grid-template-columns:repeat(2,1fr)}
}
@media (max-width:920px){
  .sidebar{transform:translateX(-105%)}
  .sidebar.open{transform:translateX(0);box-shadow:0 0 60px rgba(0,0,0,.7)}
  .content{margin-left:0}
  .hamburger{display:flex}
  .chip-mono{display:none}
  .page-wrap{padding:16px 14px 24px}
  .topbar{padding:10px 14px}
  .crumb-root{display:none}
  .grid-3{grid-template-columns:1fr}
  .grid-sensors{grid-template-columns:repeat(auto-fill,minmax(160px,1fr))}
  .big-val{font-size:27px}
  .mini-stats{flex-direction:column;gap:6px}
  .foot{justify-content:center;text-align:center}
}
@media (max-width:520px){
  .grid-sensors{grid-template-columns:1fr}
  .top-right .chip-live{font-size:10px;padding:0 10px}
  .form-actions{flex-direction:column;align-items:stretch}
  .form-actions .btn{justify-content:center}
  .matrix-actions{flex-direction:column;align-items:stretch}
  .matrix-actions .btn{justify-content:center}
}
@media (prefers-reduced-motion:reduce){*{animation-duration:.01ms!important;transition-duration:.01ms!important;scroll-behavior:auto!important}}
</style>
</head>
<body>
<div class="bg-fx"></div>

<aside class="sidebar" id="sidebar" aria-label="Settings navigation">
  <div class="brand">
    <div class="brand-mark" aria-hidden="true">
      <svg viewBox="0 0 48 48"><path d="M38 9C22 10.6 13.5 19.6 12 35.5 27.8 37 37 27.6 38 9Z" fill="#d1fae5"/><path d="M15 33c7.5-9.2 14.5-14.6 22-18.6" stroke="#065f46" stroke-width="2.4" fill="none" stroke-linecap="round"/></svg>
    </div>
    <div class="brand-text"><strong>SMART&nbsp;GARDEN</strong><span>ESP32 Unified Node</span></div>
  </div>
  <nav class="nav" id="navList">
    <div class="nav-label">Overview</div>
    <button class="nav-item active" data-page="dashboard"><svg viewBox="0 0 24 24"><rect x="3" y="3" width="7.5" height="7.5" rx="1.8"/><rect x="13.5" y="3" width="7.5" height="7.5" rx="1.8"/><rect x="3" y="13.5" width="7.5" height="7.5" rx="1.8"/><rect x="13.5" y="13.5" width="7.5" height="7.5" rx="1.8"/></svg><span>Dashboard</span></button>
    <div class="nav-label">Sensors</div>
    <button class="nav-item" data-page="bme"><svg viewBox="0 0 24 24"><path d="M10 13.6V5a2 2 0 0 1 4 0v8.6a4.5 4.5 0 1 1-4 0Z"/><path d="M12 9v5"/></svg><span>BME280 Temp·Hum·Pres</span><span class="nav-pin">21/22</span></button>
    <button class="nav-item" data-page="bh"><svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="4.4"/><path d="M12 2.5v3M12 18.5v3M2.5 12h3M18.5 12h3M4.9 4.9l2.1 2.1M17 17l2.1 2.1M19.1 4.9 17 7M7 17l-2.1 2.1"/></svg><span>BH1750 Light</span><span class="nav-pin">21/22</span></button>
    <button class="nav-item" data-page="mq"><svg viewBox="0 0 24 24"><path d="M9.5 3h5M10.5 3v4.2L6.4 15a3.4 3.4 0 0 0 6.6 1.7l.2-.6.2.6a3.4 3.4 0 0 0 6.6-1.7L15.5 7.2V3"/><path d="M8.5 13h7"/></svg><span>MQ-2 Gas / Smoke</span><span class="nav-pin">GPIO 32</span></button>
    <button class="nav-item" data-page="soil"><svg viewBox="0 0 24 24"><path d="M4 20h16M6 20v-5.5M10 20v-8M14 20v-11M18 20v-6.5"/><path d="M6 12.5c2-1.6 3-4 3-6.5-2.6.4-4.4 2.3-5 5.2M14 8V4.5M14 4.5c2.2 0 3.7 1.4 4.2 3.6-2.4.2-3.9-.8-4.2-3.6Z"/></svg><span>Soil Moisture</span><span class="nav-pin">GPIO 34</span></button>
    <button class="nav-item" data-page="rain"><svg viewBox="0 0 24 24"><path d="M7 13a5 5 0 1 1 .9-9.92A6 6 0 0 1 19 8.5 4.25 4.25 0 0 1 17.5 17H8"/><path d="M8.5 17v3M12.5 17.5v3.5M16.5 17v3"/></svg><span>Rain Sensor</span><span class="nav-pin">GPIO 35</span></button>
    <div class="nav-label">Outputs</div>
    <button class="nav-item" data-page="oled"><svg viewBox="0 0 24 24"><rect x="2.5" y="5" width="19" height="13" rx="2"/><path d="M8 21h8M12 18v3"/></svg><span>SSD1306 OLED</span><span class="nav-pin">21/22</span></button>
    <button class="nav-item" data-page="relay"><svg viewBox="0 0 24 24"><rect x="3" y="7" width="18" height="10" rx="2"/><path d="M7 12h3M14 12h.01M17 12h.01M9.5 12a2.5 2.5 0 0 0 5 0"/></svg><span>Relay Matrix</span><span class="nav-pin">6×</span></button>
    <button class="nav-item" data-page="buzzer"><svg viewBox="0 0 24 24"><path d="M11 5 6.5 9H3v6h3.5L11 19V5Z"/><path d="M15 9.5a4 4 0 0 1 0 5M18 6.8a8 8 0 0 1 0 10.4"/></svg><span>Buzzer</span><span class="nav-pin">GPIO 26</span></button>
    <button class="nav-item" data-page="led"><svg viewBox="0 0 24 24"><path d="M9 18h6M10 21h4"/><path d="M12 3a6.5 6.5 0 0 1 4 11.6c-.9.7-1 1.4-1 2.4H9c0-1-.1-1.7-1-2.4A6.5 6.5 0 0 1 12 3Z"/></svg><span>Status LED</span><span class="nav-pin">GPIO 33</span></button>
    <div class="nav-label">System</div>
    <button class="nav-item" data-page="system"><svg viewBox="0 0 24 24"><rect x="6" y="6" width="12" height="12" rx="2"/><path d="M9 2.5v3.5M15 2.5v3.5M9 18v3.5M15 18v3.5M2.5 9H6M2.5 15H6M18 9h3.5M18 15h3.5"/></svg><span>System Status</span></button>
    <button class="nav-item" data-page="wifiap"><svg viewBox="0 0 24 24"><path d="M2.5 9.5a15 15 0 0 1 19 0M5.5 13a10.5 10.5 0 0 1 13 0M8.6 16.4a6 6 0 0 1 6.8 0"/><circle cx="12" cy="19.4" r="1.15" fill="currentColor" stroke="none"/></svg><span>WiFi AP Settings</span></button>
  </nav>
  <div class="side-foot"><span class="pulse-dot"></span><span>FW <b id="sideFw">2.4.1</b> · <span id="sideIp">192.168.4.1</span></span></div>
</aside>
<div class="scrim" id="scrim"></div>

<main class="content">
<header class="topbar">
  <button class="icon-btn hamburger" id="btnMenu" aria-label="Toggle navigation"><svg width="19" height="19" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><path d="M4 7h16M4 12h16M4 17h16"/></svg></button>
  <div class="crumb"><span class="crumb-root">SMART&nbsp;GARDEN</span><span class="crumb-sep">/</span><span id="crumbPage">Dashboard</span></div>
  <div class="top-right">
    <div class="chip chip-live" id="connPill"><span class="live-dot"></span><span id="connText">AP · 192.168.4.1</span></div>
    <div class="chip chip-mono" id="uptimeChip">UP 00:00:00</div>
  </div>
</header>

<div class="page-wrap">

<section class="panel active" id="panel-dashboard" role="tabpanel">
  <div class="alert-banner" id="alertBanner" role="alert">
    <svg viewBox="0 0 24 24"><path d="M12 3.5 2.8 19.4h18.4L12 3.5Z"/><path d="M12 9.5v5M12 17.2h.01"/></svg>
    <div><strong>Attention — active condition</strong><span id="alertList">—</span></div>
  </div>

  <div class="grid grid-sensors">
    <article class="card sensor-card">
      <header class="card-head">
        <div class="sensor-ico bme"><svg viewBox="0 0 24 24"><path d="M10 13.6V5a2 2 0 0 1 4 0v8.6a4.5 4.5 0 1 1-4 0Z"/><path d="M12 9v5"/></svg></div>
        <div><h3>BME280</h3><p>Temp · Humidity · Pressure</p></div>
        <span class="badge off" data-badge="bme">—</span>
      </header>
      <div class="big-read"><span class="big-val" id="v-temp">--.-</span><span class="big-unit">°C</span></div>
      <div class="mini-stats">
        <div><label>Humidity</label><b id="v-humidity">--.- %</b></div>
        <div><label>Pressure</label><b id="v-pressure">---- hPa</b></div>
        <div><label>Est. Altitude</label><b id="v-altitude">--- m</b></div>
      </div>
      <div class="meter" id="meter-bme"><i></i></div>
      <canvas class="spark" id="spark-bme" data-color="#38bdf8" aria-hidden="true"></canvas>
      <footer class="card-foot"><span>I2C · SDA 21 / SCL 22</span><span id="bme-addr">0x76</span></footer>
    </article>

    <article class="card sensor-card">
      <header class="card-head">
        <div class="sensor-ico bh"><svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="4.4"/><path d="M12 2.5v3M12 18.5v3M2.5 12h3M18.5 12h3M4.9 4.9l2.1 2.1M17 17l2.1 2.1M19.1 4.9 17 7M7 17l-2.1 2.1"/></svg></div>
        <div><h3>BH1750</h3><p>Ambient Light · Lux</p></div>
        <span class="badge off" data-badge="bh">—</span>
      </header>
      <div class="big-read"><span class="big-val" id="v-lux">----</span><span class="big-unit">lux</span></div>
      <div class="mini-stats">
        <div><label>Condition</label><b id="v-sky">—</b></div>
        <div><label>Raw mode</label><b>Cont. H-Res</b></div>
      </div>
      <div class="meter" id="meter-bh"><i></i></div>
      <canvas class="spark" id="spark-bh" data-color="#fde047" aria-hidden="true"></canvas>
      <footer class="card-foot"><span>I2C · SDA 21 / SCL 22</span><span>0x23</span></footer>
    </article>

    <article class="card sensor-card">
      <header class="card-head">
        <div class="sensor-ico mq"><svg viewBox="0 0 24 24"><path d="M9.5 3h5M10.5 3v4.2L6.4 15a3.4 3.4 0 0 0 6.6 1.7l.2-.6.2.6a3.4 3.4 0 0 0 6.6-1.7L15.5 7.2V3"/><path d="M8.5 13h7"/></svg></div>
        <div><h3>MQ-2</h3><p>LPG · Smoke · Alcohol · H₂</p></div>
        <span class="badge off" data-badge="mq">—</span>
      </header>
      <div class="big-read"><span class="big-val" id="v-gas">--.-</span><span class="big-unit">% gas</span></div>
      <div class="mini-stats">
        <div><label>Raw ADC</label><b id="v-gasraw">---- / 4095</b></div>
        <div><label>Voltage</label><b id="v-gasvolt">-.-- V</b></div>
      </div>
      <div class="meter" id="meter-mq"><i></i></div>
      <canvas class="spark" id="spark-mq" data-color="#fb923c" aria-hidden="true"></canvas>
      <footer class="card-foot"><span>ADC1 · GPIO 32 · 12-bit</span><span id="mq-samples">8× avg</span></footer>
    </article>

    <article class="card sensor-card">
      <header class="card-head">
        <div class="sensor-ico soil"><svg viewBox="0 0 24 24"><path d="M4 20h16M6 20v-5.5M10 20v-8M14 20v-11M18 20v-6.5"/><path d="M14 8V4.5M14 4.5c2.2 0 3.7 1.4 4.2 3.6-2.4.2-3.9-.8-4.2-3.6Z"/></svg></div>
        <div><h3>Soil Moisture</h3><p>Capacitive · VWC %</p></div>
        <span class="badge off" data-badge="soil">—</span>
      </header>
      <div class="big-read"><span class="big-val" id="v-soil">--.-</span><span class="big-unit">% moisture</span></div>
      <div class="mini-stats">
        <div><label>Raw ADC</label><b id="v-soilraw">---- / 4095</b></div>
        <div><label>Status</label><b id="v-soilstat">—</b></div>
      </div>
      <div class="meter" id="meter-soil"><i></i></div>
      <canvas class="spark" id="spark-soil" data-color="#4ade80" aria-hidden="true"></canvas>
      <footer class="card-foot"><span>ADC1 · GPIO 34 · input-only</span><span id="soil-cal">cal 3100/1400</span></footer>
    </article>

    <article class="card sensor-card">
      <header class="card-head">
        <div class="sensor-ico rain"><svg viewBox="0 0 24 24"><path d="M7 13a5 5 0 1 1 .9-9.92A6 6 0 0 1 19 8.5 4.25 4.25 0 0 1 17.5 17H8"/><path d="M8.5 17v3M12.5 17.5v3.5M16.5 17v3"/></svg></div>
        <div><h3>Rain Sensor</h3><p>Resistive · Precip %</p></div>
        <span class="badge off" data-badge="rain">—</span>
      </header>
      <div class="big-read"><span class="big-val" id="v-rain">--.-</span><span class="big-unit">% precip</span></div>
      <div class="mini-stats">
        <div><label>Raw ADC</label><b id="v-rainraw">---- / 4095</b></div>
        <div><label>Sky state</label><b id="v-rainstat">—</b></div>
      </div>
      <div class="meter" id="meter-rain"><i></i></div>
      <canvas class="spark" id="spark-rain" data-color="#93c5fd" aria-hidden="true"></canvas>
      <footer class="card-foot"><span>ADC1 · GPIO 35 · input-only</span><span id="rain-cal">cal 3500/900</span></footer>
    </article>
  </div>

  <div class="grid grid-2" style="margin-top:16px">
    <article class="card">
      <div class="card-title">Relay Output Matrix <small>GPIO25 unified · auto = source-triggered</small></div>
      <div class="relay-grid" id="relayGrid"></div>
      <div class="output-duo">
        <div class="out-pill"><div class="out-ico"><svg viewBox="0 0 24 24"><path d="M11 5 6.5 9H3v6h3.5L11 19V5Z"/><path d="M15 9.5a4 4 0 0 1 0 5"/></svg></div><div><small>Buzzer · GPIO 26</small><b id="out-buzzer">IDLE</b></div></div>
        <div class="out-pill"><div class="out-ico"><svg viewBox="0 0 24 24"><path d="M9 18h6M10 21h4"/><path d="M12 3a6.5 6.5 0 0 1 4 11.6c-.9.7-1 1.4-1 2.4H9c0-1-.1-1.7-1-2.4A6.5 6.5 0 0 1 12 3Z"/></svg></div><div><small>LED · GPIO 33</small><b id="out-led">OFF</b></div></div>
        <div class="out-pill"><div class="out-ico"><svg viewBox="0 0 24 24"><rect x="2.5" y="5" width="19" height="13" rx="2"/><path d="M8 21h8"/></svg></div><div><small>OLED · 128×64</small><b id="out-oled">ON</b></div></div>
      </div>
    </article>

    <article class="card">
      <div class="card-title">System Snapshot <small>live</small></div>
      <div class="kv">
        <div class="kv-row"><span>Uptime (64-bit)</span><b id="kv-uptime">—</b></div>
        <div class="kv-row"><span>Free heap</span><b id="kv-heap">—</b></div>
        <div class="kv-row"><span>Lowest heap</span><b id="kv-minheap">—</b></div>
        <div class="kv-row"><span>Heap fragmentation</span><b id="kv-frag">—</b></div>
        <div class="kv-row"><span>AP clients</span><b id="kv-clients">—</b></div>
        <div class="kv-row"><span>Config integrity</span><b id="kv-cfg" class="good">—</b></div>
        <div class="kv-row"><span>NVS writes (this boot)</span><b id="kv-nvs">—</b></div>
      </div>
      <div class="form-actions" style="border:none;padding-top:12px;margin-top:6px">
        <button class="btn btn-ghost btn-sm" id="btnRefreshNow"><svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M21 12a9 9 0 1 1-2.64-6.36M21 3v6h-6"/></svg>Refresh now</button>
        <button class="btn btn-ghost btn-sm" data-goto="system">System details →</button>
      </div>
    </article>
  </div>
</section>
<section class="panel" id="panel-bme" role="tabpanel">
  <div class="page-head">
    <div><h2>BME280 — Temperature · Humidity · Pressure</h2><p>Bosch BME280 environmental sensor on shared I2C bus (<code>SDA GPIO21</code> · <code>SCL GPIO22</code>), address <code>0x76</code>/<code>0x77</code> auto-probe. Drives GPIO15 dedicated relay and/or GPIO25 unified relay on trigger.</p></div>
    <span class="badge big off" data-badge="bme">—</span>
  </div>
  <div class="settings-grid">
    <form class="card form-card" data-section="bme" autocomplete="off">
      <div class="section-sep"><span>Sensor</span></div>
      <div class="field-grid">
        <div class="toggle-row full" style="grid-column:1/-1"><div class="tr-text"><b>BME280 enabled</b><small>Polls temperature, relative humidity and barometric pressure</small></div><label class="switch"><input type="checkbox" name="enabled"><span class="slider-ui"></span></label></div>
        <div class="field"><label for="bme-interval">Read interval</label><select class="text-input" id="bme-interval" name="intervalIdx" data-num><option value="0">1 second</option><option value="1" selected>2 seconds</option><option value="2">5 seconds</option><option value="3">10 seconds</option><option value="4">30 seconds</option><option value="5">60 seconds</option></select></div>
        <div class="field"><label>&nbsp;</label><button type="button" class="btn btn-ghost" data-test="bme">Test sensor</button></div>
      </div>
      <div class="test-out" id="test-bme"></div>

      <div class="section-sep"><span>Calibration offsets</span></div>
      <div class="field-grid">
        <div class="field"><label for="bme-toff">Temperature offset</label><div class="input-unit"><input class="text-input" id="bme-toff" name="tempOffset" type="number" step="0.1" min="-20" max="20" data-num><em>°C</em></div></div>
        <div class="field"><label for="bme-hoff">Humidity offset</label><div class="input-unit"><input class="text-input" id="bme-hoff" name="humOffset" type="number" step="0.1" min="-50" max="50" data-num><em>%RH</em></div></div>
        <div class="field"><label for="bme-poff">Pressure offset</label><div class="input-unit"><input class="text-input" id="bme-poff" name="presOffset" type="number" step="0.1" min="-100" max="100" data-num><em>hPa</em></div></div>
      </div>

      <div class="section-sep"><span>Trigger &amp; relay control</span></div>
      <div class="field-grid">
        <div class="field"><label for="bme-metric">Trigger metric</label><select class="text-input" id="bme-metric" name="metric" data-num><option value="0" selected>Temperature</option><option value="1">Humidity</option><option value="2">Pressure</option></select></div>
        <div class="field"><label for="bme-cond">Condition</label><select class="text-input" id="bme-cond" name="condition" data-num><option value="0" selected>Above threshold</option><option value="1">Below threshold</option></select></div>
        <div class="field"><label for="bme-thr">Threshold</label><input class="text-input" id="bme-thr" name="threshold" type="number" step="0.1" data-num><span class="hint">°C · %RH · hPa according to metric</span></div>
        <div class="field"><label for="bme-hyst">Hysteresis</label><input class="text-input" id="bme-hyst" name="hysteresis" type="number" step="0.1" min="0" max="100" data-num><span class="hint">Schmitt band — prevents relay chatter</span></div>
        <div class="field"><label for="bme-cool">Alert cooldown</label><div class="input-unit"><input class="text-input" id="bme-cool" name="cooldownSec" type="number" step="1" min="0" max="86400" data-num><em>sec</em></div><span class="hint">Minimum gap between repeat alerts</span></div>
      </div>
      <div class="check-grid" style="margin-top:12px">
        <div class="check-pill"><input type="checkbox" id="bme-relay" name="relay"><label for="bme-relay"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Dedicated relay · GPIO15</label></div>
        <div class="check-pill"><input type="checkbox" id="bme-unified" name="unified"><label for="bme-unified"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Unified relay · GPIO25</label></div>
        <div class="check-pill"><input type="checkbox" id="bme-buzz" name="buzzer"><label for="bme-buzz"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Buzzer alert · GPIO26</label></div>
        <div class="check-pill"><input type="checkbox" id="bme-led" name="led"><label for="bme-led"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>LED alert · GPIO33</label></div>
        <div class="check-pill"><input type="checkbox" id="bme-trig" name="trigger"><label for="bme-trig"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Trigger enabled</label></div>
      </div>
      <div class="form-actions">
        <button type="button" class="btn btn-primary" data-save="bme"><svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2Z"/><path d="M17 21v-8H7v8M7 3v5h8"/></svg>Save changes</button>
        <span class="save-hint">commit → NVS only when values differ</span>
      </div>
    </form>

    <aside class="side-col">
      <div class="card live-card">
        <div class="card-title">Live Reading <small>auto-refresh 2 s</small></div>
        <div class="big-read"><span class="big-val" id="lv-bme-temp">--.-</span><span class="big-unit">°C</span></div>
        <div class="live-rows">
          <div class="live-row"><span>Humidity</span><b id="lv-bme-hum">--.- %RH</b></div>
          <div class="live-row"><span>Pressure</span><b id="lv-bme-pres">----.- hPa</b></div>
          <div class="live-row"><span>Trigger state</span><b id="lv-bme-trig">IDLE</b></div>
          <div class="live-row"><span>Device</span><b id="lv-bme-dev">—</b></div>
        </div>
      </div>
      <div class="card note-card">
        <h4><svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="12" cy="12" r="9"/><path d="M12 8v5M12 16.5h.01"/></svg>Wiring notes</h4>
        <p>Shares the <code>SDA 21 / SCL 22</code> bus with the OLED and BH1750. BMP280 boards marked 3.3V only must not receive 5V. Add 4.7kΩ pull-ups on both lines if your breakout lacks them. </p>
      </div>
    </aside>
  </div>
</section>

<section class="panel" id="panel-bh" role="tabpanel">
  <div class="page-head">
    <div><h2>BH1750 — Ambient Light Sensor</h2><p>ROHM BH1750FVI digital lux sensor on shared I2C bus (<code>SDA GPIO21</code> · <code>SCL GPIO22</code>), fixed address <code>0x23</code> (ADDR low). Drives GPIO2 dedicated relay and/or GPIO25 unified relay on trigger.</p></div>
    <span class="badge big off" data-badge="bh">—</span>
  </div>
  <div class="settings-grid">
    <form class="card form-card" data-section="bh" autocomplete="off">
      <div class="section-sep"><span>Sensor</span></div>
      <div class="field-grid">
        <div class="toggle-row full" style="grid-column:1/-1"><div class="tr-text"><b>BH1750 enabled</b><small>Continuous high-resolution lux measurement (1 lx resolution)</small></div><label class="switch"><input type="checkbox" name="enabled"><span class="slider-ui"></span></label></div>
        <div class="field"><label for="bh-interval">Read interval</label><select class="text-input" id="bh-interval" name="intervalIdx" data-num><option value="0">1 second</option><option value="1" selected>2 seconds</option><option value="2">5 seconds</option><option value="3">10 seconds</option><option value="4">30 seconds</option><option value="5">60 seconds</option></select></div>
        <div class="field"><label>&nbsp;</label><button type="button" class="btn btn-ghost" data-test="bh">Test sensor</button></div>
      </div>
      <div class="test-out" id="test-bh"></div>

      <div class="section-sep"><span>Trigger &amp; relay control</span></div>
      <div class="field-grid">
        <div class="field"><label for="bh-cond">Condition</label><select class="text-input" id="bh-cond" name="condition" data-num><option value="0">Above threshold</option><option value="1" selected>Below threshold</option></select></div>
        <div class="field"><label for="bh-thr">Threshold</label><div class="input-unit"><input class="text-input" id="bh-thr" name="threshold" type="number" step="1" min="0" max="200000" data-num><em>lux</em></div></div>
        <div class="field"><label for="bh-hyst">Hysteresis</label><div class="input-unit"><input class="text-input" id="bh-hyst" name="hysteresis" type="number" step="1" min="0" max="100000" data-num><em>lux</em></div><span class="hint">Schmitt band — prevents relay chatter</span></div>
        <div class="field"><label for="bh-cool">Alert cooldown</label><div class="input-unit"><input class="text-input" id="bh-cool" name="cooldownSec" type="number" step="1" min="0" max="86400" data-num><em>sec</em></div></div>
      </div>
      <div class="check-grid" style="margin-top:12px">
        <div class="check-pill"><input type="checkbox" id="bh-relay" name="relay"><label for="bh-relay"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Dedicated relay · GPIO2</label></div>
        <div class="check-pill"><input type="checkbox" id="bh-unified" name="unified"><label for="bh-unified"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Unified relay · GPIO25</label></div>
        <div class="check-pill"><input type="checkbox" id="bh-buzz" name="buzzer"><label for="bh-buzz"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Buzzer alert · GPIO26</label></div>
        <div class="check-pill"><input type="checkbox" id="bh-led" name="led"><label for="bh-led"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>LED alert · GPIO33</label></div>
        <div class="check-pill"><input type="checkbox" id="bh-trig" name="trigger"><label for="bh-trig"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Trigger enabled</label></div>
      </div>
      <div class="form-actions">
        <button type="button" class="btn btn-primary" data-save="bh"><svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2Z"/><path d="M17 21v-8H7v8M7 3v5h8"/></svg>Save changes</button>
        <span class="save-hint">commit → NVS only when values differ</span>
      </div>
    </form>

    <aside class="side-col">
      <div class="card live-card">
        <div class="card-title">Live Reading <small>auto-refresh 2 s</small></div>
        <div class="big-read"><span class="big-val" id="lv-bh-lux">----</span><span class="big-unit">lux</span></div>
        <div class="live-rows">
          <div class="live-row"><span>Sky condition</span><b id="lv-bh-sky">—</b></div>
          <div class="live-row"><span>Trigger state</span><b id="lv-bh-trig">IDLE</b></div>
          <div class="live-row"><span>Device</span><b id="lv-bh-dev">—</b></div>
        </div>
      </div>
      <div class="card note-card">
        <h4><svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="12" cy="12" r="9"/><path d="M12 8v5M12 16.5h.01"/></svg>Wiring notes</h4>
        <p>Fixed I2C address <code>0x23</code> with ADDR pin low, or <code>0x5C</code> if high — this firmware probes <code>0x23</code> first. Typical sunlight 10k–100k lx, overcast 1k lx, full moon 0.1 lx. </p>
      </div>
    </aside>
  </div>
</section>

<section class="panel" id="panel-mq" role="tabpanel">
  <div class="page-head">
    <div><h2>MQ-2 — Gas / Smoke Sensor</h2><p>Semiconductor SnO₂ sensor for LPG, smoke, alcohol, propane, H₂ and CO. Analog output to <code>GPIO32</code> (ADC1_CH4), 12-bit. Drives GPIO4 dedicated relay and/or GPIO25 unified relay on trigger.</p></div>
    <span class="badge big off" data-badge="mq">—</span>
  </div>
  <div class="settings-grid">
    <form class="card form-card" data-section="mq" autocomplete="off">
      <div class="section-sep"><span>Sensor</span></div>
      <div class="field-grid">
        <div class="toggle-row full" style="grid-column:1/-1"><div class="tr-text"><b>MQ-2 enabled</b><small>Analog acquisition with exponential smoothing (EMA α = 0.25)</small></div><label class="switch"><input type="checkbox" name="enabled"><span class="slider-ui"></span></label></div>
        <div class="field"><label for="mq-interval">Read interval</label><select class="text-input" id="mq-interval" name="intervalIdx" data-num><option value="0">1 second</option><option value="1">2 seconds</option><option value="2" selected>5 seconds</option><option value="3">10 seconds</option><option value="4">30 seconds</option><option value="5">60 seconds</option></select></div>
        <div class="field"><label for="mq-samples">Samples per read</label><select class="text-input" id="mq-samples" name="samplesIdx" data-num><option value="0">4 samples</option><option value="1" selected>8 samples</option><option value="2">16 samples</option><option value="3">32 samples</option></select><span class="hint">64-bit accumulator, overflow-safe averaging</span></div>
        <div class="field"><label>&nbsp;</label><button type="button" class="btn btn-ghost" data-test="mq">Test sensor</button></div>
      </div>
      <div class="test-out" id="test-mq"></div>

      <div class="section-sep"><span>Trigger &amp; relay control</span></div>
      <div class="field-grid">
        <div class="field"><label for="mq-cond">Condition</label><select class="text-input" id="mq-cond" name="condition" data-num><option value="0" selected>Above threshold</option><option value="1">Below threshold</option></select></div>
        <div class="field"><label for="mq-thr">Gas threshold</label><div class="input-unit"><input class="text-input" id="mq-thr" name="thresholdPct" type="number" step="0.5" min="0" max="100" data-num><em>% gas</em></div><span class="hint">% of full-scale ADC (raw ÷ 4095)</span></div>
        <div class="field"><label for="mq-hyst">Hysteresis</label><div class="input-unit"><input class="text-input" id="mq-hyst" name="hysteresis" type="number" step="0.5" min="0" max="100" data-num><em>%</em></div></div>
        <div class="field"><label for="mq-cool">Alert cooldown</label><div class="input-unit"><input class="text-input" id="mq-cool" name="cooldownSec" type="number" step="1" min="0" max="86400" data-num><em>sec</em></div></div>
      </div>
      <div class="check-grid" style="margin-top:12px">
        <div class="check-pill"><input type="checkbox" id="mq-relay" name="relay"><label for="mq-relay"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Dedicated relay · GPIO4</label></div>
        <div class="check-pill"><input type="checkbox" id="mq-unified" name="unified"><label for="mq-unified"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Unified relay · GPIO25</label></div>
        <div class="check-pill"><input type="checkbox" id="mq-buzz" name="buzzer"><label for="mq-buzz"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Buzzer alert · GPIO26</label></div>
        <div class="check-pill"><input type="checkbox" id="mq-led" name="led"><label for="mq-led"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>LED alert · GPIO33</label></div>
        <div class="check-pill"><input type="checkbox" id="mq-trig" name="trigger"><label for="mq-trig"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Trigger enabled</label></div>
      </div>
      <div class="form-actions">
        <button type="button" class="btn btn-primary" data-save="mq"><svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2Z"/><path d="M17 21v-8H7v8M7 3v5h8"/></svg>Save changes</button>
        <span class="save-hint">commit → NVS only when values differ</span>
      </div>
    </form>

    <aside class="side-col">
      <div class="card live-card">
        <div class="card-title">Live Reading <small>auto-refresh 2 s</small></div>
        <div class="big-read"><span class="big-val" id="lv-mq-gas">--.-</span><span class="big-unit">% gas</span></div>
        <div class="live-rows">
          <div class="live-row"><span>Raw ADC</span><b id="lv-mq-raw">---- / 4095</b></div>
          <div class="live-row"><span>Heater warm-up</span><b>burn-in OK · 24 h recommended</b></div>
          <div class="live-row"><span>Trigger state</span><b id="lv-mq-trig">IDLE</b></div>
        </div>
      </div>
      <div class="card note-card">
        <h4><svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="12" cy="12" r="9"/><path d="M12 8v5M12 16.5h.01"/></svg>Wiring &amp; burn-in</h4>
        <p>Power the module at <code>5 V</code> if it has an onboard regulator with heater, but read <code>AOUT</code> through a divider into <code>GPIO32</code> if it can exceed 3.3 V. New MQ-2 elements need a 24–48 h burn-in for stable baselines. </p>
      </div>
    </aside>
  </div>
</section>
<section class="panel" id="panel-soil" role="tabpanel">
  <div class="page-head">
    <div><h2>Soil Moisture — Capacitive Probe</h2><p>Capacitive VWC sensor on <code>GPIO34</code> (ADC1_CH6, input-only). Normalized to 0–100 % with dry/wet raw calibration. Trigger default: moisture <b>below</b> threshold energizes GPIO5 pump relay and/or GPIO25 unified relay.</p></div>
    <span class="badge big off" data-badge="soil">—</span>
  </div>
  <div class="settings-grid">
    <form class="card form-card" data-section="soil" autocomplete="off">
      <div class="section-sep"><span>Sensor</span></div>
      <div class="field-grid">
        <div class="toggle-row full" style="grid-column:1/-1"><div class="tr-text"><b>Soil moisture enabled</b><small>Averaged analog read with EMA α = 0.25 and 64-bit accumulator</small></div><label class="switch"><input type="checkbox" name="enabled"><span class="slider-ui"></span></label></div>
        <div class="field"><label for="soil-interval">Read interval</label><select class="text-input" id="soil-interval" name="intervalIdx" data-num><option value="0">1 second</option><option value="1" selected>2 seconds</option><option value="2">5 seconds</option><option value="3">10 seconds</option><option value="4">30 seconds</option><option value="5">60 seconds</option></select></div>
        <div class="field"><label>&nbsp;</label><button type="button" class="btn btn-ghost" data-test="soil">Test sensor</button></div>
      </div>
      <div class="test-out" id="test-soil"></div>

      <div class="section-sep"><span>Calibration</span></div>
      <div class="field-grid">
        <div class="field"><label for="soil-dry">Dry raw (in air)</label><input class="text-input" id="soil-dry" name="dryRaw" type="number" step="1" min="0" max="4095" data-num><span class="hint">Capacitive probes read high in air</span></div>
        <div class="field"><label for="soil-wet">Wet raw (in water)</label><input class="text-input" id="soil-wet" name="wetRaw" type="number" step="1" min="0" max="4095" data-num><span class="hint">…and read low submerged</span></div>
      </div>
      <div class="test-out bad" id="test-soilcal" style="min-height:0"></div>

      <div class="section-sep"><span>Trigger &amp; relay control</span></div>
      <div class="field-grid">
        <div class="field"><label for="soil-cond">Condition</label><select class="text-input" id="soil-cond" name="condition" data-num><option value="0">Above threshold</option><option value="1" selected>Below threshold</option></select></div>
        <div class="field"><label for="soil-thr">Moisture threshold</label><div class="input-unit"><input class="text-input" id="soil-thr" name="thresholdPct" type="number" step="1" min="0" max="100" data-num><em>% VWC</em></div></div>
        <div class="field"><label for="soil-hyst">Hysteresis</label><div class="input-unit"><input class="text-input" id="soil-hyst" name="hysteresis" type="number" step="1" min="0" max="50" data-num><em>%</em></div><span class="hint">Pump de-rates at threshold, releases below −hysteresis… set ≥ 4 %</span></div>
        <div class="field"><label for="soil-cool">Alert cooldown</label><div class="input-unit"><input class="text-input" id="soil-cool" name="cooldownSec" type="number" step="1" min="0" max="86400" data-num><em>sec</em></div></div>
      </div>
      <div class="check-grid" style="margin-top:12px">
        <div class="check-pill"><input type="checkbox" id="soil-relay" name="relay"><label for="soil-relay"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Pump relay · GPIO5</label></div>
        <div class="check-pill"><input type="checkbox" id="soil-unified" name="unified"><label for="soil-unified"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Unified relay · GPIO25</label></div>
        <div class="check-pill"><input type="checkbox" id="soil-buzz" name="buzzer"><label for="soil-buzz"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Buzzer alert · GPIO26</label></div>
        <div class="check-pill"><input type="checkbox" id="soil-led" name="led"><label for="soil-led"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>LED alert · GPIO33</label></div>
        <div class="check-pill"><input type="checkbox" id="soil-trig" name="trigger"><label for="soil-trig"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Trigger enabled</label></div>
      </div>
      <div class="form-actions">
        <button type="button" class="btn btn-primary" data-save="soil"><svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2Z"/><path d="M17 21v-8H7v8M7 3v5h8"/></svg>Save changes</button>
        <span class="save-hint">commit → NVS only when values differ</span>
      </div>
    </form>

    <aside class="side-col">
      <div class="card live-card">
        <div class="card-title">Live Reading <small>auto-refresh 2 s</small></div>
        <div class="big-read"><span class="big-val" id="lv-soil-pct">--.-</span><span class="big-unit">% moisture</span></div>
        <div class="live-rows">
          <div class="live-row"><span>Raw ADC</span><b id="lv-soil-raw">---- / 4095</b></div>
          <div class="live-row"><span>Irrigation state</span><b id="lv-soil-trig">IDLE</b></div>
          <div class="live-row"><span>Condition</span><b id="lv-soil-cond">below threshold</b></div>
        </div>
      </div>
      <div class="card note-card">
        <h4><svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="12" cy="12" r="9"/><path d="M12 8v5M12 16.5h.01"/></svg>Calibration procedure</h4>
        <p>1. Hold probe in dry air → set <b>dry raw</b> to the reading shown. 2. Submerge tip in water (not the electronics) → set <b>wet raw</b>. 3. Verify the live % responds. Typical capacitive v1.2: dry ≈ 3000–3400, wet ≈ 1300–1600. </p>
      </div>
    </aside>
  </div>
</section>

<section class="panel" id="panel-rain" role="tabpanel">
  <div class="page-head">
    <div><h2>Rain — Precipitation Sensor</h2><p>Resistive rain board analog output on <code>GPIO35</code> (ADC1_CH7, input-only). Normalized 0–100 % with dry/wet calibration. Trigger default: precipitation <b>above</b> threshold energizes GPIO18 and/or GPIO25 unified relay.</p></div>
    <span class="badge big off" data-badge="rain">—</span>
  </div>
  <div class="settings-grid">
    <form class="card form-card" data-section="rain" autocomplete="off">
      <div class="section-sep"><span>Sensor</span></div>
      <div class="field-grid">
        <div class="toggle-row full" style="grid-column:1/-1"><div class="tr-text"><b>Rain sensor enabled</b><small>Averaged analog read with EMA α = 0.25 and 64-bit accumulator</small></div><label class="switch"><input type="checkbox" name="enabled"><span class="slider-ui"></span></label></div>
        <div class="field"><label for="rain-interval">Read interval</label><select class="text-input" id="rain-interval" name="intervalIdx" data-num><option value="0">1 second</option><option value="1" selected>2 seconds</option><option value="2">5 seconds</option><option value="3">10 seconds</option><option value="4">30 seconds</option><option value="5">60 seconds</option></select></div>
        <div class="field"><label>&nbsp;</label><button type="button" class="btn btn-ghost" data-test="rain">Test sensor</button></div>
      </div>
      <div class="test-out" id="test-rain"></div>

      <div class="section-sep"><span>Calibration</span></div>
      <div class="field-grid">
        <div class="field"><label for="rain-dry">Dry raw (bone dry)</label><input class="text-input" id="rain-dry" name="dryRaw" type="number" step="1" min="0" max="4095" data-num><span class="hint">Typically near full-scale, ≈ 3300–4095</span></div>
        <div class="field"><label for="rain-wet">Wet raw (soaked)</label><input class="text-input" id="rain-wet" name="wetRaw" type="number" step="1" min="0" max="4095" data-num><span class="hint">Typically ≈ 400–1200 when saturated</span></div>
      </div>
      <div class="test-out bad" id="test-raincal" style="min-height:0"></div>

      <div class="section-sep"><span>Trigger &amp; relay control</span></div>
      <div class="field-grid">
        <div class="field"><label for="rain-cond">Condition</label><select class="text-input" id="rain-cond" name="condition" data-num><option value="0" selected>Above threshold</option><option value="1">Below threshold</option></select></div>
        <div class="field"><label for="rain-thr">Precipitation threshold</label><div class="input-unit"><input class="text-input" id="rain-thr" name="thresholdPct" type="number" step="1" min="0" max="100" data-num><em>% precip</em></div></div>
        <div class="field"><label for="rain-hyst">Hysteresis</label><div class="input-unit"><input class="text-input" id="rain-hyst" name="hysteresis" type="number" step="1" min="0" max="50" data-num><em>%</em></div></div>
        <div class="field"><label for="rain-cool">Alert cooldown</label><div class="input-unit"><input class="text-input" id="rain-cool" name="cooldownSec" type="number" step="1" min="0" max="86400" data-num><em>sec</em></div></div>
      </div>
      <div class="check-grid" style="margin-top:12px">
        <div class="check-pill"><input type="checkbox" id="rain-relay" name="relay"><label for="rain-relay"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Dedicated relay · GPIO18</label></div>
        <div class="check-pill"><input type="checkbox" id="rain-unified" name="unified"><label for="rain-unified"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Unified relay · GPIO25</label></div>
        <div class="check-pill"><input type="checkbox" id="rain-buzz" name="buzzer"><label for="rain-buzz"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Buzzer alert · GPIO26</label></div>
        <div class="check-pill"><input type="checkbox" id="rain-led" name="led"><label for="rain-led"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>LED alert · GPIO33</label></div>
        <div class="check-pill"><input type="checkbox" id="rain-trig" name="trigger"><label for="rain-trig"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Trigger enabled</label></div>
      </div>
      <div class="form-actions">
        <button type="button" class="btn btn-primary" data-save="rain"><svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2Z"/><path d="M17 21v-8H7v8M7 3v5h8"/></svg>Save changes</button>
        <span class="save-hint">commit → NVS only when values differ</span>
      </div>
    </form>

    <aside class="side-col">
      <div class="card live-card">
        <div class="card-title">Live Reading <small>auto-refresh 2 s</small></div>
        <div class="big-read"><span class="big-val" id="lv-rain-pct">--.-</span><span class="big-unit">% precip</span></div>
        <div class="live-rows">
          <div class="live-row"><span>Raw ADC</span><b id="lv-rain-raw">---- / 4095</b></div>
          <div class="live-row"><span>Sky state</span><b id="lv-rain-trig">DRY</b></div>
          <div class="live-row"><span>Condition</span><b id="lv-rain-cond">above threshold</b></div>
        </div>
      </div>
      <div class="card note-card">
        <h4><svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="12" cy="12" r="9"/><path d="M12 8v5M12 16.5h.01"/></svg>Probe care</h4>
        <p>Resistive rain boards corrode under constant bias. Mount the panel facing the sky, and consider powering it from a GPIO on an interval — this firmware samples always-on by default; for longest probe life use a long read interval. </p>
      </div>
    </aside>
  </div>
</section>
<section class="panel" id="panel-oled" role="tabpanel">
  <div class="page-head">
    <div><h2>SSD1306 — OLED 128×64 Display</h2><p>Monochrome OLED on shared I2C bus (<code>SDA GPIO21</code> · <code>SCL GPIO22</code>), address <code>0x3C</code>/<code>0x3D</code>. Auto-rotates the enabled pages. Wakes on any alert or web request; sleeps after the configured idle timeout.</p></div>
    <span class="badge big ok" id="oledBadge">ONLINE</span>
  </div>
  <div class="settings-grid">
    <form class="card form-card" data-section="oled" autocomplete="off">
      <div class="section-sep"><span>Display</span></div>
      <div class="field-grid">
        <div class="toggle-row full" style="grid-column:1/-1"><div class="tr-text"><b>OLED enabled</b><small>Master switch for the 128×64 panel</small></div><label class="switch"><input type="checkbox" name="enabled"><span class="slider-ui"></span></label></div>
        <div class="toggle-row full" style="grid-column:1/-1"><div class="tr-text"><b>Flip display 180°</b><small>Use when the module is mounted upside-down (setRotation 2)</small></div><label class="switch"><input type="checkbox" name="flip"><span class="slider-ui"></span></label></div>
        <div class="field full"><label for="oled-contrast">Contrast / brightness</label><div class="range-wrap"><input type="range" id="oled-contrast" name="contrast" min="0" max="255" step="1" data-num><span class="range-val" id="oled-contrast-val">160</span></div></div>
        <div class="field"><label for="oled-timeout">Idle sleep timeout</label><select class="text-input" id="oled-timeout" name="timeoutIdx" data-num><option value="0" selected>Never sleep</option><option value="1">30 seconds</option><option value="2">1 minute</option><option value="3">5 minutes</option><option value="4">10 minutes</option></select></div>
        <div class="field"><label for="oled-cycle">Page rotation</label><select class="text-input" id="oled-cycle" name="cycleIdx" data-num><option value="0">Every 2 s</option><option value="1" selected>Every 4 s</option><option value="2">Every 6 s</option><option value="3">Every 8 s</option></select></div>
      </div>

      <div class="section-sep"><span>Pages to rotate</span></div>
      <div class="check-grid">
        <div class="check-pill"><input type="checkbox" id="oled-pg-overview" name="pageOverview" checked><label for="oled-pg-overview"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Sensors overview</label></div>
        <div class="check-pill"><input type="checkbox" id="oled-pg-status" name="pageStatus" checked><label for="oled-pg-status"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>System status</label></div>
        <div class="check-pill"><input type="checkbox" id="oled-pg-alerts" name="pageAlerts" checked><label for="oled-pg-alerts"><span class="box"><svg viewBox="0 0 12 12"><path d="M1.5 6.5 4.5 9.5 10.5 2.5"/></svg></span>Alerts page</label></div>
      </div>

      <div class="section-sep"><span>Actions</span></div>
      <div class="field-grid">
        <div class="field"><label>&nbsp;</label><button type="button" class="btn btn-ghost" id="btnOledTest"><svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><rect x="2.5" y="5" width="19" height="13" rx="2"/><path d="M8 21h8M12 18v3"/></svg>Redraw now</button></div>
      </div>
      <div class="test-out" id="test-oled"></div>

      <div class="form-actions">
        <button type="button" class="btn btn-primary" data-save="oled"><svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2Z"/><path d="M17 21v-8H7v8M7 3v5h8"/></svg>Save changes</button>
        <span class="save-hint">commit → NVS only when values differ</span>
      </div>
    </form>

    <aside class="side-col">
      <div class="card">
        <div class="card-title">Live Preview <small>128×64 canvas</small></div>
        <div class="oled-bezel"><canvas id="oledPreview" width="128" height="64"></canvas><small>SSD1306 · live mock render</small></div>
      </div>
      <div class="card note-card">
        <h4><svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="12" cy="12" r="9"/><path d="M12 8v5M12 16.5h.01"/></svg>Wiring notes</h4>
        <p>I2C default address <code>0x3C</code> (also probes <code>0x3D</code>). Shares bus with BME280 and BH1750. Logic level is 3.3 V — never apply 5 V to SDA/SCL. Factory-reset hold (GPIO0, 5 s) is drawn on this panel.</p>
      </div>
    </aside>
  </div>
</section>

<section class="panel" id="panel-relay" role="tabpanel">
  <div class="page-head">
    <div><h2>Relay Matrix — 6 Channels</h2><p><code>GPIO25</code> unified relay fires on any enabled sensor trigger. Dedicated channels: <code>GPIO15</code> BME280 · <code>GPIO2</code> BH1750 · <code>GPIO4</code> MQ-2 · <code>GPIO5</code> Soil · <code>GPIO18</code> Rain. Each channel has its own enable, AUTO/manual override, active level and per-sensor source selection.</p></div>
    <span class="badge big ok" id="relayBadge">MATRIX READY</span>
  </div>

  <article class="card">
    <div class="card-title">Relay Control Matrix <small>AUTO energizes while a selected source trigger is active · toggle chips to assign sources individually</small></div>
    <div class="table-wrap">
      <table class="matrix" aria-label="Relay matrix">
        <thead><tr><th>Relay</th><th>Enable</th><th>Mode</th><th>Coil level</th><th>Sensor sources (individual control)</th><th>State</th></tr></thead>
        <tbody id="relayRows"></tbody>
      </table>
    </div>
    <div class="legend"><span>BME = BME280</span><span>LUX = BH1750</span><span>GAS = MQ-2</span><span>SOIL = Moisture</span><span>RAIN = Rain</span><span>UNIFIED GPIO25 = any enabled source</span></div>
    <div class="matrix-actions">
      <button class="btn btn-danger-ghost" id="btnAllRelaysOff"><svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M12 3v9"/><path d="M6.3 6.5a8 8 0 1 0 11.4 0"/></svg>All relays OFF</button>
      <button class="btn btn-primary" id="btnSaveRelays"><svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2Z"/><path d="M17 21v-8H7v8M7 3v5h8"/></svg>Save relay matrix</button>
    </div>
  </article>

  <div class="grid grid-3" style="margin-top:16px">
    <div class="card note-card">
      <h4><svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="12" cy="12" r="9"/><path d="M12 8v5M12 16.5h.01"/></svg>How AUTO works</h4>
      <p>A source bit assignment means: <i>while that sensor's trigger condition is active</i>, this relay energizes. Dedicated channels default to their own sensor only; GPIO25 defaults to all five. Hysteresis on each sensor prevents relay chatter. GPIO2 idles LOW so the board still boots with flash mode intact.</p>
    </div>
    <div class="card note-card">
      <h4><svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="12" cy="12" r="9"/><path d="M12 8v5M12 16.5h.01"/></svg>Manual override</h4>
      <p><b>Manual ON</b> forces the coil regardless of sensors. <b>Manual OFF</b> forces release. <b>Auto</b> follows the selected sources. Manual overrides take precedence and are persisted in NVS like every other setting — use with care for pumps and valves.</p>
    </div>
    <div class="card note-card">
      <h4><svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="12" cy="12" r="9"/><path d="M12 8v5M12 16.5h.01"/></svg>Coil level</h4>
      <p>Most opto-isolated boards energize on <b>LOW</b> (active-low, default). Bare transistor stages may need <b>HIGH</b>. Set per channel — the firmware inverts the GPIO write accordingly; observed state on the dashboard always shows the true coil state.</p>
    </div>
  </div>
</section>

<section class="panel" id="panel-buzzer" role="tabpanel">
  <div class="page-head">
    <div><h2>Buzzer — Audio Alert</h2><p>Piezo buzzer on <code>GPIO26</code> driven by LEDC PWM at 2093 Hz (C7) with 8-bit duty for volume. Plays the selected pattern while any enabled sensor trigger is active; auto-mutes after the configured continuous-alert limit.</p></div>
    <span class="badge big ok" id="buzzerBadge">READY</span>
  </div>
  <div class="settings-grid">
    <form class="card form-card" data-section="buzzer" autocomplete="off">
      <div class="section-sep"><span>Buzzer</span></div>
      <div class="field-grid">
        <div class="toggle-row full" style="grid-column:1/-1"><div class="tr-text"><b>Buzzer enabled</b><small>Master switch for all acoustic alerts</small></div><label class="switch"><input type="checkbox" name="enabled"><span class="slider-ui"></span></label></div>
        <div class="field"><label for="buzz-pattern">Alert pattern</label><select class="text-input" id="buzz-pattern" name="pattern" data-num><option value="0">Single beep</option><option value="1" selected>Double beep</option><option value="2">Triple beep</option><option value="3">Long tone (1.2 s)</option><option value="4">Pulse train ×5</option></select></div>
        <div class="field"><label for="buzz-maxdur">Continuous alert limit</label><select class="text-input" id="buzz-maxdur" name="maxAlertIdx" data-num><option value="0">5 seconds</option><option value="1" selected>15 seconds</option><option value="2">30 seconds</option><option value="3">60 seconds</option><option value="4">Unlimited</option></select><span class="hint">Silences the beeper until the condition clears</span></div>
        <div class="field full"><label for="buzz-volume">Volume (PWM duty)</label><div class="range-wrap"><input type="range" id="buzz-volume" name="volume" min="8" max="255" step="1" data-num><span class="range-val" id="buzz-volume-val">160</span></div><span class="hint">For DC-type active buzzers use ≥ 200; passive piezo tracks actual duty</span></div>
      </div>

      <div class="section-sep"><span>Test</span></div>
      <div class="field-grid">
        <div class="field"><label for="buzz-testpat">Pattern to test</label><select class="text-input" id="buzz-testpat"><option value="0">Single beep</option><option value="1" selected>Double beep</option><option value="2">Triple beep</option><option value="3">Long tone</option><option value="4">Pulse train ×5</option></select></div>
        <div class="field"><label>&nbsp;</label><button type="button" class="btn btn-ghost" id="btnBuzzerTest"><svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M11 5 6.5 9H3v6h3.5L11 19V5Z"/><path d="M15 9.5a4 4 0 0 1 0 5M18 6.8a8 8 0 0 1 0 10.4"/></svg>Play test</button></div>
      </div>
      <div class="test-out" id="test-buzzer"></div>

      <div class="form-actions">
        <button type="button" class="btn btn-primary" data-save="buzzer"><svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2Z"/><path d="M17 21v-8H7v8M7 3v5h8"/></svg>Save changes</button>
        <span class="save-hint">commit → NVS only when values differ</span>
      </div>
    </form>

    <aside class="side-col">
      <div class="card live-card">
        <div class="card-title">Live State</div>
        <div class="live-rows">
          <div class="live-row"><span>Beeper</span><b id="lv-buzz">IDLE</b></div>
          <div class="live-row"><span>Pattern</span><b id="lv-buzzpat">Double beep</b></div>
          <div class="live-row"><span>Alert sources</span><b>BME · LUX · GAS · SOIL · RAIN</b></div>
        </div>
      </div>
      <div class="card note-card">
        <h4><svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="12" cy="12" r="9"/><path d="M12 8v5M12 16.5h.01"/></svg>Wiring notes</h4>
        <p><code>GPIO26 → buzzer signal</code>, <code>GND → GND</code>. Active buzzers draw more current than a GPIO can source from a weak supply — use a transistor or a module with onboard driver, and never feed 5 V logic back into the pin.</p>
      </div>
    </aside>
  </div>
</section>

<section class="panel" id="panel-led" role="tabpanel">
  <div class="page-head">
    <div><h2>Status LED — Anode on GPIO33</h2><p>High-side anode LED with series resistor on <code>GPIO33</code>, driven by LEDC PWM (8-bit) for dimmable brightness. Modes: follow alerts, always on, or off. Blink 2 Hz while any trigger is active.</p></div>
    <span class="badge big ok" id="ledBadge">READY</span>
  </div>
  <div class="settings-grid">
    <form class="card form-card" data-section="led" autocomplete="off">
      <div class="section-sep"><span>LED</span></div>
      <div class="field-grid">
        <div class="toggle-row full" style="grid-column:1/-1"><div class="tr-text"><b>LED enabled</b><small>Master switch for the status anode</small></div><label class="switch"><input type="checkbox" name="enabled"><span class="slider-ui"></span></label></div>
        <div class="field"><label for="led-mode">Mode</label><select class="text-input" id="led-mode" name="mode" data-num><option value="0" selected>Auto — on alert only</option><option value="1">Always on</option><option value="2">Off</option></select></div>
        <div class="toggle-row"><div class="tr-text"><b>Blink on alert</b><small>2 Hz square wave while a trigger is active</small></div><label class="switch"><input type="checkbox" name="blink"><span class="slider-ui"></span></label></div>
        <div class="field full"><label for="led-brightness">Brightness (PWM duty)</label><div class="range-wrap"><input type="range" id="led-brightness" name="brightness" min="1" max="255" step="1" data-num><span class="range-val" id="led-brightness-val">128</span></div><span class="hint">Use ≤ 40 for panel-mount high-efficiency LEDs</span></div>
      </div>

      <div class="section-sep"><span>Test</span></div>
      <div class="field-grid">
        <div class="field"><label>&nbsp;</label><button type="button" class="btn btn-ghost" id="btnLedTest"><svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M9 18h6M10 21h4"/><path d="M12 3a6.5 6.5 0 0 1 4 11.6c-.9.7-1 1.4-1 2.4H9c0-1-.1-1.7-1-2.4A6.5 6.5 0 0 1 12 3Z"/></svg>Blink ×3</button></div>
      </div>
      <div class="test-out" id="test-led"></div>

      <div class="form-actions">
        <button type="button" class="btn btn-primary" data-save="led"><svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2Z"/><path d="M17 21v-8H7v8M7 3v5h8"/></svg>Save changes</button>
        <span class="save-hint">commit → NVS only when values differ</span>
      </div>
    </form>

    <aside class="side-col">
      <div class="card live-card">
        <div class="card-title">Live State</div>
        <div class="live-rows">
          <div class="live-row"><span>Lamp</span><b id="lv-led">OFF</b></div>
          <div class="live-row"><span>Mode</span><b id="lv-ledmode">Auto — on alert only</b></div>
          <div class="live-row"><span>Alert sources</span><b>BME · LUX · GAS · SOIL · RAIN</b></div>
        </div>
      </div>
      <div class="card note-card">
        <h4><svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="12" cy="12" r="9"/><path d="M12 8v5M12 16.5h.01"/></svg>Wiring notes</h4>
        <p>Anode to <code>GPIO33</code> through a 220–330 Ω resistor; cathode to GND. GPIO33 is a safe output (no boot-strap role) and shares nothing with the input-only ADC pins 34/35.</p>
      </div>
    </aside>
  </div>
</section>
<section class="panel" id="panel-system" role="tabpanel">
  <div class="page-head">
    <div><h2>System Status — Diagnostics</h2><p>Live silicon, memory and NVS integrity telemetry. Values refresh every 2 s from <code>/api/state</code>. NVS commits are A/B slotted with CRC-32 guard and skipped entirely when nothing changed — safe for decade-scale deployments.</p></div>
    <span class="badge big ok" id="systemBadge">NOMINAL</span>
  </div>
  <div class="status-grid">
    <article class="card">
      <div class="card-title">Telemetry <small>64-bit counters · no rollover before year 2554</small></div>
      <div class="kv">
        <div class="kv-row"><span>Firmware version</span><b id="sy-fw">—</b></div>
        <div class="kv-row"><span>Uptime (64-bit)</span><b id="sy-uptime">—</b></div>
        <div class="kv-row"><span>Boot count (NVS)</span><b id="sy-boots">—</b></div>
        <div class="kv-row"><span>Last reset reason</span><b id="sy-reset">—</b></div>
        <div class="kv-row"><span>Chip model</span><b id="sy-chip">—</b></div>
        <div class="kv-row"><span>Chip revision</span><b id="sy-rev">—</b></div>
        <div class="kv-row"><span>CPU frequency</span><b id="sy-cpu">—</b></div>
        <div class="kv-row"><span>Flash size</span><b id="sy-flash">—</b></div>
        <div class="kv-row"><span>ESP-IDF / Arduino SDK</span><b id="sy-sdk">—</b></div>
        <div class="kv-row"><span>Free heap</span><b id="sy-heap">—</b></div>
        <div class="kv-row"><span>Lowest heap watermark</span><b id="sy-minheap">—</b></div>
        <div class="kv-row"><span>Heap fragmentation</span><b id="sy-frag">—</b></div>
        <div class="kv-row"><span>Loop rate</span><b id="sy-loophz">—</b></div>
        <div class="kv-row"><span>AP clients connected</span><b id="sy-clients">—</b></div>
        <div class="kv-row"><span>AP address</span><b id="sy-ip">—</b></div>
        <div class="kv-row"><span>AP MAC</span><b id="sy-mac">—</b></div>
        <div class="kv-row"><span>Config slot / CRC-32</span><b id="sy-cfgcrc">—</b></div>
        <div class="kv-row"><span>NVS commits this boot</span><b id="sy-nvswrites">—</b></div>
      </div>
      <div class="form-actions" style="border:none;padding-top:12px">
        <button class="btn btn-ghost btn-sm" id="btnRescanI2c"><svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="11" cy="11" r="7"/><path d="m21 21-4.3-4.3"/></svg>Rescan I2C bus</button>
        <button class="btn btn-ghost btn-sm" id="btnReboot"><svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M21 12a9 9 0 1 1-2.64-6.36M21 3v6h-6"/></svg>Reboot controller</button>
      </div>
      <div class="test-out" id="test-i2c"></div>
    </article>

    <div class="side-col">
      <article class="card">
        <div class="card-title">Health Checks <small>live</small></div>
        <div class="health-list" id="healthList"></div>
      </article>

      <article class="factory-box">
        <h4>Danger zone</h4>
        <p>Factory reset wipes every stored configuration block (A/B slots, sequence counter, boot statistics) and reboots into firmware defaults. Alternatively, hold the onboard <b>GPIO0 BOOT</b> button for <b>5 seconds</b> — the OLED shows a progress bar.</p>
        <button class="btn btn-danger" id="btnFactoryReset" style="width:100%;justify-content:center"><svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M10.3 3.9 1.8 18a2 2 0 0 0 1.7 3h17a2 2 0 0 0 1.7-3L13.7 3.9a2 2 0 0 0-3.4 0Z"/><path d="M12 9.5v5M12 17.5h.01"/></svg>Factory reset</button>
      </article>
    </div>
  </div>
</section>

<section class="panel" id="panel-wifiap" role="tabpanel">
  <div class="page-head">
    <div><h2>WiFi Access Point — Captive Portal</h2><p>SoftAP settings. All HTTP traffic from connected clients is intercepted by the DNS hijack and redirected to <code>http://192.168.4.1</code>. Saving applies the new radio configuration live — no reboot required; clients reconnect to the new network.</p></div>
    <span class="badge big ok" id="wifiBadge">AP ACTIVE</span>
  </div>
  <div class="wifi-grid">
    <form class="card form-card" data-section="wifiap" autocomplete="off">
      <div class="section-sep"><span>Network identity</span></div>
      <div class="field-grid">
        <div class="field full"><label for="wifi-ssid">SSID (1–32 chars)</label><input class="text-input" id="wifi-ssid" name="ssid" type="text" maxlength="32" spellcheck="false" autocapitalize="off" placeholder="ESP32_Multi_Smart_Garden"><span class="hint">Default: <code class="mono" style="color:#6ee7b7">ESP32_Multi_Smart_Garden</code></span></div>
        <div class="field full"><label for="wifi-pass">WPA2 password (8–63 chars)</label>
          <div class="pass-wrap"><input class="text-input" id="wifi-pass" name="pass" type="password" minlength="8" maxlength="63" spellcheck="false" autocapitalize="off" autocomplete="new-password" placeholder="12345678"><button type="button" class="pass-toggle" id="btnTogglePass">SHOW</button></div>
          <div class="strength"><i id="passStrength"></i></div>
          <div class="inline-err" id="wifi-err"></div>
        </div>
        <div class="field"><label for="wifi-channel">Channel</label><select class="text-input" id="wifi-channel" name="channel" data-num><option value="1">1</option><option value="2">2</option><option value="3">3</option><option value="4">4</option><option value="5">5</option><option value="6" selected>6 — default</option><option value="7">7</option><option value="8">8</option><option value="9">9</option><option value="10">10</option><option value="11">11</option><option value="12">12</option><option value="13">13</option></select></div>
        <div class="field"><label for="wifi-maxconn">Max clients</label><select class="text-input" id="wifi-maxconn" name="maxConn" data-num><option value="1">1</option><option value="2">2</option><option value="3">3</option><option value="4" selected>4</option></select></div>
        <div class="toggle-row full" style="grid-column:1/-1"><div class="tr-text"><b>Hide SSID</b><small>Clients must know the exact SSID; still reachable via captive portal flow</small></div><label class="switch"><input type="checkbox" name="hidden"><span class="slider-ui"></span></label></div>
      </div>

      <div class="section-sep"><span>Captive portal</span></div>
      <p class="hint" style="font-size:11px;color:var(--text-dim);line-height:1.7">The onboard DNS server answers <b>every</b> query with <code class="mono" style="color:#6ee7b7">192.168.4.1</code>, and OS connectivity-check endpoints (Apple CNA, Android/Chrome <code>generate_204</code>, Windows NCSI) all reply with a redirect to the portal. Any path that is not an API route lands on this page.</p>

      <div class="form-actions">
        <button type="button" class="btn btn-primary" data-save="wifiap"><svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M5 13a10 10 0 0 1 14 0M8.5 16.5a5.5 5.5 0 0 1 7 0M2 9a15 15 0 0 1 20 0"/><path d="M12 20h.01"/></svg>Save &amp; apply live</button>
        <span class="save-hint">AP restarts · clients re-join automatically</span>
      </div>
    </form>

    <aside class="side-col">
      <div class="card live-card">
        <div class="card-title">Access Point <small>live</small></div>
        <div class="live-rows">
          <div class="live-row"><span>SSID</span><b id="lv-ssid">ESP32_Multi_Smart_Garden</b></div>
          <div class="live-row"><span>Address</span><b>192.168.4.1</b></div>
          <div class="live-row"><span>Clients connected</span><b id="lv-clients">0</b></div>
          <div class="live-row"><span>Channel</span><b id="lv-channel">6</b></div>
          <div class="live-row"><span>Security</span><b>WPA2-PSK</b></div>
        </div>
      </div>
      <div class="card note-card">
        <h4><svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="12" cy="12" r="9"/><path d="M12 8v5M12 16.5h.01"/></svg>Recovery</h4>
        <p>Forgot the password? Hold <code>GPIO0 BOOT</code> for <b>5 s</b> — WiFi credentials return to <code>ESP32_Multi_Smart_Garden</code> / <code>12345678</code> along with every other setting.</p>
      </div>
    </aside>
  </div>
</section>

</div><footer class="foot"><span>ESP32 Multi Smart Garden · Unified Firmware <b id="footFw">2.4.1</b></span><span>Captive portal → 192.168.4.1 · NVS A/B + CRC-32</span></footer>
</main>

<div class="modal-overlay" id="modalFactory" role="dialog" aria-modal="true" aria-labelledby="mf-title">
  <div class="modal">
    <h3><svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="#f87171" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M10.3 3.9 1.8 18a2 2 0 0 0 1.7 3h17a2 2 0 0 0 1.7-3L13.7 3.9a2 2 0 0 0-3.4 0Z"/><path d="M12 9.5v5M12 17.5h.01"/></svg><span id="mf-title">Factory reset?</span></h3>
    <p>This erases all stored configuration and boot statistics, then reboots. Type <b class="mono" style="color:#fecaca">RESET</b> to confirm.</p>
    <input class="text-input" id="factoryConfirmInput" placeholder="RESET" autocomplete="off" spellcheck="false" style="margin-top:12px;text-transform:uppercase">
    <div class="modal-actions"><button class="btn btn-ghost" id="btnFactoryCancel">Cancel</button><button class="btn btn-danger" id="btnFactoryConfirm" disabled>Erase &amp; reboot</button></div>
  </div>
</div>

<div class="modal-overlay" id="modalReboot" role="dialog" aria-modal="true" aria-labelledby="mr-title">
  <div class="modal">
    <h3><svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="#7dd3fc" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M21 12a9 9 0 1 1-2.64-6.36M21 3v6h-6"/></svg><span id="mr-title">Reboot controller?</span></h3>
    <p>The ESP32 will restart. The AP disappears for a few seconds, sensor history buffers clear, and the captive portal comes back automatically.</p>
    <div class="modal-actions"><button class="btn btn-ghost" id="btnRebootCancel">Cancel</button><button class="btn btn-primary" id="btnRebootConfirm">Reboot now</button></div>
  </div>
</div>

<div class="modal-overlay" id="modalAllOff" role="dialog" aria-modal="true" aria-labelledby="ma-title">
  <div class="modal">
    <h3><svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="#fcd34d" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M12 3v9"/><path d="M6.3 6.5a8 8 0 1 0 11.4 0"/></svg><span id="ma-title">Force all relays OFF?</span></h3>
    <p>Manually forces every coil — including GPIO25 — to the OFF state until each channel's mode is set again. AUTO sources still determine state on the next evaluation if left in AUTO… so this also switches every channel to <b>Manual OFF</b>.</p>
    <div class="modal-actions"><button class="btn btn-ghost" id="btnAllOffCancel">Cancel</button><button class="btn btn-danger" id="btnAllOffConfirm">Force all OFF</button></div>
  </div>
</div>

<div class="toast" id="toast" role="status" aria-live="polite"></div>
<script>
"use strict";
const $ = (s, r) => (r || document).querySelector(s);
const $$ = (s, r) => Array.from((r || document).querySelectorAll(s));
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const fmtUptime = (sec) => {
  sec = Math.max(0, Math.floor(sec));
  const d = Math.floor(sec / 86400), h = Math.floor((sec % 86400) / 3600), m = Math.floor((sec % 3600) / 60), s = sec % 60;
  const pad = (n) => String(n).padStart(2, "0");
  return (d > 0 ? d + "d " : "") + pad(h) + ":" + pad(m) + ":" + pad(s);
};
const toast = (msg, bad) => {
  const t = $("#toast");
  t.textContent = msg;
  t.classList.toggle("bad", !!bad);
  t.classList.add("show");
  clearTimeout(toast._tm);
  toast._tm = setTimeout(() => t.classList.remove("show"), 3400);
};

const state = {
  demo: false,
  lastState: null,
  config: null,
  serverUptime: 0,
  uptimeAnchor: Date.now(),
  sparks: {}
};

const DEMO = {
  state: () => ({
    fw: "2.4.1", uptimeSec: 1183453, boots: 7, reset: "poweron", chip: "ESP32-D0WDQ6", rev: "1", cpuMHz: 240, flashKB: 4096, sdk: "v5.1.2 (demo)",
    heap: 168432, minHeap: 149208, fragPct: 14, loopHz: 880, clients: 1, ip: "192.168.4.1", mac: "24:6F:28:A1:B2:C3",
    cfgSlot: "A", cfgCrcOk: true, nvsWrites: 3, alerts: ["Soil moisture 22% below 30% — GPIO5 pump relay energized"],
    bme: { present: true, enabled: true, t: 27.4, h: 58.2, p: 1011.8, trig: false },
    bh: { present: true, enabled: true, lux: 842, trig: false },
    mq: { enabled: true, pct: 11.6, raw: 476, trig: false },
    soil: { enabled: true, pct: 22.4, raw: 2980, trig: true },
    rain: { enabled: true, pct: 3.1, raw: 3680, trig: false },
    relays: [{ on: 1, mode: 0 }, { on: 0, mode: 0 }, { on: 0, mode: 0 }, { on: 0, mode: 0 }, { on: 1, mode: 0 }, { on: 0, mode: 0 }],
    buzzer: { playing: false }, led: { on: true }, oled: { on: true }, ssid: "ESP32_Multi_Smart_Garden"
  }),
  config: () => ({
    fw: "2.4.1",
    oled: { enabled: 1, flip: 0, contrast: 160, timeoutIdx: 0, cycleIdx: 1, pageOverview: 1, pageStatus: 1, pageAlerts: 1 },
    bme: { enabled: 1, intervalIdx: 1, tempOffset: 0.0, humOffset: 0.0, presOffset: 0.0, metric: 0, condition: 0, threshold: 35.0, hysteresis: 1.0, cooldownSec: 30, relay: 1, unified: 0, buzzer: 1, led: 1, trigger: 0 },
    bh: { enabled: 1, intervalIdx: 1, condition: 1, threshold: 400, hysteresis: 50, cooldownSec: 30, relay: 0, unified: 0, buzzer: 0, led: 0, trigger: 0 },
    mq: { enabled: 1, intervalIdx: 2, samplesIdx: 1, condition: 0, thresholdPct: 40.0, hysteresis: 5.0, cooldownSec: 60, relay: 1, unified: 1, buzzer: 1, led: 1, trigger: 0 },
    soil: { enabled: 1, intervalIdx: 1, dryRaw: 3100, wetRaw: 1400, condition: 1, thresholdPct: 30, hysteresis: 5, cooldownSec: 60, relay: 1, unified: 1, buzzer: 1, led: 1, trigger: 1 },
    rain: { enabled: 1, intervalIdx: 1, dryRaw: 3500, wetRaw: 900, condition: 0, thresholdPct: 60, hysteresis: 8, cooldownSec: 60, relay: 1, unified: 1, buzzer: 1, led: 1, trigger: 0 },
    buzzer: { enabled: 1, pattern: 1, volume: 160, maxAlertIdx: 1 },
    led: { enabled: 1, mode: 0, brightness: 128, blink: 1 },
    relays: { enabled: [1, 1, 1, 1, 1, 1], mode: [0, 0, 0, 0, 0, 0], activeLow: [1, 1, 1, 1, 1, 1], sources: [31, 1, 2, 4, 8, 16] },
    wifiap: { ssid: "ESP32_Multi_Smart_Garden", pass: "12345678", channel: 6, hidden: 0, maxConn: 4 }
  })
};

const apiFetch = async (url, opts) => {
  const ctrl = new AbortController();
  const tm = setTimeout(() => ctrl.abort(), 4500);
  try {
    const res = await fetch(url, Object.assign({ signal: ctrl.signal }, opts || {}));
    clearTimeout(tm);
    return res;
  } catch (e) {
    clearTimeout(tm);
    throw e;
  }
};
const apiJson = async (url, opts) => {
  const res = await apiFetch(url, opts);
  let body = null;
  try { body = await res.json(); } catch (e) { body = null; }
  if (!res.ok) {
    const msg = (body && (body.err || body.error)) || ("HTTP " + res.status);
    const err = new Error(msg);
    err.status = res.status;
    throw err;
  }
  return body;
};

const RELAY_DEFS = [
  { key: "UNIFIED", pin: 25, sub: "any enabled source" },
  { key: "BME280", pin: 15, sub: "temperature / humidity / pressure" },
  { key: "BH1750", pin: 2, sub: "ambient light lux" },
  { key: "MQ-2 GAS", pin: 4, sub: "LPG · smoke · alcohol" },
  { key: "SOIL", pin: 5, sub: "moisture pump driver" },
  { key: "RAIN", pin: 18, sub: "precipitation actuator" }
];
const SRC_BITS = [{ bit: 0, label: "BME" }, { bit: 1, label: "LUX" }, { bit: 2, label: "GAS" }, { bit: 3, label: "SOIL" }, { bit: 4, label: "RAIN" }];
const BUZZ_NAMES = ["Single beep", "Double beep", "Triple beep", "Long tone", "Pulse ×5"];
const LED_MODES = ["Auto — on alert only", "Always on", "Off"];

const PAGE_TITLES = { dashboard: "Dashboard", bme: "BME280 Settings", bh: "BH1750 Settings", mq: "MQ-2 Gas Settings", soil: "Soil Moisture Settings", rain: "Rain Sensor Settings", oled: "SSD1306 OLED Settings", relay: "Relay Matrix", buzzer: "Buzzer Settings", led: "Status LED Settings", system: "System Status", wifiap: "WiFi AP Settings" };

const showPage = (name) => {
  if (!$("#panel-" + name)) return;
  $$(".panel").forEach(p => p.classList.toggle("active", p.id === "panel-" + name));
  $$(".nav-item").forEach(n => n.classList.toggle("active", n.dataset.page === name));
  $("#crumbPage").textContent = PAGE_TITLES[name] || name;
  $("#sidebar").classList.remove("open");
  $("#scrim").classList.remove("show");
  try { localStorage.setItem("sg_page", name); } catch (e) {}
  if (name === "oled") drawOledPreview();
  try { window.scrollTo({ top: 0, behavior: "smooth" }); } catch (e) {}
};
$$(".nav-item").forEach(btn => btn.addEventListener("click", () => showPage(btn.dataset.page)));
$$("[data-goto]").forEach(btn => btn.addEventListener("click", () => showPage(btn.dataset.goto)));
$("#btnMenu").addEventListener("click", () => { $("#sidebar").classList.toggle("open"); $("#scrim").classList.toggle("show", $("#sidebar").classList.contains("open")); });
$("#scrim").addEventListener("click", () => { $("#sidebar").classList.remove("open"); $("#scrim").classList.remove("show"); });
document.addEventListener("keydown", (e) => { if (e.key === "Escape") { $("#sidebar").classList.remove("open"); $("#scrim").classList.remove("show"); closeModal($("#modalFactory")); closeModal($("#modalReboot")); closeModal($("#modalAllOff")); } });

const openModal = (m) => m.classList.add("show");
const closeModal = (m) => { if (m) m.classList.remove("show"); };
$$(".modal-overlay").forEach(m => m.addEventListener("click", (e) => { if (e.target === m) closeModal(m); }));

function renderSpark(canvas, hist, color) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth || 220, h = canvas.clientHeight || 46;
  if (canvas.width !== Math.floor(w * dpr)) { canvas.width = Math.floor(w * dpr); canvas.height = Math.floor(h * dpr); }
  const ctx = canvas.getContext ? canvas.getContext("2d") : null;
  if (!ctx) return;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  if (hist.length < 2) return;
  let min = Infinity, max = -Infinity;
  for (const v of hist) { if (v < min) min = v; if (v > max) max = v; }
  if (max - min < 1e-6) { max = min + 1; }
  const pad = 3;
  ctx.beginPath();
  hist.forEach((v, i) => {
    const x = (i / (hist.length - 1)) * (w - pad * 2) + pad;
    const y = h - pad - ((v - min) / (max - min)) * (h - pad * 2);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.6;
  ctx.shadowColor = color;
  ctx.shadowBlur = 6;
  ctx.stroke();
  ctx.shadowBlur = 0;
  ctx.lineTo(w - pad, h); ctx.lineTo(pad, h); ctx.closePath();
  const grad = ctx.createLinearGradient(0, 0, 0, h);
  grad.addColorStop(0, color + "44");
  grad.addColorStop(1, color + "00");
  ctx.fillStyle = grad;
  ctx.fill();
}
const pushSpark = (id, v) => {
  if (v === null || v === undefined || isNaN(v)) return;
  const s = (state.sparks[id] = state.sparks[id] || []);
  s.push(v);
  if (s.length > 48) s.shift();
  const c = document.getElementById(id);
  if (c) renderSpark(c, s, c.dataset.color || "#34d399");
};

const setBadge = (name, mode, text) => {
  $$('[data-badge="' + name + '"]').forEach(b => { b.className = "badge " + (b.classList.contains("big") ? "big " : "") + mode; b.textContent = text; });
};
const meterSet = (id, pct, warnAt, dangerAt) => {
  const el = document.getElementById(id);
  if (!el) return;
  const bar = el.querySelector("i");
  bar.style.width = clamp(pct, 0, 100).toFixed(1) + "%";
  el.classList.toggle("warn", pct >= warnAt && pct < dangerAt);
  el.classList.toggle("danger", pct >= dangerAt);
};
const skyText = (lux) => lux < 50 ? "Dark" : lux < 400 ? "Dim indoor" : lux < 2500 ? "Bright room" : lux < 20000 ? "Overcast day" : "Full sun";
const soilText = (p) => p < 20 ? "PARCHED" : p < 30 ? "DRY" : p <= 60 ? "MOIST" : p <= 80 ? "WET" : "SATURATED";
const rainText = (p) => p < 12 ? "DRY" : p < 40 ? "SPRINKLE" : p < 70 ? "RAINING" : "DOWNPOUR";
const dewAlt = (pHpa) => 44330 * (1 - Math.pow(pHpa / 1013.25, 0.190284));

const renderRelayChips = (relays) => {
  const grid = $("#relayGrid");
  if (!grid.childElementCount) {
    grid.innerHTML = RELAY_DEFS.map((d, i) =>
      '<div class="relay-chip" id="relayChip' + i + '"><div class="rc-top"><span class="rc-name">' + d.key + '</span><span class="rc-pin">GPIO ' + d.pin + '</span></div><div class="rc-bottom"><span class="rc-mode">AUTO</span><span class="rc-state">OFF</span></div></div>'
    ).join("");
  }
  RELAY_DEFS.forEach((d, i) => {
    const r = (relays && relays[i]) || { on: 0, mode: 0 };
    const chip = document.getElementById("relayChip" + i);
    chip.classList.toggle("on", !!r.on);
    chip.querySelector(".rc-mode").textContent = r.mode === 1 ? "MANUAL·ON" : r.mode === 2 ? "MANUAL·OFF" : "AUTO";
    chip.querySelector(".rc-state").textContent = r.on ? "ON" : "OFF";
  });
};

const renderAlerts = (alerts) => {
  const banner = $("#alertBanner");
  if (alerts && alerts.length) { $("#alertList").textContent = alerts.join("  ·  "); banner.classList.add("show"); }
  else banner.classList.remove("show");
};

const renderHealth = (s) => {
  const items = [];
  const add = (ok, warn, label, detail) => items.push({ cls: ok ? "ok" : (warn ? "warn" : "bad"), label, detail });
  add(s.bme ? s.bme.present : false, false, "BME280 on I2C bus", s.bme && s.bme.present ? "found on bus" : "not detected");
  add(s.bh ? s.bh.present : false, false, "BH1750 on I2C bus", s.bh && s.bh.present ? "found 0x23" : "not detected");
  add(true, false, "SSD1306 OLED on I2C bus", "128×64 @ 0x3C");
  add(!!s.cfgCrcOk, false, "Config CRC-32 + A/B slots", s.cfgCrcOk ? ("slot " + (s.cfgSlot || "A") + " valid") : "recovered defaults");
  add(s.fragPct < 45, s.fragPct < 70, "Heap fragmentation", s.fragPct + "% largest-block loss");
  add(s.heap > 60000, s.heap > 30000, "Free heap headroom", Math.round(s.heap / 1024) + " KB free");
  const check = '<svg viewBox="0 0 24 24"><path d="M4 12.5 9.5 18 20 6.5"/></svg>';
  const warn = '<svg viewBox="0 0 24 24"><path d="M12 3.5 2.8 19.4h18.4L12 3.5Z"/><path d="M12 9.5v5M12 17.2h.01"/></svg>';
  $("#healthList").innerHTML = items.map(it => '<div class="health-item ' + it.cls + '">' + (it.cls === "ok" ? check : warn) + '<span>' + it.label + '</span><b>' + it.detail + '</b></div>').join("");
};

const renderState = (s) => {
  state.lastState = s;
  state.serverUptime = s.uptimeSec || 0;
  state.uptimeAnchor = Date.now();
  $("#uptimeChip").textContent = "UP " + fmtUptime(state.serverUptime);

  const bme = s.bme || {}, bh = s.bh || {}, mq = s.mq || {}, soil = s.soil || {}, rain = s.rain || {};
  $("#v-temp").textContent = bme.enabled && bme.present ? bme.t.toFixed(1) : "--.-";
  $("#v-humidity").textContent = bme.enabled && bme.present ? bme.h.toFixed(1) + " %" : "--";
  $("#v-pressure").textContent = bme.enabled && bme.present ? bme.p.toFixed(1) + " hPa" : "-- hPa";
  $("#v-altitude").textContent = bme.enabled && bme.present ? dewAlt(bme.p).toFixed(0) + " m" : "-- m";
  $("#v-lux").textContent = bh.enabled && bh.present ? (bh.lux < 10 ? bh.lux.toFixed(1) : Math.round(bh.lux)) : "----";
  $("#v-sky").textContent = bh.enabled && bh.present ? skyText(bh.lux) : "—";
  $("#v-gas").textContent = mq.enabled ? mq.pct.toFixed(1) : "--.-";
  $("#v-gasraw").textContent = mq.enabled ? String(Math.round(mq.raw)).padStart(4, "0") + " / 4095" : "---- / 4095";
  $("#v-gasvolt").textContent = mq.enabled ? ((mq.raw / 4095) * 3.3).toFixed(2) + " V" : "--";
  $("#v-soil").textContent = soil.enabled ? soil.pct.toFixed(1) : "--.-";
  $("#v-soilraw").textContent = soil.enabled ? String(Math.round(soil.raw)).padStart(4, "0") + " / 4095" : "---- / 4095";
  $("#v-soilstat").textContent = soil.enabled ? soilText(soil.pct) : "—";
  $("#v-rain").textContent = rain.enabled ? rain.pct.toFixed(1) : "--.-";
  $("#v-rainraw").textContent = rain.enabled ? String(Math.round(rain.raw)).padStart(4, "0") + " / 4095" : "---- / 4095";
  $("#v-rainstat").textContent = rain.enabled ? rainText(rain.pct) : "—";

  setBadge("bme", !bme.enabled ? "off" : (!bme.present ? "err" : (bme.trig ? "trig" : "ok")), !bme.enabled ? "Disabled" : (!bme.present ? "No I2C" : (bme.trig ? "Trigger" : "OK")));
  setBadge("bh", !bh.enabled ? "off" : (!bh.present ? "err" : (bh.trig ? "trig" : "ok")), !bh.enabled ? "Disabled" : (!bh.present ? "No I2C" : (bh.trig ? "Trigger" : "OK")));
  setBadge("mq", !mq.enabled ? "off" : (mq.trig ? "trig" : "ok"), !mq.enabled ? "Disabled" : (mq.trig ? "Trigger" : "OK"));
  setBadge("soil", !soil.enabled ? "off" : (soil.trig ? "trig" : "ok"), !soil.enabled ? "Disabled" : (soil.trig ? "Trigger" : "OK"));
  setBadge("rain", !rain.enabled ? "off" : (rain.trig ? "trig" : "ok"), !rain.enabled ? "Disabled" : (rain.trig ? "Trigger" : "OK"));

  meterSet("meter-bme", bme.enabled && bme.present ? clamp((bme.t / 50) * 100, 0, 100) : 0, 70, 88);
  meterSet("meter-bh", bh.enabled && bh.present ? clamp((Math.log10(Math.max(1, bh.lux)) / 5) * 100, 0, 100) : 0, 80, 101);
  meterSet("meter-mq", mq.enabled ? mq.pct : 0, 55, 80);
  meterSet("meter-soil", soil.enabled ? soil.pct : 0, 0, -101);
  document.querySelector("#meter-soil i").style.background = soil.pct < 20 ? "linear-gradient(90deg,#ef4444,#f87171)" : soil.pct < 35 ? "linear-gradient(90deg,#f59e0b,#fbbf24)" : "linear-gradient(90deg,#10b981,#22d3ee)";
  meterSet("meter-rain", rain.enabled ? rain.pct : 0, 70, 101);

  pushSpark("spark-bme", bme.enabled && bme.present ? bme.t : null);
  pushSpark("spark-bh", bh.enabled && bh.present ? bh.lux : null);
  pushSpark("spark-mq", mq.enabled ? mq.pct : null);
  pushSpark("spark-soil", soil.enabled ? soil.pct : null);
  pushSpark("spark-rain", rain.enabled ? rain.pct : null);

  renderRelayChips(s.relays);
  renderAlerts(s.alerts || []);
  $("#out-buzzer").textContent = s.buzzer && s.buzzer.playing ? "BEEPING" : "IDLE";
  $("#out-led").textContent = s.led && s.led.on ? "ON" : "OFF";
  $("#out-oled").textContent = s.oled && s.oled.on ? "ON" : "SLEEP";
  $("#kv-uptime").textContent = fmtUptime(s.uptimeSec || 0);
  $("#kv-heap").textContent = Math.round((s.heap || 0) / 1024) + " KB free";
  $("#kv-minheap").textContent = Math.round((s.minHeap || 0) / 1024) + " KB low-water";
  $("#kv-frag").textContent = (s.fragPct != null ? s.fragPct : "—") + " %";
  $("#kv-clients").textContent = String(s.clients != null ? s.clients : "—");
  $("#kv-cfg").textContent = s.cfgCrcOk ? ("SLOT " + (s.cfgSlot || "?") + " · CRC OK") : "RECOVERED → DEFAULTS";
  $("#kv-cfg").classList.toggle("good", !!s.cfgCrcOk);
  $("#kv-nvs").textContent = String(s.nvsWrites != null ? s.nvsWrites : "—");

  $("#sy-fw").textContent = s.fw || "—";
  $("#footFw").textContent = s.fw || "—";
  $("#sideFw").textContent = s.fw || "—";
  $("#sy-uptime").textContent = fmtUptime(s.uptimeSec || 0);
  $("#sy-boots").textContent = String(s.boots != null ? s.boots : "—");
  $("#sy-reset").textContent = s.reset || "—";
  $("#sy-chip").textContent = s.chip || "—";
  $("#sy-rev").textContent = s.rev != null ? String(s.rev) : "—";
  $("#sy-cpu").textContent = s.cpuMHz ? s.cpuMHz + " MHz" : "—";
  $("#sy-flash").textContent = s.flashKB ? (s.flashKB / 1024).toFixed(0) + " MB" : "—";
  $("#sy-sdk").textContent = s.sdk || "—";
  $("#sy-heap").textContent = Math.round((s.heap || 0) / 1024) + " KB (" + (s.heap || 0) + " B)";
  $("#sy-minheap").textContent = Math.round((s.minHeap || 0) / 1024) + " KB";
  $("#sy-frag").textContent = (s.fragPct != null ? s.fragPct : "—") + " %";
  $("#sy-loophz").textContent = s.loopHz ? s.loopHz + " Hz" : "—";
  $("#sy-clients").textContent = String(s.clients != null ? s.clients : "—");
  $("#sy-ip").textContent = s.ip || "—";
  $("#sy-mac").textContent = s.mac || "—";
  $("#sy-cfgcrc").textContent = s.cfgCrcOk ? ("SLOT " + (s.cfgSlot || "?") + " · CRC-32 OK") : "CRC FAIL → DEFAULTS";
  $("#sy-nvswrites").textContent = String(s.nvsWrites != null ? s.nvsWrites : "—");
  $("#lv-clients").textContent = String(s.clients != null ? s.clients : "0");
  if (s.ssid) { $("#lv-ssid").textContent = s.ssid; const sideIp = $("#sideIp"); if (sideIp) sideIp.textContent = s.ip || "192.168.4.1"; }

  $("#lv-bme-temp").textContent = bme.enabled && bme.present ? bme.t.toFixed(1) : "--.-";
  $("#lv-bme-hum").textContent = bme.enabled && bme.present ? bme.h.toFixed(1) + " %RH" : "--";
  $("#lv-bme-pres").textContent = bme.enabled && bme.present ? bme.p.toFixed(1) + " hPa" : "-- hPa";
  $("#lv-bme-trig").textContent = !bme.enabled ? "DISABLED" : (bme.trig ? "TRIGGERED" : "IDLE");
  $("#lv-bme-dev").textContent = bme.present ? "0x76 / 0x77 detected" : "not detected";
  $("#lv-bh-lux").textContent = bh.enabled && bh.present ? Math.round(bh.lux) : "----";
  $("#lv-bh-sky").textContent = bh.enabled && bh.present ? skyText(bh.lux) : "—";
  $("#lv-bh-trig").textContent = !bh.enabled ? "DISABLED" : (bh.trig ? "TRIGGERED" : "IDLE");
  $("#lv-bh-dev").textContent = bh.present ? "0x23 detected" : "not detected";
  $("#lv-mq-gas").textContent = mq.enabled ? mq.pct.toFixed(1) : "--.-";
  $("#lv-mq-raw").textContent = mq.enabled ? String(Math.round(mq.raw)).padStart(4, "0") + " / 4095" : "---- / 4095";
  $("#lv-mq-trig").textContent = !mq.enabled ? "DISABLED" : (mq.trig ? "TRIGGERED" : "IDLE");
  $("#lv-soil-pct").textContent = soil.enabled ? soil.pct.toFixed(1) : "--.-";
  $("#lv-soil-raw").textContent = soil.enabled ? String(Math.round(soil.raw)).padStart(4, "0") + " / 4095" : "---- / 4095";
  $("#lv-soil-trig").textContent = !soil.enabled ? "DISABLED" : (soil.trig ? "IRRIGATING" : "IDLE");
  $("#lv-soil-cond").textContent = (state.config && state.config.soil && state.config.soil.condition === 1) ? "below threshold" : "above threshold";
  $("#lv-rain-pct").textContent = rain.enabled ? rain.pct.toFixed(1) : "--.-";
  $("#lv-rain-raw").textContent = rain.enabled ? String(Math.round(rain.raw)).padStart(4, "0") + " / 4095" : "---- / 4095";
  $("#lv-rain-trig").textContent = !rain.enabled ? "DISABLED" : (rain.pct < 12 ? "CLEAR / DRY" : rainText(rain.pct));
  $("#lv-rain-cond").textContent = (state.config && state.config.rain && state.config.rain.condition === 0) ? "above threshold" : "below threshold";
  $("#lv-buzz").textContent = s.buzzer && s.buzzer.playing ? "SOUNDING" : "IDLE";
  $("#lv-led").textContent = s.led && s.led.on ? "ON" : "OFF";

  $("#bme-addr").textContent = bme.present ? (bme.addr ? "0x" + Number(bme.addr).toString(16).toUpperCase() : "detected") : "—";
  $("#mq-samples").textContent = ((state.config && state.config.mq && ["4×", "8×", "16×", "32×"][state.config.mq.samplesIdx || 1]) || "8×") + " avg";
  $("#soil-cal").textContent = state.config && state.config.soil ? ("cal " + state.config.soil.dryRaw + "/" + state.config.soil.wetRaw) : "cal";
  $("#rain-cal").textContent = state.config && state.config.rain ? ("cal " + state.config.rain.dryRaw + "/" + state.config.rain.wetRaw) : "cal";
  renderHealth(s);
  syncRelayStates(s.relays);
};

const connOk = (ok) => {
  const pill = $("#connPill");
  pill.classList.toggle("lost", !ok);
  $("#connText").textContent = ok ? (state.demo ? "DEMO · UI PREVIEW" : "AP · 192.168.4.1") : "CONNECTION LOST";
};

let demoDriftT = 0;
const fetchState = async () => {
  try {
    const s = await apiJson("api/state");
    if (state.demo) { state.demo = false; toast("Device reconnected — live data restored"); }
    connOk(true);
    renderState(s);
  } catch (e) {
    if (!state.demo) { state.demo = true; toast("Device unreachable — running UI demo preview", false); }
    const s = DEMO.state();
    demoDriftT += 0.35;
    s.bme.t = +(s.bme.t + Math.sin(demoDriftT * 0.7) * 0.4).toFixed(1);
    s.bme.h = clamp(s.bme.h + Math.cos(demoDriftT * 0.5) * 0.8, 20, 95);
    s.soil.pct = clamp(22.4 + Math.sin(demoDriftT * 0.22) * 9, 2, 80);
    s.soil.raw = Math.round(3100 - (s.soil.pct / 100) * (3100 - 1400));
    s.soil.trig = s.soil.pct < 30;
    s.relays[4].on = s.soil.trig ? 1 : 0;
    s.relays[0].on = s.relays[4].on;
    s.alerts = s.soil.trig ? ["Soil moisture " + s.soil.pct.toFixed(0) + "% below 30% — GPIO5 pump relay energized"] : [];
    s.buzzer.playing = false;
    s.led.on = s.soil.trig;
    s.uptimeSec = 1183453 + Math.floor(demoDriftT * 4);
    connOk(true);
    renderState(s);
  }
};

const formFields = (form) => {
  const out = {};
  $$("input[name], select[name]", form).forEach(el => {
    if (el.disabled) return;
    if (el.type === "checkbox") out[el.name] = el.checked ? 1 : 0;
    else if (el.dataset.num !== undefined) out[el.name] = el.value === "" ? 0 : Number(el.value);
    else out[el.name] = el.value;
  });
  return out;
};
const applyConfig = (cfg) => {
  state.config = cfg;
  Object.keys(cfg).forEach(section => {
    const data = cfg[section];
    if (!data || typeof data !== "object" || Array.isArray(data)) return;
    const form = document.querySelector('form[data-section="' + section + '"]');
    if (!form) return;
    Object.keys(data).forEach(key => {
      const el = form.querySelector('[name="' + key + '"]');
      if (!el) return;
      if (el.type === "checkbox") el.checked = !!Number(data[key]);
      else el.value = data[key];
      if (el.type === "range") { paintRange(el); const out = document.getElementById(el.id + "-val"); if (out) out.textContent = el.value; }
    });
  });
  $("#oled-contrast-val").textContent = String(cfg.oled.contrast);
  $("#buzz-volume-val").textContent = String(cfg.buzzer.volume);
  $("#led-brightness-val").textContent = String(cfg.led.brightness);
  $("#lv-buzzpat").textContent = BUZZ_NAMES[cfg.buzzer.pattern] || "Custom";
  $("#lv-ledmode").textContent = LED_MODES[cfg.led.mode] || "—";
  $("#lv-ssid").textContent = cfg.wifiap.ssid;
  $("#lv-channel").textContent = String(cfg.wifiap.channel);
  renderRelayMatrix(cfg.relays);
};

const fetchConfig = async () => {
  try {
    const cfg = await apiJson("api/config");
    if (state.demo) { state.demo = false; toast("Device reconnected — live configuration restored"); }
    applyConfig(cfg);
  } catch (e) {
    if (!state.demo) { state.demo = true; connOk(true); toast("Device unreachable — demo configuration loaded", false); }
    applyConfig(DEMO.config());
  }
};

const validateSection = (section, fields) => {
  if (section === "wifiap") {
    const ssid = String(fields.ssid || "").trim();
    const pass = String(fields.pass || "");
    const errEl = $("#wifi-err");
    errEl.textContent = "";
    if (ssid.length < 1 || ssid.length > 32) { errEl.textContent = "SSID must be 1–32 characters."; return "SSID must be 1–32 characters."; }
    if (pass.length < 8 || pass.length > 63) { errEl.textContent = "WPA2 password must be 8–63 characters."; return "WPA2 password must be 8–63 characters."; }
  }
  if (section === "soil" || section === "rain") {
    if (Number(fields.dryRaw) === Number(fields.wetRaw)) {
      const el = section === "soil" ? $("#test-soilcal") : $("#test-raincal");
      if (el) el.textContent = "Dry and wet raw values must differ.";
      return "dryRaw and wetRaw must differ";
    }
  }
  return null;
};

$$("[data-save]").forEach(btn => btn.addEventListener("click", async () => {
  const section = btn.dataset.save;
  const form = document.querySelector('form[data-section="' + section + '"]');
  const fields = formFields(form);
  const verdict = validateSection(section, fields);
  if (verdict) { toast(verdict, true); return; }
  const original = btn.innerHTML;
  btn.disabled = true;
  btn.innerHTML = '<svg class="spin" width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round"><path d="M21 12a9 9 0 1 1-6.2-8.56"/></svg>Saving…';
  try {
    const body = Object.assign({ section }, fields);
    if (state.demo) { await new Promise(r => setTimeout(r, 260)); state.config[section] = Object.assign({}, state.config[section], fields); toast("Saved (demo) — device would commit to NVS"); }
    else {
      const res = await apiJson("api/config", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
      toast(res.changed === false ? "No changes — NVS untouched" : "Saved & committed to NVS");
      if (section === "wifiap") setTimeout(fetchState, 2500);
    }
    await fetchConfig();
  } catch (e) {
    toast("Save failed: " + e.message, true);
  } finally {
    btn.disabled = false;
    btn.innerHTML = original;
  }
}));

$$("[data-test]").forEach(btn => btn.addEventListener("click", async () => {
  const kind = btn.dataset.test;
  const out = document.getElementById("test-" + kind);
  out.textContent = "Probing…";
  out.classList.remove("bad");
  try {
    if (state.demo) { await new Promise(r => setTimeout(r, 300)); out.textContent = "OK — demo device responds (real hardware performs live probe)"; return; }
    const res = state.demo ? {} : await apiJson("api/test/" + kind, { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" });
    if (res.ok === false) { out.textContent = "FAIL — " + (res.err || "no response"); out.classList.add("bad"); return; }
    if (kind === "bme") out.textContent = "OK — " + Number(res.t).toFixed(1) + " °C · " + Number(res.h).toFixed(1) + " %RH · " + Number(res.p).toFixed(1) + " hPa (addr 0x" + Number(res.addr).toString(16).toUpperCase() + ")";
    else if (kind === "bh") out.textContent = "OK — " + Number(res.lux).toFixed(1) + " lux";
    else if (kind === "mq") out.textContent = "OK — raw " + Math.round(res.raw) + " · " + Number(res.pct).toFixed(1) + " % gas";
    else if (kind === "soil") { out.textContent = "OK — raw " + Math.round(res.raw) + " · " + Number(res.pct).toFixed(1) + " % moisture"; const c = $("#test-soilcal"); if (c) c.textContent = ""; }
    else if (kind === "rain") { out.textContent = "OK — raw " + Math.round(res.raw) + " · " + Number(res.pct).toFixed(1) + " % precip"; const c = $("#test-raincal"); if (c) c.textContent = ""; }
  } catch (e) {
    out.textContent = "FAIL — " + e.message;
    out.classList.add("bad");
  }
}));

$("#btnBuzzerTest").addEventListener("click", async () => {
  const out = $("#test-buzzer");
  out.textContent = "Playing…";
  out.classList.remove("bad");
  try {
    if (state.demo) { await new Promise(r => setTimeout(r, 400)); out.textContent = "OK — pattern played (demo)"; return; }
    await apiJson("api/buzzer/test", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ pattern: Number($("#buzz-testpat").value) }) });
    out.textContent = "OK — pattern played";
  } catch (e) { out.textContent = "FAIL — " + e.message; out.classList.add("bad"); }
});

$("#btnLedTest").addEventListener("click", async () => {
  const out = $("#test-led");
  out.textContent = "Blinking…";
  out.classList.remove("bad");
  try {
    if (state.demo) { await new Promise(r => setTimeout(r, 900)); out.textContent = "OK — 3 blinks queued (demo)"; return; }
    await apiJson("api/led/test", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" });
    out.textContent = "OK — 3 blinks queued";
  } catch (e) { out.textContent = "FAIL — " + e.message; out.classList.add("bad"); }
});

$("#btnOledTest").addEventListener("click", async () => {
  const out = $("#test-oled");
  out.textContent = "Redrawing…";
  out.classList.remove("bad");
  try {
    if (state.demo) { await new Promise(r => setTimeout(r, 200)); out.textContent = "OK — frame pushed (demo)"; drawOledPreview(); return; }
    await apiJson("api/oled/test", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" });
    out.textContent = "OK — frame pushed to SSD1306";
    drawOledPreview();
  } catch (e) { out.textContent = "FAIL — " + e.message; out.classList.add("bad"); }
});

const renderRelayMatrix = (relays) => {
  const tbody = $("#relayRows");
  const enabled = relays.enabled || [1, 1, 1, 1, 1, 1];
  const mode = relays.mode || [0, 0, 0, 0, 0, 0];
  const activeLow = relays.activeLow || [1, 1, 1, 1, 1, 1];
  const sources = relays.sources || [31, 1, 2, 4, 8, 16];
  tbody.innerHTML = RELAY_DEFS.map((d, i) => {
    const chips = SRC_BITS.map(s => '<span class="src-chip"><input type="checkbox" id="src-' + i + "-" + s.bit + '" data-relay="' + i + '" data-bit="' + s.bit + '"' + (((sources[i] >> s.bit) & 1) ? " checked" : "") + '><label for="src-' + i + "-" + s.bit + '">' + s.label + "</label></span>").join("");
    return '<tr' + (i === 0 ? ' class="unified"' : "") + '><td class="relay-name-cell"><b>' + d.key + '</b><small>GPIO ' + d.pin + " · " + d.sub + '</small></td>'
      + '<td><label class="switch"><input type="checkbox" id="ren-' + i + '"' + (enabled[i] ? " checked" : "") + '><span class="slider-ui"></span></label></td>'
      + '<td><select class="text-input mini" id="rmode-' + i + '"><option value="0"' + (mode[i] === 0 ? " selected" : "") + '>AUTO</option><option value="1"' + (mode[i] === 1 ? " selected" : "") + '>MANUAL ON</option><option value="2"' + (mode[i] === 2 ? " selected" : "") + '>MANUAL OFF</option></select></td>'
      + '<td><select class="text-input mini" id="ract-' + i + '"><option value="1"' + (activeLow[i] ? " selected" : "") + '>LOW (active-low)</option><option value="0"' + (!activeLow[i] ? " selected" : "") + '>HIGH (active-high)</option></select></td>'
      + '<td><div class="src-chips">' + chips + '</div></td>'
      + '<td><span class="state-pill" id="rstate-' + i + '">OFF</span></td></tr>';
  }).join("");
};
const syncRelayStates = (relays) => {
  RELAY_DEFS.forEach((d, i) => {
    const pill = document.getElementById("rstate-" + i);
    if (pill && relays && relays[i]) { pill.textContent = relays[i].on ? "ON" : "OFF"; pill.classList.toggle("on", !!relays[i].on); }
  });
};
$("#btnSaveRelays").addEventListener("click", async () => {
  const btn = $("#btnSaveRelays");
  const original = btn.innerHTML;
  const enabled = [], mode = [], activeLow = [], sources = [];
  for (let i = 0; i < 6; i++) {
    enabled.push($("#ren-" + i).checked ? 1 : 0);
    mode.push(Number($("#rmode-" + i).value));
    activeLow.push(Number($("#ract-" + i).value));
    let mask = 0;
    for (let bit = 0; bit < 5; bit++) { const cb = document.getElementById("src-" + i + "-" + bit); if (cb && cb.checked) mask |= (1 << bit); }
    sources.push(mask);
  }
  btn.disabled = true;
  btn.innerHTML = '<svg class="spin" width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round"><path d="M21 12a9 9 0 1 1-6.2-8.56"/></svg>Saving…';
  try {
    if (state.demo) { await new Promise(r => setTimeout(r, 300)); state.config.relays = { enabled, mode, activeLow, sources }; toast("Relay matrix saved (demo)"); }
    else {
      const res = await apiJson("api/config", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ section: "relays", enabled, mode, activeLow, sources }) });
      toast(res.changed === false ? "No changes — NVS untouched" : "Relay matrix saved to NVS");
    }
    await fetchConfig();
    await fetchState();
  } catch (e) { toast("Save failed: " + e.message, true); }
  finally { btn.disabled = false; btn.innerHTML = original; }
});

$("#btnAllRelaysOff").addEventListener("click", () => openModal($("#modalAllOff")));
$("#btnAllOffCancel").addEventListener("click", () => closeModal($("#modalAllOff")));
$("#btnAllOffConfirm").addEventListener("click", async () => {
  closeModal($("#modalAllOff"));
  try {
    if (state.demo) { state.config.relays.mode = [2, 2, 2, 2, 2, 2]; toast("All relays forced OFF (demo)"); await fetchState(); return; }
    await apiJson("api/relays/alloff", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" });
    toast("All relays forced OFF — modes set to MANUAL OFF");
    await fetchConfig();
    await fetchState();
  } catch (e) { toast("Failed: " + e.message, true); }
});

$("#btnRescanI2c").addEventListener("click", async () => {
  const out = $("#test-i2c");
  out.textContent = "Scanning 0x08–0x77…";
  out.classList.remove("bad");
  try {
    if (state.demo) { await new Promise(r => setTimeout(r, 500)); out.textContent = "FOUND — 0x3C (OLED) · 0x23 (BH1750) · 0x76 (BME280)"; return; }
    const res = await apiJson("api/i2c-scan", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" });
    const names = { 60: "0x3C OLED", 61: "0x3D OLED", 35: "0x23 BH1750", 92: "0x5C BH1750", 118: "0x76 BME280", 119: "0x77 BME280" };
    out.textContent = res.found && res.found.length ? "FOUND — " + res.found.map(a => names[a] || ("0x" + a.toString(16).padStart(2, "0").toUpperCase())).join(" · ") : "NONE — bus is silent, check SDA/SCL wiring and pull-ups";
  } catch (e) { out.textContent = "FAIL — " + e.message; out.classList.add("bad"); }
});

$("#btnReboot").addEventListener("click", () => openModal($("#modalReboot")));
$("#btnRebootCancel").addEventListener("click", () => closeModal($("#modalReboot")));
$("#btnRebootConfirm").addEventListener("click", async () => {
  closeModal($("#modalReboot"));
  try {
    if (state.demo) { toast("Reboot skipped — demo mode"); return; }
    await apiJson("api/reboot", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" });
    toast("Rebooting — reload in a few seconds…");
    setTimeout(fetchState, 6000);
  } catch (e) { toast("Reboot failed: " + e.message, true); }
});

$("#btnFactoryReset").addEventListener("click", () => openModal($("#modalFactory")));
$("#btnFactoryCancel").addEventListener("click", () => { closeModal($("#modalFactory")); $("#factoryConfirmInput").value = ""; $("#btnFactoryConfirm").disabled = true; });
$("#factoryConfirmInput").addEventListener("input", (e) => { $("#btnFactoryConfirm").disabled = e.target.value.trim().toUpperCase() !== "RESET"; });
$("#btnFactoryConfirm").addEventListener("click", async () => {
  closeModal($("#modalFactory"));
  try {
    if (state.demo) { toast("Factory reset skipped — demo mode"); return; }
    await apiJson("api/factory-reset", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ confirm: "FACTORY_RESET" }) });
    toast("Configuration erased — device rebooting…");
    setTimeout(fetchState, 7000);
  } catch (e) { toast("Factory reset failed: " + e.message, true); }
});
$("#factoryConfirmInput").addEventListener("keydown", (e) => { if (e.key === "Enter" && !$("#btnFactoryConfirm").disabled) $("#btnFactoryConfirm").click(); });

const paintRange = (el) => {
  const min = Number(el.min || 0), max = Number(el.max || 100), v = Number(el.value);
  const pct = ((v - min) / (max - min)) * 100;
  el.style.background = "linear-gradient(90deg,#10b981 " + pct + "%,#16241d " + pct + "%)";
};
$$("input[type=range]").forEach(el => {
  paintRange(el);
  el.addEventListener("input", () => {
    paintRange(el);
    const out = document.getElementById(el.id + "-val");
    if (out) out.textContent = el.value;
    if (el.id === "oled-contrast") drawOledPreview();
  });
});
const oledFlipInput = document.querySelector('form[data-section="oled"] input[name="flip"]');
if (oledFlipInput) oledFlipInput.addEventListener("change", () => drawOledPreview());

const passEl = $("#wifi-pass");
const strengthEl = $("#passStrength");
const scorePass = (p) => {
  let score = 0;
  if (p.length >= 8) score++;
  if (p.length >= 12) score++;
  if (/[a-z]/.test(p) && /[A-Z]/.test(p)) score++;
  if (/\d/.test(p) && /[^A-Za-z0-9]/.test(p)) score++;
  return score;
};
passEl.addEventListener("input", () => {
  const s = scorePass(passEl.value);
  strengthEl.style.width = ([8, 28, 52, 76, 100][s]) + "%";
  strengthEl.style.background = ["#ef4444", "#ef4444", "#f59e0b", "#fbbf24", "#34d399"][s];
  $("#wifi-err").textContent = "";
});
$("#btnTogglePass").addEventListener("click", () => {
  const show = passEl.type === "password";
  passEl.type = show ? "text" : "password";
  $("#btnTogglePass").textContent = show ? "HIDE" : "SHOW";
});

const drawOledPreview = () => {
  const cv = $("#oledPreview");
  if (!cv) return;
  const ctx = cv.getContext ? cv.getContext("2d") : null;
  if (!ctx) return;
  const W = 128, H = 64;
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.clearRect(0, 0, W, H);
  const flip = $('form[data-section="oled"] input[name="flip"]');
  if (flip && flip.checked) { ctx.translate(W, H); ctx.rotate(Math.PI); }
  const contrastEl = $("#oled-contrast");
  const contrast = contrastEl ? Number(contrastEl.value) : 160;
  const lum = clamp(contrast / 255, 0.12, 1);
  const on = "rgba(190,242,255," + lum.toFixed(3) + ")";
  const dim = "rgba(140,220,245," + (lum * 0.72).toFixed(3) + ")";
  const s = state.lastState;
  const t = s && s.bme && s.bme.present ? s.bme.t.toFixed(1) + "C" : "--.-C";
  const h = s && s.bme && s.bme.present ? s.bme.h.toFixed(0) + "%" : "--%";
  const lux = s && s.bh && s.bh.present ? String(Math.round(s.bh.lux)) : "---";
  const g = s && s.mq ? s.mq.pct.toFixed(0) + "%" : "--%";
  const so = s && s.soil ? s.soil.pct.toFixed(0) + "%" : "--%";
  const rn = s && s.rain ? s.rain.pct.toFixed(0) + "%" : "--%";
  ctx.fillStyle = on;
  ctx.fillRect(0, 0, W, 12);
  ctx.fillStyle = "#010409";
  ctx.font = "bold 8px monospace";
  ctx.fillText("SMART GARDEN", 3, 9);
  ctx.fillStyle = "#010409";
  ctx.fillText(s && s.clients > 0 ? "AP·" + s.clients : "AP", W - 22, 9);
  ctx.fillStyle = on;
  ctx.font = "8px monospace";
  ctx.fillText("T:" + t + "  H:" + h, 3, 22);
  ctx.fillText((s && s.bme && s.bme.present ? s.bme.p.toFixed(0) + "hPa" : "----hPa"), 3, 32);
  ctx.fillText("Lx:" + lux, 68, 32);
  ctx.fillStyle = dim;
  ctx.fillText("Gas:" + g, 3, 43);
  ctx.fillText("Soil:" + so, 66, 43);
  ctx.fillText("Rain:" + rn, 3, 54);
  ctx.fillText("UP " + fmtUptime(s ? s.uptimeSec : 0).replace(/^0d /, ""), 66, 54);
  ctx.strokeStyle = dim;
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0, 60.5); ctx.lineTo(W, 60.5); ctx.stroke();
};
const oledContrastInput = $("#oled-contrast");
if (oledContrastInput) oledContrastInput.addEventListener("input", drawOledPreview);

$("#btnRefreshNow").addEventListener("click", () => fetchState());

setInterval(() => {
  const delta = (Date.now() - state.uptimeAnchor) / 1000;
  $("#uptimeChip").textContent = "UP " + fmtUptime(state.serverUptime + delta);
}, 1000);

setInterval(() => { fetchState(); }, 2000);

(async () => {
  try {
    const saved = localStorage.getItem("sg_page");
    if (saved && $("#panel-" + saved)) showPage(saved);
  } catch (e) {}
  await Promise.allSettled([fetchConfig(), fetchState()]);
  drawOledPreview();
})();
</script>
</body>
</html>
)rawliteral";

// ======================================================
//
// JSON WRITER - FIXED BUFFER, ZERO HEAP CHURN PER REQUEST
// ======================================================
struct JsonBuf {
  char* buf;
  size_t cap;
  size_t len;
};

static void jsonBufInit(JsonBuf* w, char* buf, size_t cap) {
  w->buf = buf;
  w->cap = cap;
  w->len = 0;
  if (cap > 0) buf[0] = '\0';
}

static void jsonBufRaw(JsonBuf* w, const char* s) {
  const size_t n = strlen(s);
  if (w->len + n >= w->cap) return;
  memcpy(w->buf + w->len, s, n);
  w->len += n;
  w->buf[w->len] = '\0';
}

static void jsonBufAddf(JsonBuf* w, const char* fmt, ...) {
  char tmp[320];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);
  jsonBufRaw(w, tmp);
}

static void jsonEscapeInto(JsonBuf* w, const char* src) {
  if (src == nullptr) {
    jsonBufRaw(w, "");
    return;
  }
  for (size_t i = 0; src[i] != '\0'; i++) {
    const char ch = src[i];
    if (ch == '"' || ch == '\\') {
      char esc[2] = {'\\', ch};
      jsonBufRaw(w, esc);
    } else if ((unsigned char)ch < 0x20) {
      continue;
    } else {
      char one[2] = {ch, '\0'};
      jsonBufRaw(w, one);
    }
  }
}

static void sendJson(AsyncWebServerRequest* request, int code, const char* json) {
  AsyncWebServerResponse* response = request->beginResponse(code, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

static void sendPortalRedirect(AsyncWebServerRequest* request) {
  AsyncWebServerResponse* response = request->beginResponse(302, "text/plain", "Redirecting to the captive portal");
  response->addHeader("Location", "http://192.168.4.1/");
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

static const char* resetReasonString() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "int watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_EXT: return "external pin";
    case ESP_RST_DEEPSLEEP: return "deep sleep";
    default: return "unknown";
  }
}

// ======================================================
//
// STATE SNAPSHOT - MANUAL JSON INTO STATIC BUFFER
// ======================================================
static char g_stateBuf[2600];

static void alertMessage(const Config& c, uint8_t bit, char* out, size_t cap) {
  switch (bit) {
    case SRC_BIT_BME:
      if (c.bmeMetric == 1) snprintf(out, cap, "BME280 humidity %.1f%% is %s %.1f%%", g_bmeH, c.bmeCondition == 0 ? "above" : "below", c.bmeThreshold);
      else if (c.bmeMetric == 2) snprintf(out, cap, "BME280 pressure %.1f hPa is %s %.1f hPa", g_bmeP, c.bmeCondition == 0 ? "above" : "below", c.bmeThreshold);
      else snprintf(out, cap, "BME280 temperature %.1fC is %s %.1fC", g_bmeT, c.bmeCondition == 0 ? "above" : "below", c.bmeThreshold);
      break;
    case SRC_BIT_BH: snprintf(out, cap, "BH1750 light %.0f lux is %s %.0f lux", g_bhLux, c.bhCondition == 0 ? "above" : "below", c.bhThresholdLux); break;
    case SRC_BIT_MQ: snprintf(out, cap, "MQ-2 gas %.1f%% is %s %.1f%%", g_mqPct, c.mqCondition == 0 ? "above" : "below", c.mqThresholdPct); break;
    case SRC_BIT_SOIL: snprintf(out, cap, "Soil moisture %.1f%% is %s %.1f%%", g_soilPct, c.soilCondition == 0 ? "above" : "below", c.soilThresholdPct); break;
    case SRC_BIT_RAIN: snprintf(out, cap, "Rain precipitation %.1f%% is %s %.1f%%", g_rainPct, c.rainCondition == 0 ? "above" : "below", c.rainThresholdPct); break;
    default: snprintf(out, cap, "sensor trigger"); break;
  }
}

static void buildStateJson() {
  JsonBuf w;
  jsonBufInit(&w, g_stateBuf, sizeof(g_stateBuf));
  Config cfgSnap;
  portENTER_CRITICAL(&g_cfgMux);
  memcpy(&cfgSnap, &g_cfg, sizeof(Config));
  portEXIT_CRITICAL(&g_cfgMux);
  const size_t freeHeap = ESP.getFreeHeap();
  const size_t largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  const uint32_t fragPct = freeHeap > 0 ? (uint32_t)(100u - (largestBlock * 100u) / freeHeap) : 0u;
  const uint32_t minHeap = esp_get_minimum_free_heap_size();
  const uint32_t flashKB = (uint32_t)(ESP.getFlashChipSize() / 1024UL);
  uint8_t mac[6] = {0, 0, 0, 0, 0, 0};
  esp_wifi_get_mac(WIFI_IF_AP, mac);
  jsonBufAddf(&w, "{\"fw\":\"%s\",\"uptimeSec\":%llu,\"boots\":%lu,\"reset\":\"%s\",\"chip\":\"%s\",\"rev\":%d,\"cpuMHz\":%u,\"flashKB\":%lu,\"sdk\":\"%s\",",
              FW_VERSION, (unsigned long long)uptimeSeconds(), (unsigned long)g_bootCount, resetReasonString(), ESP.getChipModel(), ESP.getChipRevision(), (unsigned)ESP.getCpuFreqMHz(), (unsigned long)flashKB, ESP.getSdkVersion());
  jsonBufAddf(&w, "\"heap\":%u,\"minHeap\":%u,\"fragPct\":%u,\"loopHz\":%u,\"clients\":%u,\"ip\":\"%u.%u.%u.%u\",\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\",",
              (unsigned)freeHeap, (unsigned)minHeap, (unsigned)fragPct, (unsigned)g_loopHz, (unsigned)WiFi.softAPgetStationNum(),
              AP_IP_OCTETS[0], AP_IP_OCTETS[1], AP_IP_OCTETS[2], AP_IP_OCTETS[3],
              mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  jsonBufAddf(&w, "\"cfgSlot\":\"%c\",\"cfgCrcOk\":%s,\"nvsWrites\":%lu,\"alerts\":[",
              g_cfgSource == 1 ? 'A' : (g_cfgSource == 2 ? 'B' : '-'), g_cfgSource != 0 ? "true" : "false", (unsigned long)g_nvsWrites);
  const uint8_t bits = alertBitsEffective(cfgSnap);
  bool first = true;
  for (uint8_t bit = 0; bit < 5; bit++) {
    if ((bits & (uint8_t)(1u << bit)) == 0) continue;
    char msg[96];
    alertMessage(cfgSnap, bit, msg, sizeof(msg));
    char escaped[160];
    JsonBuf esc;
    jsonBufInit(&esc, escaped, sizeof(escaped));
    jsonEscapeInto(&esc, msg);
    if (!first) jsonBufRaw(&w, ",");
    first = false;
    jsonBufAddf(&w, "\"%s\"", escaped);
  }
  jsonBufRaw(&w, "],");
  jsonBufAddf(&w, "\"bme\":{\"enabled\":%u,\"present\":%u,\"addr\":%u,\"valid\":%u,\"t\":%.1f,\"h\":%.1f,\"p\":%.1f,\"trig\":%u},",
              cfgSnap.bmeEnabled, g_bmePresent ? 1 : 0, g_bmeAddr, g_bmeValid ? 1 : 0, g_bmeT, g_bmeH, g_bmeP, (g_bmeTrig && cfgSnap.bmeTrigger) ? 1 : 0);
  jsonBufAddf(&w, "\"bh\":{\"enabled\":%u,\"present\":%u,\"valid\":%u,\"lux\":%.1f,\"trig\":%u},",
              cfgSnap.bhEnabled, g_bhPresent ? 1 : 0, g_bhValid ? 1 : 0, g_bhLux, (g_bhTrig && cfgSnap.bhTrigger) ? 1 : 0);
  jsonBufAddf(&w, "\"mq\":{\"enabled\":%u,\"valid\":%u,\"pct\":%.1f,\"raw\":%u,\"trig\":%u},",
              cfgSnap.mqEnabled, g_mqValid ? 1 : 0, g_mqPct, g_mqRaw, (g_mqTrig && cfgSnap.mqTrigger) ? 1 : 0);
  jsonBufAddf(&w, "\"soil\":{\"enabled\":%u,\"valid\":%u,\"pct\":%.1f,\"raw\":%u,\"trig\":%u},",
              cfgSnap.soilEnabled, g_soilValid ? 1 : 0, g_soilPct, g_soilRaw, (g_soilTrig && cfgSnap.soilTrigger) ? 1 : 0);
  jsonBufAddf(&w, "\"rain\":{\"enabled\":%u,\"valid\":%u,\"pct\":%.1f,\"raw\":%u,\"trig\":%u},",
              cfgSnap.rainEnabled, g_rainValid ? 1 : 0, g_rainPct, g_rainRaw, (g_rainTrig && cfgSnap.rainTrigger) ? 1 : 0);
  jsonBufRaw(&w, "\"relays\":[");
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    jsonBufAddf(&w, "%s{\"on\":%u,\"mode\":%u}", i == 0 ? "" : ",", g_relayState[i] ? 1 : 0, cfgSnap.relayMode[i]);
  }
  const bool oledOn = (cfgSnap.oledEnabled != 0) && g_oledReady && !g_oledSleeping;
  jsonBufAddf(&w, "],\"buzzer\":{\"playing\":%u},\"led\":{\"on\":%u},\"oled\":{\"on\":%u},\"ssid\":\"",
              g_buzzPlaying ? 1 : 0, g_ledOn ? 1 : 0, oledOn ? 1 : 0);
  jsonEscapeInto(&w, cfgSnap.apSsid);
  jsonBufRaw(&w, "\"}");
}

// ======================================================
//
// CONFIG SNAPSHOT - FULL ECHO FOR THE SETTINGS UI
// ======================================================
static char g_configBuf[2600];

static void buildConfigJson() {
  JsonBuf w;
  jsonBufInit(&w, g_configBuf, sizeof(g_configBuf));
  Config cfgSnap;
  portENTER_CRITICAL(&g_cfgMux);
  memcpy(&cfgSnap, &g_cfg, sizeof(Config));
  portEXIT_CRITICAL(&g_cfgMux);
  jsonBufAddf(&w, "{\"fw\":\"%s\",", FW_VERSION);
  jsonBufAddf(&w, "\"oled\":{\"enabled\":%u,\"flip\":%u,\"contrast\":%u,\"timeoutIdx\":%u,\"cycleIdx\":%u,\"pageOverview\":%u,\"pageStatus\":%u,\"pageAlerts\":%u},",
              cfgSnap.oledEnabled, cfgSnap.oledFlip, cfgSnap.oledContrast, cfgSnap.oledTimeoutIdx, cfgSnap.oledCycleIdx, cfgSnap.oledPageOverview, cfgSnap.oledPageStatus, cfgSnap.oledPageAlerts);
  jsonBufAddf(&w, "\"bme\":{\"enabled\":%u,\"intervalIdx\":%u,\"tempOffset\":%.1f,\"humOffset\":%.1f,\"presOffset\":%.1f,\"metric\":%u,\"condition\":%u,\"threshold\":%.1f,\"hysteresis\":%.1f,\"cooldownSec\":%u,\"relay\":%u,\"unified\":%u,\"buzzer\":%u,\"led\":%u,\"trigger\":%u},",
              cfgSnap.bmeEnabled, cfgSnap.bmeIntervalIdx, (float)cfgSnap.bmeTempOffsetX10 / 10.0f, (float)cfgSnap.bmeHumOffsetX10 / 10.0f, (float)cfgSnap.bmePresOffsetX10 / 10.0f,
              cfgSnap.bmeMetric, cfgSnap.bmeCondition, cfgSnap.bmeThreshold, cfgSnap.bmeHysteresis, cfgSnap.bmeCooldownSec,
              cfgSnap.bmeRelay, cfgSnap.bmeUnified, cfgSnap.bmeBuzzer, cfgSnap.bmeLed, cfgSnap.bmeTrigger);
  jsonBufAddf(&w, "\"bh\":{\"enabled\":%u,\"intervalIdx\":%u,\"condition\":%u,\"threshold\":%.1f,\"hysteresis\":%.1f,\"cooldownSec\":%u,\"relay\":%u,\"unified\":%u,\"buzzer\":%u,\"led\":%u,\"trigger\":%u},",
              cfgSnap.bhEnabled, cfgSnap.bhIntervalIdx, cfgSnap.bhCondition, cfgSnap.bhThresholdLux, cfgSnap.bhHysteresisLux, cfgSnap.bhCooldownSec,
              cfgSnap.bhRelay, cfgSnap.bhUnified, cfgSnap.bhBuzzer, cfgSnap.bhLed, cfgSnap.bhTrigger);
  jsonBufAddf(&w, "\"mq\":{\"enabled\":%u,\"intervalIdx\":%u,\"samplesIdx\":%u,\"condition\":%u,\"thresholdPct\":%.1f,\"hysteresis\":%.1f,\"cooldownSec\":%u,\"relay\":%u,\"unified\":%u,\"buzzer\":%u,\"led\":%u,\"trigger\":%u},",
              cfgSnap.mqEnabled, cfgSnap.mqIntervalIdx, cfgSnap.mqSamplesIdx, cfgSnap.mqCondition, cfgSnap.mqThresholdPct, cfgSnap.mqHysteresisPct, cfgSnap.mqCooldownSec,
              cfgSnap.mqRelay, cfgSnap.mqUnified, cfgSnap.mqBuzzer, cfgSnap.mqLed, cfgSnap.mqTrigger);
  jsonBufAddf(&w, "\"soil\":{\"enabled\":%u,\"intervalIdx\":%u,\"dryRaw\":%u,\"wetRaw\":%u,\"condition\":%u,\"thresholdPct\":%.1f,\"hysteresis\":%.1f,\"cooldownSec\":%u,\"relay\":%u,\"unified\":%u,\"buzzer\":%u,\"led\":%u,\"trigger\":%u},",
              cfgSnap.soilEnabled, cfgSnap.soilIntervalIdx, cfgSnap.soilDryRaw, cfgSnap.soilWetRaw, cfgSnap.soilCondition, cfgSnap.soilThresholdPct, cfgSnap.soilHysteresisPct, cfgSnap.soilCooldownSec,
              cfgSnap.soilRelay, cfgSnap.soilUnified, cfgSnap.soilBuzzer, cfgSnap.soilLed, cfgSnap.soilTrigger);
  jsonBufAddf(&w, "\"rain\":{\"enabled\":%u,\"intervalIdx\":%u,\"dryRaw\":%u,\"wetRaw\":%u,\"condition\":%u,\"thresholdPct\":%.1f,\"hysteresis\":%.1f,\"cooldownSec\":%u,\"relay\":%u,\"unified\":%u,\"buzzer\":%u,\"led\":%u,\"trigger\":%u},",
              cfgSnap.rainEnabled, cfgSnap.rainIntervalIdx, cfgSnap.rainDryRaw, cfgSnap.rainWetRaw, cfgSnap.rainCondition, cfgSnap.rainThresholdPct, cfgSnap.rainHysteresisPct, cfgSnap.rainCooldownSec,
              cfgSnap.rainRelay, cfgSnap.rainUnified, cfgSnap.rainBuzzer, cfgSnap.rainLed, cfgSnap.rainTrigger);
  jsonBufAddf(&w, "\"buzzer\":{\"enabled\":%u,\"pattern\":%u,\"volume\":%u,\"maxAlertIdx\":%u},",
              cfgSnap.buzzEnabled, cfgSnap.buzzPattern, cfgSnap.buzzVolume, cfgSnap.buzzMaxAlertIdx);
  jsonBufAddf(&w, "\"led\":{\"enabled\":%u,\"mode\":%u,\"brightness\":%u,\"blink\":%u},",
              cfgSnap.ledEnabled, cfgSnap.ledMode, cfgSnap.ledBrightness, cfgSnap.ledBlink);
  jsonBufRaw(&w, "\"relays\":{\"enabled\":[");
  for (uint8_t i = 0; i < RELAY_COUNT; i++) jsonBufAddf(&w, "%s%u", i == 0 ? "" : ",", cfgSnap.relayEnabled[i]);
  jsonBufRaw(&w, "],\"mode\":[");
  for (uint8_t i = 0; i < RELAY_COUNT; i++) jsonBufAddf(&w, "%s%u", i == 0 ? "" : ",", cfgSnap.relayMode[i]);
  jsonBufRaw(&w, "],\"activeLow\":[");
  for (uint8_t i = 0; i < RELAY_COUNT; i++) jsonBufAddf(&w, "%s%u", i == 0 ? "" : ",", cfgSnap.relayActiveLow[i]);
  jsonBufRaw(&w, "],\"sources\":[");
  for (uint8_t i = 0; i < RELAY_COUNT; i++) jsonBufAddf(&w, "%s%u", i == 0 ? "" : ",", cfgSnap.relaySources[i]);
  jsonBufRaw(&w, "]},");
  jsonBufRaw(&w, "\"wifiap\":{\"ssid\":\"");
  jsonEscapeInto(&w, cfgSnap.apSsid);
  jsonBufRaw(&w, "\",\"pass\":\"");
  jsonEscapeInto(&w, cfgSnap.apPass);
  jsonBufAddf(&w, "\",\"channel\":%u,\"hidden\":%u,\"maxConn\":%u}}",
              cfgSnap.apChannel, cfgSnap.apHidden, cfgSnap.apMaxConn);
}
// ======================================================
//
// FORWARD DECLARATIONS
// ======================================================
static void applyApConfig();

// ======================================================
//
// TYPED JSON FIELD EXTRACTION (partial saves are legal)
// ======================================================
static void jBool(const JsonDocument& doc, const char* key, uint8_t* dst) {
  if (doc[key].isNull()) return;
  *dst = doc[key].as<int>() != 0 ? 1 : 0;
}

static void jU8(const JsonDocument& doc, const char* key, uint8_t* dst, uint8_t lo, uint8_t hi) {
  if (doc[key].isNull()) return;
  const int64_t v = doc[key].as<int64_t>();
  *dst = clampu8((int)v, lo, hi);
}

static void jU16(const JsonDocument& doc, const char* key, uint16_t* dst, int64_t lo, int64_t hi) {
  if (doc[key].isNull()) return;
  const int64_t v = doc[key].as<int64_t>();
  const int64_t lov = lo < 0 ? 0 : lo;
  const int64_t hiv = hi > 65535 ? 65535 : hi;
  if (v < lov) *dst = (uint16_t)lov;
  else if (v > hiv) *dst = (uint16_t)hiv;
  else *dst = (uint16_t)v;
}

static void jFloat(const JsonDocument& doc, const char* key, float* dst, float lo, float hi) {
  if (doc[key].isNull()) return;
  const double v = doc[key].as<double>();
  if (isnan(v) || !isfinite(v)) return;
  *dst = clampf((float)v, lo, hi);
}

static void jOffset(const JsonDocument& doc, const char* key, int16_t* dst, float lo, float hi) {
  if (doc[key].isNull()) return;
  const double v = doc[key].as<double>();
  if (isnan(v) || !isfinite(v)) return;
  const double clamped = v < lo ? lo : (v > hi ? hi : v);
  *dst = (int16_t)lround(clamped * 10.0);
}

static void jString(const JsonDocument& doc, const char* key, char* dst, size_t cap) {
  const char* s = doc[key].as<const char*>();
  if (s == nullptr) return;
  size_t j = 0;
  for (size_t i = 0; s[i] != '\0' && j + 1 < cap; i++) {
    const uint8_t ch = (uint8_t)s[i];
    if (ch < 0x20 || ch == 0x7F) continue;
    dst[j++] = (char)ch;
  }
  dst[j] = '\0';
}

// ======================================================
//
// SECTION APPLY - STAGED WRITE, VALIDATE, THEN COMMIT
// ======================================================
static bool applySection(const char* section, const JsonDocument& doc) {
  Config staged;
  memcpy(&staged, &g_cfg, sizeof(Config));
  if (strcmp(section, "oled") == 0) {
    jBool(doc, "enabled", &staged.oledEnabled);
    jBool(doc, "flip", &staged.oledFlip);
    jU8(doc, "contrast", &staged.oledContrast, 0, 255);
    jU8(doc, "timeoutIdx", &staged.oledTimeoutIdx, 0, 4);
    jU8(doc, "cycleIdx", &staged.oledCycleIdx, 0, 3);
    jBool(doc, "pageOverview", &staged.oledPageOverview);
    jBool(doc, "pageStatus", &staged.oledPageStatus);
    jBool(doc, "pageAlerts", &staged.oledPageAlerts);
  } else if (strcmp(section, "bme") == 0) {
    jBool(doc, "enabled", &staged.bmeEnabled);
    jU8(doc, "intervalIdx", &staged.bmeIntervalIdx, 0, 5);
    jOffset(doc, "tempOffset", &staged.bmeTempOffsetX10, -20.0, 20.0);
    jOffset(doc, "humOffset", &staged.bmeHumOffsetX10, -50.0, 50.0);
    jOffset(doc, "presOffset", &staged.bmePresOffsetX10, -100.0, 100.0);
    jU8(doc, "metric", &staged.bmeMetric, 0, 2);
    jU8(doc, "condition", &staged.bmeCondition, 0, 1);
    jFloat(doc, "threshold", &staged.bmeThreshold, -10000.0f, 10000.0f);
    jFloat(doc, "hysteresis", &staged.bmeHysteresis, 0.0f, 1000.0f);
    jU16(doc, "cooldownSec", &staged.bmeCooldownSec, 0, 86400);
    jBool(doc, "relay", &staged.bmeRelay);
    jBool(doc, "unified", &staged.bmeUnified);
    jBool(doc, "buzzer", &staged.bmeBuzzer);
    jBool(doc, "led", &staged.bmeLed);
    jBool(doc, "trigger", &staged.bmeTrigger);
  } else if (strcmp(section, "bh") == 0) {
    jBool(doc, "enabled", &staged.bhEnabled);
    jU8(doc, "intervalIdx", &staged.bhIntervalIdx, 0, 5);
    jU8(doc, "condition", &staged.bhCondition, 0, 1);
    jFloat(doc, "threshold", &staged.bhThresholdLux, 0.0f, 200000.0f);
    jFloat(doc, "hysteresis", &staged.bhHysteresisLux, 0.0f, 100000.0f);
    jU16(doc, "cooldownSec", &staged.bhCooldownSec, 0, 86400);
    jBool(doc, "relay", &staged.bhRelay);
    jBool(doc, "unified", &staged.bhUnified);
    jBool(doc, "buzzer", &staged.bhBuzzer);
    jBool(doc, "led", &staged.bhLed);
    jBool(doc, "trigger", &staged.bhTrigger);
  } else if (strcmp(section, "mq") == 0) {
    jBool(doc, "enabled", &staged.mqEnabled);
    jU8(doc, "intervalIdx", &staged.mqIntervalIdx, 0, 5);
    jU8(doc, "samplesIdx", &staged.mqSamplesIdx, 0, 3);
    jU8(doc, "condition", &staged.mqCondition, 0, 1);
    jFloat(doc, "thresholdPct", &staged.mqThresholdPct, 0.0f, 100.0f);
    jFloat(doc, "hysteresis", &staged.mqHysteresisPct, 0.0f, 100.0f);
    jU16(doc, "cooldownSec", &staged.mqCooldownSec, 0, 86400);
    jBool(doc, "relay", &staged.mqRelay);
    jBool(doc, "unified", &staged.mqUnified);
    jBool(doc, "buzzer", &staged.mqBuzzer);
    jBool(doc, "led", &staged.mqLed);
    jBool(doc, "trigger", &staged.mqTrigger);
  } else if (strcmp(section, "soil") == 0) {
    jBool(doc, "enabled", &staged.soilEnabled);
    jU8(doc, "intervalIdx", &staged.soilIntervalIdx, 0, 5);
    jU16(doc, "dryRaw", &staged.soilDryRaw, 0, 4095);
    jU16(doc, "wetRaw", &staged.soilWetRaw, 0, 4095);
    jU8(doc, "condition", &staged.soilCondition, 0, 1);
    jFloat(doc, "thresholdPct", &staged.soilThresholdPct, 0.0f, 100.0f);
    jFloat(doc, "hysteresis", &staged.soilHysteresisPct, 0.0f, 100.0f);
    jU16(doc, "cooldownSec", &staged.soilCooldownSec, 0, 86400);
    jBool(doc, "relay", &staged.soilRelay);
    jBool(doc, "unified", &staged.soilUnified);
    jBool(doc, "buzzer", &staged.soilBuzzer);
    jBool(doc, "led", &staged.soilLed);
    jBool(doc, "trigger", &staged.soilTrigger);
    if (staged.soilDryRaw == staged.soilWetRaw) return false;
  } else if (strcmp(section, "rain") == 0) {
    jBool(doc, "enabled", &staged.rainEnabled);
    jU8(doc, "intervalIdx", &staged.rainIntervalIdx, 0, 5);
    jU16(doc, "dryRaw", &staged.rainDryRaw, 0, 4095);
    jU16(doc, "wetRaw", &staged.rainWetRaw, 0, 4095);
    jU8(doc, "condition", &staged.rainCondition, 0, 1);
    jFloat(doc, "thresholdPct", &staged.rainThresholdPct, 0.0f, 100.0f);
    jFloat(doc, "hysteresis", &staged.rainHysteresisPct, 0.0f, 100.0f);
    jU16(doc, "cooldownSec", &staged.rainCooldownSec, 0, 86400);
    jBool(doc, "relay", &staged.rainRelay);
    jBool(doc, "unified", &staged.rainUnified);
    jBool(doc, "buzzer", &staged.rainBuzzer);
    jBool(doc, "led", &staged.rainLed);
    jBool(doc, "trigger", &staged.rainTrigger);
    if (staged.rainDryRaw == staged.rainWetRaw) return false;
  } else if (strcmp(section, "buzzer") == 0) {
    jBool(doc, "enabled", &staged.buzzEnabled);
    jU8(doc, "pattern", &staged.buzzPattern, 0, 4);
    jU8(doc, "volume", &staged.buzzVolume, 8, 255);
    jU8(doc, "maxAlertIdx", &staged.buzzMaxAlertIdx, 0, 4);
  } else if (strcmp(section, "led") == 0) {
    jBool(doc, "enabled", &staged.ledEnabled);
    jU8(doc, "mode", &staged.ledMode, 0, 2);
    jU8(doc, "brightness", &staged.ledBrightness, 1, 255);
    jBool(doc, "blink", &staged.ledBlink);
  } else if (strcmp(section, "relays") == 0) {
    JsonArrayConst enabled = doc["enabled"].as<JsonArrayConst>();
    JsonArrayConst mode = doc["mode"].as<JsonArrayConst>();
    JsonArrayConst activeLow = doc["activeLow"].as<JsonArrayConst>();
    JsonArrayConst sources = doc["sources"].as<JsonArrayConst>();
    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
      if (!enabled.isNull() && i < enabled.size()) staged.relayEnabled[i] = enabled[i].as<int>() != 0 ? 1 : 0;
      if (!mode.isNull() && i < mode.size()) staged.relayMode[i] = clampu8(mode[i].as<int>(), 0, 2);
      if (!activeLow.isNull() && i < activeLow.size()) staged.relayActiveLow[i] = activeLow[i].as<int>() != 0 ? 1 : 0;
      if (!sources.isNull() && i < sources.size()) staged.relaySources[i] = (uint8_t)(sources[i].as<int>() & 0x1F);
    }
  } else if (strcmp(section, "wifiap") == 0) {
    jString(doc, "ssid", staged.apSsid, sizeof(staged.apSsid));
    jString(doc, "pass", staged.apPass, sizeof(staged.apPass));
    jU8(doc, "channel", &staged.apChannel, 1, 13);
    jBool(doc, "hidden", &staged.apHidden);
    jU8(doc, "maxConn", &staged.apMaxConn, 1, 4);
    const size_t ssidLen = strnlen(staged.apSsid, sizeof(staged.apSsid));
    const size_t passLen = strnlen(staged.apPass, sizeof(staged.apPass));
    if (ssidLen == 0 || ssidLen > 32) return false;
    if (passLen < 8 || passLen > 63) return false;
  } else {
    return false;
  }
  portENTER_CRITICAL(&g_cfgMux);
  memcpy(&g_cfg, &staged, sizeof(Config));
  portEXIT_CRITICAL(&g_cfgMux);
  if (strcmp(section, "oled") == 0) {
    g_oled.setRotation(g_cfg.oledFlip != 0 ? 2 : 0);
    oledWriteContrast();
    if (g_oledSleeping && g_cfg.oledEnabled != 0) oledWake(millis());
    g_oledDirty = true;
  }
  return true;
}

static bool sectionChangedAndSaved(Config& before) {
  Config current;
  portENTER_CRITICAL(&g_cfgMux);
  memcpy(&current, &g_cfg, sizeof(Config));
  portEXIT_CRITICAL(&g_cfgMux);
  if (configContentEqual(before, current)) return false;
  configSave();
  return true;
}

// ======================================================
//
// HANDLERS - STATE & CONFIG
// ======================================================
static void handleState(AsyncWebServerRequest* request) {
  buildStateJson();
  sendJson(request, 200, g_stateBuf);
}

static void handleConfigGet(AsyncWebServerRequest* request) {
  buildConfigJson();
  sendJson(request, 200, g_configBuf);
}

static void handleConfigPost(AsyncWebServerRequest* request) {
  if (!request->hasParam("plain", true)) {
    sendJson(request, 400, "{\"ok\":false,\"err\":\"missing request body\"}");
    return;
  }
  const String& body = request->getParam("plain", true)->value();
  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, body);
  if (err) {
    sendJson(request, 400, "{\"ok\":false,\"err\":\"invalid JSON body\"}");
    return;
  }
  const char* section = doc["section"].as<const char*>();
  if (section == nullptr || section[0] == '\0') {
    sendJson(request, 400, "{\"ok\":false,\"err\":\"missing section\"}");
    return;
  }
  Config before;
  portENTER_CRITICAL(&g_cfgMux);
  memcpy(&before, &g_cfg, sizeof(Config));
  portEXIT_CRITICAL(&g_cfgMux);
  if (!applySection(section, doc)) {
    portENTER_CRITICAL(&g_cfgMux);
    memcpy(&g_cfg, &before, sizeof(Config));
    portEXIT_CRITICAL(&g_cfgMux);
    sendJson(request, 400, "{\"ok\":false,\"err\":\"invalid values for section\"}");
    return;
  }
  const bool changed = sectionChangedAndSaved(before);
  if (changed && strcmp(section, "wifiap") == 0) {
    g_pendingApRestartMs = millis() + 700UL;
  }
  if (changed) oledWake(millis());
  sendJson(request, 200, changed ? "{\"ok\":true,\"changed\":true}" : "{\"ok\":true,\"changed\":false}");
}

// ======================================================
//
// HANDLERS - SENSOR & ACTUATOR PROBES
// ======================================================
static void handleTestBme(AsyncWebServerRequest* request) {
  if (!g_bmePresent) {
    sendJson(request, 200, "{\"ok\":false,\"err\":\"BME280 not detected on I2C bus - check SDA/SCL and 0x76/0x77 address\"}");
    return;
  }
  const float t = g_bme.readTemperature() + ((float)g_cfg.bmeTempOffsetX10 / 10.0f);
  const float h = g_bme.readHumidity() + ((float)g_cfg.bmeHumOffsetX10 / 10.0f);
  const float p = (g_bme.readPressure() / 100.0f) + ((float)g_cfg.bmePresOffsetX10 / 10.0f);
  char out[200];
  snprintf(out, sizeof(out), "{\"ok\":true,\"t\":%.1f,\"h\":%.1f,\"p\":%.1f,\"addr\":%u}", t, h, p, g_bmeAddr);
  sendJson(request, 200, out);
}

static void handleTestBh(AsyncWebServerRequest* request) {
  if (!g_bhPresent) {
    sendJson(request, 200, "{\"ok\":false,\"err\":\"BH1750 not detected on I2C bus - check SDA/SCL and ADDR pin\"}");
    return;
  }
  char out[120];
  snprintf(out, sizeof(out), "{\"ok\":true,\"lux\":%.1f,\"addr\":%u}", g_bh.readLightLevel(), g_bhAddr);
  sendJson(request, 200, out);
}

static void handleTestMq(AsyncWebServerRequest* request) {
  const uint16_t raw = analogAverage64(PIN_MQ2_AOUT, MQ_SAMPLE_COUNT[clampu8(g_cfg.mqSamplesIdx, 0, 3)]);
  const float pct = ((float)raw * 100.0f) / 4095.0f;
  char out[120];
  snprintf(out, sizeof(out), "{\"ok\":true,\"raw\":%u,\"pct\":%.1f}", raw, pct);
  sendJson(request, 200, out);
}

static void handleTestSoil(AsyncWebServerRequest* request) {
  const uint16_t raw = analogAverage64(PIN_SOIL_AOUT, 16);
  const float pct = analogToPercent(raw, g_cfg.soilDryRaw, g_cfg.soilWetRaw);
  char out[120];
  snprintf(out, sizeof(out), "{\"ok\":true,\"raw\":%u,\"pct\":%.1f}", raw, pct);
  sendJson(request, 200, out);
}

static void handleTestRain(AsyncWebServerRequest* request) {
  const uint16_t raw = analogAverage64(PIN_RAIN_AOUT, 16);
  const float pct = analogToPercent(raw, g_cfg.rainDryRaw, g_cfg.rainWetRaw);
  char out[120];
  snprintf(out, sizeof(out), "{\"ok\":true,\"raw\":%u,\"pct\":%.1f}", raw, pct);
  sendJson(request, 200, out);
}

static void handleI2cScan(AsyncWebServerRequest* request) {
  char out[256];
  JsonBuf w;
  jsonBufInit(&w, out, sizeof(out));
  jsonBufRaw(&w, "{\"ok\":true,\"found\":[");
  bool first = true;
  for (uint8_t addr = 8; addr < 120; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      jsonBufAddf(&w, "%s%u", first ? "" : ",", addr);
      first = false;
    }
    yield();
  }
  jsonBufRaw(&w, "]}");
  sendJson(request, 200, out);
}

static void handleBuzzerTest(AsyncWebServerRequest* request) {
  uint8_t pattern = 1;
  if (request->hasParam("plain", true)) {
    JsonDocument doc;
    if (deserializeJson(doc, request->getParam("plain", true)->value()) == DeserializationError::Ok) {
      if (!doc["pattern"].isNull()) pattern = clampu8(doc["pattern"].as<int>(), 0, 4);
    }
  }
  g_buzzTestPattern = pattern;
  g_buzzTestUntil = millis() + 2500UL;
  g_buzzPhase = 0;
  g_buzzPulseIndex = 0;
  g_buzzPlaying = false;
  oledWake(millis());
  sendJson(request, 200, "{\"ok\":true}");
}

static void handleLedTest(AsyncWebServerRequest* request) {
  g_ledTestUntil = millis() + 1500UL;
  oledWake(millis());
  sendJson(request, 200, "{\"ok\":true}");
}

static void handleOledTest(AsyncWebServerRequest* request) {
  if (!g_oledReady) {
    sendJson(request, 200, "{\"ok\":false,\"err\":\"SSD1306 not detected on I2C bus (0x3C/0x3D)\"}");
    return;
  }
  oledWake(millis());
  const uint8_t pageCount = oledEnabledPageCount();
  const uint8_t page = oledPageAt((uint8_t)(g_oledPageIndex % (pageCount == 0 ? 1u : pageCount)));
  g_oledSleeping = false;
  g_oled.dim(false);
  oledRender(page);
  sendJson(request, 200, "{\"ok\":true}");
}

static void handleRelaysAllOff(AsyncWebServerRequest* request) {
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    g_cfg.relayMode[i] = 2;
  }
  configSave();
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    relayApplyLevel(i, false);
  }
  oledWake(millis());
  sendJson(request, 200, "{\"ok\":true}");
}

static void handleReboot(AsyncWebServerRequest* request) {
  g_pendingRestartMs = millis() + 800UL;
  sendJson(request, 200, "{\"ok\":true}");
}

static void handleFactoryReset(AsyncWebServerRequest* request) {
  if (!request->hasParam("plain", true)) {
    sendJson(request, 400, "{\"ok\":false,\"err\":\"missing request body\"}");
    return;
  }
  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, request->getParam("plain", true)->value());
  if (err) {
    sendJson(request, 400, "{\"ok\":false,\"err\":\"invalid JSON body\"}");
    return;
  }
  const char* confirm = doc["confirm"].as<const char*>();
  if (confirm == nullptr || strcmp(confirm, "FACTORY_RESET") != 0) {
    sendJson(request, 400, "{\"ok\":false,\"err\":\"confirmation token mismatch\"}");
    return;
  }
  Preferences prefs;
  if (prefs.begin(NVS_NAMESPACE, false)) {
    prefs.clear();
    prefs.end();
  }
  g_pendingRestartMs = millis() + 1200UL;
  sendJson(request, 200, "{\"ok\":true}");
}
// ======================================================
//
// WEB SERVER + CAPTIVE PORTAL
// ======================================================
static DNSServer dnsServer;
static AsyncWebServer server(80);
static const uint16_t DNS_PORT = 53;

static void applyApConfig() {
  const IPAddress apIp(AP_IP_OCTETS[0], AP_IP_OCTETS[1], AP_IP_OCTETS[2], AP_IP_OCTETS[3]);
  WiFi.softAPdisconnect(true);
  delay(80);
  WiFi.softAPConfig(apIp, apIp, IPAddress(255, 255, 255, 0));
  WiFi.softAP(g_cfg.apSsid, g_cfg.apPass, g_cfg.apChannel, g_cfg.apHidden != 0 ? 1 : 0, g_cfg.apMaxConn);
  dnsServer.stop();
  dnsServer.start(DNS_PORT, "*", apIp);
}

static const char* CAPTIVE_PORTAL_PATHS[] = {
  "/generate_204",
  "/gen_204",
  "/hotspot-detect.html",
  "/library/test/success.html",
  "/ncsi.txt",
  "/connecttest.txt",
  "/connectivity-check.html",
  "/success.txt",
  "/canonical.html",
  "/redirect",
  "/redirect.html",
  "/mobile/status.php",
  "/hotspot-detect.php",
  "/fwlink",
  "/fwlink/",
  "/kindle-wifi/wifistub.html",
  "/wifi.shtml",
  "/start.html",
  "/hotspot.html",
  "/library/test/success.txt"
};
static const size_t CAPTIVE_PORTAL_PATH_COUNT = sizeof(CAPTIVE_PORTAL_PATHS) / sizeof(CAPTIVE_PORTAL_PATHS[0]);

static void registerRoutes() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse* response = request->beginResponse_P(200, "text/html", INDEX_HTML);
    response->addHeader("Cache-Control", "no-store");
    request->send(response);
  });
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(204);
  });
  server.on("/api/state", HTTP_GET, handleState);
  server.on("/api/config", HTTP_GET, handleConfigGet);
  server.on("/api/config", HTTP_POST, handleConfigPost);
  server.on("/api/test/bme", HTTP_POST, handleTestBme);
  server.on("/api/test/bh", HTTP_POST, handleTestBh);
  server.on("/api/test/mq", HTTP_POST, handleTestMq);
  server.on("/api/test/soil", HTTP_POST, handleTestSoil);
  server.on("/api/test/rain", HTTP_POST, handleTestRain);
  server.on("/api/i2c-scan", HTTP_POST, handleI2cScan);
  server.on("/api/buzzer/test", HTTP_POST, handleBuzzerTest);
  server.on("/api/led/test", HTTP_POST, handleLedTest);
  server.on("/api/oled/test", HTTP_POST, handleOledTest);
  server.on("/api/relays/alloff", HTTP_POST, handleRelaysAllOff);
  server.on("/api/reboot", HTTP_POST, handleReboot);
  server.on("/api/factory-reset", HTTP_POST, handleFactoryReset);
  for (size_t i = 0; i < CAPTIVE_PORTAL_PATH_COUNT; i++) {
    server.on(CAPTIVE_PORTAL_PATHS[i], HTTP_GET, [](AsyncWebServerRequest* request) {
      sendPortalRedirect(request);
    });
  }
  server.onNotFound([](AsyncWebServerRequest* request) {
    const String& url = request->url();
    if (url.startsWith("/api/")) {
      sendJson(request, 404, "{\"ok\":false,\"err\":\"unknown api route\"}");
      return;
    }
    sendPortalRedirect(request);
  });
  server.begin();
}

// ======================================================
//
// HARDWARE INITIALIZATION
// ======================================================
static void initI2cDevices() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(16);
  if (g_oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    g_oledReady = true;
  } else if (g_oled.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
    g_oledReady = true;
  } else {
    g_oledReady = false;
  }
  if (g_oledReady) {
    g_oled.setRotation(g_cfg.oledFlip != 0 ? 2 : 0);
    g_oled.ssd1306_command(SSD1306_SETCONTRAST);
    g_oled.ssd1306_command(g_cfg.oledContrast);
    g_oled.setTextColor(SSD1306_WHITE);
    g_oled.dim(false);
    g_oled.clearDisplay();
    g_oled.setTextSize(2);
    g_oled.setCursor(4, 4);
    g_oled.println("SMART");
    g_oled.println("GARDEN");
    g_oled.setTextSize(1);
    g_oled.setCursor(4, 44);
    g_oled.printf("FW %s", FW_VERSION);
    g_oled.setCursor(4, 55);
    g_oled.print("192.168.4.1");
    g_oled.display();
    g_oledLastActivityMs = millis();
    g_oledPageSwitchMs = millis();
    g_oledLastDrawMs = millis();
  }
  if (g_bme.begin(0x76)) {
    g_bmePresent = true;
    g_bmeAddr = 0x76;
  } else if (g_bme.begin(0x77)) {
    g_bmePresent = true;
    g_bmeAddr = 0x77;
  } else {
    g_bmePresent = false;
  }
  if (g_bh.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23)) {
    g_bhPresent = true;
    g_bhAddr = 0x23;
  } else if (g_bh.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x5C)) {
    g_bhPresent = true;
    g_bhAddr = 0x5C;
  } else {
    g_bhPresent = false;
  }
}

static void initAdc() {
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(PIN_MQ2_AOUT, INPUT);
  pinMode(PIN_SOIL_AOUT, INPUT);
  pinMode(PIN_RAIN_AOUT, INPUT);
}

static void initOutputs() {
  pinMode(PIN_BUTTON_GPIO0, INPUT_PULLUP);
  relayWritePinsInactiveFirst();
  buzzerAttach();
  ledAttach();
}

static void initWifiAndPortal() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  applyApConfig();
  registerRoutes();
}

// ======================================================
//
// SETUP
// ======================================================
void setup() {
  configLoad();
  const uint32_t bootMs = millis();
  (void)bootMs;
  g_lastUptimeTick = millis();
  g_loopHzWindowStart = millis();
  initAdc();
  initOutputs();
  initI2cDevices();
  initWifiAndPortal();
  bootCountIncrement();
  g_oledDirty = true;
}

// ======================================================
//
// RESTART WATCHDOG (reboot / factory reset scheduling)
// ======================================================
static void restartTick(uint32_t now) {
  if (g_pendingApRestartMs != 0 && (int32_t)(now - g_pendingApRestartMs) >= 0) {
    g_pendingApRestartMs = 0;
    applyApConfig();
  }
  if (g_pendingRestartMs != 0 && (int32_t)(now - g_pendingRestartMs) >= 0) {
    g_pendingRestartMs = 0;
    buzzerDuty(0);
    ledDuty(0);
    ESP.restart();
  }
}

// ======================================================
//
// MAIN LOOP - FULLY NON-BLOCKING
// ======================================================
void loop() {
  dnsServer.processNextRequest();
  const uint32_t now = millis();
  const uint32_t delta = now - g_lastUptimeTick;
  g_lastUptimeTick = now;
  g_uptimeMs += (uint64_t)delta;
  g_loopIterations++;
  const uint32_t hzWindow = now - g_loopHzWindowStart;
  if (hzWindow >= 1000UL) {
    g_loopHz = (uint32_t)(((uint64_t)g_loopIterations * 1000ULL) / (uint64_t)hzWindow);
    g_loopIterations = 0;
    g_loopHzWindowStart = now;
  }
  buttonTick(now);
  bmeTick(now);
  bhTick(now);
  mqTick(now);
  soilTick(now);
  rainTick(now);
  relayTick(now);
  buzzerTick(now);
  ledTick(now);
  oledTick(now);
  restartTick(now);
  esp_task_wdt_reset();
  yield();
}
