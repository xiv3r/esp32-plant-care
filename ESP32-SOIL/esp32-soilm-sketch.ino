// ======================================================
// ESP32 SOIL MOISTURE SENSOR + SD1306 + BUZZER + LED + RELAY
// ======================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <driver/ledc.h>
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_wifi.h"

// ======================================================
// PIN DEFINITIONS
// ======================================================
#define PIN_SOIL_ADC    34
#define PIN_RELAY       25
#define PIN_BUZZER      26
#define PIN_LED         33
#define PIN_OLED_SDA    21
#define PIN_OLED_SCL    22
#define PIN_BOOT_BTN    0

// ======================================================
// CONSTANTS
// ======================================================
#define FW_VERSION            300
#define CFG_MAGIC             0x534F494Cul
#define CFG_VERSION           3ul
#define CFG_NS                "soilcfg"
#define CFG_SLOT_A            "slotA"
#define CFG_SLOT_B            "slotB"
#define CFG_SAVE_DEBOUNCE_MS  2000ull
#define CFG_SAVE_MIN_MS       30000ull
#define STATS_SAVE_MS         3600000ull
#define HISTORY_SIZE          240u
#define OLED_ADDR             0x3C
#define AP_IP_STR             "192.168.4.1"
#define WDT_TIMEOUT_S         30
#define FACTORY_HOLD_MS       5000ull
#define BTN_DEBOUNCE_MS       50ull
#define HEAP_CRITICAL_BYTES   4096u
#define HEAP_CRITICAL_MS      60000ull

// ======================================================
// CONFIG STRUCTURE
// ======================================================
struct AppConfig {
  uint32_t magic;
  uint32_t version;
  uint32_t crc;
  uint64_t seq;
  uint64_t soilDryRaw;
  uint64_t soilWetRaw;
  uint64_t soilThresholdPct;
  uint64_t soilHysteresisPct;
  uint64_t sampleIntervalMs;
  uint64_t avgWindow;
  uint64_t relayMode;
  uint64_t relayActiveLow;
  uint64_t relayMinOnMs;
  uint64_t relayMinOffMs;
  uint64_t buzzerEnabled;
  uint64_t buzzerBeepMs;
  uint64_t buzzerGapMs;
  uint64_t buzzerCount;
  uint64_t buzzerTrigger;
  uint64_t buzzerPeriodMs;
  uint64_t ledEnabled;
  uint64_t ledMode;
  uint64_t ledBrightness;
  uint64_t ledOnMs;
  uint64_t ledOffMs;
  uint64_t oledEnabled;
  uint64_t oledPage;
  uint64_t oledRotateMs;
  uint64_t oledContrast;
  uint64_t apChannel;
  uint64_t apMaxConn;
  uint64_t apHidden;
  uint64_t apPowerDbm;
  uint64_t bootCount;
  uint64_t totalRelayOnMs;
  uint64_t totalSamples;
  uint64_t saveCount;
  char apSsid[33];
  char apPass[65];
};

// ======================================================
// GLOBAL STATE
// ======================================================
static AppConfig g_cfg;
static bool g_cfgDirty = false;
static uint64_t g_lastChangeMs = 0;
static uint64_t g_lastSaveMs = 0;
static uint64_t g_cfgSeqNext = 1;
static bool g_saveSlot = false;
static uint64_t g_bootMs = 0;
static uint64_t g_lastStatsSaveMs = 0;
static uint64_t g_totalSamples = 0;
static uint64_t g_relayTotalMs = 0;
static bool g_wifiRestartPending = false;
static uint8_t g_wifiRestartStep = 0;
static uint64_t g_wifiRestartAtMs = 0;
static bool g_rebootPending = false;
static uint64_t g_rebootAtMs = 0;
static bool g_factoryPending = false;
static uint64_t g_factoryAtMs = 0;
static uint64_t g_heapLowSinceMs = 0;

static WebServer server(80);
static DNSServer dnsServer;
static Adafruit_SSD1306 display(128, 64, &Wire, -1);

static uint64_t histT[HISTORY_SIZE];
static uint64_t histV[HISTORY_SIZE];
static uint32_t histHead = 0;
static uint32_t histCount = 0;

static uint64_t soilAccSum = 0;
static uint64_t soilAccCount = 0;
static uint64_t soilRawAvg = 0;
static uint64_t soilPct = 0;
static uint64_t soilLastPublishMs = 0;
static uint64_t soilLastSampleMs = 0;

static bool relayOn = false;
static uint64_t relayLastChangeMs = 0;
static uint64_t relayOnSinceMs = 0;

static bool buzzerPinOn = false;
static bool buzzerActive = false;
static uint64_t buzzerPhaseStartMs = 0;
static uint64_t buzzerBeepIndex = 0;
static uint64_t buzzerNextAllowedMs = 0;
static bool buzzerTestReq = false;

static uint64_t ledPhaseStartMs = 0;
static bool ledPhaseOn = false;
static uint8_t ledPhaseIdx = 0;
static uint32_t ledLastDuty = 0xFFFFFFFFul;

static bool oledReady = false;
static bool oledOffSent = false;
static uint64_t oledLastDrawMs = 0;
static uint64_t oledLastPageChangeMs = 0;
static uint64_t oledLastContrast = 0xFFFFFFFFull;

static bool btnRawPrev = true;
static bool btnStable = true;
static uint64_t btnRawChangeMs = 0;
static bool btnPressed = false;
static uint64_t btnPressStartMs = 0;
static bool btnResetFired = false;

static uint64_t healthLastMs = 0;

// ======================================================
// TIME SYSTEM
// ======================================================
static inline uint64_t timeNowMs() {
  return (uint64_t)(esp_timer_get_time() / 1000);
}
static void fmtUptime(uint64_t ms, char* buf, size_t n) {
  uint64_t s = ms / 1000ull;
  uint64_t d = s / 86400ull; s %= 86400ull;
  uint64_t h = s / 3600ull; s %= 3600ull;
  uint64_t m = s / 60ull;
  snprintf(buf, n, "%llud %02lluh %02llum", (unsigned long long)d, (unsigned long long)h, (unsigned long long)m);
}

// ======================================================
// CRC32
// ======================================================
static uint32_t crc32_ieee(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFul;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint32_t)data[i];
    for (int b = 0; b < 8; b++) {
      uint32_t mask = (uint32_t)(-(int32_t)(crc & 1ul));
      crc = (crc >> 1) ^ (0xEDB88320ul & mask);
    }
  }
  return crc ^ 0xFFFFFFFFul;
}
static uint32_t configCrc(const AppConfig* c) {
  AppConfig tmp = *c;
  tmp.crc = 0;
  return crc32_ieee((const uint8_t*)&tmp, sizeof(tmp));
}

// ======================================================
// CONFIG MANAGER
// ======================================================
static void configDefaults() {
  memset(&g_cfg, 0, sizeof(g_cfg));
  g_cfg.magic = CFG_MAGIC;
  g_cfg.version = CFG_VERSION;
  g_cfg.seq = 0;
  g_cfg.soilDryRaw = 3000;
  g_cfg.soilWetRaw = 1400;
  g_cfg.soilThresholdPct = 35;
  g_cfg.soilHysteresisPct = 5;
  g_cfg.sampleIntervalMs = 1000;
  g_cfg.avgWindow = 32;
  g_cfg.relayMode = 0;
  g_cfg.relayActiveLow = 0;
  g_cfg.relayMinOnMs = 5000;
  g_cfg.relayMinOffMs = 5000;
  g_cfg.buzzerEnabled = 1;
  g_cfg.buzzerBeepMs = 200;
  g_cfg.buzzerGapMs = 200;
  g_cfg.buzzerCount = 3;
  g_cfg.buzzerTrigger = 1;
  g_cfg.buzzerPeriodMs = 60000;
  g_cfg.ledEnabled = 1;
  g_cfg.ledMode = 2;
  g_cfg.ledBrightness = 180;
  g_cfg.ledOnMs = 500;
  g_cfg.ledOffMs = 500;
  g_cfg.oledEnabled = 1;
  g_cfg.oledPage = 0;
  g_cfg.oledRotateMs = 10000;
  g_cfg.oledContrast = 207;
  g_cfg.apChannel = 6;
  g_cfg.apMaxConn = 4;
  g_cfg.apHidden = 0;
  g_cfg.apPowerDbm = 20;
  g_cfg.bootCount = 0;
  g_cfg.totalRelayOnMs = 0;
  g_cfg.totalSamples = 0;
  g_cfg.saveCount = 0;
  strncpy(g_cfg.apSsid, "ESP32_Soil_Sensor", sizeof(g_cfg.apSsid) - 1);
  strncpy(g_cfg.apPass, "12345678", sizeof(g_cfg.apPass) - 1);
  g_cfg.crc = configCrc(&g_cfg);
}
static bool configLoadSlot(Preferences& p, const char* key, AppConfig& out) {
  if (p.getBytesLength(key) != sizeof(AppConfig)) return false;
  if (p.getBytes(key, &out, sizeof(out)) != sizeof(out)) return false;
  if (out.magic != CFG_MAGIC || out.version != CFG_VERSION) return false;
  if (out.crc != configCrc(&out)) return false;
  return true;
}
static void configLoad() {
  configDefaults();
  Preferences p;
  if (!p.begin(CFG_NS, true)) return;
  AppConfig a, b;
  bool okA = configLoadSlot(p, CFG_SLOT_A, a);
  bool okB = configLoadSlot(p, CFG_SLOT_B, b);
  p.end();
  if (okA && okB) {
    g_cfg = (a.seq >= b.seq) ? a : b;
    g_saveSlot = (a.seq >= b.seq);
  } else if (okA) {
    g_cfg = a;
    g_saveSlot = true;
  } else if (okB) {
    g_cfg = b;
    g_saveSlot = false;
  } else {
    return;
  }
  g_cfgSeqNext = g_cfg.seq + 1;
  g_totalSamples = g_cfg.totalSamples;
  g_relayTotalMs = g_cfg.totalRelayOnMs;
}
static void configMarkDirty() {
  g_cfgDirty = true;
  g_lastChangeMs = timeNowMs();
}
static bool configSave(bool force) {
  uint64_t now = timeNowMs();
  if (!force) {
    if (!g_cfgDirty) return false;
    if (now - g_lastChangeMs < CFG_SAVE_DEBOUNCE_MS) return false;
    if (g_lastSaveMs != 0 && now - g_lastSaveMs < CFG_SAVE_MIN_MS) return false;
  }
  g_cfg.totalSamples = g_totalSamples;
  g_cfg.totalRelayOnMs = g_relayTotalMs + (relayOn ? (now - relayOnSinceMs) : 0);
  g_cfg.seq = g_cfgSeqNext++;
  g_cfg.saveCount++;
  g_cfg.crc = 0;
  g_cfg.crc = configCrc(&g_cfg);
  Preferences p;
  if (!p.begin(CFG_NS, false)) return false;
  const char* key = g_saveSlot ? CFG_SLOT_B : CFG_SLOT_A;
  size_t w = p.putBytes(key, &g_cfg, sizeof(g_cfg));
  p.end();
  if (w == sizeof(g_cfg)) {
    g_saveSlot = !g_saveSlot;
    g_cfgDirty = false;
    g_lastSaveMs = now;
    return true;
  }
  return false;
}
static void factoryReset() {
  Preferences p;
  if (p.begin(CFG_NS, false)) {
    p.clear();
    p.end();
  }
  ESP.restart();
}

// ======================================================
// NAME HELPERS
// ======================================================
static const char* relayModeName(uint64_t m) {
  return m == 1 ? "Manual ON" : (m == 2 ? "Manual OFF" : "Auto");
}
static const char* buzzerTriggerName(uint64_t t) {
  switch (t) {
    case 1: return "Low moisture";
    case 2: return "High moisture";
    case 3: return "Relay on";
    case 4: return "Periodic";
    default: return "Never";
  }
}
static const char* ledModeName(uint64_t m) {
  switch (m) {
    case 1: return "On";
    case 2: return "Blink";
    case 3: return "Pattern";
    default: return "Off";
  }
}
static const char* oledPageName(uint64_t p) {
  switch (p) {
    case 1: return "Soil";
    case 2: return "Buzzer";
    case 3: return "LED";
    case 4: return "WiFi";
    case 5: return "System";
    default: return "Dashboard";
  }
}

// ======================================================
// SOIL MOISTURE SENSOR
// ======================================================
static void soilPushHistory(uint64_t t, uint64_t v) {
  histT[histHead] = t;
  histV[histHead] = v;
  histHead = (histHead + 1) % HISTORY_SIZE;
  if (histCount < HISTORY_SIZE) histCount++;
}
static uint64_t soilToPct(uint64_t raw) {
  int64_t dry = (int64_t)g_cfg.soilDryRaw;
  int64_t wet = (int64_t)g_cfg.soilWetRaw;
  int64_t r = (int64_t)raw;
  if (dry == wet) return 0;
  int64_t pct = (dry - r) * 100 / (dry - wet);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (uint64_t)pct;
}
static void taskSoil(uint64_t now) {
  if (now - soilLastSampleMs >= 20) {
    soilLastSampleMs = now;
    if (soilAccCount < g_cfg.avgWindow) {
      soilAccSum += (uint64_t)analogRead(PIN_SOIL_ADC);
      soilAccCount++;
    }
  }
  if (now - soilLastPublishMs >= g_cfg.sampleIntervalMs) {
    soilLastPublishMs = now;
    if (soilAccCount > 0) soilRawAvg = soilAccSum / soilAccCount;
    soilAccSum = 0;
    soilAccCount = 0;
    soilPct = soilToPct(soilRawAvg);
    soilPushHistory(now, soilPct);
    g_totalSamples++;
  }
}

// ======================================================
// RELAY CONTROLLER
// ======================================================
static inline uint8_t relayLevelFor(bool on) {
  uint8_t hi = g_cfg.relayActiveLow ? LOW : HIGH;
  uint8_t lo = g_cfg.relayActiveLow ? HIGH : LOW;
  return on ? hi : lo;
}
static void relayApply(bool on, uint64_t now) {
  if (on == relayOn) return;
  relayOn = on;
  digitalWrite(PIN_RELAY, relayLevelFor(on));
  if (on) {
    relayOnSinceMs = now;
  } else {
    g_relayTotalMs += (now - relayOnSinceMs);
  }
  relayLastChangeMs = now;
}
static void taskRelay(uint64_t now) {
  bool want = relayOn;
  if (g_cfg.relayMode == 1) {
    want = true;
  } else if (g_cfg.relayMode == 2) {
    want = false;
  } else {
    if (soilPct <= g_cfg.soilThresholdPct) want = true;
    else if (soilPct >= g_cfg.soilThresholdPct + g_cfg.soilHysteresisPct) want = false;
  }
  if (want && !relayOn && (now - relayLastChangeMs) >= g_cfg.relayMinOffMs) relayApply(true, now);
  else if (!want && relayOn && (now - relayLastChangeMs) >= g_cfg.relayMinOnMs) relayApply(false, now);
}

// ======================================================
// BUZZER CONTROLLER
// ======================================================
static void buzzerSetPin(bool on) {
  if (on == buzzerPinOn) return;
  buzzerPinOn = on;
  digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
}
static bool buzzerConditionMet() {
  switch (g_cfg.buzzerTrigger) {
    case 1: return soilPct <= g_cfg.soilThresholdPct;
    case 2: return soilPct >= g_cfg.soilThresholdPct + g_cfg.soilHysteresisPct;
    case 3: return relayOn;
    case 4: return true;
    default: return false;
  }
}
static void taskBuzzer(uint64_t now) {
  if (!g_cfg.buzzerEnabled) {
    buzzerActive = false;
    buzzerSetPin(false);
    return;
  }
  if (!buzzerActive) {
    buzzerSetPin(false);
    bool trig = buzzerTestReq || (buzzerConditionMet() && now >= buzzerNextAllowedMs);
    if (trig) {
      buzzerTestReq = false;
      buzzerActive = true;
      buzzerBeepIndex = 0;
      buzzerPhaseStartMs = now;
      buzzerSetPin(true);
    }
    return;
  }
  uint64_t el = now - buzzerPhaseStartMs;
  if (buzzerPinOn) {
    if (el >= g_cfg.buzzerBeepMs) {
      buzzerSetPin(false);
      buzzerPhaseStartMs = now;
    }
  } else {
    if (el >= g_cfg.buzzerGapMs) {
      buzzerBeepIndex++;
      if (buzzerBeepIndex >= g_cfg.buzzerCount) {
        buzzerActive = false;
        buzzerNextAllowedMs = now + g_cfg.buzzerPeriodMs;
      } else {
        buzzerSetPin(true);
        buzzerPhaseStartMs = now;
      }
    }
  }
}

// ======================================================
// LED CONTROLLER
// ======================================================
static void ledPwmInit() {
  ledc_timer_config_t tc{};
  tc.speed_mode = LEDC_LOW_SPEED_MODE;
  tc.duty_resolution = LEDC_TIMER_8_BIT;
  tc.timer_num = LEDC_TIMER_0;
  tc.freq_hz = 5000;
  tc.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&tc);
  ledc_channel_config_t cc{};
  cc.gpio_num = PIN_LED;
  cc.speed_mode = LEDC_LOW_SPEED_MODE;
  cc.channel = LEDC_CHANNEL_0;
  cc.intr_type = LEDC_INTR_DISABLE;
  cc.timer_sel = LEDC_TIMER_0;
  cc.duty = 0;
  cc.hpoint = 0;
  ledc_channel_config(&cc);
}
static void ledSetDuty(uint32_t duty) {
  if (duty == ledLastDuty) return;
  ledLastDuty = duty;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}
static void taskLed(uint64_t now) {
  if (!g_cfg.ledEnabled || g_cfg.ledMode == 0) {
    ledSetDuty(0);
    ledPhaseStartMs = now;
    return;
  }
  uint32_t bright = (uint32_t)g_cfg.ledBrightness;
  if (g_cfg.ledMode == 1) {
    ledSetDuty(bright);
    ledPhaseStartMs = now;
    return;
  }
  uint64_t el = now - ledPhaseStartMs;
  if (g_cfg.ledMode == 2) {
    if (ledPhaseOn && el >= g_cfg.ledOnMs) {
      ledPhaseOn = false;
      ledPhaseStartMs = now;
      ledSetDuty(0);
    } else if (!ledPhaseOn && el >= g_cfg.ledOffMs) {
      ledPhaseOn = true;
      ledPhaseStartMs = now;
      ledSetDuty(bright);
    }
  } else {
    static const uint8_t patOn[6] = {1, 0, 1, 0, 1, 0};
    static const uint8_t patMul[6] = {1, 1, 1, 1, 3, 3};
    uint64_t u = g_cfg.ledOnMs;
    if (u == 0) u = 100;
    if (el >= u * patMul[ledPhaseIdx]) {
      ledPhaseIdx = (ledPhaseIdx + 1) % 6;
      ledPhaseOn = patOn[ledPhaseIdx] != 0;
      ledPhaseStartMs = now;
      ledSetDuty(ledPhaseOn ? bright : 0);
    }
  }
  ledSetDuty(ledPhaseOn ? bright : 0);
}

// ======================================================
// OLED DISPLAY MANAGER
// ======================================================
static void oledInit() {
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    oledReady = true;
    display.ssd1306_command(0x81);
    display.ssd1306_command((uint8_t)g_cfg.oledContrast);
    oledLastContrast = g_cfg.oledContrast;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.display();
  }
}
static void oledLine(uint8_t y, const char* s) {
  display.setCursor(0, y);
  display.print(s);
}
static void oledNextPage() {
  g_cfg.oledPage = (g_cfg.oledPage + 1) % 6;
  oledLastPageChangeMs = timeNowMs();
  oledLastDrawMs = 0;
  configMarkDirty();
}
static void oledPrevPage() {
  g_cfg.oledPage = (g_cfg.oledPage + 5) % 6;
  oledLastPageChangeMs = timeNowMs();
  oledLastDrawMs = 0;
  configMarkDirty();
}
static void oledDrawPage(uint8_t page, uint64_t now) {
  char l[40];
  char u[24];
  switch (page) {
    case 0:
      oledLine(0, "ESP32 SOIL SENSOR");
      snprintf(l, sizeof(l), "Moisture: %llu%%", (unsigned long long)soilPct);
      oledLine(10, l);
      snprintf(l, sizeof(l), "Raw ADC : %llu", (unsigned long long)soilRawAvg);
      oledLine(20, l);
      snprintf(l, sizeof(l), "Relay   : %s %s", relayOn ? "ON" : "OFF", g_cfg.relayMode == 0 ? "AUTO" : (g_cfg.relayMode == 1 ? "MAN-ON" : "MAN-OFF"));
      oledLine(30, l);
      fmtUptime(now - g_bootMs, u, sizeof(u));
      snprintf(l, sizeof(l), "Uptime  : %s", u);
      oledLine(40, l);
      snprintf(l, sizeof(l), "IP 192.168.4.1 C:%u", (unsigned)WiFi.softAPgetStationNum());
      oledLine(50, l);
      break;
    case 1:
      oledLine(0, "SOIL SETTINGS");
      snprintf(l, sizeof(l), "Threshold: %llu%%", (unsigned long long)g_cfg.soilThresholdPct);
      oledLine(10, l);
      snprintf(l, sizeof(l), "Hysteresis:%llu%%", (unsigned long long)g_cfg.soilHysteresisPct);
      oledLine(20, l);
      snprintf(l, sizeof(l), "Dry %llu Wet %llu", (unsigned long long)g_cfg.soilDryRaw, (unsigned long long)g_cfg.soilWetRaw);
      oledLine(30, l);
      snprintf(l, sizeof(l), "Interval : %llums", (unsigned long long)g_cfg.sampleIntervalMs);
      oledLine(40, l);
      snprintf(l, sizeof(l), "RelayMin : %llu/%llu", (unsigned long long)g_cfg.relayMinOnMs, (unsigned long long)g_cfg.relayMinOffMs);
      oledLine(50, l);
      break;
    case 2:
      oledLine(0, "BUZZER SETTINGS");
      snprintf(l, sizeof(l), "Enabled  : %s", g_cfg.buzzerEnabled ? "YES" : "NO");
      oledLine(10, l);
      snprintf(l, sizeof(l), "Trigger  : %s", buzzerTriggerName(g_cfg.buzzerTrigger));
      oledLine(20, l);
      snprintf(l, sizeof(l), "Beep %llu Gap %llu", (unsigned long long)g_cfg.buzzerBeepMs, (unsigned long long)g_cfg.buzzerGapMs);
      oledLine(30, l);
      snprintf(l, sizeof(l), "Count    : %llu", (unsigned long long)g_cfg.buzzerCount);
      oledLine(40, l);
      snprintf(l, sizeof(l), "Period   : %llums", (unsigned long long)g_cfg.buzzerPeriodMs);
      oledLine(50, l);
      break;
    case 3:
      oledLine(0, "LED SETTINGS");
      snprintf(l, sizeof(l), "Enabled  : %s", g_cfg.ledEnabled ? "YES" : "NO");
      oledLine(10, l);
      snprintf(l, sizeof(l), "Mode     : %s", ledModeName(g_cfg.ledMode));
      oledLine(20, l);
      snprintf(l, sizeof(l), "Bright   : %llu", (unsigned long long)g_cfg.ledBrightness);
      oledLine(30, l);
      snprintf(l, sizeof(l), "On %llu Off %llu", (unsigned long long)g_cfg.ledOnMs, (unsigned long long)g_cfg.ledOffMs);
      oledLine(40, l);
      oledLine(50, "GPIO33 anode");
      break;
    case 4:
      oledLine(0, "WIFI AP SETTINGS");
      snprintf(l, sizeof(l), "SSID %.21s", g_cfg.apSsid);
      oledLine(10, l);
      snprintf(l, sizeof(l), "Channel  : %llu", (unsigned long long)g_cfg.apChannel);
      oledLine(20, l);
      snprintf(l, sizeof(l), "Clients  : %u / %llu", (unsigned)WiFi.softAPgetStationNum(), (unsigned long long)g_cfg.apMaxConn);
      oledLine(30, l);
      snprintf(l, sizeof(l), "Hidden   : %s", g_cfg.apHidden ? "YES" : "NO");
      oledLine(40, l);
      snprintf(l, sizeof(l), "Power    : %lludBm", (unsigned long long)g_cfg.apPowerDbm);
      oledLine(50, l);
      break;
    default:
      oledLine(0, "SYSTEM");
      snprintf(l, sizeof(l), "Heap     : %lluKB", (unsigned long long)(esp_get_free_heap_size() / 1024));
      oledLine(10, l);
      snprintf(l, sizeof(l), "Boots    : %llu", (unsigned long long)g_cfg.bootCount);
      oledLine(20, l);
      snprintf(l, sizeof(l), "CfgSeq   : %llu", (unsigned long long)g_cfg.seq);
      oledLine(30, l);
      snprintf(l, sizeof(l), "Saves    : %llu", (unsigned long long)g_cfg.saveCount);
      oledLine(40, l);
      snprintf(l, sizeof(l), "Samples  : %llu", (unsigned long long)g_totalSamples);
      oledLine(50, l);
      break;
  }
}
static void taskOled(uint64_t now) {
  if (!oledReady) return;
  if (!g_cfg.oledEnabled) {
    if (!oledOffSent) {
      display.ssd1306_command(SSD1306_DISPLAYOFF);
      oledOffSent = true;
    }
    return;
  }
  if (oledOffSent) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    oledOffSent = false;
  }
  if (g_cfg.oledContrast != oledLastContrast) {
    oledLastContrast = g_cfg.oledContrast;
    display.ssd1306_command(0x81);
    display.ssd1306_command((uint8_t)g_cfg.oledContrast);
  }
  if (g_cfg.oledRotateMs > 0 && now - oledLastPageChangeMs >= g_cfg.oledRotateMs) {
    oledLastPageChangeMs = now;
    g_cfg.oledPage = (g_cfg.oledPage + 1) % 6;
  }
  if (now - oledLastDrawMs < 250) return;
  oledLastDrawMs = now;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  if (now - g_bootMs < 1500) {
    oledLine(10, "ESP32 SOIL SENSOR");
    oledLine(24, "Production Console");
    oledLine(38, "Starting systems...");
  } else if (btnPressed && (now - btnPressStartMs) >= 2000) {
    oledLine(10, "FACTORY RESET");
    oledLine(24, "Release button");
    oledLine(38, "to cancel");
    oledLine(52, "Keep holding...");
  } else {
    oledDrawPage((uint8_t)g_cfg.oledPage, now);
  }
  display.display();
}

// ======================================================
// BOOT BUTTON MANAGER
// ======================================================
static void taskButton(uint64_t now) {
  bool raw = digitalRead(PIN_BOOT_BTN);
  if (raw != btnRawPrev) {
    btnRawPrev = raw;
    btnRawChangeMs = now;
  }
  if (raw != btnStable && (now - btnRawChangeMs) >= BTN_DEBOUNCE_MS) {
    bool was = btnStable;
    btnStable = raw;
    if (!btnStable && was) {
      btnPressed = true;
      btnPressStartMs = now;
      btnResetFired = false;
    } else if (btnStable && !was) {
      btnPressed = false;
      if (!btnResetFired && now - btnPressStartMs < FACTORY_HOLD_MS) oledNextPage();
    }
  }
  if (btnPressed && !btnResetFired && (now - btnPressStartMs) >= FACTORY_HOLD_MS) {
    btnResetFired = true;
    factoryReset();
  }
}

// ======================================================
// WIFI AP MANAGER
// ======================================================
static void wifiStart() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(g_cfg.apSsid, g_cfg.apPass, (int)g_cfg.apChannel, g_cfg.apHidden != 0, (int)g_cfg.apMaxConn);
  esp_wifi_set_max_tx_power((int8_t)(g_cfg.apPowerDbm * 4));
}
static void taskWifi(uint64_t now) {
  if (g_wifiRestartPending && now >= g_wifiRestartAtMs) {
    if (g_wifiRestartStep == 0) {
      WiFi.softAPdisconnect(true);
      g_wifiRestartStep = 1;
      g_wifiRestartAtMs = now + 300;
    } else {
      g_wifiRestartStep = 0;
      g_wifiRestartPending = false;
      wifiStart();
    }
  }
}

// ======================================================
// WEB SERVER SHARED
// ======================================================
static const char HTML_HEAD[] PROGMEM = R"HTML(<!DOCTYPE html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><meta name="theme-color" content="#0b1220"><title>ESP32 Soil Sensor</title><style>:root{--bg:#0b1220;--panel:#111c30;--panel2:#0e1729;--line:#1f2f4d;--txt:#e6eefc;--mut:#8aa3c7;--acc:#22c55e;--acc2:#38bdf8;--warn:#f59e0b;--bad:#ef4444;--rad:14px}*{box-sizing:border-box}html,body{margin:0;padding:0}body{font-family:system-ui,-apple-system,"Segoe UI",Roboto,Ubuntu,sans-serif;background:var(--bg);color:var(--txt)}a{color:var(--acc2);text-decoration:none}.layout{display:flex;min-height:100vh}.sidebar{width:248px;flex:0 0 248px;background:var(--panel2);border-right:1px solid var(--line);padding:20px 14px;position:fixed;top:0;bottom:0;left:0;overflow-y:auto}.brand{font-weight:800;font-size:18px;letter-spacing:.3px;margin:2px 8px 6px}.brand small{display:block;font-weight:500;color:var(--mut);font-size:12px;margin-top:2px}.nav{display:block;padding:10px 12px;border-radius:10px;color:var(--txt);margin:4px 0;font-size:14px}.nav:hover{background:#16233d}.nav.active{background:linear-gradient(90deg,#14532d,#166534);color:#fff}.sidefoot{margin-top:18px;padding:12px;color:var(--mut);font-size:12px;border-top:1px solid var(--line);line-height:1.6}.main{flex:1;margin-left:248px;padding:22px;max-width:1180px}.topbar{display:flex;align-items:center;gap:10px;flex-wrap:wrap;margin-bottom:16px}.topbar h1{font-size:22px;margin:0 8px 0 0}.pill{background:var(--panel);border:1px solid var(--line);border-radius:999px;padding:6px 12px;font-size:12px;color:var(--mut)}.pill b{color:var(--txt)}.pill.on b{color:var(--acc)}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:14px}.card{background:var(--panel);border:1px solid var(--line);border-radius:var(--rad);padding:16px}.card .k{color:var(--mut);font-size:12px;text-transform:uppercase;letter-spacing:.6px}.card .v{font-size:28px;font-weight:800;margin-top:6px}.card .s{color:var(--mut);font-size:12px;margin-top:4px}.card.wide{grid-column:1/-1}canvas.graph{width:100%;height:230px;display:block;margin-top:8px}form .row{display:grid;grid-template-columns:1fr 1fr;gap:12px}.field{margin-bottom:12px}label{display:block;color:var(--mut);font-size:12px;margin-bottom:6px}input,select{width:100%;background:#0b1526;border:1px solid var(--line);color:var(--txt);border-radius:10px;padding:10px 12px;font-size:14px}input:focus,select:focus{outline:2px solid var(--acc2);border-color:transparent}.btn{display:inline-block;background:var(--acc);color:#04150a;border:0;border-radius:10px;padding:11px 18px;font-weight:700;font-size:14px;cursor:pointer}.btn:hover{filter:brightness(1.08)}.btn.ghost{background:#16233d;color:var(--txt);border:1px solid var(--line)}.btn.danger{background:var(--bad);color:#fff}.btnrow{display:flex;gap:10px;flex-wrap:wrap;margin-top:10px}.note{color:var(--mut);font-size:12px;line-height:1.6}.toast{position:fixed;right:16px;bottom:16px;background:#052e16;border:1px solid var(--acc);color:#bbf7d0;padding:12px 16px;border-radius:12px;font-size:13px;opacity:0;transition:opacity .25s;pointer-events:none;max-width:90vw;z-index:50}.toast.show{opacity:1}table{width:100%;border-collapse:collapse;font-size:13px}td,th{padding:8px 6px;border-bottom:1px solid var(--line);text-align:left}th{color:var(--mut);font-weight:600}@media (max-width:900px){.layout{flex-direction:column}.sidebar{position:static;width:auto;flex:none;display:flex;align-items:center;gap:6px;overflow-x:auto;padding:10px}.brand{white-space:nowrap;margin:0 8px 0 0;font-size:15px}.brand small{display:none}.nav{white-space:nowrap;margin:0}.sidefoot{display:none}.main{margin-left:0;padding:14px}form .row{grid-template-columns:1fr}.card .v{font-size:24px}canvas.graph{height:190px}}</style></head><body><div class="layout">)HTML";

static const char HTML_FOOT[] PROGMEM = R"HTML(</div></div><div class="toast" id="toast"></div><script>function toast(m){var t=document.getElementById('toast');if(!t)return;t.textContent=m;t.classList.add('show');clearTimeout(t._h);t._h=setTimeout(function(){t.classList.remove('show')},2600)}function pad(n){return(n<10?'0':'')+n}function fmtU(ms){var s=Math.floor(ms/1000);var d=Math.floor(s/86400);s=s%86400;var h=Math.floor(s/3600);s=s%3600;var m=Math.floor(s/60);s=s%60;return d+'d '+pad(h)+'h '+pad(m)+'m '+pad(s)+'s'}function setTxt(id,v){var e=document.getElementById(id);if(e)e.textContent=v}function refreshStatus(){fetch('/api/status').then(function(r){return r.json()}).then(function(j){setTxt('st-moisture',j.moisturePct+'%');setTxt('st-raw',j.rawAdc);setTxt('st-relay',j.relayOn?'ON':'OFF');setTxt('st-threshold',j.soilThresholdPct+'%');setTxt('st-uptime',fmtU(j.uptimeMs));setTxt('st-heap',(j.heapFree/1024).toFixed(0)+' KB');setTxt('st-heapmin',(j.heapMin/1024).toFixed(0)+' KB');setTxt('st-clients',j.clients);setTxt('st-boots',j.bootCount);setTxt('st-buzzer',j.buzzerOn?'BEEPING':'IDLE');setTxt('st-led',j.ledOn?'ON':'OFF');setTxt('st-relaytotal',(j.totalRelayOnMs/3600000).toFixed(1)+' h');setTxt('st-samples',j.totalSamples);setTxt('st-seq',j.configSeq);setTxt('st-saves',j.saveCount);var pr=document.getElementById('pill-relay');if(pr){pr.className='pill'+(j.relayOn?' on':'');pr.innerHTML='Relay <b>'+(j.relayOn?'ON':'OFF')+'</b>'}var pm=document.getElementById('pill-moisture');if(pm){pm.innerHTML='Moisture <b>'+j.moisturePct+'%</b>'}var pu=document.getElementById('pill-uptime');if(pu){pu.innerHTML='Uptime <b>'+fmtU(j.uptimeMs)+'</b>'}}).catch(function(){})}function drawGraph(){var c=document.getElementById('graph');if(!c)return;fetch('/api/graph').then(function(r){return r.json()}).then(function(j){var pts=j.points||[];var dpr=window.devicePixelRatio||1;var w=c.clientWidth,h=c.clientHeight;if(w===0)return;c.width=w*dpr;c.height=h*dpr;var x=c.getContext('2d');x.setTransform(dpr,0,0,dpr,0,0);x.clearRect(0,0,w,h);x.strokeStyle='#1f2f4d';x.lineWidth=1;for(var gy=0;gy<=4;gy++){var y=8+(h-24)*gy/4;x.beginPath();x.moveTo(34,y);x.lineTo(w-6,y);x.stroke();x.fillStyle='#8aa3c7';x.font='10px system-ui';x.fillText(String(100-gy*25),6,y+3)}if(pts.length>0){var th=j.threshold||0;var ty=8+(h-24)*(1-th/100);x.strokeStyle='#f59e0b';x.setLineDash([5,4]);x.beginPath();x.moveTo(34,ty);x.lineTo(w-6,ty);x.stroke();x.setLineDash([]);x.strokeStyle='#22c55e';x.lineWidth=2;x.beginPath();for(var i=0;i<pts.length;i++){var px=34+(w-40)*(pts.length===1?0:i/(pts.length-1));var py=8+(h-24)*(1-pts[i][1]/100);if(i===0)x.moveTo(px,py);else x.lineTo(px,py)}x.stroke()}}).catch(function(){})}document.addEventListener('DOMContentLoaded',function(){refreshStatus();drawGraph();setInterval(refreshStatus,2000);setInterval(drawGraph,5000);document.addEventListener('submit',function(ev){var f=ev.target;if(!f.classList||!f.classList.contains('cfgform'))return;ev.preventDefault();var fd=new FormData(f);var p=new URLSearchParams();fd.forEach(function(v,k){p.set(k,v)});fetch('/api/config',{method:'POST',body:p}).then(function(r){return r.json()}).then(function(j){toast(j.ok?'Settings saved':'Save failed');if(j.wifi)setTimeout(function(){toast('WiFi AP restarting, please reconnect')},600)}).catch(function(){toast('Save failed')})});document.addEventListener('click',function(ev){var b=ev.target.closest?ev.target.closest('[data-action]'):null;if(!b)return;var a=b.getAttribute('data-action');if(b.hasAttribute('data-confirm')&&!window.confirm(b.getAttribute('data-confirm')))return;var p=new URLSearchParams();p.set('action',a);fetch('/api/action',{method:'POST',body:p}).then(function(r){return r.json()}).then(function(j){toast(j.ok?'Done':'Failed');refreshStatus()}).catch(function(){toast('Failed')})});});</script></body></html>)HTML";

static const char HTML_SIDEBAR_OPEN[] PROGMEM = R"HTML(<aside class="sidebar"><div class="brand">ESP32 Soil Sensor<small>Production Console</small></div>)HTML";
static const char HTML_SIDEBAR_FOOT[] PROGMEM = R"HTML(<div class="sidefoot">AP: 192.168.4.1<br>Hold BOOT 5s = factory reset<br>Short press = next OLED page</div></aside><div class="main">)HTML";
static const char FORM_OPEN[] PROGMEM = R"HTML(<div class="card" style="margin-top:14px"><form class="cfgform">)HTML";
static const char FORM_CLOSE[] PROGMEM = R"HTML(<div class="btnrow"><button class="btn" type="submit">Save Settings</button></div></form></div>)HTML";

static void uiNav(const char* href, const char* label, const char* id, const char* active) {
  char b[160];
  snprintf(b, sizeof(b), "<a class=\"nav%s\" href=\"%s\">%s</a>", (strcmp(id, active) == 0 ? " active" : ""), href, label);
  server.sendContent(b);
}
static void uiSidebar(const char* active) {
  server.sendContent_P(HTML_SIDEBAR_OPEN);
  uiNav("/", "Dashboard", "dash", active);
  uiNav("/soil", "Soil Moisture", "soil", active);
  uiNav("/buzzer", "Buzzer", "buzzer", active);
  uiNav("/led", "LED", "led", active);
  uiNav("/oled", "OLED Display", "oled", active);
  uiNav("/wifi", "WiFi AP", "wifi", active);
  uiNav("/system", "System", "system", active);
  server.sendContent_P(HTML_SIDEBAR_FOOT);
}
static void pageBegin(const char* title, const char* active) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html; charset=utf-8", "");
  server.sendContent_P(HTML_HEAD);
  uiSidebar(active);
  char b[512];
  snprintf(b, sizeof(b), "<div class=\"topbar\"><h1>%s</h1><span class=\"pill\" id=\"pill-moisture\">Moisture <b>--</b></span><span class=\"pill\" id=\"pill-relay\">Relay <b>--</b></span><span class=\"pill\" id=\"pill-uptime\">Uptime <b>--</b></span></div>", title);
  server.sendContent(b);
}
static void pageEnd() {
  server.sendContent_P(HTML_FOOT);
}
static void uiNum(const char* label, const char* name, uint64_t v, uint64_t mn, uint64_t mx) {
  char b[288];
  snprintf(b, sizeof(b), "<div class=\"field\"><label>%s</label><input type=\"number\" name=\"%s\" value=\"%llu\" min=\"%llu\" max=\"%llu\"></div>", label, name, (unsigned long long)v, (unsigned long long)mn, (unsigned long long)mx);
  server.sendContent(b);
}
static void uiText(const char* label, const char* name, const char* v, uint32_t maxlen) {
  char b[384];
  snprintf(b, sizeof(b), "<div class=\"field\"><label>%s</label><input type=\"text\" name=\"%s\" value=\"%s\" maxlength=\"%u\"></div>", label, name, v, (unsigned)maxlen);
  server.sendContent(b);
}
static void uiSelect(const char* label, const char* name, uint64_t cur, const char* const* vals, const char* const* labs, uint8_t n) {
  char b[1024];
  size_t off = 0;
  off += (size_t)snprintf(b + off, sizeof(b) - off, "<div class=\"field\"><label>%s</label><select name=\"%s\">", label, name);
  for (uint8_t i = 0; i < n; i++) {
    off += (size_t)snprintf(b + off, sizeof(b) - off, "<option value=\"%s\"%s>%s</option>", vals[i], (cur == strtoull(vals[i], nullptr, 10) ? " selected" : ""), labs[i]);
  }
  snprintf(b + off, sizeof(b) - off, "</select></div>");
  server.sendContent(b);
}

// ======================================================
// WEB SERVER PAGES
// ======================================================
static void handleDashboard() {
  pageBegin("Dashboard", "dash");
  char b[2048];
  char u[24];
  fmtUptime(timeNowMs() - g_bootMs, u, sizeof(u));
  snprintf(b, sizeof(b),
    "<div class=\"grid\">"
    "<div class=\"card\"><div class=\"k\">Soil Moisture</div><div class=\"v\" id=\"st-moisture\">%llu%%</div><div class=\"s\">Raw ADC <span id=\"st-raw\">%llu</span> / threshold <span id=\"st-threshold\">%llu%%</span></div></div>"
    "<div class=\"card\"><div class=\"k\">Relay GPIO25</div><div class=\"v\" id=\"st-relay\">%s</div><div class=\"s\">Mode %s</div></div>"
    "<div class=\"card\"><div class=\"k\">Buzzer GPIO26</div><div class=\"v\" id=\"st-buzzer\">%s</div><div class=\"s\">Trigger %s</div></div>"
    "<div class=\"card\"><div class=\"k\">LED GPIO33</div><div class=\"v\" id=\"st-led\">%s</div><div class=\"s\">Mode %s</div></div>"
    "<div class=\"card\"><div class=\"k\">Uptime</div><div class=\"v\" id=\"st-uptime\" style=\"font-size:20px\">%s</div><div class=\"s\">Boots <span id=\"st-boots\">%llu</span></div></div>"
    "<div class=\"card\"><div class=\"k\">Free Heap</div><div class=\"v\" id=\"st-heap\">%llu KB</div><div class=\"s\">Min <span id=\"st-heapmin\">%llu KB</span></div></div>"
    "<div class=\"card\"><div class=\"k\">AP Clients</div><div class=\"v\" id=\"st-clients\">%u</div><div class=\"s\">%s</div></div>"
    "<div class=\"card wide\"><div class=\"k\">Moisture History</div><canvas class=\"graph\" id=\"graph\"></canvas><div class=\"s\">Orange dashed line = relay threshold. Graph refreshes every 5s.</div></div>"
    "</div>"
    "<div class=\"btnrow\">"
    "<button class=\"btn\" data-action=\"relay_on\">Relay ON</button>"
    "<button class=\"btn ghost\" data-action=\"relay_off\">Relay OFF</button>"
    "<button class=\"btn ghost\" data-action=\"relay_auto\">Relay AUTO</button>"
    "<button class=\"btn ghost\" data-action=\"buzzer_test\">Test Buzzer</button>"
    "</div>",
    (unsigned long long)soilPct,
    (unsigned long long)soilRawAvg,
    (unsigned long long)g_cfg.soilThresholdPct,
    relayOn ? "ON" : "OFF",
    relayModeName(g_cfg.relayMode),
    buzzerPinOn ? "BEEPING" : "IDLE",
    buzzerTriggerName(g_cfg.buzzerTrigger),
    (ledLastDuty > 0 && ledLastDuty != 0xFFFFFFFFul) ? "ON" : "OFF",
    ledModeName(g_cfg.ledMode),
    u,
    (unsigned long long)g_cfg.bootCount,
    (unsigned long long)(esp_get_free_heap_size() / 1024),
    (unsigned long long)(esp_get_minimum_free_heap_size() / 1024),
    (unsigned)WiFi.softAPgetStationNum(),
    g_cfg.apSsid);
  server.sendContent(b);
  pageEnd();
}

static void handleSoilPage() {
  pageBegin("Soil Moisture & Relay", "soil");
  char b[1024];
  snprintf(b, sizeof(b),
    "<div class=\"grid\">"
    "<div class=\"card\"><div class=\"k\">Live Reading</div><div class=\"v\" id=\"st-moisture\">%llu%%</div><div class=\"s\">Raw ADC <span id=\"st-raw\">%llu</span></div></div>"
    "<div class=\"card\"><div class=\"k\">Relay GPIO25</div><div class=\"v\" id=\"st-relay\">%s</div><div class=\"s\">Mode %s</div></div>"
    "</div>",
    (unsigned long long)soilPct,
    (unsigned long long)soilRawAvg,
    relayOn ? "ON" : "OFF",
    relayModeName(g_cfg.relayMode));
  server.sendContent(b);
  server.sendContent_P(FORM_OPEN);
  server.sendContent("<div class=\"row\">");
  uiNum("Dry ADC value (GPIO34)", "soilDryRaw", g_cfg.soilDryRaw, 0, 4095);
  uiNum("Wet ADC value (GPIO34)", "soilWetRaw", g_cfg.soilWetRaw, 0, 4095);
  uiNum("Relay ON at or below (%)", "soilThresholdPct", g_cfg.soilThresholdPct, 0, 100);
  uiNum("Hysteresis (%)", "soilHysteresisPct", g_cfg.soilHysteresisPct, 0, 50);
  uiNum("Sample interval (ms)", "sampleIntervalMs", g_cfg.sampleIntervalMs, 100, 60000);
  uiNum("Averaging window (samples)", "avgWindow", g_cfg.avgWindow, 1, 256);
  uiNum("Relay min ON time (ms)", "relayMinOnMs", g_cfg.relayMinOnMs, 0, 3600000);
  uiNum("Relay min OFF time (ms)", "relayMinOffMs", g_cfg.relayMinOffMs, 0, 3600000);
  server.sendContent("</div>");
  static const char* const rmV[] = {"0", "1", "2"};
  static const char* const rmL[] = {"Auto (threshold)", "Manual ON", "Manual OFF"};
  uiSelect("Relay mode", "relayMode", g_cfg.relayMode, rmV, rmL, 3);
  static const char* const alV[] = {"0", "1"};
  static const char* const alL[] = {"Active HIGH", "Active LOW"};
  uiSelect("Relay active level", "relayActiveLow", g_cfg.relayActiveLow, alV, alL, 2);
  server.sendContent_P(FORM_CLOSE);
  server.sendContent("<div class=\"btnrow\"><button class=\"btn ghost\" data-action=\"calib_dry\">Set Dry = Current Raw</button><button class=\"btn ghost\" data-action=\"calib_wet\">Set Wet = Current Raw</button></div><p class=\"note\">Calibration captures the live ADC average. Dry is the reading in dry soil, wet in saturated soil. Moisture % = (dry - raw) / (dry - wet) x 100. Relay turns ON at or below the threshold and OFF at threshold + hysteresis.</p>");
  pageEnd();
}

static void handleBuzzerPage() {
  pageBegin("Buzzer Settings", "buzzer");
  char b[1024];
  snprintf(b, sizeof(b),
    "<div class=\"grid\">"
    "<div class=\"card\"><div class=\"k\">Buzzer GPIO26</div><div class=\"v\" id=\"st-buzzer\">%s</div><div class=\"s\">Trigger %s</div></div>"
    "</div>",
    buzzerPinOn ? "BEEPING" : "IDLE",
    buzzerTriggerName(g_cfg.buzzerTrigger));
  server.sendContent(b);
  server.sendContent_P(FORM_OPEN);
  server.sendContent("<div class=\"row\">");
  static const char* const enV[] = {"0", "1"};
  static const char* const enL[] = {"Disabled", "Enabled"};
  uiSelect("Buzzer enabled", "buzzerEnabled", g_cfg.buzzerEnabled, enV, enL, 2);
  static const char* const trV[] = {"0", "1", "2", "3", "4"};
  static const char* const trL[] = {"Never", "Low moisture", "High moisture", "Relay on", "Periodic"};
  uiSelect("Trigger condition", "buzzerTrigger", g_cfg.buzzerTrigger, trV, trL, 5);
  uiNum("Beep duration (ms)", "buzzerBeepMs", g_cfg.buzzerBeepMs, 10, 5000);
  uiNum("Gap between beeps (ms)", "buzzerGapMs", g_cfg.buzzerGapMs, 10, 5000);
  uiNum("Beep count", "buzzerCount", g_cfg.buzzerCount, 1, 20);
  uiNum("Repeat period (ms)", "buzzerPeriodMs", g_cfg.buzzerPeriodMs, 1000, 86400000);
  server.sendContent("</div>");
  server.sendContent_P(FORM_CLOSE);
  server.sendContent("<div class=\"btnrow\"><button class=\"btn ghost\" data-action=\"buzzer_test\">Test Buzzer</button></div><p class=\"note\">When the trigger condition is met, the buzzer plays the configured beep pattern, then waits the repeat period before it can trigger again.</p>");
  pageEnd();
}

static void handleLedPage() {
  pageBegin("LED Settings", "led");
  char b[1024];
  snprintf(b, sizeof(b),
    "<div class=\"grid\">"
    "<div class=\"card\"><div class=\"k\">LED GPIO33</div><div class=\"v\" id=\"st-led\">%s</div><div class=\"s\">Mode %s, brightness %llu</div></div>"
    "</div>",
    (ledLastDuty > 0 && ledLastDuty != 0xFFFFFFFFul) ? "ON" : "OFF",
    ledModeName(g_cfg.ledMode),
    (unsigned long long)g_cfg.ledBrightness);
  server.sendContent(b);
  server.sendContent_P(FORM_OPEN);
  server.sendContent("<div class=\"row\">");
  static const char* const enV[] = {"0", "1"};
  static const char* const enL[] = {"Disabled", "Enabled"};
  uiSelect("LED enabled", "ledEnabled", g_cfg.ledEnabled, enV, enL, 2);
  static const char* const mdV[] = {"0", "1", "2", "3"};
  static const char* const mdL[] = {"Off", "On", "Blink", "Pattern"};
  uiSelect("LED mode", "ledMode", g_cfg.ledMode, mdV, mdL, 4);
  uiNum("Brightness (0-255)", "ledBrightness", g_cfg.ledBrightness, 0, 255);
  uiNum("ON time (ms)", "ledOnMs", g_cfg.ledOnMs, 50, 60000);
  uiNum("OFF time (ms)", "ledOffMs", g_cfg.ledOffMs, 50, 60000);
  server.sendContent("</div>");
  server.sendContent_P(FORM_CLOSE);
  server.sendContent("<div class=\"btnrow\"><button class=\"btn ghost\" data-action=\"led_test\">Restart Pattern</button></div><p class=\"note\">LED is wired anode-first on GPIO33 and driven with PWM. Pattern mode plays short-short-long using the ON time as the unit.</p>");
  pageEnd();
}

static void handleOledPage() {
  pageBegin("OLED Display Settings", "oled");
  char b[1024];
  snprintf(b, sizeof(b),
    "<div class=\"grid\">"
    "<div class=\"card\"><div class=\"k\">SSD1306 128x64</div><div class=\"v\" style=\"font-size:20px\">%s</div><div class=\"s\">I2C SDA GPIO21 / SCL GPIO22 / addr 0x3C</div></div>"
    "</div>",
    oledPageName(g_cfg.oledPage));
  server.sendContent(b);
  server.sendContent_P(FORM_OPEN);
  server.sendContent("<div class=\"row\">");
  static const char* const enV[] = {"0", "1"};
  static const char* const enL[] = {"Disabled", "Enabled"};
  uiSelect("OLED enabled", "oledEnabled", g_cfg.oledEnabled, enV, enL, 2);
  static const char* const pgV[] = {"0", "1", "2", "3", "4", "5"};
  static const char* const pgL[] = {"Dashboard", "Soil", "Buzzer", "LED", "WiFi", "System"};
  uiSelect("Current page", "oledPage", g_cfg.oledPage, pgV, pgL, 6);
  uiNum("Auto-rotate interval (ms, 0 = manual)", "oledRotateMs", g_cfg.oledRotateMs, 0, 60000);
  uiNum("Contrast (0-255)", "oledContrast", g_cfg.oledContrast, 0, 255);
  server.sendContent("</div>");
  server.sendContent_P(FORM_CLOSE);
  server.sendContent("<div class=\"btnrow\"><button class=\"btn ghost\" data-action=\"oled_prev\">Previous Page</button><button class=\"btn ghost\" data-action=\"oled_next\">Next Page</button></div><p class=\"note\">The BOOT button short-press also cycles OLED pages. Hold BOOT for 5 seconds to factory reset.</p>");
  pageEnd();
}

static void handleWifiPage() {
  pageBegin("WiFi AP Settings", "wifi");
  char b[1024];
  snprintf(b, sizeof(b),
    "<div class=\"grid\">"
    "<div class=\"card\"><div class=\"k\">Access Point</div><div class=\"v\" style=\"font-size:20px\">%s</div><div class=\"s\">%s / clients <span id=\"st-clients\">%u</span></div></div>"
    "</div>",
    g_cfg.apSsid,
    AP_IP_STR,
    (unsigned)WiFi.softAPgetStationNum());
  server.sendContent(b);
  server.sendContent_P(FORM_OPEN);
  server.sendContent("<div class=\"row\">");
  uiText("SSID", "apSsid", g_cfg.apSsid, 32);
  uiText("Password (min 8 chars)", "apPass", g_cfg.apPass, 63);
  uiNum("Channel", "apChannel", g_cfg.apChannel, 1, 13);
  uiNum("Max clients", "apMaxConn", g_cfg.apMaxConn, 1, 10);
  static const char* const hdV[] = {"0", "1"};
  static const char* const hdL[] = {"Visible", "Hidden"};
  uiSelect("SSID visibility", "apHidden", g_cfg.apHidden, hdV, hdL, 2);
  uiNum("TX power (dBm)", "apPowerDbm", g_cfg.apPowerDbm, 2, 20);
  server.sendContent("</div>");
  server.sendContent_P(FORM_CLOSE);
  server.sendContent("<p class=\"note\">Changing WiFi settings restarts the access point about 1 second after saving. Reconnect to the new SSID and open http://192.168.4.1. Defaults: SSID ESP32_Soil_Sensor, password 12345678.</p>");
  pageEnd();
}

static void handleSystemPage() {
  pageBegin("System", "system");
  char b[4096];
  char u[24];
  fmtUptime(timeNowMs() - g_bootMs, u, sizeof(u));
  uint64_t relayTotal = g_relayTotalMs + (relayOn ? (timeNowMs() - relayOnSinceMs) : 0);
  snprintf(b, sizeof(b),
    "<div class=\"card\"><table>"
    "<tr><th>Item</th><th>Value</th></tr>"
    "<tr><td>Firmware version</td><td>%u</td></tr>"
    "<tr><td>Uptime</td><td id=\"st-uptime\">%s</td></tr>"
    "<tr><td>Free heap</td><td id=\"st-heap\">%llu KB</td></tr>"
    "<tr><td>Minimum heap since boot</td><td id=\"st-heapmin\">%llu KB</td></tr>"
    "<tr><td>Boot count</td><td id=\"st-boots\">%llu</td></tr>"
    "<tr><td>Config sequence</td><td id=\"st-seq\">%llu</td></tr>"
    "<tr><td>Config saves</td><td id=\"st-saves\">%llu</td></tr>"
    "<tr><td>Total relay ON time</td><td id=\"st-relaytotal\">%llu ms</td></tr>"
    "<tr><td>Total samples</td><td id=\"st-samples\">%llu</td></tr>"
    "<tr><td>AP SSID</td><td>%s</td></tr>"
    "<tr><td>AP IP</td><td>%s</td></tr>"
    "</table></div>"
    "<div class=\"btnrow\">"
    "<button class=\"btn ghost\" data-action=\"save_now\">Save Config Now</button>"
    "<button class=\"btn ghost\" data-action=\"reboot\" data-confirm=\"Reboot the device?\">Reboot</button>"
    "<button class=\"btn danger\" data-action=\"factory_reset\" data-confirm=\"Erase all settings and reboot?\">Factory Reset</button>"
    "</div>"
    "<p class=\"note\">Config is stored in dual NVS slots with CRC32 integrity checks. Writes are debounced and rate-limited to protect flash for multi-year operation. A hardware watchdog and heap guard protect against hangs and memory exhaustion.</p>",
    (unsigned)FW_VERSION,
    u,
    (unsigned long long)(esp_get_free_heap_size() / 1024),
    (unsigned long long)(esp_get_minimum_free_heap_size() / 1024),
    (unsigned long long)g_cfg.bootCount,
    (unsigned long long)g_cfg.seq,
    (unsigned long long)g_cfg.saveCount,
    (unsigned long long)relayTotal,
    (unsigned long long)g_totalSamples,
    g_cfg.apSsid,
    AP_IP_STR);
  server.sendContent(b);
  pageEnd();
}

// ======================================================
// WEB SERVER API
// ======================================================
static void handleApiStatus() {
  char b[1024];
  uint64_t now = timeNowMs();
  uint64_t relayTotal = g_relayTotalMs + (relayOn ? (now - relayOnSinceMs) : 0);
  snprintf(b, sizeof(b),
    "{\"ok\":1,\"uptimeMs\":%llu,\"heapFree\":%llu,\"heapMin\":%llu,\"clients\":%u,\"ip\":\"%s\",\"moisturePct\":%llu,\"rawAdc\":%llu,\"relayOn\":%u,\"relayMode\":%llu,\"buzzerOn\":%u,\"ledOn\":%u,\"oledPage\":%llu,\"bootCount\":%llu,\"configSeq\":%llu,\"saveCount\":%llu,\"totalRelayOnMs\":%llu,\"totalSamples\":%llu,\"soilThresholdPct\":%llu,\"apSsid\":\"%s\",\"version\":%u}",
    (unsigned long long)(now - g_bootMs),
    (unsigned long long)esp_get_free_heap_size(),
    (unsigned long long)esp_get_minimum_free_heap_size(),
    (unsigned)WiFi.softAPgetStationNum(),
    AP_IP_STR,
    (unsigned long long)soilPct,
    (unsigned long long)soilRawAvg,
    relayOn ? 1u : 0u,
    (unsigned long long)g_cfg.relayMode,
    buzzerPinOn ? 1u : 0u,
    (ledLastDuty > 0 && ledLastDuty != 0xFFFFFFFFul) ? 1u : 0u,
    (unsigned long long)g_cfg.oledPage,
    (unsigned long long)g_cfg.bootCount,
    (unsigned long long)g_cfg.seq,
    (unsigned long long)g_cfg.saveCount,
    (unsigned long long)relayTotal,
    (unsigned long long)g_totalSamples,
    (unsigned long long)g_cfg.soilThresholdPct,
    g_cfg.apSsid,
    (unsigned)FW_VERSION);
  server.send(200, "application/json", b);
}
static void handleApiGraph() {
  static char b[16384];
  size_t off = 0;
  off += (size_t)snprintf(b + off, sizeof(b) - off, "{\"threshold\":%llu,\"points\":[", (unsigned long long)g_cfg.soilThresholdPct);
  uint32_t n = histCount;
  uint32_t start = (histHead + HISTORY_SIZE - n) % HISTORY_SIZE;
  for (uint32_t i = 0; i < n; i++) {
    uint32_t idx = (start + i) % HISTORY_SIZE;
    off += (size_t)snprintf(b + off, sizeof(b) - off, "%s[%llu,%llu]", i ? "," : "", (unsigned long long)histT[idx], (unsigned long long)histV[idx]);
  }
  snprintf(b + off, sizeof(b) - off, "]}");
  server.send(200, "application/json", b);
}
static void handleApiConfigGet() {
  char b[2048];
  snprintf(b, sizeof(b),
    "{\"soilDryRaw\":%llu,\"soilWetRaw\":%llu,\"soilThresholdPct\":%llu,\"soilHysteresisPct\":%llu,\"sampleIntervalMs\":%llu,\"avgWindow\":%llu,\"relayMode\":%llu,\"relayActiveLow\":%llu,\"relayMinOnMs\":%llu,\"relayMinOffMs\":%llu,\"buzzerEnabled\":%llu,\"buzzerBeepMs\":%llu,\"buzzerGapMs\":%llu,\"buzzerCount\":%llu,\"buzzerTrigger\":%llu,\"buzzerPeriodMs\":%llu,\"ledEnabled\":%llu,\"ledMode\":%llu,\"ledBrightness\":%llu,\"ledOnMs\":%llu,\"ledOffMs\":%llu,\"oledEnabled\":%llu,\"oledPage\":%llu,\"oledRotateMs\":%llu,\"oledContrast\":%llu,\"apSsid\":\"%s\",\"apPass\":\"%s\",\"apChannel\":%llu,\"apMaxConn\":%llu,\"apHidden\":%llu,\"apPowerDbm\":%llu}",
    (unsigned long long)g_cfg.soilDryRaw,
    (unsigned long long)g_cfg.soilWetRaw,
    (unsigned long long)g_cfg.soilThresholdPct,
    (unsigned long long)g_cfg.soilHysteresisPct,
    (unsigned long long)g_cfg.sampleIntervalMs,
    (unsigned long long)g_cfg.avgWindow,
    (unsigned long long)g_cfg.relayMode,
    (unsigned long long)g_cfg.relayActiveLow,
    (unsigned long long)g_cfg.relayMinOnMs,
    (unsigned long long)g_cfg.relayMinOffMs,
    (unsigned long long)g_cfg.buzzerEnabled,
    (unsigned long long)g_cfg.buzzerBeepMs,
    (unsigned long long)g_cfg.buzzerGapMs,
    (unsigned long long)g_cfg.buzzerCount,
    (unsigned long long)g_cfg.buzzerTrigger,
    (unsigned long long)g_cfg.buzzerPeriodMs,
    (unsigned long long)g_cfg.ledEnabled,
    (unsigned long long)g_cfg.ledMode,
    (unsigned long long)g_cfg.ledBrightness,
    (unsigned long long)g_cfg.ledOnMs,
    (unsigned long long)g_cfg.ledOffMs,
    (unsigned long long)g_cfg.oledEnabled,
    (unsigned long long)g_cfg.oledPage,
    (unsigned long long)g_cfg.oledRotateMs,
    (unsigned long long)g_cfg.oledContrast,
    g_cfg.apSsid,
    g_cfg.apPass,
    (unsigned long long)g_cfg.apChannel,
    (unsigned long long)g_cfg.apMaxConn,
    (unsigned long long)g_cfg.apHidden,
    (unsigned long long)g_cfg.apPowerDbm);
  server.send(200, "application/json", b);
}
static bool argU64(const char* key, uint64_t& out) {
  if (!server.hasArg(key)) return false;
  out = strtoull(server.arg(key).c_str(), nullptr, 10);
  return true;
}
static uint64_t clampU(uint64_t v, uint64_t mn, uint64_t mx) {
  return v < mn ? mn : (v > mx ? mx : v);
}
static bool strSafe(const char* s) {
  for (const char* p = s; *p; p++) {
    unsigned char c = (unsigned char)*p;
    if (c < 32 || c > 126 || c == '"' || c == '\\' || c == '<' || c == '>' || c == '&' || c == '\'') return false;
  }
  return true;
}
static void handleApiConfigPost() {
  bool changed = false;
  bool wifiChanged = false;
  uint64_t v;
#define SETU(field, mn, mx) do { if (argU64(#field, v)) { v = clampU(v, mn, mx); if (g_cfg.field != v) { g_cfg.field = v; changed = true; } } } while (0)
#define SETUW(field, mn, mx) do { if (argU64(#field, v)) { v = clampU(v, mn, mx); if (g_cfg.field != v) { g_cfg.field = v; changed = true; wifiChanged = true; } } } while (0)
  SETU(soilDryRaw, 0, 4095);
  SETU(soilWetRaw, 0, 4095);
  SETU(soilThresholdPct, 0, 100);
  SETU(soilHysteresisPct, 0, 50);
  SETU(sampleIntervalMs, 100, 60000);
  SETU(avgWindow, 1, 256);
  SETU(relayMode, 0, 2);
  SETU(relayActiveLow, 0, 1);
  SETU(relayMinOnMs, 0, 3600000);
  SETU(relayMinOffMs, 0, 3600000);
  SETU(buzzerEnabled, 0, 1);
  SETU(buzzerBeepMs, 10, 5000);
  SETU(buzzerGapMs, 10, 5000);
  SETU(buzzerCount, 1, 20);
  SETU(buzzerTrigger, 0, 4);
  SETU(buzzerPeriodMs, 1000, 86400000);
  SETU(ledEnabled, 0, 1);
  SETU(ledMode, 0, 3);
  SETU(ledBrightness, 0, 255);
  SETU(ledOnMs, 50, 60000);
  SETU(ledOffMs, 50, 60000);
  SETU(oledEnabled, 0, 1);
  SETU(oledPage, 0, 5);
  SETU(oledRotateMs, 0, 60000);
  SETU(oledContrast, 0, 255);
  SETUW(apChannel, 1, 13);
  SETUW(apMaxConn, 1, 10);
  SETUW(apHidden, 0, 1);
  SETUW(apPowerDbm, 2, 20);
#undef SETU
#undef SETUW
  if (server.hasArg("apSsid")) {
    String s = server.arg("apSsid");
    if (s.length() >= 1 && s.length() <= 32 && strSafe(s.c_str())) {
      if (strcmp(g_cfg.apSsid, s.c_str()) != 0) {
        strncpy(g_cfg.apSsid, s.c_str(), sizeof(g_cfg.apSsid) - 1);
        g_cfg.apSsid[sizeof(g_cfg.apSsid) - 1] = '\0';
        changed = true;
        wifiChanged = true;
      }
    }
  }
  if (server.hasArg("apPass")) {
    String s = server.arg("apPass");
    if (s.length() >= 8 && s.length() <= 63 && strSafe(s.c_str())) {
      if (strcmp(g_cfg.apPass, s.c_str()) != 0) {
        strncpy(g_cfg.apPass, s.c_str(), sizeof(g_cfg.apPass) - 1);
        g_cfg.apPass[sizeof(g_cfg.apPass) - 1] = '\0';
        changed = true;
        wifiChanged = true;
      }
    }
  }
  if (changed) configMarkDirty();
  char b[64];
  snprintf(b, sizeof(b), "{\"ok\":1,\"wifi\":%u}", wifiChanged ? 1u : 0u);
  server.send(200, "application/json", b);
  if (wifiChanged) {
    g_wifiRestartPending = true;
    g_wifiRestartStep = 0;
    g_wifiRestartAtMs = timeNowMs() + 800;
  }
}
static void handleApiAction() {
  String a = server.arg("action");
  bool ok = true;
  if (a == "buzzer_test") {
    buzzerTestReq = true;
  } else if (a == "led_test") {
    ledPhaseStartMs = timeNowMs();
    ledPhaseOn = true;
    ledPhaseIdx = 0;
  } else if (a == "relay_on") {
    g_cfg.relayMode = 1;
    configMarkDirty();
  } else if (a == "relay_off") {
    g_cfg.relayMode = 2;
    configMarkDirty();
  } else if (a == "relay_auto") {
    g_cfg.relayMode = 0;
    configMarkDirty();
  } else if (a == "calib_dry") {
    g_cfg.soilDryRaw = soilRawAvg;
    configMarkDirty();
  } else if (a == "calib_wet") {
    g_cfg.soilWetRaw = soilRawAvg;
    configMarkDirty();
  } else if (a == "oled_next") {
    oledNextPage();
  } else if (a == "oled_prev") {
    oledPrevPage();
  } else if (a == "save_now") {
    ok = configSave(true);
  } else if (a == "reboot") {
    g_rebootPending = true;
    g_rebootAtMs = timeNowMs() + 500;
  } else if (a == "factory_reset") {
    g_factoryPending = true;
    g_factoryAtMs = timeNowMs() + 500;
  } else {
    ok = false;
  }
  server.send(200, "application/json", ok ? "{\"ok\":1}" : "{\"ok\":0}");
}

// ======================================================
// CAPTIVE PORTAL
// ======================================================
static void handleNotFound() {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Location", "http://192.168.4.1/");
  server.send(302, "text/plain", "Redirect");
}

// ======================================================
// WEB SERVER ROUTES
// ======================================================
static void webRoutes() {
  server.on("/", HTTP_GET, handleDashboard);
  server.on("/soil", HTTP_GET, handleSoilPage);
  server.on("/buzzer", HTTP_GET, handleBuzzerPage);
  server.on("/led", HTTP_GET, handleLedPage);
  server.on("/oled", HTTP_GET, handleOledPage);
  server.on("/wifi", HTTP_GET, handleWifiPage);
  server.on("/system", HTTP_GET, handleSystemPage);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/graph", HTTP_GET, handleApiGraph);
  server.on("/api/config", HTTP_GET, handleApiConfigGet);
  server.on("/api/config", HTTP_POST, handleApiConfigPost);
  server.on("/api/action", HTTP_POST, handleApiAction);
  server.onNotFound(handleNotFound);
}

// ======================================================
// SYSTEM HEALTH
// ======================================================
static void wdtInit() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wc{};
  wc.timeout_ms = WDT_TIMEOUT_S * 1000;
  wc.idle_core_mask = 0;
  wc.trigger_panic = true;
  esp_task_wdt_init(&wc);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);
}
static void taskHealth(uint64_t now) {
  esp_task_wdt_reset();
  configSave(false);
  if (now - healthLastMs >= 1000) {
    healthLastMs = now;
    if (esp_get_free_heap_size() < HEAP_CRITICAL_BYTES) {
      if (g_heapLowSinceMs == 0) {
        g_heapLowSinceMs = now;
      } else if (now - g_heapLowSinceMs >= HEAP_CRITICAL_MS) {
        configSave(true);
        ESP.restart();
      }
    } else {
      g_heapLowSinceMs = 0;
    }
  }
  if (now - g_lastStatsSaveMs >= STATS_SAVE_MS) {
    g_lastStatsSaveMs = now;
    configMarkDirty();
  }
  if (g_rebootPending && now >= g_rebootAtMs) {
    configSave(true);
    ESP.restart();
  }
  if (g_factoryPending && now >= g_factoryAtMs) {
    factoryReset();
  }
}

// ======================================================
// SETUP
// ======================================================
void setup() {
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BOOT_BTN, INPUT_PULLUP);
  pinMode(PIN_SOIL_ADC, INPUT);
  analogSetPinAttenuation(PIN_SOIL_ADC, ADC_11db);
  configLoad();
  digitalWrite(PIN_RELAY, relayLevelFor(false));
  digitalWrite(PIN_BUZZER, LOW);
  ledPwmInit();
  g_bootMs = timeNowMs();
  relayLastChangeMs = g_bootMs - g_cfg.relayMinOffMs;
  ledPhaseStartMs = g_bootMs;
  ledPhaseOn = true;
  soilRawAvg = (uint64_t)analogRead(PIN_SOIL_ADC);
  soilPct = soilToPct(soilRawAvg);
  soilLastPublishMs = g_bootMs;
  soilLastSampleMs = g_bootMs;
  soilPushHistory(g_bootMs, soilPct);
  g_cfg.bootCount++;
  configMarkDirty();
  oledInit();
  wifiStart();
  dnsServer.start(53, "*", WiFi.softAPIP());
  webRoutes();
  server.begin(80);
  wdtInit();
}

// ======================================================
// LOOP
// ======================================================
void loop() {
  uint64_t now = timeNowMs();
  taskButton(now);
  taskSoil(now);
  taskRelay(now);
  taskBuzzer(now);
  taskLed(now);
  taskOled(now);
  taskWifi(now);
  dnsServer.processNextRequest();
  server.handleClient();
  taskHealth(now);
}
