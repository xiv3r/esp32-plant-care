/*
==========================================
* ESP32 Unified Environmental Relay Controller
* Author: Raff Alds
* Github: github.com/xiv3r
* License: MIT
==========================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <ESPmDNS.h>
#include <math.h>
#include <ctype.h>
#include <stdlib.h>

// -------------------- Hardware pins / constants --------------------

#define RELAY_PIN             25
#define BOOT_BUTTON_PIN       0
#define SOIL_PIN              34
#define I2C_SDA_PIN           21
#define I2C_SCL_PIN           22

static const uint8_t SHT31_ADDR_PRIMARY = 0x44;
static const uint8_t SHT31_ADDR_ALT     = 0x45;

static const char* APP_VERSION = "1.0.2-audited";

static const unsigned long BOOT_RESET_HOLD_MS   = 5000UL;
static const unsigned long STA_RETRY_MS         = 30000UL;
static const unsigned long AP_RETRY_MS          = 5000UL;
static const unsigned long SENSOR_STALE_MIN_MS  = 30000UL;
static const unsigned long SHT_RETRY_MS         = 10000UL;

// -------------------- Runtime types --------------------

enum RuntimeMode {
  MODE_AUTO = 0,
  MODE_OFF  = 1,
  MODE_ON   = 2
};

struct Settings {
  String hostname;

  bool staEnabled;
  String staSsid;
  String staPass;

  bool apEnabled;
  bool apFallback;
  bool captivePortal;
  bool apHidden;
  uint8_t apChannel;
  String apSsid;
  String apPass;
  uint8_t apMaxClients;

  bool relayActiveLow;
  bool failSafeOnSensorFault;
  uint32_t relayMinOnMs;
  uint32_t relayMinOffMs;
  uint16_t sensorInterval;

  bool soilEnabled;
  float soilThreshold;
  float soilHysteresis;
  float soilDryRaw;
  float soilWetRaw;

  bool tempEnabled;
  float tempThreshold;
  float tempHysteresis;
  bool tempTriggerAbove;

  bool humEnabled;
  float humThreshold;
  float humHysteresis;
  bool humTriggerAbove;

  bool combineAny; 
};

struct AutoEval {
  bool target;
  bool fault;
  bool soilActive;
  bool tempActive;
  bool humActive;
  int enabledCount;
  bool combineAny;
  const char* reason;
};

// -------------------- Globals --------------------

Preferences prefs;
WebServer server(80);
DNSServer dnsServer;
Adafruit_SHT31 sht31 = Adafruit_SHT31();

Settings settings;

RuntimeMode runtimeMode = MODE_AUTO;

bool relayState = false;
const char* relayReason = "Boot - auto mode";
unsigned long lastRelayChangeMs = 0;

AutoEval lastEval;

float soilRawFiltered = NAN;
float soilPercent = NAN;
bool soilValid = false;
unsigned long soilLastValidMs = 0;

float tempC = NAN;
float humidity = NAN;
bool shtValid = false;
bool shtReady = false;
uint8_t shtAddr = 0;
uint8_t shtFailCount = 0;
unsigned long lastShtInitAttempt = 0;
unsigned long shtLastValidMs = 0;

unsigned long lastSensorReadMs = 0;

bool soilActivePrev = false;
bool tempActivePrev = false;
bool humActivePrev = false;

bool apActive = false;
bool dnsActive = false;

bool networkDirty = false;
unsigned long networkApplyAt = 0;
unsigned long lastStaAttempt = 0;
unsigned long lastAPAttempt = 0;
unsigned long lastDNSAttempt = 0;
unsigned long lastMdnsAttempt = 0;

bool mdnsStarted = false;
String lastMdnsHostname = "";
IPAddress lastMdnsIp(0, 0, 0, 0);

bool bootButtonDown = false;
unsigned long bootButtonDownMs = 0;
bool bootResetTriggered = false;

// Forward declarations
void sendJson(int code, const String& body);
void sendError(int code, const String& message);
void serviceRelay();
void reconfigureNetwork();
void serviceNetwork();
void resetConditionHysteresis();
void doFactoryReset();

// -------------------- Utility functions --------------------

String jsonEscape(const String& s) {
  String out;
  out.reserve(s.length() * 2 + 16);

  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\b':
        out += "\\b";
        break;
      case '\f':
        out += "\\f";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if ((unsigned char)c < 0x20) {
          char buf[7];
          snprintf(buf, sizeof(buf), "\\u%04x", (unsigned char)c);
          out += buf;
        } else {
          out += c;
        }
        break;
    }
  }

  return out;
}

String jsonEscape(const char* s) {
  if (s == nullptr) {
    return "";
  }
  return jsonEscape(String(s));
}

String boolJson(bool v) {
  return v ? "true" : "false";
}

String jf(float v) {
  if (!isfinite(v)) {
    return "null";
  }
  return String(v, 2);
}

float clampFloat(float v, float lo, float hi) {
  if (!isfinite(v)) {
    return lo;
  }
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

String sanitizeHostname(String s) {
  s.trim();
  String out;
  out.reserve(s.length());

  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (isalnum((unsigned char)c) || c == '-') {
      out += c;
    }
  }

  while (out.length() > 0 && out[0] == '-') {
    out.remove(0, 1);
  }

  while (out.length() > 0 && out[out.length() - 1] == '-') {
    out.remove(out.length() - 1);
  }

  if (out.length() == 0) {
    out = "esp32-env";
  }

  if (out.length() > 32) {
    out = out.substring(0, 32);
  }

  while (out.length() > 0 && out[out.length() - 1] == '-') {
    out.remove(out.length() - 1);
  }

  if (out.length() == 0) {
    out = "esp32-env";
  }

  return out;
}

const char* modeName(RuntimeMode m) {
  switch (m) {
    case MODE_AUTO: return "auto";
    case MODE_OFF:  return "off";
    case MODE_ON:   return "on";
    default:        return "auto";
  }
}

const char* wifiStatusName(wl_status_t s) {
  switch (s) {
    case WL_IDLE_STATUS:      return "idle";
    case WL_NO_SSID_AVAIL:    return "no_ssid";
    case WL_SCAN_COMPLETED:   return "scan_completed";
    case WL_CONNECTED:        return "connected";
    case WL_CONNECT_FAILED:   return "connect_failed";
    case WL_CONNECTION_LOST:  return "connection_lost";
    case WL_DISCONNECTED:     return "disconnected";
    default:                  return "unknown";
  }
}

bool getBoolArg(const char* name) {
  if (!server.hasArg(name)) {
    return false;
  }

  String v = server.arg(name);
  v.toLowerCase();

  return v == "1" || v == "true" || v == "on" || v == "yes";
}

bool getStringArg(const char* name, String& out, String& err) {
  if (!server.hasArg(name)) {
    err = String("Missing field: ") + name;
    return false;
  }
  out = server.arg(name);
  return true;
}

bool readFloatArg(const char* name, float& out, float minValue, float maxValue, String& err) {
  if (!server.hasArg(name)) {
    err = String("Missing field: ") + name;
    return false;
  }

  String s = server.arg(name);
  s.trim();

  if (s.length() == 0) {
    err = String("Empty field: ") + name;
    return false;
  }

  char* end = nullptr;
  double v = strtod(s.c_str(), &end);

  if (end == s.c_str() || *end != '\0' || !isfinite(v)) {
    err = String("Invalid number: ") + name;
    return false;
  }

  if (v < minValue || v > maxValue) {
    err = String("Out of range: ") + name;
    return false;
  }

  out = (float)v;
  return true;
}

bool readIntArg(const char* name, int& out, int minValue, int maxValue, String& err) {
  if (!server.hasArg(name)) {
    err = String("Missing field: ") + name;
    return false;
  }

  String s = server.arg(name);
  s.trim();

  if (s.length() == 0) {
    err = String("Empty field: ") + name;
    return false;
  }

  char* end = nullptr;
  long v = strtol(s.c_str(), &end, 10);

  if (end == s.c_str() || *end != '\0') {
    err = String("Invalid integer: ") + name;
    return false;
  }

  if (v < minValue || v > maxValue) {
    err = String("Out of range: ") + name;
    return false;
  }

  out = (int)v;
  return true;
}

bool readULongArg(const char* name, uint32_t& out, uint32_t minValue, uint32_t maxValue, String& err) {
  if (!server.hasArg(name)) {
    err = String("Missing field: ") + name;
    return false;
  }

  String s = server.arg(name);
  s.trim();

  if (s.length() == 0) {
    err = String("Empty field: ") + name;
    return false;
  }

  if (s.startsWith("-")) {
    err = String("Negative value not allowed: ") + name;
    return false;
  }

  char* end = nullptr;
  unsigned long v = strtoul(s.c_str(), &end, 10);

  if (end == s.c_str() || *end != '\0') {
    err = String("Invalid unsigned integer: ") + name;
    return false;
  }

  if (v < minValue || v > maxValue) {
    err = String("Out of range: ") + name;
    return false;
  }

  out = (uint32_t)v;
  return true;
}

void sendJson(int code, const String& body) {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  if (body.length() == 0) {
    server.send(code, "application/json", "{\"ok\":false}");
  } else {
    server.send(code, "application/json", body);
  }
}

void sendError(int code, const String& message) {
  String body = "{\"ok\":false,\"error\":\"";
  body += jsonEscape(message);
  body += "\"}";
  sendJson(code, body);
}

// -------------------- Settings persistence --------------------

void loadDefaults() {
  settings.hostname = "esp32-env";

  settings.staEnabled = false;
  settings.staSsid = "";
  settings.staPass = "";

  settings.apEnabled = true;
  settings.apFallback = true;
  settings.captivePortal = true;
  settings.apHidden = false;
  settings.apChannel = 1;
  settings.apSsid = "ESP32-Env";
  settings.apPass = "12345678";
  settings.apMaxClients = 4;

  settings.relayActiveLow = true;
  settings.failSafeOnSensorFault = true;
  settings.relayMinOnMs = 5000UL;
  settings.relayMinOffMs = 5000UL;
  settings.sensorInterval = 2;

  settings.soilEnabled = true;
  settings.soilThreshold = 35.0f;
  settings.soilHysteresis = 5.0f;
  settings.soilDryRaw = 3200.0f;
  settings.soilWetRaw = 1200.0f;

  settings.tempEnabled = false;
  settings.tempThreshold = 35.0f;
  settings.tempHysteresis = 2.0f;
  settings.tempTriggerAbove = true;

  settings.humEnabled = false;
  settings.humThreshold = 40.0f;
  settings.humHysteresis = 5.0f;
  settings.humTriggerAbove = false;

  settings.combineAny = false;
}

void sanitizeLoadedSettings() {
  settings.hostname = sanitizeHostname(settings.hostname);

  if (settings.staSsid.length() > 32) {
    settings.staSsid = settings.staSsid.substring(0, 32);
  }

  if (settings.staPass.length() > 0 &&
      (settings.staPass.length() < 8 || settings.staPass.length() > 63)) {
    settings.staPass = "";
  }

  if (settings.apSsid.length() > 32) {
    settings.apSsid = settings.apSsid.substring(0, 32);
  }

  if (settings.apSsid.length() == 0) {
    settings.apSsid = "ESP32-Env";
  }

  if (settings.apPass.length() > 0 &&
      (settings.apPass.length() < 8 || settings.apPass.length() > 63)) {
    settings.apPass = "12345678";
  }

  if (settings.apChannel < 1 || settings.apChannel > 13) {
    settings.apChannel = 1;
  }

  if (settings.apMaxClients < 1 || settings.apMaxClients > 4) {
    settings.apMaxClients = 4;
  }

  if (settings.staEnabled && settings.staSsid.length() == 0) {
    settings.staEnabled = false;
  }

  if (!settings.staEnabled && !settings.apEnabled) {
    settings.apEnabled = true;
  }

  settings.soilThreshold = clampFloat(settings.soilThreshold, 0.0f, 100.0f);
  settings.soilHysteresis = clampFloat(settings.soilHysteresis, 0.0f, 100.0f);
  settings.soilDryRaw = clampFloat(settings.soilDryRaw, 0.0f, 4095.0f);
  settings.soilWetRaw = clampFloat(settings.soilWetRaw, 0.0f, 4095.0f);

  if (fabs(settings.soilDryRaw - settings.soilWetRaw) < 25.0f) {
    settings.soilDryRaw = 3200.0f;
    settings.soilWetRaw = 1200.0f;
  }

  settings.tempThreshold = clampFloat(settings.tempThreshold, -40.0f, 125.0f);
  settings.tempHysteresis = clampFloat(settings.tempHysteresis, 0.0f, 125.0f);

  settings.humThreshold = clampFloat(settings.humThreshold, 0.0f, 100.0f);
  settings.humHysteresis = clampFloat(settings.humHysteresis, 0.0f, 100.0f);

  if (settings.sensorInterval < 1) {
    settings.sensorInterval = 2;
  }
  if (settings.sensorInterval > 60) {
    settings.sensorInterval = 60;
  }

  if (settings.relayMinOnMs > 3600000UL) {
    settings.relayMinOnMs = 3600000UL;
  }
  if (settings.relayMinOffMs > 3600000UL) {
    settings.relayMinOffMs = 3600000UL;
  }
}

void loadSettings() {
  loadDefaults();

  if (prefs.begin("envmon", true)) {
    settings.hostname = prefs.getString("host", settings.hostname);

    settings.staEnabled = prefs.getBool("staEn", settings.staEnabled);
    settings.staSsid = prefs.getString("staSsid", settings.staSsid);
    settings.staPass = prefs.getString("staPass", settings.staPass);

    settings.apEnabled = prefs.getBool("apEn", settings.apEnabled);
    settings.apFallback = prefs.getBool("apFall", settings.apFallback);
    settings.captivePortal = prefs.getBool("portal", settings.captivePortal);
    settings.apHidden = prefs.getBool("apHide", settings.apHidden);
    settings.apChannel = (uint8_t)prefs.getUInt("apCh", settings.apChannel);
    settings.apSsid = prefs.getString("apSsid", settings.apSsid);
    settings.apPass = prefs.getString("apPass", settings.apPass);
    settings.apMaxClients = (uint8_t)prefs.getUInt("apMax", settings.apMaxClients);

    settings.relayActiveLow = prefs.getBool("relayLow", settings.relayActiveLow);
    settings.failSafeOnSensorFault = prefs.getBool("failSafe", settings.failSafeOnSensorFault);
    settings.relayMinOnMs = prefs.getULong("minOn", settings.relayMinOnMs);
    settings.relayMinOffMs = prefs.getULong("minOff", settings.relayMinOffMs);
    settings.sensorInterval = (uint16_t)prefs.getUInt("sensorInt", settings.sensorInterval);

    settings.soilEnabled = prefs.getBool("soilEn", settings.soilEnabled);
    settings.soilThreshold = prefs.getFloat("soilThr", settings.soilThreshold);
    settings.soilHysteresis = prefs.getFloat("soilHys", settings.soilHysteresis);
    settings.soilDryRaw = prefs.getFloat("soilDry", settings.soilDryRaw);
    settings.soilWetRaw = prefs.getFloat("soilWet", settings.soilWetRaw);

    settings.tempEnabled = prefs.getBool("tempEn", settings.tempEnabled);
    settings.tempThreshold = prefs.getFloat("tempThr", settings.tempThreshold);
    settings.tempHysteresis = prefs.getFloat("tempHys", settings.tempHysteresis);
    settings.tempTriggerAbove = prefs.getBool("tempAbove", settings.tempTriggerAbove);

    settings.humEnabled = prefs.getBool("humEn", settings.humEnabled);
    settings.humThreshold = prefs.getFloat("humThr", settings.humThreshold);
    settings.humHysteresis = prefs.getFloat("humHys", settings.humHysteresis);
    settings.humTriggerAbove = prefs.getBool("humAbove", settings.humTriggerAbove);

    settings.combineAny = prefs.getBool("combAny", settings.combineAny);

    prefs.end();
  }

  sanitizeLoadedSettings();
}

void saveSettings() {
  if (prefs.begin("envmon", false)) {
    prefs.putString("host", settings.hostname);

    prefs.putBool("staEn", settings.staEnabled);
    prefs.putString("staSsid", settings.staSsid);
    prefs.putString("staPass", settings.staPass);

    prefs.putBool("apEn", settings.apEnabled);
    prefs.putBool("apFall", settings.apFallback);
    prefs.putBool("portal", settings.captivePortal);
    prefs.putBool("apHide", settings.apHidden);
    prefs.putUInt("apCh", settings.apChannel);
    prefs.putString("apSsid", settings.apSsid);
    prefs.putString("apPass", settings.apPass);
    prefs.putUInt("apMax", settings.apMaxClients);

    prefs.putBool("relayLow", settings.relayActiveLow);
    prefs.putBool("failSafe", settings.failSafeOnSensorFault);
    prefs.putULong("minOn", settings.relayMinOnMs);
    prefs.putULong("minOff", settings.relayMinOffMs);
    prefs.putUInt("sensorInt", settings.sensorInterval);

    prefs.putBool("soilEn", settings.soilEnabled);
    prefs.putFloat("soilThr", settings.soilThreshold);
    prefs.putFloat("soilHys", settings.soilHysteresis);
    prefs.putFloat("soilDry", settings.soilDryRaw);
    prefs.putFloat("soilWet", settings.soilWetRaw);

    prefs.putBool("tempEn", settings.tempEnabled);
    prefs.putFloat("tempThr", settings.tempThreshold);
    prefs.putFloat("tempHys", settings.tempHysteresis);
    prefs.putBool("tempAbove", settings.tempTriggerAbove);

    prefs.putBool("humEn", settings.humEnabled);
    prefs.putFloat("humThr", settings.humThreshold);
    prefs.putFloat("humHys", settings.humHysteresis);
    prefs.putBool("humAbove", settings.humTriggerAbove);

    prefs.putBool("combAny", settings.combineAny);

    prefs.end();
  }
}

bool validateSettings(const Settings& s, String& err) {
  if (s.hostname.length() == 0 || s.hostname.length() > 32) {
    err = "Hostname must be 1-32 characters.";
    return false;
  }

  if (s.staSsid.length() > 32) {
    err = "STA SSID must be <=32 characters.";
    return false;
  }

  if (s.staEnabled && s.staSsid.length() == 0) {
    err = "STA is enabled but SSID is empty.";
    return false;
  }

  if (s.staPass.length() > 0 &&
      (s.staPass.length() < 8 || s.staPass.length() > 63)) {
    err = "STA password must be empty or 8-63 characters.";
    return false;
  }

  if (s.apSsid.length() > 32) {
    err = "AP SSID must be <=32 characters.";
    return false;
  }

  bool apRequired = s.apEnabled || (s.staEnabled && s.apFallback);

  if (apRequired && s.apSsid.length() == 0) {
    err = "AP SSID is required when AP or fallback AP can be active.";
    return false;
  }

  if (s.apPass.length() > 0 &&
      (s.apPass.length() < 8 || s.apPass.length() > 63)) {
    err = "AP password must be empty or 8-63 characters.";
    return false;
  }

  if (s.apChannel < 1 || s.apChannel > 13) {
    err = "AP channel must be 1-13.";
    return false;
  }

  if (s.apMaxClients < 1 || s.apMaxClients > 4) {
    err = "AP max clients must be 1-4.";
    return false;
  }

  if (!s.staEnabled && !s.apEnabled) {
    err = "At least one of STA or AP must be enabled.";
    return false;
  }

  if (s.soilThreshold < 0.0f || s.soilThreshold > 100.0f) {
    err = "Soil threshold must be 0-100.";
    return false;
  }

  if (s.soilHysteresis < 0.0f || s.soilHysteresis > 100.0f) {
    err = "Soil hysteresis must be 0-100.";
    return false;
  }

  if (s.soilDryRaw < 0.0f || s.soilDryRaw > 4095.0f ||
      s.soilWetRaw < 0.0f || s.soilWetRaw > 4095.0f) {
    err = "Soil calibration raw values must be 0-4095.";
    return false;
  }

  if (fabs(s.soilDryRaw - s.soilWetRaw) < 25.0f) {
    err = "Soil dry and wet calibration raw values must differ by at least 25.";
    return false;
  }

  if (s.tempThreshold < -40.0f || s.tempThreshold > 125.0f) {
    err = "Temperature threshold must be -40 to 125 C.";
    return false;
  }

  if (s.tempHysteresis < 0.0f || s.tempHysteresis > 125.0f) {
    err = "Temperature hysteresis must be 0-125.";
    return false;
  }

  if (s.humThreshold < 0.0f || s.humThreshold > 100.0f) {
    err = "Humidity threshold must be 0-100.";
    return false;
  }

  if (s.humHysteresis < 0.0f || s.humHysteresis > 100.0f) {
    err = "Humidity hysteresis must be 0-100.";
    return false;
  }

  if (s.sensorInterval < 1 || s.sensorInterval > 60) {
    err = "Sensor interval must be 1-60 seconds.";
    return false;
  }

  if (s.relayMinOnMs > 3600000UL || s.relayMinOffMs > 3600000UL) {
    err = "Relay minimum on/off time must be <=3600000 ms.";
    return false;
  }

  return true;
}

bool networkSettingsChanged(const Settings& a, const Settings& b) {
  return a.hostname != b.hostname ||
         a.staEnabled != b.staEnabled ||
         a.staSsid != b.staSsid ||
         a.staPass != b.staPass ||
         a.apEnabled != b.apEnabled ||
         a.apFallback != b.apFallback ||
         a.captivePortal != b.captivePortal ||
         a.apHidden != b.apHidden ||
         a.apChannel != b.apChannel ||
         a.apSsid != b.apSsid ||
         a.apPass != b.apPass ||
         a.apMaxClients != b.apMaxClients;
}

// -------------------- Sensor functions --------------------

int readSoilRawAverage(int samples) {
  if (samples < 1) {
    samples = 1;
  }

  long sum = 0;
  int count = 0;

  for (int i = 0; i < samples; i++) {
    int v = analogRead(SOIL_PIN);
    if (v >= 0 && v <= 4095) {
      sum += v;
      count++;
    }
  }

  if (count == 0) {
    return -1;
  }

  return (int)((sum + count / 2) / count);
}

float rawToMoisture(float raw) {
  float dry = settings.soilDryRaw;
  float wet = settings.soilWetRaw;

  if (!isfinite(dry) || !isfinite(wet)) {
    return NAN;
  }

  if (fabs(dry - wet) < 25.0f) {
    return NAN;
  }

  float pct = (raw - dry) * 100.0f / (wet - dry);

  if (pct < 0.0f) {
    pct = 0.0f;
  }
  if (pct > 100.0f) {
    pct = 100.0f;
  }

  return pct;
}

bool beginSHT31() {
  lastShtInitAttempt = millis();

  uint8_t addrs[2] = { SHT31_ADDR_PRIMARY, SHT31_ADDR_ALT };

  for (uint8_t i = 0; i < 2; i++) {
    if (sht31.begin(addrs[i])) {
      shtAddr = addrs[i];
      sht31.heater(false);
      shtReady = true;
      shtFailCount = 0;
      return true;
    }
  }

  shtReady = false;
  return false;
}

bool soilReadingFresh() {
  if (!soilValid) {
    return false;
  }

  unsigned long staleMs = (unsigned long)settings.sensorInterval * 3000UL;
  if (staleMs < SENSOR_STALE_MIN_MS) {
    staleMs = SENSOR_STALE_MIN_MS;
  }

  return (millis() - soilLastValidMs) <= staleMs;
}

bool shtReadingFresh() {
  if (!shtValid) {
    return false;
  }

  unsigned long staleMs = (unsigned long)settings.sensorInterval * 3000UL;
  if (staleMs < SENSOR_STALE_MIN_MS) {
    staleMs = SENSOR_STALE_MIN_MS;
  }

  if (shtFailCount >= 2) {
    unsigned long tightMs = (unsigned long)settings.sensorInterval * 2000UL;
    if (tightMs < 5000UL) tightMs = 5000UL;
    if (tightMs < staleMs) staleMs = tightMs;
  } else if (shtFailCount == 1) {
    unsigned long tightMs = (unsigned long)settings.sensorInterval * 4000UL;
    if (tightMs < 10000UL) tightMs = 10000UL;
    if (tightMs < staleMs) staleMs = tightMs;
  }

  return (millis() - shtLastValidMs) <= staleMs;
}

void readSensors(bool force) {
  unsigned long now = millis();
  unsigned long intervalMs = (unsigned long)settings.sensorInterval * 1000UL;

  if (!force && (now - lastSensorReadMs) < intervalMs) {
    return;
  }

  lastSensorReadMs = now;

  // ----- Capacitive soil moisture -----
  int raw = readSoilRawAverage(16);

  if (raw >= 0) {
    if (!soilValid || isnan(soilRawFiltered)) {
      soilRawFiltered = (float)raw;
    } else {
      soilRawFiltered = soilRawFiltered * 0.70f + (float)raw * 0.30f;
    }

    float pct = rawToMoisture(soilRawFiltered);

    if (isnan(pct)) {
      soilPercent = NAN;
      soilValid = false;
    } else {
      soilPercent = pct;
      soilValid = true;
      soilLastValidMs = now;
    }
  } else {
    soilValid = false;
    soilPercent = NAN;
  }

  // ----- SHT31 temperature / humidity -----
  if (!shtReady) {
    if (now - lastShtInitAttempt > SHT_RETRY_MS) {
      beginSHT31();
    }
  }

  if (shtReady) {
    float t = sht31.readTemperature();
    float h = sht31.readHumidity();

    bool good = !isnan(t) && !isnan(h) &&
                t >= -40.0f && t <= 125.0f &&
                h >= 0.0f && h <= 100.0f;

    if (good) {
      if (shtValid) {
        tempC = tempC * 0.70f + t * 0.30f;
        humidity = humidity * 0.70f + h * 0.30f;
      } else {
        tempC = t;
        humidity = h;
      }

      shtValid = true;
      shtFailCount = 0;
      shtLastValidMs = now;
    } else {
      if (shtFailCount < 255) {
        shtFailCount++;
      }

      if (shtFailCount >= 3) {
        shtValid = false;
        shtReady = false;
      }
    }
  }
}

bool applyBelowHysteresis(bool prev, float value, float threshold, float hysteresis) {
  if (!isfinite(value)) {
    return false;
  }

  if (hysteresis < 0.0f) {
    hysteresis = 0.0f;
  }

  if (prev) {
    return value <= (threshold + hysteresis);
  }

  return value <= threshold;
}

bool applyAboveHysteresis(bool prev, float value, float threshold, float hysteresis) {
  if (!isfinite(value)) {
    return false;
  }

  if (hysteresis < 0.0f) {
    hysteresis = 0.0f;
  }

  if (prev) {
    return value >= (threshold - hysteresis);
  }

  return value >= threshold;
}

void resetConditionHysteresis() {
  soilActivePrev = false;
  tempActivePrev = false;
  humActivePrev = false;
}

AutoEval evaluateAuto() {
  AutoEval e;
  e.target = false;
  e.fault = false;
  e.soilActive = false;
  e.tempActive = false;
  e.humActive = false;
  e.enabledCount = 0;
  e.combineAny = settings.combineAny;
  e.reason = "Auto OFF";

  bool soilFresh = soilReadingFresh();
  bool shtFresh = shtReadingFresh();

  // Soil condition
  if (settings.soilEnabled) {
    e.enabledCount++;

    if (!soilFresh) {
      e.fault = true;
      e.soilActive = false;
      soilActivePrev = false;
    } else {
      e.soilActive = applyBelowHysteresis(
        soilActivePrev,
        soilPercent,
        settings.soilThreshold,
        settings.soilHysteresis
      );
      soilActivePrev = e.soilActive;
    }
  } else {
    soilActivePrev = false;
  }

  // Temperature condition
  if (settings.tempEnabled) {
    e.enabledCount++;

    if (!shtFresh) {
      e.fault = true;
      e.tempActive = false;
      tempActivePrev = false;
    } else {
      e.tempActive = settings.tempTriggerAbove
        ? applyAboveHysteresis(tempActivePrev, tempC, settings.tempThreshold, settings.tempHysteresis)
        : applyBelowHysteresis(tempActivePrev, tempC, settings.tempThreshold, settings.tempHysteresis);
      tempActivePrev = e.tempActive;
    }
  } else {
    tempActivePrev = false;
  }

  // Humidity condition
  if (settings.humEnabled) {
    e.enabledCount++;

    if (!shtFresh) {
      e.fault = true;
      e.humActive = false;
      humActivePrev = false;
    } else {
      e.humActive = settings.humTriggerAbove
        ? applyAboveHysteresis(humActivePrev, humidity, settings.humThreshold, settings.humHysteresis)
        : applyBelowHysteresis(humActivePrev, humidity, settings.humThreshold, settings.humHysteresis);
      humActivePrev = e.humActive;
    }
  } else {
    humActivePrev = false;
  }

  bool anyActive = e.soilActive || e.tempActive || e.humActive;

  bool allActive = true;
  if (settings.soilEnabled) {
    allActive = allActive && e.soilActive;
  }
  if (settings.tempEnabled) {
    allActive = allActive && e.tempActive;
  }
  if (settings.humEnabled) {
    allActive = allActive && e.humActive;
  }

  if (settings.failSafeOnSensorFault && e.fault) {
    e.target = false;
    e.reason = "Auto OFF: enabled sensor fault";
    return e;
  }

  if (e.enabledCount == 0) {
    e.target = false;
    e.reason = "Auto OFF: no enabled sensor rule";
    return e;
  }

  e.target = settings.combineAny ? anyActive : allActive;

  bool faultIgnored = e.fault && !settings.failSafeOnSensorFault;
  if (e.target) {
    e.reason = faultIgnored
      ? "Auto ON: condition met (sensor fault ignored)"
      : "Auto ON: condition met";
  } else {
    e.reason = faultIgnored
      ? "Auto OFF: condition not met (sensor fault ignored)"
      : "Auto OFF: condition not met";
  }

  return e;
}

// -------------------- Relay functions --------------------

void writeRelayOutput(bool on) {
  int level = settings.relayActiveLow ? (on ? LOW : HIGH) : (on ? HIGH : LOW);
  digitalWrite(RELAY_PIN, level);
}

void setRelay(bool on, const char* reason) {
  bool changed = (relayState != on);

  writeRelayOutput(on);
  relayState = on;

  if (changed) {
    lastRelayChangeMs = millis();
  }

  relayReason = reason;
}

void serviceRelay() {
  lastEval = evaluateAuto();

  bool safetyFault = lastEval.fault && settings.failSafeOnSensorFault;

  bool desired = false;
  const char* reason = nullptr;

  if (runtimeMode == MODE_ON) {
    desired = true;
    reason = "Manual ON";
  } else if (runtimeMode == MODE_OFF) {
    desired = false;
    reason = "Manual OFF";
  } else {
    desired = lastEval.target;
    reason = lastEval.reason;
  }

  if (runtimeMode == MODE_AUTO && safetyFault) {
    if (relayState) {
      setRelay(false, reason);
    } else {
      relayReason = reason;
    }
    return;
  }

  if (desired == relayState) {
    relayReason = reason;
    return;
  }

  unsigned long age = millis() - lastRelayChangeMs;

  bool canSwitch = true;

  if (runtimeMode == MODE_AUTO) {
    if (desired && age < settings.relayMinOffMs) {
      canSwitch = false;
      reason = "Auto ON: minimum OFF time not elapsed";
    }

    if (!desired && age < settings.relayMinOnMs) {
      canSwitch = false;
      reason = "Auto OFF: minimum ON time not elapsed";
    }
  }

  if (canSwitch) {
    setRelay(desired, reason);
  } else {
    relayReason = reason;
  }
}

// -------------------- Network functions --------------------

void startDNS() {
  lastDNSAttempt = millis();

  if (!apActive) {
    return;
  }

  IPAddress ip = WiFi.softAPIP();

  if (ip == IPAddress(0, 0, 0, 0)) {
    return;
  }

  if (dnsActive) {
    dnsServer.stop();
    dnsActive = false;
  }

  dnsActive = dnsServer.start(53, "*", ip);
}

void stopDNS() {
  if (dnsActive) {
    dnsServer.stop();
    dnsActive = false;
  }
}

String apRedirectUrl() {
  IPAddress ip = WiFi.softAPIP();

  if (ip == IPAddress(0, 0, 0, 0)) {
    return "/";
  }

  String out = "http://";
  out += ip.toString();
  out += "/";
  return out;
}

void stopMDNS() {
  if (mdnsStarted) {
    MDNS.end();
    mdnsStarted = false;
    lastMdnsHostname = "";
    lastMdnsIp = IPAddress(0, 0, 0, 0);
  }
}

void startMDNS() {
  if (WiFi.getMode() == WIFI_OFF) {
    stopMDNS();
    return;
  }

  IPAddress ip(0, 0, 0, 0);

  if (WiFi.status() == WL_CONNECTED) {
    ip = WiFi.localIP();
  } else if (apActive) {
    ip = WiFi.softAPIP();
  }

  if (ip == IPAddress(0, 0, 0, 0)) {
    stopMDNS();
    return;
  }

  if (mdnsStarted &&
      lastMdnsHostname == settings.hostname &&
      lastMdnsIp == ip) {
    return;
  }

  if (lastMdnsAttempt != 0 && (millis() - lastMdnsAttempt) < 5000UL) {
    return;
  }

  lastMdnsAttempt = millis();

  if (mdnsStarted) {
    MDNS.end();
    mdnsStarted = false;
  }

  if (MDNS.begin(settings.hostname.c_str())) {
    MDNS.addService("http", "tcp", 80);
    mdnsStarted = true;
    lastMdnsHostname = settings.hostname;
    lastMdnsIp = ip;
  }
}

bool softAPStart() {
  if (apActive) {
    return true;
  }

  const char* apPassword = settings.apPass.length() > 0 ? settings.apPass.c_str() : nullptr;

  bool ok = WiFi.softAP(
    settings.apSsid.c_str(),
    apPassword,
    settings.apChannel,
    settings.apHidden,
    settings.apMaxClients
  );

  if (ok) {
    apActive = true;

    if (settings.captivePortal) {
      startDNS();
    }
  }

  return ok;
}

bool startAP() {
  if (apActive) {
    return true;
  }

  if (WiFi.getMode() != WIFI_AP_STA) {
    WiFi.mode(WIFI_AP_STA);
    delay(20);
  }

  return softAPStart();
}

void stopAP() {
  stopDNS();

  if (apActive) {
    WiFi.softAPdisconnect(false);
    apActive = false;
  }

  if (settings.staEnabled) {
    if (WiFi.getMode() != WIFI_STA) {
      WiFi.mode(WIFI_STA);
    }
  } else if (!settings.apEnabled) {
    WiFi.mode(WIFI_OFF);
  }

  startMDNS();
}

void reconfigureNetwork() {
  networkDirty = false;

  stopDNS();
  stopMDNS();

  if (apActive) {
    WiFi.softAPdisconnect(false);
    apActive = false;
  }

  WiFi.disconnect(false, false);
  delay(60);

  bool wantAP = settings.apEnabled || (settings.staEnabled && settings.apFallback);

  wifi_mode_t mode = WIFI_OFF;

  if (settings.staEnabled && wantAP) {
    mode = WIFI_AP_STA;
  } else if (settings.staEnabled) {
    mode = WIFI_STA;
  } else if (wantAP) {
    mode = WIFI_AP_STA;
  }

  WiFi.mode(mode);
  delay(30);

  WiFi.setHostname(settings.hostname.c_str());
  WiFi.setAutoReconnect(settings.staEnabled);

  if (!settings.staEnabled) {
    WiFi.disconnect(false, false);
  }

  if (wantAP) {
    lastAPAttempt = millis();
    softAPStart();
  }

  if (settings.staEnabled) {
    WiFi.begin(settings.staSsid.c_str(), settings.staPass.c_str());
    lastStaAttempt = millis();
  }

  startMDNS();
}

void serviceNetwork() {
  if (!settings.staEnabled) {
    if (WiFi.status() == WL_CONNECTED) {
      WiFi.disconnect(false, false);
    }
  } else {
    if (WiFi.status() != WL_CONNECTED) {
      if ((millis() - lastStaAttempt) > STA_RETRY_MS) {
        WiFi.disconnect(false, false);
        WiFi.begin(settings.staSsid.c_str(), settings.staPass.c_str());
        lastStaAttempt = millis();
      }
    }
  }

  // AP / fallback AP management
  bool fallbackNeeded = settings.staEnabled &&
                        settings.apFallback &&
                        (WiFi.status() != WL_CONNECTED);

  bool shouldAP = settings.apEnabled || fallbackNeeded;

  if (shouldAP) {
    if (!apActive && (millis() - lastAPAttempt > AP_RETRY_MS)) {
      lastAPAttempt = millis();
      startAP();
    }
  } else {
    if (apActive) {
      stopAP();
    }
  }

  // Captive portal DNS
  if (apActive && settings.captivePortal) {
    if (!dnsActive && (millis() - lastDNSAttempt > 1000UL)) {
      startDNS();
    }
  } else {
    if (dnsActive) {
      stopDNS();
    }
  }

  startMDNS();
}

// -------------------- Boot button / factory reset --------------------

void doFactoryReset() {
  setRelay(false, "Factory reset");
  writeRelayOutput(false);

  stopDNS();
  stopMDNS();

  if (apActive) {
    WiFi.softAPdisconnect(false);
    apActive = false;
  }

  WiFi.disconnect(true, true);
  delay(120);

  if (prefs.begin("envmon", false)) {
    prefs.clear();
    prefs.end();
  }

  delay(250);
  ESP.restart();
}

void handleBootButton() {
  bool down = (digitalRead(BOOT_BUTTON_PIN) == LOW);

  if (down) {
    if (!bootButtonDown) {
      bootButtonDown = true;
      bootButtonDownMs = millis();
    }

    if (!bootResetTriggered &&
        (millis() - bootButtonDownMs >= BOOT_RESET_HOLD_MS)) {
      bootResetTriggered = true;
      setRelay(false, "Factory reset boot button");
      delay(100);
      doFactoryReset();
    }
  } else {
    bootButtonDown = false;
    bootResetTriggered = false;
  }
}

// -------------------- Embedded dashboard --------------------

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Sensor Relay Dashboard</title>
<style>
:root{
  color-scheme:dark;
  --bg:#08111f;
  --card:#101c31;
  --card2:#0c1729;
  --line:#26364f;
  --text:#e5eefb;
  --muted:#93a4bd;
  --blue:#3b82f6;
  --green:#22c55e;
  --red:#ef4444;
  --amber:#f59e0b;
}
*{box-sizing:border-box}
html,body{margin:0;padding:0}
body{
  min-height:100vh;
  background:radial-gradient(circle at top left,#14233f 0,var(--bg) 38%,#050b14 100%);
  color:var(--text);
  font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,Cantarell,Noto Sans,Arial,sans-serif;
}
header{
  max-width:1200px;
  margin:0 auto;
  padding:18px 16px 8px;
  display:flex;
  justify-content:space-between;
  align-items:flex-start;
  gap:12px;
}
h1{font-size:21px;margin:0 0 3px}
h2{font-size:16px;margin:0 0 10px;color:#bfdbfe}
h3{font-size:13px;margin:18px 0 8px;color:#93c5fd;text-transform:uppercase;letter-spacing:.06em}
main{max-width:1200px;margin:0 auto;padding:0 16px 28px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(245px,1fr));gap:14px}
.card{
  background:linear-gradient(180deg,rgba(23,37,61,.92),rgba(10,21,38,.96));
  border:1px solid var(--line);
  border-radius:16px;
  padding:14px;
  margin-bottom:14px;
  box-shadow:0 10px 30px rgba(0,0,0,.18);
}
.big{font-size:34px;line-height:1.1;font-weight:800;margin:4px 0}
.small{font-size:12px}
.muted{color:var(--muted)}
.good{color:var(--green)}
.bad{color:var(--red)}
.on{color:var(--green)}
.off{color:var(--muted)}
.pill{
  border:1px solid var(--line);
  border-radius:999px;
  padding:6px 11px;
  font-weight:700;
  font-size:13px;
  white-space:nowrap;
  background:rgba(15,23,42,.75);
}
.pill.good{border-color:rgba(34,197,94,.45);color:var(--green)}
.pill.bad{border-color:rgba(239,68,68,.5);color:var(--red)}
.msg{
  margin:0 0 14px;
  padding:10px 12px;
  border-radius:12px;
  border:1px solid var(--line);
  background:rgba(15,23,42,.8);
}
.msg:empty{display:none}
.msg.ok{border-color:rgba(34,197,94,.5);color:#bbf7d0}
.msg.bad{border-color:rgba(239,68,68,.55);color:#fecaca}
.msg.info{color:#dbeafe}
.form-grid{
  display:grid;
  grid-template-columns:repeat(auto-fit,minmax(190px,1fr));
  gap:10px;
}
label{
  display:flex;
  flex-direction:column;
  gap:5px;
  min-width:0;
  color:#cbd5e1;
  font-size:13px;
}
label.check{
  flex-direction:row;
  align-items:center;
  gap:8px;
  min-height:38px;
}
input,select,button{
  font:inherit;
}
input,select{
  width:100%;
  min-width:0;
  background:#0b1628;
  color:var(--text);
  border:1px solid #334155;
  border-radius:10px;
  padding:9px 10px;
  outline:none;
  font-size:16px;
}
input:focus,select:focus{border-color:var(--blue);box-shadow:0 0 0 2px rgba(59,130,246,.18)}
label.check input{width:auto}
button{
  cursor:pointer;
  border:1px solid #334155;
  background:#1e293b;
  color:var(--text);
  border-radius:10px;
  padding:9px 12px;
  font-weight:700;
  white-space:nowrap;
}
button:hover{border-color:#64748b}
button.primary{background:#1d4ed8;border-color:#3b82f6}
button.warn{background:#78350f;border-color:#f59e0b}
button.danger{background:#7f1d1d;border-color:#ef4444}
.btnrow{display:flex;flex-wrap:wrap;gap:8px;margin-top:10px}
.scanList{
  display:grid;
  gap:6px;
  margin-top:10px;
  max-height:280px;
  overflow:auto;
}
.scan-item{
  display:flex;
  justify-content:space-between;
  align-items:center;
  text-align:left;
  gap:12px;
  width:100%;
}
.scan-item span:last-child{color:var(--muted);font-size:12px}
.hr{height:1px;background:var(--line);margin:14px 0}
@media (max-width:560px){
  header{flex-direction:column}
  .big{font-size:30px}
}
</style>
</head>
<body>
<header>
  <div>
    <h1>ESP32 Environmental Relay</h1>
    <div class="muted small">Soil moisture + SHT31 + GPIO25 relay + WiFi management</div>
  </div>
  <div id="connPill" class="pill bad">Connecting...</div>
</header>

<main>
  <div id="msg" class="msg"></div>

  <section class="grid">
    <div class="card">
      <h2>Relay GPIO25</h2>
      <div id="relayState" class="big off">--</div>
      <div class="muted small">Reason: <span id="relayReason">--</span></div>
      <div class="btnrow">
        <button type="button" id="btnModeAuto" onclick="setMode('auto')">Auto</button>
        <button type="button" id="btnModeOff"  onclick="setMode('off')">Manual Off</button>
        <button type="button" id="btnModeOn"   onclick="setMode('on')">Manual On</button>
      </div>
      <div class="muted small" style="margin-top:8px">
        Mode: <span id="controlMode">--</span> &middot; Last change: <span id="relayAge">--</span>
      </div>
    </div>

    <div class="card">
      <h2>Soil Moisture</h2>
      <div class="big"><span id="soilPct">--</span><span class="muted">%</span></div>
      <div class="small">Raw: <span id="soilRaw">--</span> &middot; Threshold: <span id="soilThresholdView">--</span>%</div>
      <div class="small">Sensor: <span id="soilValid">--</span> &middot; Rule: <span id="soilActive">--</span></div>
    </div>

    <div class="card">
      <h2>SHT31</h2>
      <div class="big"><span id="tempC">--</span><span class="muted">&deg;C</span></div>
      <div class="small">Humidity: <span id="humPct">--</span>%</div>
      <div class="small">Sensor: <span id="shtValid">--</span></div>
      <div class="small">Temp rule: <span id="tempActive">--</span> &middot; Humidity rule: <span id="humActive">--</span></div>
    </div>

    <div class="card">
      <h2>Network</h2>
      <div class="small">STA: <span id="staStatus">--</span> <span id="staSsidView">--</span></div>
      <div class="small">STA IP: <span id="staIp">--</span> &middot; RSSI: <span id="staRssi">--</span></div>
      <div class="small">AP: <span id="apStatus">--</span> <span id="apSsidView">--</span> &middot; clients: <span id="apClients">--</span></div>
      <div class="small">AP IP: <span id="apIp">--</span></div>
      <div class="small">Captive portal: <span id="portalStatus">--</span></div>
      <div class="small muted" style="margin-top:8px">
        Uptime: <span id="uptime">--</span> &middot; Free heap: <span id="heap">--</span> bytes
      </div>
    </div>
  </section>

  <form id="settingsForm">
    <section class="card">
      <h2>Automatic Relay Rules</h2>
      <div class="form-grid">
        <label>Combine enabled conditions
          <select name="combine" id="combine">
            <option value="all">ALL enabled conditions must be true</option>
            <option value="any">ANY enabled condition may be true</option>
          </select>
        </label>
        <label class="check">
          <input type="checkbox" name="failSafeOnSensorFault" id="failSafeOnSensorFault">
          Fail-safe OFF if an enabled sensor is invalid
        </label>
      </div>

      <h3>Soil Moisture Rule</h3>
      <div class="form-grid">
        <label class="check">
          <input type="checkbox" name="soilEnabled" id="soilEnabled">
          Enable soil rule
        </label>
        <label>Dry threshold (%)
          <input type="number" required step="0.1" min="0" max="100" name="soilThreshold" id="soilThreshold">
        </label>
        <label>Hysteresis (%)
          <input type="number" required step="0.1" min="0" max="100" name="soilHysteresis" id="soilHysteresis">
        </label>
        <label>Dry calibration raw
          <input type="number" required step="1" min="0" max="4095" name="soilDryRaw" id="soilDryRaw">
        </label>
        <label>Wet calibration raw
          <input type="number" required step="1" min="0" max="4095" name="soilWetRaw" id="soilWetRaw">
        </label>
        <div class="btnrow">
          <button type="button" onclick="calibrateSoil('dry')">Calibrate Dry Now</button>
          <button type="button" onclick="calibrateSoil('wet')">Calibrate Wet Now</button>
        </div>
      </div>
      <p class="muted small">Soil rule becomes active at moisture &lt;= threshold. It clears at moisture &gt;= threshold + hysteresis.</p>

      <h3>SHT31 Temperature Rule</h3>
      <div class="form-grid">
        <label class="check">
          <input type="checkbox" name="tempEnabled" id="tempEnabled">
          Enable temperature rule
        </label>
        <label>Threshold (&deg;C)
          <input type="number" required step="0.1" min="-40" max="125" name="tempThreshold" id="tempThreshold">
        </label>
        <label>Hysteresis (&deg;C)
          <input type="number" required step="0.1" min="0" max="125" name="tempHysteresis" id="tempHysteresis">
        </label>
        <label>Trigger direction
          <select name="tempDirection" id="tempDirection">
            <option value="above">Above threshold</option>
            <option value="below">Below threshold</option>
          </select>
        </label>
      </div>
      <p class="muted small">Temperature rule activates when the reading crosses the threshold, and clears at threshold &plusmn; hysteresis (sign depends on direction).</p>

      <h3>SHT31 Humidity Rule</h3>
      <div class="form-grid">
        <label class="check">
          <input type="checkbox" name="humEnabled" id="humEnabled">
          Enable humidity rule
        </label>
        <label>Threshold (%RH)
          <input type="number" required step="0.1" min="0" max="100" name="humThreshold" id="humThreshold">
        </label>
        <label>Hysteresis (%RH)
          <input type="number" required step="0.1" min="0" max="100" name="humHysteresis" id="humHysteresis">
        </label>
        <label>Trigger direction
          <select name="humDirection" id="humDirection">
            <option value="above">Above threshold</option>
            <option value="below">Below threshold</option>
          </select>
        </label>
      </div>
      <p class="muted small">Humidity rule activates when the reading crosses the threshold, and clears at threshold &plusmn; hysteresis (sign depends on direction).</p>
    </section>

    <section class="card">
      <h2>Relay Hardware &amp; Sampling</h2>
      <div class="form-grid">
        <label class="check">
          <input type="checkbox" name="relayActiveLow" id="relayActiveLow">
          Relay is active LOW
        </label>
        <label>Minimum ON time (ms)
          <input type="number" required min="0" max="3600000" step="100" name="relayMinOnMs" id="relayMinOnMs">
        </label>
        <label>Minimum OFF time (ms)
          <input type="number" required min="0" max="3600000" step="100" name="relayMinOffMs" id="relayMinOffMs">
        </label>
        <label>Sensor sample interval (seconds)
          <input type="number" required min="1" max="60" step="1" name="sensorInterval" id="sensorInterval">
        </label>
      </div>
    </section>

    <section class="card">
      <h2>Wi-Fi Station (STA)</h2>
      <div class="form-grid">
        <label class="check">
          <input type="checkbox" name="staEnabled" id="staEnabled">
          Enable station connection
        </label>
        <label>SSID
          <input type="text" maxlength="32" name="staSsid" id="staSsid" autocomplete="off">
        </label>
        <label>Password
          <input type="password" maxlength="63" name="staPass" id="staPass" autocomplete="new-password">
        </label>
        <label class="check">
          <input type="checkbox" name="clearStaPass" id="clearStaPass">
          Clear stored STA password
        </label>
        <div class="btnrow">
          <button type="button" onclick="forgetSta()">Forget STA</button>
        </div>
      </div>
      <p class="muted small">Leave password empty for an open network. Use the WiFi scan section below to fill an SSID.</p>

      <h3>Access Point (AP)</h3>
      <div class="form-grid">
        <label class="check">
          <input type="checkbox" name="apEnabled" id="apEnabled">
          Always enable AP
        </label>
        <label class="check">
          <input type="checkbox" name="apFallback" id="apFallback">
          Fallback AP when STA is disconnected
        </label>
        <label class="check">
          <input type="checkbox" name="captivePortal" id="captivePortal">
          Enable captive portal DNS
        </label>
        <label class="check">
          <input type="checkbox" name="apHidden" id="apHidden">
          Hidden AP
        </label>
        <label>AP SSID
          <input type="text" maxlength="32" name="apSsid" id="apSsid">
        </label>
        <label>AP password
          <input type="password" maxlength="63" name="apPass" id="apPass" autocomplete="new-password">
        </label>
        <label class="check">
          <input type="checkbox" name="clearApPass" id="clearApPass">
          Clear stored AP password
        </label>
        <label>AP channel
          <input type="number" required min="1" max="13" name="apChannel" id="apChannel">
        </label>
        <label>AP max clients
          <input type="number" required min="1" max="4" name="apMaxClients" id="apMaxClients">
        </label>
      </div>
      <p class="muted small">AP password empty = open. Otherwise use 8-63 characters.</p>

      <h3>System</h3>
      <div class="form-grid">
        <label>Hostname
          <input type="text" maxlength="32" name="hostname" id="hostname">
        </label>
      </div>
    </section>

    <div class="btnrow">
      <button type="submit" class="primary">Save &amp; Apply</button>
      <button type="button" onclick="loadSettings()">Reload from Device</button>
    </div>
  </form>

  <section class="card">
    <h2>Wi-Fi Scan &amp; Connect</h2>
    <p class="muted small">Scan station networks, click one to fill the STA SSID, enter its password, then press Save &amp; Apply.</p>
    <button type="button" onclick="scanWifi()">Scan WiFi</button>
    <div id="scanList" class="scanList"></div>
  </section>

  <section class="card">
    <h2>Danger Zone</h2>
    <div class="btnrow">
      <button type="button" class="warn" onclick="rebootDevice()">Reboot</button>
      <button type="button" class="danger" onclick="factoryResetDevice()">Factory Reset</button>
    </div>
    <p class="muted small">Factory reset clears stored app settings and Wi-Fi credentials. Holding GPIO0 LOW for 5 seconds also performs a factory reset.</p>
  </section>
</main>

<script>
const $ = id => document.getElementById(id);
const boolKeys = [
  'staEnabled','apEnabled','apFallback','captivePortal','apHidden',
  'relayActiveLow','failSafeOnSensorFault','soilEnabled','tempEnabled','humEnabled'
];
const textKeys = ['hostname','staSsid','staPass','apSsid','apPass'];
const numKeys = [
  'apChannel','apMaxClients','relayMinOnMs','relayMinOffMs','sensorInterval',
  'soilThreshold','soilHysteresis','soilDryRaw','soilWetRaw',
  'tempThreshold','tempHysteresis','humThreshold','humHysteresis'
];

let msgTimer = null;
let statusBusy = false;
let scanInFlight = false;

function showMsg(text, type) {
  const el = $('msg');
  el.textContent = text;
  el.className = 'msg ' + (type || 'info');
  clearTimeout(msgTimer);
  msgTimer = setTimeout(() => {
    el.textContent = '';
    el.className = 'msg';
  }, 6500);
}

function setConn(online) {
  const el = $('connPill');
  el.textContent = online ? 'Online' : 'Offline';
  el.className = 'pill ' + (online ? 'good' : 'bad');
}

function fmt(v, digits) {
  if (v === null || v === undefined) return '--';
  const n = Number(v);
  if (!Number.isFinite(n)) return '--';
  return n.toFixed(digits === undefined ? 1 : digits);
}

function formatDuration(totalSeconds) {
  totalSeconds = Math.max(0, Number(totalSeconds || 0));
  const h = Math.floor(totalSeconds / 3600);
  const m = Math.floor((totalSeconds % 3600) / 60);
  const s = Math.floor(totalSeconds % 60);
  return (h ? h + 'h ' : '') + (m ? m + 'm ' : '') + s + 's';
}

function setBadge(id, value, trueText, falseText, falseIsBad) {
  const el = $(id);
  if (!el) return;
  el.textContent = value ? trueText : falseText;
  el.className = value ? 'good' : (falseIsBad ? 'bad' : 'muted');
}

async function fetchJson(url, options, timeoutMs) {
  options = options || {};
  timeoutMs = timeoutMs || 15000;

  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeoutMs);

  try {
    const res = await fetch(url, {
      ...options,
      signal: controller.signal,
      cache: 'no-store'
    });

    const text = await res.text();
    let data = {};

    if (text) {
      try {
        data = JSON.parse(text);
      } catch (e) {
        data = { raw: text };
      }
    }

    if (!res.ok) {
      throw new Error(data.error || ('HTTP ' + res.status));
    }

    return data;
  } finally {
    clearTimeout(timer);
  }
}

async function loadSettings(silent) {
  try {
    const d = await fetchJson('/api/settings', {}, 9000);

    for (const k of boolKeys) {
      const el = $(k);
      if (el) el.checked = !!d[k];
    }

    for (const k of textKeys.concat(numKeys)) {
      const el = $(k);
      if (el) el.value = (d[k] === undefined || d[k] === null) ? '' : d[k];
    }

    $('staPass').value = '';
    $('staPass').placeholder = d.staPassSet ? '(unchanged — check below to clear)' : '';
    $('apPass').value = '';
    $('apPass').placeholder = d.apPassSet ? '(unchanged — check below to clear)' : '';
    if ($('clearStaPass')) $('clearStaPass').checked = false;
    if ($('clearApPass')) $('clearApPass').checked = false;

    if ($('staPass')) $('staPass').disabled = false;
    if ($('apPass')) $('apPass').disabled = false;

    $('combine').value = d.combineAny ? 'any' : 'all';
    $('tempDirection').value = d.tempTriggerAbove ? 'above' : 'below';
    $('humDirection').value = d.humTriggerAbove ? 'above' : 'below';

    if (!silent) {
      showMsg('Settings loaded from device.', 'ok');
    }
  } catch (e) {
    if (!silent) {
      showMsg('Could not load settings: ' + e.message, 'bad');
    }
  }
}

async function pollStatus() {
  if (statusBusy || scanInFlight) return;
  statusBusy = true;

  try {
    const d = await fetchJson('/api/status', {}, 6000);
    setConn(true);

    const relayOn = !!d.control.relayOn;
    $('relayState').textContent = relayOn ? 'ON' : 'OFF';
    $('relayState').className = 'big ' + (relayOn ? 'on' : 'off');

    $('relayReason').textContent = d.control.reason || '--';
    $('controlMode').textContent = (d.control.mode || '--').toUpperCase();
    $('relayAge').textContent = formatDuration(d.control.ageSec || 0);

    const modeMap = { auto: 'btnModeAuto', off: 'btnModeOff', on: 'btnModeOn' };
    for (const k in modeMap) {
      const b = $(modeMap[k]);
      if (!b) continue;
      const active = (d.control.mode === k);
      b.classList.toggle('primary', active);
    }

    $('soilPct').textContent = fmt(d.soil.percent, 1);
    $('soilRaw').textContent = fmt(d.soil.raw, 0);
    $('soilThresholdView').textContent = fmt(d.soil.threshold, 1);
    setBadge('soilValid', !!d.soil.valid, 'valid', 'invalid', true);
    setBadge('soilActive', !!d.soil.active, 'active', 'idle', false);

    $('tempC').textContent = fmt(d.sht31.tempC, 2);
    $('humPct').textContent = fmt(d.sht31.humidity, 2);
    setBadge('shtValid', !!d.sht31.valid, 'valid', 'invalid', true);
    setBadge('tempActive', !!d.auto.tempActive, 'active', 'idle', false);
    setBadge('humActive', !!d.auto.humActive, 'active', 'idle', false);

    $('staStatus').textContent = d.wifi.staConnected
      ? 'connected'
      : (d.wifi.staEnabled ? d.wifi.staStatus : 'disabled');

    $('staSsidView').textContent = d.wifi.staSsid || '--';
    $('staIp').textContent = d.wifi.localIp || '--';
    $('staRssi').textContent = d.wifi.staConnected ? ((d.wifi.rssi || 0) + ' dBm') : '--';

    $('apStatus').textContent = d.wifi.apActive ? 'active' : 'off';
    $('apSsidView').textContent = d.wifi.apSsid || '--';
    $('apIp').textContent = d.wifi.apIp || '--';
    $('apClients').textContent = d.wifi.apActive ? d.wifi.apClients : '--';

    $('portalStatus').textContent = d.wifi.captivePortal
      ? (d.wifi.dnsActive ? 'DNS running' : 'enabled')
      : 'disabled';

    $('uptime').textContent = formatDuration(d.uptime || 0);
    $('heap').textContent = d.freeHeap || 0;
  } catch (e) {
    setConn(false);
  } finally {
    statusBusy = false;
  }
}

async function setMode(mode) {
  try {
    const body = new URLSearchParams();
    body.append('mode', mode);

    await fetchJson('/api/control', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: body
    }, 8000);

    showMsg('Control mode set to ' + mode.toUpperCase() + '.', 'ok');
    await pollStatus();
  } catch (e) {
    showMsg('Control failed: ' + e.message, 'bad');
  }
}

async function calibrateSoil(type) {
  if (!confirm('Use the current raw soil reading as the ' + type.toUpperCase() + ' calibration point?')) {
    return;
  }

  try {
    const body = new URLSearchParams();
    body.append('type', type);

    const d = await fetchJson('/api/calibrate-soil', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: body
    }, 12000);

    showMsg('Calibration captured: ' + type + ' raw = ' + d.raw, 'ok');
    await loadSettings(true);
    await pollStatus();
  } catch (e) {
    showMsg('Calibration failed: ' + e.message, 'bad');
  }
}

async function forgetSta() {
  if (!confirm('Remove the stored station SSID/password and disable STA? AP will be enabled if needed. If you are currently connected via STA, this connection will drop — reconnect to the device AP afterwards.')) {
    return;
  }

  try {
    await fetchJson('/api/forget-sta', {
      method: 'POST'
    }, 10000);

    showMsg('STA credentials cleared.', 'ok');
    await loadSettings(true);
    await pollStatus();
  } catch (e) {
    showMsg('Forget STA failed: ' + e.message, 'bad');
  }
}

async function scanWifi() {
  const list = $('scanList');
  list.textContent = 'Scanning...';
  scanInFlight = true;

  try {
    const d = await fetchJson('/api/scan', {}, 25000);
    const nets = (d.networks || []).sort((a, b) => (b.rssi || -999) - (a.rssi || -999));

    list.innerHTML = '';

    if (nets.length === 0) {
      list.textContent = 'No networks found.';
      return;
    }

    for (const n of nets) {
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'scan-item';

      const ssid = document.createElement('span');
      ssid.textContent = n.ssid || '(hidden)';

      const rssi = document.createElement('span');
      rssi.textContent = (n.rssi || 0) + ' dBm';

      btn.appendChild(ssid);
      btn.appendChild(rssi);

      btn.addEventListener('click', () => {
        $('staSsid').value = n.ssid || '';
        $('staPass').value = '';
        $('staEnabled').checked = true;
        $('staPass').focus();
        showMsg('SSID filled. Enter password and press Save & Apply.', 'info');
      });

      list.appendChild(btn);
    }
  } catch (e) {
    list.textContent = 'Scan failed: ' + e.message;
  } finally {
    scanInFlight = false;
  }
}

async function rebootDevice() {
  if (!confirm('Reboot the ESP32 now?')) {
    return;
  }

  showMsg('Rebooting...', 'info');
  try {
    await fetchJson('/api/reboot', { method: 'POST' }, 6000);
  } catch (e) {

  }
  setTimeout(() => setConn(false), 800);
}

async function factoryResetDevice() {
  if (!confirm('Factory reset will clear settings and Wi-Fi credentials. Continue?')) {
    return;
  }

  const answer = prompt('Type RESET to confirm factory reset:');
  if (answer !== 'RESET') {
    showMsg('Factory reset cancelled.', 'info');
    return;
  }

  showMsg('Factory reset requested...', 'info');
  try {
    await fetchJson('/api/factory-reset', { method: 'POST' }, 6000);
  } catch (e) {

  }
  setTimeout(() => setConn(false), 800);
}

const clearStaEl = $('clearStaPass');
const clearApEl  = $('clearApPass');
const staPassEl  = $('staPass');
const apPassEl   = $('apPass');

if (staPassEl && clearStaEl) {
  staPassEl.addEventListener('input', () => {
    if (staPassEl.value.length > 0) clearStaEl.checked = false;
  });
}
if (apPassEl && clearApEl) {
  apPassEl.addEventListener('input', () => {
    if (apPassEl.value.length > 0) clearApEl.checked = false;
  });
}

$('settingsForm').addEventListener('submit', async (ev) => {
  ev.preventDefault();

  const fd = new FormData(ev.target);
  const body = new URLSearchParams();

  for (const [k, v] of fd.entries()) {
    body.append(k, v);
  }

  try {
    const d = await fetchJson('/api/settings', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: body
    }, 20000);

    let savedNote = '';
    if (d.hostname && d.hostname !== $('hostname').value.trim()) {
      savedNote = ' Hostname saved as "' + d.hostname + '".';
    }
    if (d.networkReconfig) {
      showMsg('Settings saved. Network is reconfiguring; reconnect if the connection drops.' + savedNote, 'ok');
    } else {
      showMsg('Settings saved and applied.' + savedNote, 'ok');
    }

    await pollStatus();
    setTimeout(() => { loadSettings(true).catch(() => {}); }, 1400);
  } catch (e) {
    showMsg('Save failed: ' + e.message, 'bad');
  }
});

loadSettings(false);
pollStatus();
setInterval(pollStatus, 2500);
</script>
</body>
</html>
)rawliteral";

// -------------------- HTTP handlers --------------------

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  bool staConn = (WiFi.status() == WL_CONNECTED);
  bool soilFresh = soilReadingFresh();
  bool shtFresh = shtReadingFresh();

  float soilOut = soilFresh ? soilPercent : NAN;
  float tempOut = shtFresh ? tempC : NAN;
  float humOut = shtFresh ? humidity : NAN;

  static String j;
  j.reserve(3400);
  j = "";

  j += "{";
  j += "\"ok\":true";
  j += ",\"version\":\"";
  j += jsonEscape(APP_VERSION);
  j += "\"";
  j += ",\"uptime\":";
  j += String(millis() / 1000UL);
  j += ",\"freeHeap\":";
  j += String(ESP.getFreeHeap());

  j += ",\"control\":{";
  j += "\"mode\":\"";
  j += modeName(runtimeMode);
  j += "\"";
  j += ",\"relayOn\":";
  j += boolJson(relayState);
  j += ",\"reason\":\"";
  j += jsonEscape(relayReason);
  j += "\"";
  j += ",\"ageSec\":";
  j += String((millis() - lastRelayChangeMs) / 1000UL);
  j += "}";

  j += ",\"auto\":{";
  j += "\"target\":";
  j += boolJson(lastEval.target);
  j += ",\"fault\":";
  j += boolJson(lastEval.fault);
  j += ",\"reason\":\"";
  j += jsonEscape(lastEval.reason);
  j += "\"";
  j += ",\"combine\":\"";
  j += lastEval.combineAny ? "ANY" : "ALL";
  j += "\"";
  j += ",\"enabledCount\":";
  j += String(lastEval.enabledCount);
  j += ",\"soilActive\":";
  j += boolJson(lastEval.soilActive);
  j += ",\"tempActive\":";
  j += boolJson(lastEval.tempActive);
  j += ",\"humActive\":";
  j += boolJson(lastEval.humActive);
  j += "}";

  j += ",\"soil\":{";
  j += "\"enabled\":";
  j += boolJson(settings.soilEnabled);
  j += ",\"valid\":";
  j += boolJson(soilFresh);
  j += ",\"raw\":";
  if (soilFresh) {
    j += jf(soilRawFiltered);
  } else {
    j += "null";
  }
  j += ",\"percent\":";
  j += jf(soilOut);
  j += ",\"threshold\":";
  j += jf(settings.soilThreshold);
  j += ",\"hysteresis\":";
  j += jf(settings.soilHysteresis);
  j += ",\"active\":";
  j += boolJson(lastEval.soilActive);
  j += "}";

  j += ",\"sht31\":{";
  j += "\"valid\":";
  j += boolJson(shtFresh);
  j += ",\"tempC\":";
  j += jf(tempOut);
  j += ",\"humidity\":";
  j += jf(humOut);
  j += ",\"tempEnabled\":";
  j += boolJson(settings.tempEnabled);
  j += ",\"humEnabled\":";
  j += boolJson(settings.humEnabled);
  j += "}";

  j += ",\"wifi\":{";
  j += "\"staEnabled\":";
  j += boolJson(settings.staEnabled);
  j += ",\"staConnected\":";
  j += boolJson(staConn);
  j += ",\"staStatus\":\"";
  j += wifiStatusName(WiFi.status());
  j += "\"";
  j += ",\"staSsid\":\"";
  j += jsonEscape(staConn ? WiFi.SSID() : settings.staSsid);
  j += "\"";
  j += ",\"localIp\":\"";
  j += jsonEscape(staConn ? WiFi.localIP().toString() : String(""));
  j += "\"";
  j += ",\"rssi\":";
  j += String(staConn ? WiFi.RSSI() : 0);
  j += ",\"mac\":\"";
  j += jsonEscape(WiFi.macAddress());
  j += "\"";
  j += ",\"hostname\":\"";
  j += jsonEscape(settings.hostname);
  j += "\"";
  j += ",\"apActive\":";
  j += boolJson(apActive);
  j += ",\"apSsid\":\"";
  j += jsonEscape(settings.apSsid);
  j += "\"";
  j += ",\"apIp\":\"";
  j += jsonEscape(apActive ? WiFi.softAPIP().toString() : String(""));
  j += "\"";
  j += ",\"apClients\":";
  j += String(apActive ? WiFi.softAPgetStationNum() : 0);
  j += ",\"captivePortal\":";
  j += boolJson(settings.captivePortal);
  j += ",\"dnsActive\":";
  j += boolJson(dnsActive);
  j += "}";

  j += "}";

  sendJson(200, j);
}

void handleGetSettings() {
  static String j;
  j.reserve(2600);
  j = "";

  j += "{";
  j += "\"ok\":true";

  j += ",\"hostname\":\"";
  j += jsonEscape(settings.hostname);
  j += "\"";

  j += ",\"staEnabled\":";
  j += boolJson(settings.staEnabled);
  j += ",\"staSsid\":\"";
  j += jsonEscape(settings.staSsid);
  j += "\"";
  j += ",\"staPass\":\"\"";
  j += ",\"staPassSet\":";
  j += boolJson(settings.staPass.length() > 0);

  j += ",\"apEnabled\":";
  j += boolJson(settings.apEnabled);
  j += ",\"apFallback\":";
  j += boolJson(settings.apFallback);
  j += ",\"captivePortal\":";
  j += boolJson(settings.captivePortal);
  j += ",\"apHidden\":";
  j += boolJson(settings.apHidden);
  j += ",\"apChannel\":";
  j += String(settings.apChannel);
  j += ",\"apSsid\":\"";
  j += jsonEscape(settings.apSsid);
  j += "\"";
  j += ",\"apPass\":\"\"";
  j += ",\"apPassSet\":";
  j += boolJson(settings.apPass.length() > 0);
  j += ",\"apMaxClients\":";
  j += String(settings.apMaxClients);

  j += ",\"relayActiveLow\":";
  j += boolJson(settings.relayActiveLow);
  j += ",\"failSafeOnSensorFault\":";
  j += boolJson(settings.failSafeOnSensorFault);
  j += ",\"relayMinOnMs\":";
  j += String(settings.relayMinOnMs);
  j += ",\"relayMinOffMs\":";
  j += String(settings.relayMinOffMs);
  j += ",\"sensorInterval\":";
  j += String(settings.sensorInterval);

  j += ",\"soilEnabled\":";
  j += boolJson(settings.soilEnabled);
  j += ",\"soilThreshold\":";
  j += jf(settings.soilThreshold);
  j += ",\"soilHysteresis\":";
  j += jf(settings.soilHysteresis);
  j += ",\"soilDryRaw\":";
  j += jf(settings.soilDryRaw);
  j += ",\"soilWetRaw\":";
  j += jf(settings.soilWetRaw);

  j += ",\"tempEnabled\":";
  j += boolJson(settings.tempEnabled);
  j += ",\"tempThreshold\":";
  j += jf(settings.tempThreshold);
  j += ",\"tempHysteresis\":";
  j += jf(settings.tempHysteresis);
  j += ",\"tempTriggerAbove\":";
  j += boolJson(settings.tempTriggerAbove);

  j += ",\"humEnabled\":";
  j += boolJson(settings.humEnabled);
  j += ",\"humThreshold\":";
  j += jf(settings.humThreshold);
  j += ",\"humHysteresis\":";
  j += jf(settings.humHysteresis);
  j += ",\"humTriggerAbove\":";
  j += boolJson(settings.humTriggerAbove);

  j += ",\"combineAny\":";
  j += boolJson(settings.combineAny);

  j += "}";

  sendJson(200, j);
}

void handlePostSettings() {
  Settings next = settings;

  String err;
  String tmp;
  int tmpInt = 0;
  uint32_t tmpULong = 0;

  if (!getStringArg("hostname", tmp, err)) {
    sendError(400, err);
    return;
  }
  next.hostname = sanitizeHostname(tmp);

  next.staEnabled = getBoolArg("staEnabled");

  if (!getStringArg("staSsid", next.staSsid, err)) {
    sendError(400, err);
    return;
  }

  if (server.hasArg("staPass")) {
    next.staPass = server.arg("staPass");
  } else {
    next.staPass = "";
  }
  if (getBoolArg("clearStaPass")) {
    next.staPass = "";
  } else if (next.staPass.length() == 0 && settings.staPass.length() > 0) {
    next.staPass = settings.staPass;
  }

  next.apEnabled = getBoolArg("apEnabled");
  next.apFallback = getBoolArg("apFallback");
  next.captivePortal = getBoolArg("captivePortal");
  next.apHidden = getBoolArg("apHidden");

  if (!readIntArg("apChannel", tmpInt, 1, 13, err)) {
    sendError(400, err);
    return;
  }
  next.apChannel = (uint8_t)tmpInt;

  if (!getStringArg("apSsid", next.apSsid, err)) {
    sendError(400, err);
    return;
  }

  if (server.hasArg("apPass")) {
    next.apPass = server.arg("apPass");
  } else {
    next.apPass = "";
  }
  if (getBoolArg("clearApPass")) {
    next.apPass = "";
  } else if (next.apPass.length() == 0 && settings.apPass.length() > 0) {
    next.apPass = settings.apPass;
  }

  if (!readIntArg("apMaxClients", tmpInt, 1, 4, err)) {
    sendError(400, err);
    return;
  }
  next.apMaxClients = (uint8_t)tmpInt;

  next.relayActiveLow = getBoolArg("relayActiveLow");
  next.failSafeOnSensorFault = getBoolArg("failSafeOnSensorFault");

  if (!readULongArg("relayMinOnMs", tmpULong, 0, 3600000UL, err)) {
    sendError(400, err);
    return;
  }
  next.relayMinOnMs = tmpULong;

  if (!readULongArg("relayMinOffMs", tmpULong, 0, 3600000UL, err)) {
    sendError(400, err);
    return;
  }
  next.relayMinOffMs = tmpULong;

  if (!readIntArg("sensorInterval", tmpInt, 1, 60, err)) {
    sendError(400, err);
    return;
  }
  next.sensorInterval = (uint16_t)tmpInt;

  next.soilEnabled = getBoolArg("soilEnabled");

  if (!readFloatArg("soilThreshold", next.soilThreshold, 0.0f, 100.0f, err)) {
    sendError(400, err);
    return;
  }

  if (!readFloatArg("soilHysteresis", next.soilHysteresis, 0.0f, 100.0f, err)) {
    sendError(400, err);
    return;
  }

  if (!readFloatArg("soilDryRaw", next.soilDryRaw, 0.0f, 4095.0f, err)) {
    sendError(400, err);
    return;
  }

  if (!readFloatArg("soilWetRaw", next.soilWetRaw, 0.0f, 4095.0f, err)) {
    sendError(400, err);
    return;
  }

  next.tempEnabled = getBoolArg("tempEnabled");

  if (!readFloatArg("tempThreshold", next.tempThreshold, -40.0f, 125.0f, err)) {
    sendError(400, err);
    return;
  }

  if (!readFloatArg("tempHysteresis", next.tempHysteresis, 0.0f, 125.0f, err)) {
    sendError(400, err);
    return;
  }

  if (!getStringArg("tempDirection", tmp, err)) {
    sendError(400, err);
    return;
  }

  tmp.toLowerCase();

  if (tmp == "above") {
    next.tempTriggerAbove = true;
  } else if (tmp == "below") {
    next.tempTriggerAbove = false;
  } else {
    sendError(400, "tempDirection must be above or below.");
    return;
  }

  next.humEnabled = getBoolArg("humEnabled");

  if (!readFloatArg("humThreshold", next.humThreshold, 0.0f, 100.0f, err)) {
    sendError(400, err);
    return;
  }

  if (!readFloatArg("humHysteresis", next.humHysteresis, 0.0f, 100.0f, err)) {
    sendError(400, err);
    return;
  }

  if (!getStringArg("humDirection", tmp, err)) {
    sendError(400, err);
    return;
  }

  tmp.toLowerCase();

  if (tmp == "above") {
    next.humTriggerAbove = true;
  } else if (tmp == "below") {
    next.humTriggerAbove = false;
  } else {
    sendError(400, "humDirection must be above or below.");
    return;
  }

  if (!getStringArg("combine", tmp, err)) {
    sendError(400, err);
    return;
  }

  tmp.toLowerCase();

  if (tmp == "any") {
    next.combineAny = true;
  } else if (tmp == "all") {
    next.combineAny = false;
  } else {
    sendError(400, "combine must be any or all.");
    return;
  }

  if (!validateSettings(next, err)) {
    sendError(400, err);
    return;
  }

  Settings old = settings;
  bool calibrationChanged =
    (old.soilDryRaw != next.soilDryRaw) ||
    (old.soilWetRaw != next.soilWetRaw);
  settings = next;

  saveSettings();
  resetConditionHysteresis();

  if (calibrationChanged) {
    soilRawFiltered = NAN;
  }

  writeRelayOutput(relayState);

  lastSensorReadMs = 0;
  readSensors(true);
  serviceRelay();

  bool netChanged = networkSettingsChanged(old, settings);
  bool hostnameOnly = netChanged &&
                      old.hostname != settings.hostname &&
                      old.staEnabled == settings.staEnabled &&
                      old.staSsid == settings.staSsid &&
                      old.staPass == settings.staPass &&
                      old.apEnabled == settings.apEnabled &&
                      old.apFallback == settings.apFallback &&
                      old.captivePortal == settings.captivePortal &&
                      old.apHidden == settings.apHidden &&
                      old.apChannel == settings.apChannel &&
                      old.apSsid == settings.apSsid &&
                      old.apPass == settings.apPass &&
                      old.apMaxClients == settings.apMaxClients;

  if (hostnameOnly) {
    stopMDNS();
    lastMdnsAttempt = 0;
    startMDNS();
  } else if (netChanged) {
    networkDirty = true;
    networkApplyAt = millis() + 800UL;
  }

  String resp = "{\"ok\":true,\"networkReconfig\":";
  resp += boolJson(netChanged);
  resp += ",\"hostname\":\"";
  resp += jsonEscape(settings.hostname);
  resp += "\"}";
  sendJson(200, resp);
}

void handleControl() {
  if (!server.hasArg("mode")) {
    sendError(400, "Missing mode.");
    return;
  }

  String mode = server.arg("mode");
  mode.toLowerCase();

  if (mode == "auto") {
    runtimeMode = MODE_AUTO;
  } else if (mode == "off") {
    runtimeMode = MODE_OFF;
  } else if (mode == "on") {
    runtimeMode = MODE_ON;
  } else {
    sendError(400, "mode must be auto, off, or on.");
    return;
  }

  serviceRelay();

  String resp = "{\"ok\":true,\"mode\":\"";
  resp += modeName(runtimeMode);
  resp += "\",\"relayOn\":";
  resp += boolJson(relayState);
  resp += "}";

  sendJson(200, resp);
}

void handleCalibrateSoil() {
  if (!server.hasArg("type")) {
    sendError(400, "Missing type.");
    return;
  }

  String type = server.arg("type");
  type.toLowerCase();

  if (type != "dry" && type != "wet") {
    sendError(400, "type must be dry or wet.");
    return;
  }

  int raw = readSoilRawAverage(24);

  if (raw < 0) {
    sendError(500, "Could not read soil ADC.");
    return;
  }

  float newDry = settings.soilDryRaw;
  float newWet = settings.soilWetRaw;

  if (type == "dry") {
    newDry = (float)raw;
  } else {
    newWet = (float)raw;
  }

  if (fabs(newDry - newWet) < 25.0f) {
    sendError(400, "Calibration points are too close. Move the sensor to a different moisture level first.");
    return;
  }

  settings.soilDryRaw = newDry;
  settings.soilWetRaw = newWet;

  saveSettings();
  resetConditionHysteresis();

  soilRawFiltered = NAN;
  lastSensorReadMs = 0;
  readSensors(true);
  serviceRelay();

  String resp = "{\"ok\":true,\"type\":\"";
  resp += jsonEscape(type);
  resp += "\",\"raw\":";
  resp += String(raw);
  resp += ",\"dryRaw\":";
  resp += jf(settings.soilDryRaw);
  resp += ",\"wetRaw\":";
  resp += jf(settings.soilWetRaw);
  resp += "}";

  sendJson(200, resp);
}

void handleConnect() {
  if (!server.hasArg("ssid")) {
    sendError(400, "Missing ssid.");
    return;
  }

  String ssid = server.arg("ssid");
  String pass = server.hasArg("pass") ? server.arg("pass") : "";

  if (ssid.length() > 32) {
    sendError(400, "SSID must be <=32 characters.");
    return;
  }

  Settings next = settings;
  next.staEnabled = true;
  next.staSsid = ssid;
  next.staPass = pass;

  String err;
  if (!validateSettings(next, err)) {
    sendError(400, err);
    return;
  }

  settings = next;
  saveSettings();

  networkDirty = true;
  networkApplyAt = millis() + 600UL;

  sendJson(200, "{\"ok\":true,\"message\":\"STA credentials saved; reconnecting.\"}");
}

void handleForgetSta() {
  Settings next = settings;
  next.staEnabled = false;
  next.staSsid = "";
  next.staPass = "";

  if (!next.apEnabled) {
    next.apEnabled = true;
  }

  String err;
  if (!validateSettings(next, err)) {
    sendError(400, err);
    return;
  }

  settings = next;
  saveSettings();

  networkDirty = true;
  networkApplyAt = millis() + 600UL;

  sendJson(200, "{\"ok\":true,\"message\":\"STA forgotten and AP kept available.\"}");
}

void handleScan() {
  int n = WiFi.scanNetworks(false, true);

  if (n < 0) {
    WiFi.scanDelete();
    sendError(500, "WiFi scan failed.");
    return;
  }

  String j = "{\"ok\":true,\"networks\":[";
  j.reserve(256 + n * 80);

  for (int i = 0; i < n; i++) {
    if (i > 0) {
      j += ",";
    }

    j += "{\"ssid\":\"";
    j += jsonEscape(WiFi.SSID(i));
    j += "\",\"rssi\":";
    j += String(WiFi.RSSI(i));
    j += ",\"encryption\":";
    j += String((int)WiFi.encryptionType(i));
    j += "}";
  }

  j += "]}";

  WiFi.scanDelete();
  sendJson(200, j);
}

void handleReboot() {
  sendJson(200, "{\"ok\":true,\"message\":\"rebooting\"}");
  delay(250);

  setRelay(false, "Reboot");
  writeRelayOutput(false);
  delay(80);

  ESP.restart();
}

void handleFactoryReset() {
  sendJson(200, "{\"ok\":true,\"message\":\"factory reset and reboot\"}");
  delay(300);

  setRelay(false, "Factory reset");
  writeRelayOutput(false);
  delay(80);

  doFactoryReset();
}

void handleNotFound() {
  String uri = server.uri();

  if (uri.startsWith("/api/") || uri == "/api") {
    sendError(404, "API endpoint not found.");
    return;
  }

  if (uri == "/favicon.ico") {
    server.send(204, "text/plain", "");
    return;
  }

  if (apActive && settings.captivePortal &&
      server.client().localIP() == WiFi.softAPIP()) {
    server.sendHeader("Cache-Control", "no-store");
    server.sendHeader("Location", apRedirectUrl());
    server.send(302, "text/plain", "");
    return;
  }

  server.send(404, "text/plain", "Not found");
}

// -------------------- Web server setup --------------------

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);

  server.on("/favicon.ico", HTTP_GET, []() {
    server.send(204, "text/plain", "");
  });

  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/settings", HTTP_GET, handleGetSettings);
  server.on("/api/settings", HTTP_POST, handlePostSettings);
  server.on("/api/control", HTTP_POST, handleControl);
  server.on("/api/calibrate-soil", HTTP_POST, handleCalibrateSoil);
  server.on("/api/connect", HTTP_POST, handleConnect);
  server.on("/api/forget-sta", HTTP_POST, handleForgetSta);
  server.on("/api/scan", HTTP_GET, handleScan);
  server.on("/api/reboot", HTTP_POST, handleReboot);
  server.on("/api/restart", HTTP_POST, handleReboot);
  server.on("/api/factory-reset", HTTP_POST, handleFactoryReset);

  server.onNotFound(handleNotFound);
  server.begin();
}

// -------------------- Arduino setup / loop --------------------

void setup() {
  delay(120);

  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

  pinMode(RELAY_PIN, INPUT_PULLDOWN);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  loadSettings();

  digitalWrite(RELAY_PIN, settings.relayActiveLow ? HIGH : LOW);
  pinMode(RELAY_PIN, OUTPUT);

  relayState = false;
  lastRelayChangeMs = millis();
  writeRelayOutput(false);
  relayReason = "Boot - auto mode";
  runtimeMode = MODE_AUTO;

  resetConditionHysteresis();

  beginSHT31();

  WiFi.persistent(false);
  WiFi.setSleep(false);

  reconfigureNetwork();
  setupWebServer();

  readSensors(true);
  serviceRelay();
}

void loop() {
  handleBootButton();

  readSensors(false);
  serviceRelay();

  if (networkDirty && (int32_t)(millis() - networkApplyAt) >= 0) {
    reconfigureNetwork();
  }

  serviceNetwork();

  server.handleClient();

  if (dnsActive) {
    dnsServer.processNextRequest();
  }

  delay(2);
}
