/*
=================================================
ESP32 Unified Weather Controlled Relay

* Author: Raff Alds
* Github: github.com/xiv3r
* License: GPLv3
=================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <time.h>

// ---------------- 64-bit monotonic uptime -------------------------------
static uint64_t g_uptimeMs64 = 0;
static uint32_t g_uptimeLast = 0;
static inline void uptimeTick() {
  uint32_t now = millis();
  g_uptimeMs64 += (uint32_t)(now - g_uptimeLast);
  g_uptimeLast  = now;
}
static inline uint64_t uptimeMs() { return g_uptimeMs64; }

// ---------------- Pins ----------------
#define RELAY_PIN        25
#define BOOT_BUTTON_PIN  0

// ---------------- Constants ----------------
const uint16_t DNS_PORT              = 53;
const uint32_t FACTORY_RESET_MS      = 5000;
const uint32_t WEATHER_TIMEOUT_MS    = 8000;
const uint32_t WEATHER_POLL_MS       = 60000UL;
const uint32_t WEATHER_STALE_MS      = 30UL * 60UL * 1000UL;
const float    HYST_MIN_TEMP         = 0.1f;
const float    HYST_MIN_RAIN         = 0.05f;
const float    HYST_MIN_WIND         = 1.0f;
const uint32_t STA_CONNECT_TIMEOUT   = 15000;
const uint32_t STA_BACKOFF_MIN_MS    = 30000UL;
const uint32_t STA_BACKOFF_MAX_MS    = 600000UL;
const uint32_t AP_RETRY_INTERVAL_MS  = 5000UL;
const uint32_t WEATHER_FAIL_BACKOFF_MIN_MS = 60000UL;
const uint32_t WEATHER_FAIL_BACKOFF_MAX_MS = 900000UL;
const uint8_t  MAX_LOCATIONS         = 8;

// ---------------- Location ----------------
struct Location {
  String name;
  float  lat;
  float  lon;
  bool   enabled;
  Location() : name(""), lat(0), lon(0), enabled(true) {}
};

// ---------------- Settings ----------------
struct Settings {
  String ssid           = "";
  String password       = "";
  String hostname       = "esp32-weather";
  bool   apEnabled      = true;
  String apSsid         = "";
  String apPassword     = "";
  bool   relayEnabled   = true;
  bool   relayActiveLow = true;
  bool   useTemp        = true;
  bool   useFeels       = false;
  bool   useRainMm      = false;
  bool   useRainCode    = false;
  bool   useSnowCode    = false;
  bool   useStormCode   = false;
  bool   useFogCode     = false;
  bool   useWind        = false;
  bool   useGusts       = false;
  bool   useHumidity    = false;
  bool   logicAll       = false;
  bool   preferOn       = true;
  bool   multiLogicAll  = false;   
  float  tempOn         = 30.0f;
  float  tempOff        = 28.0f;
  float  feelsOn        = 30.0f;
  float  feelsOff       = 28.0f;
  float  rainOn         = 0.5f;
  float  rainOff        = 0.2f;
  float  windOn         = 40.0f;
  float  windOff        = 30.0f;
  float  gustOn         = 60.0f;
  float  gustOff        = 45.0f;
  float  humOn          = 90.0f;
  float  humOff         = 80.0f;
  int8_t forceState     = 0;
} cfg;
Preferences prefs;
WebServer   server(80);
DNSServer   dns;

// ---------------- Locations ----------------
Location locations[MAX_LOCATIONS];
uint8_t  locationCount = 0;

// ---------------- Runtime state ----------------
bool     apMode          = false;
String   apSsidRuntime   = "";
bool     staConnected    = false;
unsigned long lastStaRetry      = 0;
unsigned long staBackoffMs      = STA_BACKOFF_MIN_MS;
int           staRetries        = 0;
unsigned long lastApAttemptMs   = 0;
unsigned long apRetryBackoffMs  = AP_RETRY_INTERVAL_MS;
unsigned long weatherBackoffMs  = WEATHER_POLL_MS;
unsigned long weatherFailCount  = 0;
bool          bootArmed         = false;
bool          weatherRefreshRequested = false;
bool          staManualDisable        = false;
unsigned long lastWeatherPoll         = 0;
unsigned long lastSaveAttemptMs       = 0;
bool          saveDirty               = false;
unsigned long lastEvalMs              = 0;

// ---------------- Weather (per location) ----------------
struct WeatherState {
  bool     valid      = false;
  float    tempC      = NAN;
  float    feelsC     = NAN;
  float    windKmh    = NAN;
  float    gustKmh    = NAN;
  float    precip     = 0.0f;
  int      humidity   = -1;
  int      wcode      = -1;
  String   desc       = "-";
  String   icon       = "-";
  uint64_t lastOkMs64 = 0;
  String   lastErr    = "";
} wx[MAX_LOCATIONS];
bool relayIsOn = false;
bool          bootDown = false;
unsigned long bootStart = 0;

// =====================================================================
//  WMO CODE HELPERS
// =====================================================================
bool isRainCode(int c)  { return (c >= 51 && c <= 67) || (c >= 80 && c <= 82); }
bool isSnowCode(int c)  { return (c >= 71 && c <= 77) || (c >= 85 && c <= 86); }
bool isStormCode(int c) { return c >= 95 && c <= 99; }
bool isFogCode(int c)   { return c == 45 || c == 48; }
static inline bool wxMissing(float v)   { return isnan(v); }
static inline bool wxMissingInt(int v)  { return v < 0; }

// =====================================================================
//  HTML ESCAPING
// =====================================================================
String htmlEscape(const String &in) {
  String out;
  out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    switch (c) {
      case '&':  out += F("&amp;");  break;
      case '<':  out += F("&lt;");   break;
      case '>':  out += F("&gt;");   break;
      case '"':  out += F("&quot;"); break;
      case '\'': out += F("&#39;");  break;
      default:   out += c;           break;
    }
  }
  return out;
}

// =====================================================================
//  LOCATION HELPERS
// =====================================================================
void addDefaultLocationIfEmpty() {
  if (locationCount == 0) {
    locations[0].name    = "Manila";
    locations[0].lat     = 14.5995f;
    locations[0].lon     = 120.9842f;
    locations[0].enabled = true;
    locationCount        = 1;
  }
}
int countEnabledLocations() {
  int n = 0;
  for (uint8_t i = 0; i < locationCount; i++) if (locations[i].enabled) n++;
  return n;
}

// =====================================================================
//  SETTINGS
// =====================================================================
void sanitizeSettings() {
  if (cfg.tempOn  >  60) cfg.tempOn  =  60;
  if (cfg.tempOn  < -40) cfg.tempOn  = -40;
  if (cfg.feelsOn >  70) cfg.feelsOn =  70;
  if (cfg.feelsOn < -40) cfg.feelsOn = -40;
  if (cfg.rainOn  > 100)     cfg.rainOn = 100;
  if (cfg.rainOn  < HYST_MIN_RAIN * 2) cfg.rainOn = HYST_MIN_RAIN * 2;
  if (cfg.windOn  > 300)     cfg.windOn = 300;
  if (cfg.windOn  < HYST_MIN_WIND * 2) cfg.windOn = HYST_MIN_WIND * 2;
  if (cfg.gustOn  > 400)     cfg.gustOn = 400;
  if (cfg.gustOn  < HYST_MIN_WIND * 2) cfg.gustOn = HYST_MIN_WIND * 2;
  if (cfg.humOn   > 100)     cfg.humOn = 100;
  if (cfg.humOn   < 2)       cfg.humOn = 2;
  auto fixPair = [](float &on, float &off, float lo, float hi,
                    float gap, float step) {
    if (step <= 0) step = gap;
    if (on < lo) on = lo;
    if (on > hi) on = hi;
    if (off >= on) off = on - step;
    if (off < lo) off = lo;
    if (off >= on) on = off + step;
    if (on > hi)  on = hi;
    if (off >= on) off = on - gap;
  };
  fixPair(cfg.tempOn,  cfg.tempOff,  -40, 60,  HYST_MIN_TEMP, 0.1f);
  fixPair(cfg.feelsOn, cfg.feelsOff, -40, 70,  HYST_MIN_TEMP, 0.1f);
  fixPair(cfg.rainOn,  cfg.rainOff,    0, 100, HYST_MIN_RAIN, 0.05f);
  fixPair(cfg.windOn,  cfg.windOff,    0, 300, HYST_MIN_WIND, 1.0f);
  fixPair(cfg.gustOn,  cfg.gustOff,    0, 400, HYST_MIN_WIND, 1.0f);
  fixPair(cfg.humOn,   cfg.humOff,     0, 100, 1.0f,         1.0f);
  if (cfg.hostname.length() == 0) cfg.hostname = "esp32-weather";
  {
    String h;
    h.reserve(cfg.hostname.length());
    for (size_t k = 0; k < cfg.hostname.length(); k++) {
      char ch = cfg.hostname[k];
      if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
          (ch >= '0' && ch <= '9') || ch == '-' || ch == '.') h += ch;
    }
    if (h.length() == 0) h = "esp32-weather";
    if (h.length() > 32) h = h.substring(0, 32);
    cfg.hostname = h;
  }
  if (cfg.apPassword.length() > 0 && cfg.apPassword.length() < 8) cfg.apPassword = "";
  if (cfg.apPassword.length() > 63) cfg.apPassword = cfg.apPassword.substring(0, 63);
  if (cfg.apSsid.length() > 32) cfg.apSsid = cfg.apSsid.substring(0, 32);
  if (cfg.ssid.length() > 32) cfg.ssid = cfg.ssid.substring(0, 32);
  if (cfg.password.length() > 63) cfg.password = cfg.password.substring(0, 63);
  if (locationCount > MAX_LOCATIONS) locationCount = MAX_LOCATIONS;
  for (uint8_t i = 0; i < locationCount; i++) {
    if (locations[i].name.length() > 32) locations[i].name = locations[i].name.substring(0, 32);
    if (locations[i].name.length() == 0) locations[i].name = "Location " + String(i + 1);
    if (locations[i].lat < -90)  locations[i].lat = -90;
    if (locations[i].lat >  90)  locations[i].lat =  90;
    if (locations[i].lon < -180) locations[i].lon = -180;
    if (locations[i].lon >  180) locations[i].lon =  180;
  }
  if (locationCount == 0) addDefaultLocationIfEmpty();
}

void loadLocationsLegacy() {
  bool hasLocs = prefs.isKey("locs");
  if (hasLocs) {
    String blob = prefs.getString("locs", "[]");
    DynamicJsonDocument doc(4096);
    if (deserializeJson(doc, blob) == DeserializationError::Ok && doc.is<JsonArray>()) {
      JsonArray arr = doc.as<JsonArray>();
      locationCount = 0;
      for (JsonObject o : arr) {
        if (locationCount >= MAX_LOCATIONS) break;
        locations[locationCount].name    = (const char*)(o["name"] | "Location");
        locations[locationCount].lat     = o["lat"] | 0.0f;
        locations[locationCount].lon     = o["lon"] | 0.0f;
        locations[locationCount].enabled = o["en"]  | true;
        locationCount++;
      }
    }
    if (locationCount == 0) addDefaultLocationIfEmpty();
    return;
  }
  float lat = prefs.getFloat("lat", 14.5995f);
  float lon = prefs.getFloat("lon", 120.9842f);
  String nm = prefs.getString("loc", "Manila");
  locations[0].name    = nm;
  locations[0].lat     = lat;
  locations[0].lon     = lon;
  locations[0].enabled = true;
  locationCount        = 1;
}

void saveLocations() {
  DynamicJsonDocument doc(4096);
  JsonArray arr = doc.to<JsonArray>();
  for (uint8_t i = 0; i < locationCount; i++) {
    JsonObject o = arr.createNestedObject();
    o["name"] = locations[i].name;
    o["lat"]  = locations[i].lat;
    o["lon"]  = locations[i].lon;
    o["en"]   = locations[i].enabled;
  }
  String blob;
  serializeJson(doc, blob);
  prefs.putString("locs", blob);
}

void loadSettings() {
  prefs.begin("app", true);
  cfg.ssid           = prefs.getString("ssid", cfg.ssid);
  cfg.password       = prefs.getString("pass", cfg.password);
  cfg.hostname       = prefs.getString("host", cfg.hostname);
  cfg.apEnabled      = prefs.getBool  ("apEn", cfg.apEnabled);
  cfg.apSsid         = prefs.getString("apSs", cfg.apSsid);
  cfg.apPassword     = prefs.getString("apPw", cfg.apPassword);
  cfg.relayEnabled   = prefs.getBool  ("rEn",  cfg.relayEnabled);
  cfg.relayActiveLow = prefs.getBool  ("rLow", cfg.relayActiveLow);
  cfg.useTemp        = prefs.getBool  ("uT",   cfg.useTemp);
  cfg.useFeels       = prefs.getBool  ("uF",   cfg.useFeels);
  cfg.useRainMm      = prefs.getBool  ("uRm",  cfg.useRainMm);
  cfg.useRainCode    = prefs.getBool  ("uRc",  cfg.useRainCode);
  cfg.useSnowCode    = prefs.getBool  ("uSc",  cfg.useSnowCode);
  cfg.useStormCode   = prefs.getBool  ("uSt",  cfg.useStormCode);
  cfg.useFogCode     = prefs.getBool  ("uFg",  cfg.useFogCode);
  cfg.useWind        = prefs.getBool  ("uW",   cfg.useWind);
  cfg.useGusts       = prefs.getBool  ("uG",   cfg.useGusts);
  cfg.useHumidity    = prefs.getBool  ("uH",   cfg.useHumidity);
  cfg.logicAll       = prefs.getBool  ("lAll", cfg.logicAll);
  cfg.preferOn       = prefs.getBool  ("pOn",  cfg.preferOn);
  cfg.multiLogicAll  = prefs.getBool  ("mLAll", cfg.multiLogicAll);
  cfg.tempOn         = prefs.getFloat ("tOn",  cfg.tempOn);
  cfg.tempOff        = prefs.getFloat ("tOff", cfg.tempOff);
  cfg.feelsOn        = prefs.getFloat ("fOn",  cfg.feelsOn);
  cfg.feelsOff       = prefs.getFloat ("fOff", cfg.feelsOff);
  cfg.rainOn         = prefs.getFloat ("rOn",  cfg.rainOn);
  cfg.rainOff        = prefs.getFloat ("rOff", cfg.rainOff);
  cfg.windOn         = prefs.getFloat ("wOn",  cfg.windOn);
  cfg.windOff        = prefs.getFloat ("wOff", cfg.windOff);
  cfg.gustOn         = prefs.getFloat ("gOn",  cfg.gustOn);
  cfg.gustOff        = prefs.getFloat ("gOff", cfg.gustOff);
  cfg.humOn          = prefs.getFloat ("hOn",  cfg.humOn);
  cfg.humOff         = prefs.getFloat ("hOff", cfg.humOff);
  cfg.forceState     = (int8_t)prefs.getChar("frc", cfg.forceState);
  loadLocationsLegacy();
  prefs.end();
  sanitizeSettings();
}

void flushSettingsToNVS() {
  sanitizeSettings();
  prefs.begin("app", false);
  prefs.putString("ssid", cfg.ssid);
  prefs.putString("pass", cfg.password);
  prefs.putString("host", cfg.hostname);
  prefs.putBool  ("apEn", cfg.apEnabled);
  prefs.putString("apSs", cfg.apSsid);
  prefs.putString("apPw", cfg.apPassword);
  prefs.putBool  ("rEn",  cfg.relayEnabled);
  prefs.putBool  ("rLow", cfg.relayActiveLow);
  prefs.putBool  ("uT",   cfg.useTemp);
  prefs.putBool  ("uF",   cfg.useFeels);
  prefs.putBool  ("uRm",  cfg.useRainMm);
  prefs.putBool  ("uRc",  cfg.useRainCode);
  prefs.putBool  ("uSc",  cfg.useSnowCode);
  prefs.putBool  ("uSt",  cfg.useStormCode);
  prefs.putBool  ("uFg",  cfg.useFogCode);
  prefs.putBool  ("uW",   cfg.useWind);
  prefs.putBool  ("uG",   cfg.useGusts);
  prefs.putBool  ("uH",   cfg.useHumidity);
  prefs.putBool  ("lAll", cfg.logicAll);
  prefs.putBool  ("pOn",  cfg.preferOn);
  prefs.putBool  ("mLAll", cfg.multiLogicAll);
  prefs.putFloat ("tOn",  cfg.tempOn);
  prefs.putFloat ("tOff", cfg.tempOff);
  prefs.putFloat ("fOn",  cfg.feelsOn);
  prefs.putFloat ("fOff", cfg.feelsOff);
  prefs.putFloat ("rOn",  cfg.rainOn);
  prefs.putFloat ("rOff", cfg.rainOff);
  prefs.putFloat ("wOn",  cfg.windOn);
  prefs.putFloat ("wOff", cfg.windOff);
  prefs.putFloat ("gOn",  cfg.gustOn);
  prefs.putFloat ("gOff", cfg.gustOff);
  prefs.putFloat ("hOn",  cfg.humOn);
  prefs.putFloat ("hOff", cfg.humOff);
  prefs.putChar  ("frc",  (char)cfg.forceState);
  saveLocations();
  prefs.end();
  lastSaveAttemptMs = millis();
  saveDirty         = false;
}

void saveSettings() {
  saveDirty = true;
  if (millis() - lastSaveAttemptMs < 5000UL) return;
  flushSettingsToNVS();
}

void factoryReset() {
  saveDirty = false;
  prefs.begin("app", false);
  prefs.clear();
  prefs.end();
  delay(400);
  ESP.restart();
}

// =====================================================================
//  RELAY
// =====================================================================
void applyRelayHardware(bool on) {
  if (cfg.relayActiveLow) digitalWrite(RELAY_PIN, on ? LOW : HIGH);
  else                    digitalWrite(RELAY_PIN, on ? HIGH : LOW);
  relayIsOn = on;
}

void initRelayPin() {
  if (cfg.relayActiveLow) digitalWrite(RELAY_PIN, HIGH);
  else                    digitalWrite(RELAY_PIN, LOW);
  pinMode(RELAY_PIN, OUTPUT);
  relayIsOn = false;
}
int8_t evaluateLocation(int idx) {
  if (idx < 0 || idx >= locationCount) return 0;
  if (!locations[idx].enabled) return 0;
  WeatherState &w = wx[idx];
  if (!w.valid) return 0;
  if (uptimeMs() - w.lastOkMs64 > (uint64_t)WEATHER_STALE_MS) return 0;
  int total = 0, onHits = 0, offHits = 0;
  auto num = [&](bool en, float v, float onThr, float offThr) {
    if (!en) return;
    if (wxMissing(v)) return;
    total++;
    if (v >= onThr)  onHits++;
    if (v <= offThr) offHits++;
  };
  auto numi = [&](bool en, int v, float onThr, float offThr) {
    if (!en) return;
    if (wxMissingInt(v)) return;
    total++;
    if ((float)v >= onThr) onHits++;
    if ((float)v <= offThr) offHits++;
  };
  auto cat = [&](bool en, int code, bool (*test)(int)) {
    if (!en) return;
    if (wxMissingInt(code)) return;
    total++;
    if (test(code)) onHits++; else offHits++;
  };
  num (cfg.useTemp,     w.tempC,   cfg.tempOn,  cfg.tempOff);
  num (cfg.useFeels,    w.feelsC,  cfg.feelsOn, cfg.feelsOff);
  num (cfg.useRainMm,   w.precip,  cfg.rainOn,  cfg.rainOff);
  num (cfg.useWind,     w.windKmh, cfg.windOn,  cfg.windOff);
  num (cfg.useGusts,    w.gustKmh, cfg.gustOn,  cfg.gustOff);
  numi(cfg.useHumidity, w.humidity, cfg.humOn,  cfg.humOff);
  cat (cfg.useRainCode,  w.wcode, isRainCode);
  cat (cfg.useSnowCode,  w.wcode, isSnowCode);
  cat (cfg.useStormCode, w.wcode, isStormCode);
  cat (cfg.useFogCode,   w.wcode, isFogCode);
  if (total == 0) return 0;
  if (cfg.logicAll) {
    if (onHits  == total) return  1;
    if (offHits == total) return -1;
    return 0;
  } else {
    bool onFires  = onHits  > 0;
    bool offFires = offHits > 0;
    if (onFires && offFires)  return cfg.preferOn ? 1 : -1;
    if (onFires)              return  1;
    if (offFires)             return -1;
    return 0;
  }
}

void evaluateRelay() {
  if (cfg.forceState == 1)  { if (!relayIsOn) applyRelayHardware(true);  return; }
  if (cfg.forceState == -1) { if ( relayIsOn) applyRelayHardware(false); return; }
  if (!cfg.relayEnabled) {
    if (relayIsOn) applyRelayHardware(false);
    return;
  }
  bool anyTriggers = cfg.useTemp || cfg.useFeels || cfg.useRainMm ||
                     cfg.useWind || cfg.useGusts || cfg.useHumidity ||
                     cfg.useRainCode || cfg.useSnowCode ||
                     cfg.useStormCode || cfg.useFogCode;
  if (!anyTriggers) {
    if (relayIsOn) applyRelayHardware(false);
    return;
  }
  int enabledCount = countEnabledLocations();
  if (enabledCount == 0) {
    if (relayIsOn) applyRelayHardware(false);
    return;
  }
  int onLocations = 0, offLocations = 0, decidedLocations = 0;
  int freshEnabledCount = 0;
  bool anyFresh = false;
  for (uint8_t i = 0; i < locationCount; i++) {
    if (!locations[i].enabled) continue;
    bool fresh = wx[i].valid &&
                 (uptimeMs() - wx[i].lastOkMs64 <= (uint64_t)WEATHER_STALE_MS);
    if (fresh) { anyFresh = true; freshEnabledCount++; }
    int8_t r = evaluateLocation(i);
    if (r ==  1) { onLocations++;  decidedLocations++; }
    if (r == -1) { offLocations++; decidedLocations++; }
  }
  if (!anyFresh) {
    if (relayIsOn) applyRelayHardware(false);
    return;
  }
  if (decidedLocations == 0) return;
  bool wantOn = relayIsOn;
  if (cfg.multiLogicAll) {
    if (onLocations  == freshEnabledCount) wantOn = true;
    else if (offLocations == freshEnabledCount) wantOn = false;
  } else {
    bool anyOn  = onLocations  > 0;
    bool anyOff = offLocations > 0;
    if (anyOn && anyOff)  wantOn = cfg.preferOn;
    else if (anyOn)       wantOn = true;
    else if (anyOff)      wantOn = false;
  }
  if (wantOn != relayIsOn) applyRelayHardware(wantOn);
}

// =====================================================================
//  WEATHER
// =====================================================================
String wmoDesc(int code) {
  switch (code) {
    case 0:  return "Clear";
    case 1:  return "Mainly clear";
    case 2:  return "Partly cloudy";
    case 3:  return "Overcast";
    case 45: return "Fog";
    case 48: return "Rime fog";
    case 51: return "Light drizzle";
    case 53: return "Moderate drizzle";
    case 55: return "Dense drizzle";
    case 56: return "Light freezing drizzle";
    case 57: return "Dense freezing drizzle";
    case 61: return "Slight rain";
    case 63: return "Moderate rain";
    case 65: return "Heavy rain";
    case 66: return "Light freezing rain";
    case 67: return "Heavy freezing rain";
    case 71: return "Slight snow";
    case 73: return "Moderate snow";
    case 75: return "Heavy snow";
    case 77: return "Snow grains";
    case 80: return "Slight showers";
    case 81: return "Moderate showers";
    case 82: return "Violent showers";
    case 85: return "Slight snow showers";
    case 86: return "Heavy snow showers";
    case 95: return "Thunderstorm";
    case 96: return "Thunderstorm w/ hail";
    case 99: return "Thunderstorm w/ hail";
    default: return "Unknown";
  }
}
String wmoIcon(int code) {
  if (code < 0 || code > 99) return "❓";
  if (code == 0) return "☀️";
  if (code <= 3) return "⛅";
  if (code <= 48) return "🌫️";
  if (code <= 67) return "🌧️";
  if (code <= 77) return "❄️";
  if (code <= 82) return "🌦️";
  if (code <= 86) return "🌨️";
  return "⛈️";
}
bool fetchWeather() {
  if (WiFi.status() != WL_CONNECTED) {
    for (uint8_t i = 0; i < locationCount; i++) wx[i].lastErr = "No WiFi";
    return false;
  }
  if (locationCount == 0) return false;
  String latList, lonList;
  bool firstLat = true;
  for (uint8_t i = 0; i < locationCount; i++) {
    if (!locations[i].enabled) continue;
    if (!firstLat) { latList += ","; lonList += ","; }
    latList += String(locations[i].lat, 4);
    lonList += String(locations[i].lon, 4);
    firstLat = false;
  }
  if (firstLat) {
    for (uint8_t i = 0; i < locationCount; i++)
      wx[i].lastErr = "No enabled locations";
    return false;
  }
  String url = "http://api.open-meteo.com/v1/forecast?latitude=";
  url += latList;
  url += "&longitude=";
  url += lonList;
  url += "&current=temperature_2m,relative_humidity_2m,apparent_temperature,"
         "weather_code,wind_speed_10m,wind_gusts_10m,precipitation&timezone=auto";
  HTTPClient http;
  http.setConnectTimeout(WEATHER_TIMEOUT_MS);
  http.setTimeout(WEATHER_TIMEOUT_MS);
  bool begun = http.begin(url);
  if (!begun) {
    for (uint8_t i = 0; i < locationCount; i++) wx[i].lastErr = "begin fail";
    http.end();
    return false;
  }
  int code = http.GET();
  if (code != 200) {
    String e = (code > 0) ? ("HTTP " + String(code)) : ("net " + String(code));
    for (uint8_t i = 0; i < locationCount; i++) wx[i].lastErr = e;
    http.end();
    return false;
  }
  String payload = http.getString();
  http.end();
  DynamicJsonDocument doc(24576);
  auto err = deserializeJson(doc, payload);
  if (err) {
    String e = String("JSON:") + err.c_str();
    for (uint8_t i = 0; i < locationCount; i++) wx[i].lastErr = e;
    return false;
  }
  bool anyOk = false;
  bool multiResult = doc.is<JsonArray>();
  for (uint8_t i = 0; i < locationCount; i++) {
    if (!locations[i].enabled) continue;
    JsonObject cur;
    if (multiResult) {
      JsonArray arr = doc.as<JsonArray>();
      int enabledIdx = 0;
      for (uint8_t j = 0; j < i; j++) if (locations[j].enabled) enabledIdx++;
      if (enabledIdx < (int)arr.size()) {
        cur = arr[enabledIdx]["current"];
      }
    } else {
      cur = doc["current"];
    }
    if (cur.isNull()) { wx[i].lastErr = "no current"; continue; }
    float t = cur["temperature_2m"] | NAN;
    if (isnan(t)) { wx[i].lastErr = "bad temp"; continue; }
    wx[i].tempC    = t;
    wx[i].feelsC   = cur["apparent_temperature"] | NAN;
    wx[i].humidity = cur["relative_humidity_2m"] | -1;
    wx[i].wcode    = cur["weather_code"]         | -1;
    wx[i].windKmh  = cur["wind_speed_10m"]       | NAN;
    wx[i].gustKmh  = cur["wind_gusts_10m"]       | NAN;
    wx[i].precip   = cur["precipitation"]        | 0.0f;
    if (isnan(wx[i].feelsC))  wx[i].feelsC  = t;
    if (isnan(wx[i].windKmh)) wx[i].windKmh = 0.0f;
    if (isnan(wx[i].gustKmh)) wx[i].gustKmh = 0.0f;
    wx[i].desc     = wmoDesc(wx[i].wcode);
    wx[i].icon     = wmoIcon(wx[i].wcode);
    wx[i].valid     = true;
    wx[i].lastOkMs64 = uptimeMs();
    wx[i].lastErr   = "";
    anyOk = true;
  }
  return anyOk;
}

void resetWeatherBackoff() {
  weatherFailCount = 0;
  weatherBackoffMs = WEATHER_POLL_MS;
}

// =====================================================================
//  HTML
// =====================================================================
String htmlHeader(const String &title) {
  String s = F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
               "<meta name='viewport' content='width=device-width,initial-scale=1'>");
  s += "<title>" + htmlEscape(title) + "</title><style>"
       "body{font-family:system-ui,Arial;margin:0;background:#0f172a;color:#e2e8f0}"
       ".wrap{max-width:820px;margin:0 auto;padding:16px}"
       "h1{font-size:1.4rem;color:#38bdf8;margin:.4em 0}"
       "h2{font-size:1.05rem;color:#7dd3fc;margin:.2em 0 .6em}"
       ".card{background:#1e293b;border-radius:12px;padding:16px;margin:12px 0;"
       "box-shadow:0 4px 14px rgba(0,0,0,.3)}"
       ".row{display:flex;justify-content:space-between;padding:6px 0;border-bottom:1px solid #334155}"
       ".row:last-child{border-bottom:none}.val{font-weight:600;color:#f1f5f9}"
       ".on{color:#22c55e}.off{color:#ef4444}"
       "label{display:block;margin:10px 0 4px;font-size:.9rem;color:#cbd5e1}"
       "input,select{width:100%;box-sizing:border-box;padding:9px;border-radius:8px;"
       "border:1px solid #334155;background:#0f172a;color:#e2e8f0;font-size:1rem}"
       "input[type=checkbox]{width:auto;margin-right:6px;vertical-align:middle}"
       ".grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}"
       "button{margin-top:12px;width:100%;padding:11px;border:none;border-radius:8px;"
       "background:#0ea5e9;color:#fff;font-size:1rem;font-weight:600;cursor:pointer}"
       "button.alt{background:#dc2626}button.gray{background:#334155}button.grn{background:#16a34a}"
       ".msg{background:#134e4a;padding:10px;border-radius:8px;margin:10px 0;color:#a7f3d0}"
       ".err{background:#7f1d1d;color:#fecaca}"
       ".loc{background:#0f172a;border:1px solid #334155;border-radius:8px;padding:10px;margin:8px 0}"
       "a{color:#38bdf8;text-decoration:none}"
       "</style></head><body><div class='wrap'>";
  return s;
}
String htmlFooter() { return F("</div></body></html>"); }
String cRow(const String &name, bool checked, const String &label) {
  return String("<label><input type='checkbox' name='") + name + "' value='1'"
       + (checked ? " checked" : "") + ">" + label + "</label>";
}

// =====================================================================
//  STATUS PAGE
// =====================================================================
void statusPageSend(const String &msg = "", bool err = false) {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  server.sendContent(htmlHeader("Weather Relay"));
  server.sendContent("<h1>🌦️ Weather-Controlled Relay</h1>");
  if (msg.length())
    server.sendContent("<div class='msg" + String(err ? " err" : "") + "'>" + htmlEscape(msg) + "</div>");
  server.sendContent("<div class='card'><h2>Live Weather</h2>");
  if (locationCount == 0) {
    server.sendContent("<div class='row'><span>Status</span><span class='val off'>No locations configured</span></div>");
  } else {
    for (uint8_t i = 0; i < locationCount; i++) {
      String title = locations[i].name;
      if (!locations[i].enabled) title += " (disabled)";
      server.sendContent("<div class='loc'><b>" + htmlEscape(title) + "</b>");
      if (wx[i].valid) {
        uint64_t age64 = (uptimeMs() - wx[i].lastOkMs64) / 1000ULL;
        if (age64 > 3153600000ULL) age64 = 3153600000ULL;
        unsigned long age = (unsigned long)age64;
        server.sendContent("<div class='row'><span>Condition</span><span class='val'>" + htmlEscape(wx[i].icon) + " " + htmlEscape(wx[i].desc) + "</span></div>");
        server.sendContent("<div class='row'><span>Temp</span><span class='val'>" + String(wx[i].tempC, 1) + " °C (feels " + String(wx[i].feelsC, 1) + ")</span></div>");
        server.sendContent("<div class='row'><span>Humidity</span><span class='val'>" + String(wx[i].humidity) + " %</span></div>");
        server.sendContent("<div class='row'><span>Wind</span><span class='val'>" + String(wx[i].windKmh, 1) + " km/h (gust " + String(wx[i].gustKmh, 1) + ")</span></div>");
        server.sendContent("<div class='row'><span>Precip</span><span class='val'>" + String(wx[i].precip, 2) + " mm</span></div>");
        server.sendContent("<div class='row'><span>Updated</span><span class='val'>" + String(age) + " s ago</span></div>");
        if (uptimeMs() - wx[i].lastOkMs64 > (uint64_t)WEATHER_STALE_MS)
          server.sendContent("<div class='msg err'>⚠️ Stale</div>");
      } else {
        server.sendContent("<div class='row'><span>Status</span><span class='val off'>No data</span></div>");
        if (wx[i].lastErr.length())
          server.sendContent("<div class='msg err'>" + htmlEscape(wx[i].lastErr) + "</div>");
      }
      server.sendContent("</div>");
    }
  }
  server.sendContent("</div>");
  server.sendContent("<div class='card'><h2>Relay (GPIO25)</h2>");
  server.sendContent("<div class='row'><span>State</span><span class='val " + String(relayIsOn ? "on'>ON" : "off'>OFF") + "</span></div>");
  server.sendContent("<div class='row'><span>Enabled</span><span class='val'>" + String(cfg.relayEnabled ? "Yes" : "No") + "</span></div>");
  server.sendContent("<div class='row'><span>Force</span><span class='val'>" +
       String(cfg.forceState == 1 ? "FORCE ON" : (cfg.forceState == -1 ? "FORCE OFF" : "Auto")) + "</span></div>");
  server.sendContent("<div class='row'><span>Logic</span><span class='val'>" + String(cfg.logicAll ? "ALL (AND)" : "ANY (OR)") + "</span></div>");
  server.sendContent("<div class='row'><span>Multi-location</span><span class='val'>" + String(cfg.multiLogicAll ? "ALL locations" : "ANY location") + "</span></div>");
  server.sendContent("<div class='row'><span>Triggers</span><span class='val'>");
  int cnt = 0;
  if (cfg.useTemp)      { server.sendContent("Temp ");     cnt++; }
  if (cfg.useFeels)     { server.sendContent("Feels ");    cnt++; }
  if (cfg.useRainMm)    { server.sendContent("Rain-mm ");  cnt++; }
  if (cfg.useRainCode)  { server.sendContent("Rain ");     cnt++; }
  if (cfg.useSnowCode)  { server.sendContent("Snow ");     cnt++; }
  if (cfg.useStormCode) { server.sendContent("Storm ");    cnt++; }
  if (cfg.useFogCode)   { server.sendContent("Fog ");      cnt++; }
  if (cfg.useWind)      { server.sendContent("Wind ");     cnt++; }
  if (cfg.useGusts)     { server.sendContent("Gusts ");    cnt++; }
  if (cfg.useHumidity)  { server.sendContent("Humidity "); cnt++; }
  if (!cnt) server.sendContent("none");
  server.sendContent("</span></div></div>");
  server.sendContent("<div class='card'><h2>Network</h2>");
  server.sendContent("<div class='row'><span>STA</span><span class='val " + String(staConnected ? "on'>Connected" : "off'>Disconnected") + "</span></div>");
  if (staConnected) {
    server.sendContent("<div class='row'><span>SSID</span><span class='val'>" + htmlEscape(WiFi.SSID()) + "</span></div>");
    server.sendContent("<div class='row'><span>IP</span><span class='val'>" + WiFi.localIP().toString() + "</span></div>");
    server.sendContent("<div class='row'><span>RSSI</span><span class='val'>" + String(WiFi.RSSI()) + " dBm</span></div>");
  }
  server.sendContent("<div class='row'><span>AP</span><span class='val " + String(apMode ? "on'>Active" : "off'>Off") + "</span></div>");
  if (apMode) {
    server.sendContent("<div class='row'><span>AP SSID</span><span class='val'>" + htmlEscape(apSsidRuntime) + "</span></div>");
    server.sendContent("<div class='row'><span>AP IP</span><span class='val'>" + WiFi.softAPIP().toString() + "</span></div>");
  }
  server.sendContent("</div>");
  server.sendContent(
    "<div class='card'>"
    "<a href='/settings'><button>⚙️ Settings</button></a>"
    "<a href='/locations'><button>📍 Locations</button></a>"
    "<a href='/wifi'><button class='gray'>📶 WiFi Manager</button></a>"
    "<a href='/refresh'><button class='gray'>🔄 Refresh Weather</button></a>"
    "<form action='/factory' method='POST' onsubmit=\"return confirm('Factory reset? This clears all settings.')\">"
    "<button class='alt' type='submit'>🗑️ Factory Reset</button></form>"
    "</div>");
  server.sendContent(htmlFooter());
  server.sendContent("");
}

// =====================================================================
//  LOCATIONS PAGE
// =====================================================================
String locationsPage(const String &msg = "", bool err = false) {
  String s;
  s.reserve(12000);
  s = htmlHeader("Locations");
  s += "<h1>📍 Locations</h1>";
  if (msg.length()) s += "<div class='msg" + String(err ? " err" : "") + "'>" + htmlEscape(msg) + "</div>";
  s += "<div class='card'><h2>Configured (" + String(locationCount) + "/" + String(MAX_LOCATIONS) + ")</h2>";
  s += "<form method='POST' action='/locations/save'>";
  for (uint8_t i = 0; i < locationCount; i++) {
    s += "<div class='loc'><b>Location " + String(i + 1) + "</b>";
    s += "<label>Name</label><input name='ln" + String(i) + "' value='" + htmlEscape(locations[i].name) + "' maxlength='32'>";
    s += "<div class='grid'>"
         "<div><label>Latitude</label><input name='la" + String(i) + "' type='number' step='0.0001' min='-90' max='90' value='" + String(locations[i].lat, 4) + "'></div>"
         "<div><label>Longitude</label><input name='lo" + String(i) + "' type='number' step='0.0001' min='-180' max='180' value='" + String(locations[i].lon, 4) + "'></div>"
         "</div>";
    s += "<label><input type='checkbox' name='le" + String(i) + "' value='1'" + String(locations[i].enabled ? " checked" : "") + "> Enabled</label>";
    s += "</div>";
  }
  if (locationCount > 0) {
    s += "<button type='submit'>💾 Save All Locations</button>";
  }
  s += "</form></div>";
  if (locationCount < MAX_LOCATIONS) {
    s += "<div class='card'><h2>Add Location</h2><form method='POST' action='/locations/add'>";
    s += "<label>Name</label><input name='name' maxlength='32' placeholder='e.g. Tokyo'>";
    s += "<div class='grid'>"
         "<div><label>Latitude</label><input name='lat' type='number' step='0.0001' min='-90' max='90' value='0'></div>"
         "<div><label>Longitude</label><input name='lon' type='number' step='0.0001' min='-180' max='180' value='0'></div>"
         "</div>";
    s += "<button class='grn' type='submit'>➕ Add Location</button></form></div>";
  }
  s += "<div class='card'><h2>Delete Location</h2>";
  if (locationCount > 0) {
    s += "<form method='POST' action='/locations/del' onsubmit=\"return confirm('Delete this location?')\">";
    s += "<label>Select</label><select name='idx'>";
    for (uint8_t i = 0; i < locationCount; i++) {
      s += "<option value='" + String(i) + "'>" + htmlEscape(locations[i].name) + "</option>";
    }
    s += "</select>";
    s += "<button class='alt' type='submit'>🗑️ Delete</button></form>";
  } else {
    s += "<div class='row'><span>No locations.</span></div>";
  }
  s += "</div>";
  s += "<div class='card'><h2>Combine Locations</h2><form method='POST' action='/locations/mode'>";
  s += "<label>Relay when</label><select name='mLAll'>"
       "<option value='0'" + String(!cfg.multiLogicAll ? " selected" : "") + ">ANY location triggers</option>"
       "<option value='1'" + String(cfg.multiLogicAll ? " selected" : "") + ">ALL locations trigger</option>"
       "</select>";
  s += "<button type='submit'>💾 Save Combine Mode</button></form></div>";
  s += "<a href='/'><button class='gray'>← Back</button></a>";
  s += htmlFooter();
  return s;
}

// =====================================================================
//  SETTINGS PAGE
// =====================================================================
String settingsPage(const String &msg = "", bool err = false) {
  String s;
  s.reserve(12000);
  s = htmlHeader("Settings");
  s += "<h1>⚙️ Settings</h1>";
  if (msg.length()) s += "<div class='msg" + String(err ? " err" : "") + "'>" + htmlEscape(msg) + "</div>";
  s += "<form method='POST' action='/save'>";
  s += "<div class='card'><h2>Relay Basics</h2>";
  s += "<label>Relay Enabled</label><select name='rEn'>"
       "<option value='1'" + String(cfg.relayEnabled ? " selected" : "") + ">Yes</option>"
       "<option value='0'" + String(!cfg.relayEnabled ? " selected" : "") + ">No</option></select>";
  s += "<label>Relay active LOW</label><select name='rLow'>"
       "<option value='1'" + String(cfg.relayActiveLow ? " selected" : "") + ">Yes</option>"
       "<option value='0'" + String(!cfg.relayActiveLow ? " selected" : "") + ">No</option></select>";
  s += "<label>Force override</label><select name='frc'>"
       "<option value='0'"  + String(cfg.forceState == 0  ? " selected" : "") + ">Auto</option>"
       "<option value='1'"  + String(cfg.forceState == 1  ? " selected" : "") + ">Force ON</option>"
       "<option value='-1'" + String(cfg.forceState == -1 ? " selected" : "") + ">Force OFF</option></select>";
  s += "</div>";
  s += "<div class='card'><h2>Trigger Logic</h2>";
  s += "<label>Combine triggers within a location</label><select name='lAll'>"
       "<option value='0'" + String(!cfg.logicAll ? " selected" : "") + ">ANY (OR)</option>"
       "<option value='1'" + String(cfg.logicAll ? " selected" : "") + ">ALL (AND)</option></select>";
  s += "<label>OR-mode tie-breaker (both ON and OFF fire)</label><select name='pOn'>"
       "<option value='1'" + String(cfg.preferOn ? " selected" : "") + ">Prefer ON</option>"
       "<option value='0'" + String(!cfg.preferOn ? " selected" : "") + ">Prefer OFF</option></select>";
  s += "</div>";
  s += "<div class='card'><h2>Temperature Trigger</h2>";
  s += cRow("uT", cfg.useTemp, "Enable temperature trigger");
  s += "<div class='grid'>"
       "<div><label>ON at ≥ (°C)</label><input name='tOn' type='number' step='0.1' min='-40' max='60' value='" + String(cfg.tempOn, 1) + "'></div>"
       "<div><label>OFF at ≤ (°C)</label><input name='tOff' type='number' step='0.1' min='-40' max='60' value='" + String(cfg.tempOff, 1) + "'></div>"
       "</div></div>";
  s += "<div class='card'><h2>Feels-like Trigger</h2>";
  s += cRow("uF", cfg.useFeels, "Enable feels-like trigger");
  s += "<div class='grid'>"
       "<div><label>ON at ≥ (°C)</label><input name='fOn' type='number' step='0.1' min='-40' max='70' value='" + String(cfg.feelsOn, 1) + "'></div>"
       "<div><label>OFF at ≤ (°C)</label><input name='fOff' type='number' step='0.1' min='-40' max='70' value='" + String(cfg.feelsOff, 1) + "'></div>"
       "</div></div>";
  s += "<div class='card'><h2>Rain Volume Trigger (mm in last hour)</h2>";
  s += cRow("uRm", cfg.useRainMm, "Enable rain volume trigger");
  s += "<div class='grid'>"
       "<div><label>ON at ≥ (mm)</label><input name='rOn' type='number' step='0.05' min='0.1' max='100' value='" + String(cfg.rainOn, 2) + "'></div>"
       "<div><label>OFF at ≤ (mm)</label><input name='rOff' type='number' step='0.05' min='0' max='100' value='" + String(cfg.rainOff, 2) + "'></div>"
       "</div></div>";
  s += "<div class='card'><h2>Wind Speed Trigger</h2>";
  s += cRow("uW", cfg.useWind, "Enable wind speed trigger");
  s += "<div class='grid'>"
       "<div><label>ON at ≥ (km/h)</label><input name='wOn' type='number' step='1' min='2' max='300' value='" + String(cfg.windOn, 1) + "'></div>"
       "<div><label>OFF at ≤ (km/h)</label><input name='wOff' type='number' step='1' min='0' max='300' value='" + String(cfg.windOff, 1) + "'></div>"
       "</div></div>";
  s += "<div class='card'><h2>Wind Gust Trigger</h2>";
  s += cRow("uG", cfg.useGusts, "Enable gust trigger");
  s += "<div class='grid'>"
       "<div><label>ON at ≥ (km/h)</label><input name='gOn' type='number' step='1' min='2' max='400' value='" + String(cfg.gustOn, 1) + "'></div>"
       "<div><label>OFF at ≤ (km/h)</label><input name='gOff' type='number' step='1' min='0' max='400' value='" + String(cfg.gustOff, 1) + "'></div>"
       "</div></div>";
  s += "<div class='card'><h2>Humidity Trigger</h2>";
  s += cRow("uH", cfg.useHumidity, "Enable humidity trigger");
  s += "<div class='grid'>"
       "<div><label>ON at ≥ (%)</label><input name='hOn' type='number' step='1' min='2' max='100' value='" + String(cfg.humOn, 0) + "'></div>"
       "<div><label>OFF at ≤ (%)</label><input name='hOff' type='number' step='1' min='0' max='100' value='" + String(cfg.humOff, 0) + "'></div>"
       "</div></div>";
  s += "<div class='card'><h2>Weather-Code Triggers</h2>";
  s += cRow("uRc", cfg.useRainCode,  "Rain (code 51–67, 80–82)");
  s += cRow("uSc", cfg.useSnowCode,  "Snow (code 71–77, 85–86)");
  s += cRow("uSt", cfg.useStormCode, "Thunderstorm (code 95–99)");
  s += cRow("uFg", cfg.useFogCode,   "Fog (code 45, 48)");
  s += "</div>";
  s += "<button type='submit'>💾 Save Settings</button></form>";
  s += "<a href='/'><button class='gray'>← Back</button></a>";
  s += htmlFooter();
  return s;
}

// =====================================================================
//  WIFI PAGE
// =====================================================================
String wifiPage(const String &msg = "", bool err = false) {
  String s;
  s.reserve(8000);
  s = htmlHeader("WiFi Manager");
  s += "<h1>📶 WiFi Manager</h1>";
  if (msg.length()) s += "<div class='msg" + String(err ? " err" : "") + "'>" + htmlEscape(msg) + "</div>";
  s += "<div class='card'><h2>Status</h2>";
  s += "<div class='row'><span>STA Mode</span><span class='val " + String(staConnected ? "on'>Connected" : "off'>Disconnected") + "</span></div>";
  if (staConnected) {
    s += "<div class='row'><span>SSID</span><span class='val'>" + htmlEscape(WiFi.SSID()) + "</span></div>";
    s += "<div class='row'><span>IP</span><span class='val'>" + WiFi.localIP().toString() + "</span></div>";
    s += "<div class='row'><span>Gateway</span><span class='val'>" + WiFi.gatewayIP().toString() + "</span></div>";
    s += "<div class='row'><span>RSSI</span><span class='val'>" + String(WiFi.RSSI()) + " dBm</span></div>";
  }
  s += "<div class='row'><span>Hostname</span><span class='val'>" + htmlEscape(cfg.hostname) + "</span></div>";
  s += "<div class='row'><span>AP Mode</span><span class='val " + String(apMode ? "on'>Active" : "off'>Off") + "</span></div>";
  if (apMode) {
    s += "<div class='row'><span>AP SSID</span><span class='val'>" + htmlEscape(apSsidRuntime) + "</span></div>";
    s += "<div class='row'><span>AP IP</span><span class='val'>" + WiFi.softAPIP().toString() + "</span></div>";
    int clients = WiFi.softAPgetStationNum();
    s += "<div class='row'><span>Clients</span><span class='val'>" + String(clients) + "</span></div>";
  }
  s += "</div>";
  s += "<div class='card'><h2>Station (Home WiFi)</h2><form method='POST' action='/wifi/sta'>";
  s += "<label>SSID</label><input name='ssid' value='" + htmlEscape(cfg.ssid) + "' maxlength='32'>";
  s += "<label>Password</label><input name='pass' type='password' value='' placeholder='leave blank to keep current' maxlength='64'>";
  s += "<label><input type='checkbox' name='staClr' value='1'> Clear STA password (open network)</label>";
  s += "<label>Hostname</label><input name='host' value='" + htmlEscape(cfg.hostname) + "' maxlength='32'>";
  s += "<button class='grn' type='submit'>💾 Save & Connect</button></form>";
  if (staConnected) {
    s += "<form method='POST' action='/wifi/disconnect' style='margin-top:8px'>"
         "<button class='alt' type='submit'>🔌 Disconnect STA</button></form>";
  } else if (cfg.ssid.length() > 0) {
    s += "<form method='POST' action='/wifi/reconnect' style='margin-top:8px'>"
         "<button class='grn' type='submit'>🔄 Reconnect STA</button></form>";
  }
  s += "</div>";
  s += "<div class='card'><h2>Access Point (Recovery)</h2><form method='POST' action='/wifi/ap'>";
  s += "<label>Enable AP</label><select name='apEn'>"
       "<option value='1'" + String(cfg.apEnabled ? " selected" : "") + ">Yes</option>"
       "<option value='0'" + String(!cfg.apEnabled ? " selected" : "") + ">No</option></select>";
  s += "<label>AP SSID (blank = auto)</label><input name='apSs' value='" + htmlEscape(cfg.apSsid) + "' maxlength='32'>";
  s += "<label>AP Password (8–63 chars, blank = keep current)</label><input name='apPw' type='password' value='' placeholder='leave blank to keep current' maxlength='63'>";
  s += "<label><input type='checkbox' name='apClr' value='1'> Clear AP password (open network)</label>";
  s += "<button type='submit'>💾 Save AP Settings</button></form>";
  s += "</div>";
  s += "<div class='card'><h2>Scan Nearby Networks</h2>"
       "<a href='/wifi/scan'><button class='gray'>🔍 Scan</button></a></div>";
  s += "<a href='/'><button class='gray'>← Back</button></a>";
  s += htmlFooter();
  return s;
}
String scanPage() {
  String s;
  s.reserve(4000);
  s = htmlHeader("WiFi Scan");
  s += "<h1>🔍 Scan Results</h1>";
  wifi_mode_t prev = WiFi.getMode();
  WiFi.mode(WIFI_AP_STA);
  delay(50);
  int n = WiFi.scanNetworks(false, true);
  WiFi.mode(prev);
  s += "<div class='card'>";
  if (n < 0) {
    s += "<div class='row'><span>Scan failed. Try again.</span></div>";
  } else if (n == 0) {
    s += "<div class='row'><span>No networks found.</span></div>";
  } else {
    for (int i = 0; i < n; i++) {
      String ssid = WiFi.SSID(i);
      if (ssid.length() == 0) ssid = "<hidden>";
      String lock = WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "🔓" : "🔒";
      s += "<div class='row'><span>" + htmlEscape(ssid) + "</span><span class='val'>" +
           String(WiFi.RSSI(i)) + " dBm " + lock + "</span></div>";
    }
  }
  s += "</div>";
  WiFi.scanDelete();
  s += "<a href='/wifi'><button class='gray'>← Back</button></a>";
  s += htmlFooter();
  return s;
}

// =====================================================================
//  WIFI CONTROL
// =====================================================================
bool startSTA() {
  if (cfg.ssid.length() == 0) return false;
  if (cfg.apEnabled) WiFi.mode(WIFI_AP_STA);
  else               WiFi.mode(WIFI_STA);
  WiFi.setHostname(cfg.hostname.c_str());
  WiFi.setAutoReconnect(true);
  WiFi.begin(cfg.ssid.c_str(), cfg.password.c_str());
  unsigned long start = millis();
  while (millis() - start < STA_CONNECT_TIMEOUT) {
    if (WiFi.status() == WL_CONNECTED) {
      staConnected     = true;
      staRetries       = 0;
      staBackoffMs     = STA_BACKOFF_MIN_MS;
      staManualDisable = false;
      return true;
    }
    delay(200);
  }
  staConnected = false;
  WiFi.disconnect(false, false);
  return false;
}

void startAP() {
  if (apMode) return;
  if (cfg.apSsid.length() > 0) {
    apSsidRuntime = cfg.apSsid;
  } else {
    uint8_t mac[6]; WiFi.macAddress(mac);
    char buf[24];
    snprintf(buf, sizeof(buf), "ESP32-%02X%02X%02X", mac[3], mac[4], mac[5]);
    apSsidRuntime = String(buf);
  }
  if (cfg.ssid.length() > 0) WiFi.mode(WIFI_AP_STA);
  else                       WiFi.mode(WIFI_AP);
  int apCh = 1;
  if (WiFi.status() == WL_CONNECTED) {
    int staCh = WiFi.channel();
    if (staCh >= 1 && staCh <= 13) apCh = staCh;
  }
  bool ok;
  if (cfg.apPassword.length() >= 8) {
    ok = WiFi.softAP(apSsidRuntime.c_str(), cfg.apPassword.c_str(), apCh, false, 4);
  } else {
    ok = WiFi.softAP(apSsidRuntime.c_str(), nullptr, apCh, false, 4);
  }
  if (!ok) {
    apMode = false;
    unsigned long nb = apRetryBackoffMs * 2UL;
    if (nb > 300000UL || nb < apRetryBackoffMs) nb = 300000UL;
    apRetryBackoffMs = nb;
    return;
  }
  delay(50);
  dns.stop();
  dns.start(DNS_PORT, "*", WiFi.softAPIP());
  apMode = true;
  apRetryBackoffMs = AP_RETRY_INTERVAL_MS;
}

// =====================================================================
//  HTTP HANDLERS
// =====================================================================
void handleRoot()     { statusPageSend(); }
void handleSettings() { server.send(200, "text/html", settingsPage()); }
void handleWifi()     { server.send(200, "text/html", wifiPage()); }
void handleScan()     { server.send(200, "text/html", scanPage()); }
void handleLocations(){ server.send(200, "text/html", locationsPage()); }
bool argBool(const char *k) { return server.hasArg(k) && server.arg(k) == "1"; }
bool parseFloatArg(const char *k, float *out) {
  if (!server.hasArg(k)) return false;
  String s = server.arg(k);
  s.trim();
  if (s.length() == 0) return false;
  int dots = 0;
  bool hasDigit = false;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c >= '0' && c <= '9') { hasDigit = true; }
    else if (c == '.') { dots++; if (dots > 1) return false; }
    else if ((c == '-' || c == '+') && i == 0) { }
    else return false;
  }
  if (!hasDigit) return false;
  *out = s.toFloat();
  return true;
}

void handleSave() {
  struct NumField { const char *k; float *dst; };
  NumField nums[] = {
    {"tOn",  &cfg.tempOn},  {"tOff", &cfg.tempOff},
    {"fOn",  &cfg.feelsOn}, {"fOff", &cfg.feelsOff},
    {"rOn",  &cfg.rainOn},  {"rOff", &cfg.rainOff},
    {"wOn",  &cfg.windOn},  {"wOff", &cfg.windOff},
    {"gOn",  &cfg.gustOn},  {"gOff", &cfg.gustOff},
    {"hOn",  &cfg.humOn},   {"hOff", &cfg.humOff},
  };
  for (auto &nf : nums) {
    if (server.hasArg(nf.k)) {
      float v;
      if (!parseFloatArg(nf.k, &v)) {
        server.send(400, "text/html", settingsPage("Invalid numeric value for " + String(nf.k), true));
        return;
      }
    }
  }
  cfg.relayEnabled   = argBool("rEn");
  cfg.relayActiveLow = argBool("rLow");
  cfg.logicAll       = argBool("lAll");
  cfg.preferOn       = argBool("pOn");
  cfg.useTemp        = argBool("uT");
  cfg.useFeels       = argBool("uF");
  cfg.useRainMm      = argBool("uRm");
  cfg.useRainCode    = argBool("uRc");
  cfg.useSnowCode    = argBool("uSc");
  cfg.useStormCode   = argBool("uSt");
  cfg.useFogCode     = argBool("uFg");
  cfg.useWind        = argBool("uW");
  cfg.useGusts       = argBool("uG");
  cfg.useHumidity    = argBool("uH");
  for (auto &nf : nums) {
    if (server.hasArg(nf.k)) {
      float v;
      if (parseFloatArg(nf.k, &v)) *nf.dst = v;
    }
  }
  if (server.hasArg("frc")) {
    String frc = server.arg("frc");
    if      (frc == "1")  cfg.forceState = 1;
    else if (frc == "-1") cfg.forceState = -1;
    else                  cfg.forceState = 0;
  }
  sanitizeSettings();
  saveSettings();
  applyRelayHardware(relayIsOn);
  evaluateRelay();
  weatherRefreshRequested = true;
  statusPageSend("✅ Settings saved. Weather refreshing...");
}

void handleLocationsSave() {
  for (uint8_t i = 0; i < locationCount; i++) {
    String prefixN = "ln" + String(i);
    String prefixA = "la" + String(i);
    String prefixO = "lo" + String(i);
    String prefixE = "le" + String(i);
    if (server.hasArg(prefixN.c_str())) {
      String nm = server.arg(prefixN.c_str());
      if (nm.length() > 32) nm = nm.substring(0, 32);
      if (nm.length() > 0) locations[i].name = nm;
    }
    float v;
    if (parseFloatArg(prefixA.c_str(), &v)) {
      if (v >= -90 && v <= 90) locations[i].lat = v;
    }
    if (parseFloatArg(prefixO.c_str(), &v)) {
      if (v >= -180 && v <= 180) locations[i].lon = v;
    }
    locations[i].enabled = server.hasArg(prefixE.c_str());
  }
  saveSettings();
  weatherRefreshRequested = true;
  server.send(200, "text/html", locationsPage("✅ Locations saved. Weather refreshing..."));
}

void handleLocationsAdd() {
  if (locationCount >= MAX_LOCATIONS) {
    server.send(400, "text/html", locationsPage("Maximum locations reached.", true));
    return;
  }
  String nm = server.arg("name");
  if (nm.length() == 0) nm = "Location " + String(locationCount + 1);
  if (nm.length() > 32) nm = nm.substring(0, 32);
  float lat = 0, lon = 0;
  if (!parseFloatArg("lat", &lat) || !parseFloatArg("lon", &lon)) {
    server.send(400, "text/html", locationsPage("Latitude/Longitude must be numbers.", true));
    return;
  }
  if (lat < -90 || lat > 90 || lon < -180 || lon > 180) {
    server.send(400, "text/html", locationsPage("Invalid coordinates.", true));
    return;
  }
  locations[locationCount].name    = nm;
  locations[locationCount].lat     = lat;
  locations[locationCount].lon     = lon;
  locations[locationCount].enabled = true;
  locationCount++;
  saveSettings();
  weatherRefreshRequested = true;
  server.send(200, "text/html", locationsPage("✅ Location added."));
}

void handleLocationsDel() {
  if (!server.hasArg("idx")) {
    server.send(400, "text/html", locationsPage("No index.", true));
    return;
  }
  String idxStr = server.arg("idx");
  idxStr.trim();
  bool idxNumeric = idxStr.length() > 0;
  for (size_t k = 0; k < idxStr.length(); k++) {
    char ch = idxStr[k];
    if (ch < '0' || ch > '9') { idxNumeric = false; break; }
  }
  if (!idxNumeric) {
    server.send(400, "text/html", locationsPage("Invalid index.", true));
    return;
  }
  long idxLong = idxStr.toInt();
  if (idxLong < 0 || idxLong >= (long)locationCount) {
    server.send(400, "text/html", locationsPage("Invalid index.", true));
    return;
  }
  int idx = (int)idxLong;
  for (uint8_t i = idx; i < locationCount - 1; i++) {
    locations[i] = locations[i + 1];
    wx[i]        = wx[i + 1];
  }
  wx[locationCount - 1] = WeatherState();
  locationCount--;
  if (locationCount == 0) addDefaultLocationIfEmpty();
  saveSettings();
  weatherRefreshRequested = true;
  server.send(200, "text/html", locationsPage("✅ Location deleted."));
}

void handleLocationsMode() {
  cfg.multiLogicAll = argBool("mLAll");
  saveSettings();
  evaluateRelay();
  weatherRefreshRequested = true;
  server.send(200, "text/html", locationsPage("✅ Combine mode saved."));
}

void handleRefresh() {
  lastWeatherPoll        = millis();
  weatherRefreshRequested = false;
  bool ok = fetchWeather();
  if (ok) resetWeatherBackoff();
  evaluateRelay();
  statusPageSend(ok ? "🔄 Weather updated." : ("⚠️ Update failed."), !ok);
}

void handleFactory() {
  server.send(200, "text/html",
    F("<html><body style='font-family:system-ui;background:#0f172a;color:#e2e8f0;"
      "text-align:center;padding-top:60px'><h2>Factory reset...</h2>"
      "<p>Device rebooting.</p></body></html>"));
  delay(700);
  factoryReset();
}

void handleWifiStaSave() {
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  String host = server.arg("host");
  if (ssid.length() == 0) {
    server.send(400, "text/html", wifiPage("SSID cannot be empty.", true));
    return;
  }
  if (ssid.length() > 32) ssid = ssid.substring(0, 32);
  host.trim();
  if (host.length() == 0) host = "esp32-weather";
  if (host.length() > 32) host = host.substring(0, 32);
  bool staClear = server.hasArg("staClr") && server.arg("staClr") == "1";
  if (staClear) {
    pass = "";
  } else if (pass.length() == 0) {
    pass = cfg.password;
  } else if (pass.length() < 8 || pass.length() > 63) {
    server.send(400, "text/html",
      wifiPage("STA password must be 8–63 characters, or leave blank to keep current.", true));
    return;
  }
  cfg.ssid     = ssid;
  cfg.password = pass;
  cfg.hostname = host;
  staManualDisable = false;
  staRetries   = 0;
  staBackoffMs = STA_BACKOFF_MIN_MS;
  flushSettingsToNVS();
  server.send(200, "text/html",
    F("<html><body style='font-family:system-ui;background:#0f172a;color:#e2e8f0;"
      "text-align:center;padding-top:60px'><h2>Saved.</h2>"
      "<p>Rebooting to connect...</p></body></html>"));
  delay(1500);
  ESP.restart();
}

void handleWifiDisconnect() {
  staManualDisable = true;
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false, false);
  staConnected = false;
  server.send(200, "text/html", wifiPage("🔌 Disconnected from STA. Use Reconnect or save new credentials."));
}

void handleWifiReconnect() {
  if (cfg.ssid.length() == 0) {
    server.send(400, "text/html", wifiPage("No STA credentials saved.", true));
    return;
  }
  staManualDisable = false;
  WiFi.setAutoReconnect(true);
  WiFi.mode(cfg.apEnabled || apMode ? WIFI_AP_STA : WIFI_STA);
  WiFi.begin(cfg.ssid.c_str(), cfg.password.c_str());
  lastStaRetry = millis();
  staBackoffMs = STA_BACKOFF_MIN_MS;
  server.send(200, "text/html", wifiPage("🔄 Reconnecting to STA..."));
}

void handleWifiApSave() {
  bool apEn  = server.arg("apEn") == "1";
  String ss  = server.arg("apSs");
  String pw  = server.arg("apPw");
  if (!apEn && !staConnected) {
    server.send(400, "text/html",
      wifiPage("Cannot disable AP while STA is disconnected — you would lose access. "
               "Connect STA first, or keep AP enabled.", true));
    return;
  }
  if (ss.length() > 32) ss = ss.substring(0, 32);
  if (pw.length() > 0) {
    if (pw.length() < 8 || pw.length() > 63) {
      server.send(400, "text/html", wifiPage("AP password must be 8–63 chars or blank.", true));
      return;
    }
  } else if (server.hasArg("apClr") && server.arg("apClr") == "1") {
    pw = "";
  } else {
    pw = cfg.apPassword;
  }
  cfg.apEnabled  = apEn;
  cfg.apSsid     = ss;
  cfg.apPassword = pw;
  flushSettingsToNVS();
  server.send(200, "text/html",
    F("<html><body style='font-family:system-ui;background:#0f172a;color:#e2e8f0;"
      "text-align:center;padding-top:60px'><h2>AP settings saved.</h2>"
      "<p>Rebooting to apply.</p></body></html>"));
  delay(1500);
  ESP.restart();
}

void handleNotFound() {
  if (apMode) {
    IPAddress host = server.client().localIP();
    String hostStr = host.toString();
    if (hostStr == "0.0.0.0" || hostStr.length() == 0) {
      hostStr = WiFi.softAPIP().toString();
    }
    server.sendHeader("Location", "http://" + hostStr + "/", true);
    server.send(302, "text/plain", "");
  } else {
    server.send(404, "text/plain", "Not found");
  }
}

void handleApiStatus() {
  DynamicJsonDocument d(16384);
  d["relay"]["on"]        = relayIsOn;
  d["relay"]["enabled"]   = cfg.relayEnabled;
  d["relay"]["activeLow"] = cfg.relayActiveLow;
  d["relay"]["force"]     = cfg.forceState;
  d["relay"]["logicAll"]  = cfg.logicAll;
  d["relay"]["preferOn"]  = cfg.preferOn;
  d["relay"]["multiLogicAll"] = cfg.multiLogicAll;
  d["cfg"]["useTemp"]      = cfg.useTemp;
  d["cfg"]["useFeels"]     = cfg.useFeels;
  d["cfg"]["useRainMm"]    = cfg.useRainMm;
  d["cfg"]["useRainCode"]  = cfg.useRainCode;
  d["cfg"]["useSnowCode"]  = cfg.useSnowCode;
  d["cfg"]["useStormCode"] = cfg.useStormCode;
  d["cfg"]["useFogCode"]   = cfg.useFogCode;
  d["cfg"]["useWind"]      = cfg.useWind;
  d["cfg"]["useGusts"]     = cfg.useGusts;
  d["cfg"]["useHumidity"]  = cfg.useHumidity;
  d["thr"]["tempOn"]  = cfg.tempOn;
  d["thr"]["tempOff"] = cfg.tempOff;
  d["thr"]["feelsOn"] = cfg.feelsOn;
  d["thr"]["feelsOff"]= cfg.feelsOff;
  d["thr"]["rainOn"]  = cfg.rainOn;
  d["thr"]["rainOff"] = cfg.rainOff;
  d["thr"]["windOn"]  = cfg.windOn;
  d["thr"]["windOff"] = cfg.windOff;
  d["thr"]["gustOn"]  = cfg.gustOn;
  d["thr"]["gustOff"] = cfg.gustOff;
  d["thr"]["humOn"]   = cfg.humOn;
  d["thr"]["humOff"]  = cfg.humOff;
  JsonArray locs = d.createNestedArray("locations");
  uint8_t maxEmit = locationCount;
  if (maxEmit > MAX_LOCATIONS) maxEmit = MAX_LOCATIONS;
  for (uint8_t i = 0; i < maxEmit; i++) {
    JsonObject o = locs.createNestedObject();
    o["name"]    = locations[i].name;
    o["lat"]     = locations[i].lat;
    o["lon"]     = locations[i].lon;
    o["enabled"] = locations[i].enabled;
    o["weather"]["valid"]    = wx[i].valid;
    o["weather"]["tempC"]    = (wx[i].valid && !isnan(wx[i].tempC))   ? wx[i].tempC   : 0.0f;
    o["weather"]["feelsC"]   = (wx[i].valid && !isnan(wx[i].feelsC))  ? wx[i].feelsC  : 0.0f;
    o["weather"]["humidity"] = wx[i].valid ? wx[i].humidity : 0;
    o["weather"]["windKmh"]  = (wx[i].valid && !isnan(wx[i].windKmh)) ? wx[i].windKmh : 0.0f;
    o["weather"]["gustKmh"]  = (wx[i].valid && !isnan(wx[i].gustKmh)) ? wx[i].gustKmh : 0.0f;
    o["weather"]["precip"]   = wx[i].valid ? wx[i].precip  : 0.0f;
    o["weather"]["wcode"]    = wx[i].valid ? wx[i].wcode   : -1;
    o["weather"]["desc"]     = wx[i].desc;
    o["weather"]["icon"]     = wx[i].icon;
    {
      uint64_t _age = wx[i].valid ? (uptimeMs() - wx[i].lastOkMs64) / 1000ULL : 0;
      if (_age > 3153600000ULL) _age = 3153600000ULL;
      o["weather"]["ageSec"] = (uint32_t)_age;
    }
    o["weather"]["err"]      = wx[i].lastErr;
  }
  d["net"]["staConnected"] = staConnected;
  d["net"]["staSsid"]      = staConnected ? WiFi.SSID() : "";
  d["net"]["staIp"]        = staConnected ? WiFi.localIP().toString() : "";
  d["net"]["rssi"]         = staConnected ? WiFi.RSSI() : 0;
  d["net"]["apMode"]       = apMode;
  d["net"]["apSsid"]       = apSsidRuntime;
  d["net"]["apIp"]         = apMode ? WiFi.softAPIP().toString() : "";
  d["net"]["hostname"]     = cfg.hostname;
  String out;
  serializeJson(d, out);
  server.send(200, "application/json", out);
}

// =====================================================================
//  BOOT BUTTON
// =====================================================================
void handleBootButton() {
  bool pressed = (digitalRead(BOOT_BUTTON_PIN) == LOW);
  static bool     pressCandidate   = false;
  static uint32_t pressCandidateMs = 0;
  if (!bootArmed) {
    if (!pressed) bootArmed = true;
    bootDown = false;
    pressCandidate = false;
    return;
  }
  if (pressed && !bootDown) {
    if (!pressCandidate) {
      pressCandidate   = true;
      pressCandidateMs = millis();
    } else if ((uint32_t)(millis() - pressCandidateMs) >= 100) {
      bootDown       = true;
      bootStart      = millis();
      pressCandidate = false;
    }
  } else if (!pressed) {
    if (bootDown) bootDown = false;
    pressCandidate = false;
  } else if (pressed && bootDown) {
    if (millis() - bootStart >= FACTORY_RESET_MS) {
      bootDown = false;
      factoryReset();
    }
  }
}

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
  g_uptimeLast = millis();
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  loadSettings();
  initRelayPin();
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  staManualDisable = false;
  bool staOk = false;
  if (cfg.apEnabled) startAP();
  if (cfg.ssid.length() > 0) staOk = startSTA();
  if (cfg.apEnabled || !staOk) {
    if (!apMode) startAP(); 
  }
  if (staOk) {
    fetchWeather();
    evaluateRelay();
  }
  server.on("/",                 handleRoot);
  server.on("/settings",         handleSettings);
  server.on("/save",   HTTP_POST, handleSave);
  server.on("/locations",        handleLocations);
  server.on("/locations/save", HTTP_POST, handleLocationsSave);
  server.on("/locations/add",  HTTP_POST, handleLocationsAdd);
  server.on("/locations/del",  HTTP_POST, handleLocationsDel);
  server.on("/locations/mode", HTTP_POST, handleLocationsMode);
  server.on("/refresh",          handleRefresh);
  server.on("/factory", HTTP_POST, handleFactory);
  server.on("/wifi",             handleWifi);
  server.on("/wifi/scan",        handleScan);
  server.on("/wifi/sta", HTTP_POST, handleWifiStaSave);
  server.on("/wifi/ap",  HTTP_POST, handleWifiApSave);
  server.on("/wifi/disconnect", HTTP_POST, handleWifiDisconnect);
  server.on("/wifi/reconnect",  HTTP_POST, handleWifiReconnect);
  server.on("/api/status",       handleApiStatus);
  auto redir = []() {
    if (apMode) {
      String hostStr = server.client().localIP().toString();
      if (hostStr == "0.0.0.0" || hostStr.length() == 0) {
        hostStr = WiFi.softAPIP().toString();
      }
      server.sendHeader("Location", "http://" + hostStr + "/", true);
      server.send(302, "text/plain", "");
    } else {
      server.send(404, "text/plain", "Not found");
    }
  };
  server.on("/generate_204",        redir);
  server.on("/gen_204",             redir);
  server.on("/hotspot-detect.html", redir);
  server.on("/connecttest.txt",     redir);
  server.on("/ncsi.txt",            redir);
  server.on("/fwlink",              redir);
  server.onNotFound(handleNotFound);
  server.begin();
}

// =====================================================================
//  LOOP
// =====================================================================
void loop() {
  uptimeTick();
  if (saveDirty && (millis() - lastSaveAttemptMs >= 5000UL)) {
    flushSettingsToNVS();
  }
  handleBootButton();
  if (apMode) dns.processNextRequest();
  server.handleClient();
  staConnected = (WiFi.status() == WL_CONNECTED);
  if (millis() - lastEvalMs >= 5000UL) {
    lastEvalMs = millis();
    evaluateRelay();
  }
  if (!apMode && cfg.apEnabled && (millis() - lastApAttemptMs >= apRetryBackoffMs)) {
    lastApAttemptMs = millis();
    startAP();
  }
  if (staConnected) {
    staRetries   = 0;
    staBackoffMs = STA_BACKOFF_MIN_MS;
    bool due = (millis() - lastWeatherPoll >= weatherBackoffMs);
    if (due || weatherRefreshRequested) {
      weatherRefreshRequested = false;
      lastWeatherPoll = millis();
      bool ok = fetchWeather();
      evaluateRelay();
      if (ok) {
        resetWeatherBackoff();
      } else {
        weatherFailCount++;
        unsigned long nb = weatherBackoffMs * 2UL;
        if (nb > WEATHER_FAIL_BACKOFF_MAX_MS || nb < weatherBackoffMs)
          nb = WEATHER_FAIL_BACKOFF_MAX_MS;
        weatherBackoffMs = nb;
      }
    }
  } else if (!staManualDisable && cfg.ssid.length() > 0) {
    if (millis() - lastStaRetry >= staBackoffMs) {
      lastStaRetry = millis();
      staRetries++;
      WiFi.reconnect();
      unsigned long nb = staBackoffMs * 2UL;
      if (nb > STA_BACKOFF_MAX_MS || nb < staBackoffMs) nb = STA_BACKOFF_MAX_MS;
      staBackoffMs = nb;
    }
  }
}
