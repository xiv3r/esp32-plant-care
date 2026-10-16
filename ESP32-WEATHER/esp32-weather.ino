/*
===================================================================
 * ESP32 Live Weather Monitoring
 * WiFi AP+STA | Captive Portal | Settings | Factory Reset
 * Comprehensive Weather + Air Quality + 7-Day Forecast
===================================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <time.h>

// ---------------- CONFIG ----------------
#define FW_VERSION           "3.2.0"
#define AP_SSID              "ESP32_Live_Weather"
#define AP_PASS              "12345678"
#define AP_CHANNEL           1
#define AP_HIDDEN            false
#define AP_MAX_CONN          4
#define BOOT_BTN_PIN         0
#define FACTORY_HOLD_MS      5000
#define BTN_DEBOUNCE_MS      50
#define DNS_PORT             53
#define WEB_PORT             80
#define WEATHER_LAT_DEFAULT  14.5995f
#define WEATHER_LON_DEFAULT  120.9842f
#define WEATHER_TZ_DEFAULT   "Asia/Manila"
#define WEATHER_REFRESH_MS   600000UL
#define AIR_REFRESH_MS       900000UL
#define HTTP_TIMEOUT_MS      10000
#define MIN_FREE_HEAP        45000

// ---------------- STRUCTS ----------------
struct CurrentWx {
  bool   valid = false;
  float  tempC = NAN, feelsC = NAN, humidity = NAN, dewC = NAN, wetBulb = NAN;
  float  precip = NAN, rain = NAN, showers = NAN, snowfall = NAN;
  int    code = -1; String condition = "Unknown"; int isDay = -1;
  float  cloud = NAN, cloudLow = NAN, cloudMid = NAN, cloudHigh = NAN;
  float  pressureMsl = NAN, pressureSurf = NAN;
  float  windKph = NAN, windDir = NAN, windGust = NAN;
  float  soil0 = NAN, soil6 = NAN, soil18 = NAN, soil54 = NAN;
  float  soilMoist0 = NAN, soilMoist3 = NAN, soilMoist9 = NAN, soilMoist27 = NAN;
  float  visibilityM = NAN, et0 = NAN, vpd = NAN;
  float  uvIndex = NAN, uvClear = NAN, freezingLevel = NAN;
  float  shortwave = NAN, direct = NAN, diffuse = NAN, dni = NAN;
};
struct DailyWx {
  bool   valid = false;
  float  tMax = NAN, tMin = NAN, aMax = NAN, aMin = NAN;
  String sunrise = "--:--", sunset = "--:--";
  float  daylightSec = NAN, sunshineSec = NAN;
  float  uvMax = NAN, uvClearMax = NAN;
  float  precipSum = NAN, rainSum = NAN, showersSum = NAN, snowSum = NAN;
  int    precipHours = -1, precipProbMax = -1;
  float  windMax = NAN, gustMax = NAN, windDirDom = NAN;
  float  radiationSum = NAN, et0Daily = NAN;
  int    codeDaily = -1; String conditionDaily = "Unknown";
};
struct WeekWx {
  bool valid = false;
  int   count = 0;
  String date[7];
  int    code[7];
  String cond[7];
  float  tMax[7], tMin[7], aMax[7], aMin[7];
  float  precipSum[7], rainSum[7], snowSum[7];
  int    precipProb[7];
  float  windMax[7], gustMax[7];
  float  uvMax[7];
  String sunrise[7], sunset[7];
};
struct AirWx {
  bool  valid = false;
  float pm10 = NAN, pm25 = NAN, pm1 = NAN;
  float co = NAN, no2 = NAN, so2 = NAN, o3 = NAN, nh3 = NAN;
  float dust = NAN, aod = NAN, uvAi = NAN;
  float alderP = NAN, birchP = NAN, grassP = NAN, mugwortP = NAN, oliveP = NAN, ragweedP = NAN;
  int   aqiEu = -1, aqiUs = -1;
  String aqiEuCat = "-", aqiUsCat = "-";
};
CurrentWx curWx;
DailyWx   dayWx;
WeekWx    weekWx;
AirWx     airWx;

// ---------------- GLOBALS ----------------
Preferences   prefs;
WebServer     server(WEB_PORT);
DNSServer     dnsServer;
String        cfgSsid, cfgPass, cfgLat, cfgLon, cfgTz;
bool          staConnected = false, apActive = false, portalMode = false;
unsigned long lastWeatherMs = 0, lastAirMs = 0, bootMs = 0;
unsigned long btnPressStart = 0;
bool          btnHeld = false, factoryDone = false;
String        lastWxError = "", lastAqError = "";

// ---------------- FORWARD DECL ----------------
void setupWiFi(); void setupWebServer(); void setupDNS(); void handleBootButton();
void loadConfig(); void saveConfig(); void clearConfig();
void startAP(); void startSTA(); void applyTimezone();
String htmlPage(const String&, const String&);
String makeStatusJson();
String urlEncode(const String&); String htmlEscape(const String&);
String jsonEscape(const String&);
String weatherCodeToText(int);
String formatLocalTime(time_t, const char*);
String fmtFloat(float, int);
String fmtJsonFloat(float, int);
bool fetchWeather(); bool fetchAirQuality();
bool jsonFindNumber(const String& body, const String& key, int occurrence, double& out);
bool jsonExtractNumber(const String& body, const String& key, double& out);
bool jsonExtractString(const String& body, const String& key, String& out);
bool jsonFirstFloat(const String& body, const String& key, double& out);
bool jsonFirstInt  (const String& body, const String& key, int& out);
bool jsonFirstStr  (const String& body, const String& key, String& out);
bool jsonNthFloat  (const String& body, const String& key, int n, double& out);
bool jsonNthInt    (const String& body, const String& key, int n, int& out);
bool jsonNthStr    (const String& body, const String& key, int n, String& out);
bool jsonArrayCount(const String& body, const String& key, int& out);

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200); delay(150);
  Serial.println(); Serial.println(F("=== ESP32 Firmware v3.2.0 ==="));
  pinMode(BOOT_BTN_PIN, INPUT_PULLUP);
  loadConfig();
  bootMs = millis();
  setupWiFi();
  setupDNS();
  setupWebServer();
  server.begin();
  applyTimezone();
  if (staConnected) {
    fetchWeather(); fetchAirQuality();
    lastWeatherMs = millis(); lastAirMs = millis();
  }
}

// ---------------- LOOP ----------------
void loop() {
  if (apActive) dnsServer.processNextRequest();
  server.handleClient();
  handleBootButton();
  if (staConnected && (millis() - lastWeatherMs >= WEATHER_REFRESH_MS)) {
    fetchWeather(); lastWeatherMs = millis();
  }
  if (staConnected && (millis() - lastAirMs >= AIR_REFRESH_MS)) {
    fetchAirQuality(); lastAirMs = millis();
  }
  static unsigned long lastStaCheck = 0;
  static bool wasStaConnected = false;
  if (millis() - lastStaCheck > 15000) {
    lastStaCheck = millis();
    if (cfgSsid.length() > 0 && WiFi.status() != WL_CONNECTED) {
      staConnected = false;
      WiFi.disconnect(false, false);
      WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
    } else if (WiFi.status() == WL_CONNECTED) {
      staConnected = true;
    }
  }
  if (staConnected && !wasStaConnected) {
    fetchWeather(); fetchAirQuality();
    lastWeatherMs = millis(); lastAirMs = millis();
  }
  wasStaConnected = staConnected;
  delay(2);
}

// ---------------- CONFIG ----------------
void loadConfig() {
  prefs.begin("cfg", true);
  cfgSsid = prefs.getString("ssid", "");
  cfgPass = prefs.getString("pass", "");
  cfgLat  = prefs.getString("lat",  String(WEATHER_LAT_DEFAULT, 4));
  cfgLon  = prefs.getString("lon",  String(WEATHER_LON_DEFAULT, 4));
  cfgTz   = prefs.getString("tz",   WEATHER_TZ_DEFAULT);
  prefs.end();
  if (cfgLat.length() == 0) cfgLat = String(WEATHER_LAT_DEFAULT, 4);
  if (cfgLon.length() == 0) cfgLon = String(WEATHER_LON_DEFAULT, 4);
  if (cfgTz.length()  == 0) cfgTz  = WEATHER_TZ_DEFAULT;
}

void saveConfig() {
  prefs.begin("cfg", false);
  prefs.putString("ssid", cfgSsid); prefs.putString("pass", cfgPass);
  prefs.putString("lat",  cfgLat);  prefs.putString("lon",  cfgLon);
  prefs.putString("tz",   cfgTz);
  prefs.end();
}

void clearConfig() {
  prefs.begin("cfg", false); prefs.clear(); prefs.end();
  cfgSsid=""; cfgPass="";
  cfgLat = String(WEATHER_LAT_DEFAULT, 4);
  cfgLon = String(WEATHER_LON_DEFAULT, 4);
  cfgTz  = WEATHER_TZ_DEFAULT;
}

// ---------------- WIFI ----------------
void setupWiFi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  delay(100);
  startAP();
  if (cfgSsid.length() > 0) startSTA(); else portalMode = true;
}

void startAP() {
  IPAddress ip(192,168,4,1), gw(192,168,4,1), mk(255,255,255,0);
  WiFi.softAPConfig(ip, gw, mk);
  apActive = WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, AP_HIDDEN, AP_MAX_CONN);
  if (apActive) Serial.printf("[AP] %s @ %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

void startSTA() {
  WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
  unsigned long s = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - s < 15000) { delay(200); Serial.print('.'); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) { staConnected = true; portalMode = false; }
  else { staConnected = false; portalMode = true; }
}

// ---------------- DNS ----------------
void setupDNS() { if (apActive) dnsServer.start(DNS_PORT, "*", WiFi.softAPIP()); }

// ---------------- JSON ----------------
bool jsonFindNumber(const String& body, const String& key, int occurrence, double& out) {
  String token = "\"" + key + "\":";
  int from = 0;
  int found = 0;
  while (true) {
    int idx = body.indexOf(token, from);
    if (idx < 0) return false;
    from = idx + token.length();
    int p = from;
    while (p < (int)body.length() && body[p] == ' ') p++;
    if (p >= (int)body.length()) return false;
    char c = body[p];
    if (c == '"') continue;
    if (body.startsWith("null", p)) continue;
    if (c == '[' || c == '{') continue;
    if (found == occurrence) {
      int end = p;
      while (end < (int)body.length()) {
        char cc = body[end];
        if ((cc>='0'&&cc<='9')||cc=='-'||cc=='+'||cc=='.'||cc=='e'||cc=='E') end++;
        else break;
      }
      if (end == p) return false;
      out = body.substring(p, end).toDouble();
      return true;
    }
    found++;
  }
}
bool jsonExtractNumber(const String& body, const String& key, double& out) { return jsonFindNumber(body, key, 0, out); }
bool jsonExtractString(const String& body, const String& key, String& out) {
  String token = "\"" + key + "\":\"";
  int idx = body.indexOf(token);
  if (idx < 0) return false;
  idx += token.length();
  int end = body.indexOf('"', idx);
  if (end < 0) return false;
  out = body.substring(idx, end);
  return true;
}
static bool findArrayStart(const String& body, const String& key, int& pos) {
  String token = "\"" + key + "\":[";
  pos = body.indexOf(token);
  if (pos < 0) return false;
  pos += token.length();
  return true;
}
static bool nextElement(const String& body, int& pos, String& raw, bool& isNull) {
  while (pos < (int)body.length() && (body[pos]==' '||body[pos]==',')) pos++;
  if (pos >= (int)body.length() || body[pos] == ']') return false;
  int start = pos;
  if (body[pos] == '"') {
    int end = body.indexOf('"', pos + 1);
    if (end < 0) return false;
    raw = body.substring(pos + 1, end);
    isNull = false;
    pos = end + 1;
    return true;
  }
  if (body.startsWith("null", pos)) { isNull = true; raw = ""; pos += 4; return true; }
  int end = pos;
  while (end < (int)body.length()) {
    char c = body[end];
    if ((c>='0'&&c<='9')||c=='-'||c=='+'||c=='.'||c=='e'||c=='E') end++;
    else break;
  }
  if (end == start) return false;
  raw = body.substring(start, end);
  isNull = false;
  pos = end;
  return true;
}
bool jsonNthFloat(const String& body, const String& key, int n, double& out) {
  int pos;
  if (!findArrayStart(body, key, pos)) return false;
  String raw; bool isNull;
  for (int i = 0; i <= n; i++) {
    if (!nextElement(body, pos, raw, isNull)) return false;
    if (i == n) { if (isNull) return false; out = raw.toDouble(); return true; }
  }
  return false;
}
bool jsonNthInt(const String& body, const String& key, int n, int& out) {
  double d; if (!jsonNthFloat(body, key, n, d)) return false;
  out = (int)d; return true;
}
bool jsonNthStr(const String& body, const String& key, int n, String& out) {
  int pos; if (!findArrayStart(body, key, pos)) return false;
  String raw; bool isNull;
  for (int i = 0; i <= n; i++) {
    if (!nextElement(body, pos, raw, isNull)) return false;
    if (i == n) { if (isNull) return false; out = raw; return true; }
  }
  return false;
}

bool jsonFirstFloat(const String& body, const String& key, double& out) { return jsonNthFloat(body, key, 0, out); }
bool jsonFirstInt  (const String& body, const String& key, int& out)    { return jsonNthInt(body, key, 0, out); }
bool jsonFirstStr  (const String& body, const String& key, String& out) { return jsonNthStr(body, key, 0, out); }
bool jsonArrayCount(const String& body, const String& key, int& out) {
  int pos; if (!findArrayStart(body, key, pos)) return false;
  int count = 0; String raw; bool isNull;
  while (true) {
    int before = pos;
    if (!nextElement(body, pos, raw, isNull)) break;
    if (pos == before) break;
    count++;
    if (count > 100) break;
  }
  out = count; return true;
}

// ---------------- WEATHER ----------------
String weatherCodeToText(int c) {
  if (c == 0) return "Clear sky";
  if (c == 1) return "Mainly clear";
  if (c == 2) return "Partly cloudy";
  if (c == 3) return "Overcast";
  if (c == 45 || c == 48) return "Fog";
  if (c >= 51 && c <= 55) return "Drizzle";
  if (c == 56 || c == 57) return "Freezing drizzle";
  if (c >= 61 && c <= 65) return "Rain";
  if (c == 66 || c == 67) return "Freezing rain";
  if (c >= 71 && c <= 75) return "Snow";
  if (c == 77) return "Snow grains";
  if (c >= 80 && c <= 82) return "Rain showers";
  if (c == 85 || c == 86) return "Snow showers";
  if (c == 95) return "Thunderstorm";
  if (c >= 96 && c <= 99) return "Thunderstorm w/ hail";
  return "Unknown";
}

bool fetchWeather() {
  if (WiFi.status() != WL_CONNECTED) return false;
  if (ESP.getFreeHeap() < MIN_FREE_HEAP) { lastWxError = "low heap"; return false; }
  const char* curVars =
    "temperature_2m,relative_humidity_2m,apparent_temperature,is_day,"
    "precipitation,rain,showers,snowfall,weather_code,cloud_cover,"
    "cloud_cover_low,cloud_cover_mid,cloud_cover_high,"
    "pressure_msl,surface_pressure,wind_speed_10m,wind_direction_10m,"
    "wind_gusts_10m,"
    "soil_temperature_0cm,soil_temperature_6cm,soil_temperature_18cm,soil_temperature_54cm,"
    "soil_moisture_0_to_1cm,soil_moisture_1_to_3cm,soil_moisture_3_to_9cm,soil_moisture_9_to_27cm,"
    "visibility,et0_fao_evapotranspiration,vapour_pressure_deficit,"
    "uv_index,uv_index_clear_sky,freezing_level_height,"
    "dew_point_2m,wet_bulb_temperature_2m,"
    "shortwave_radiation,direct_radiation,diffuse_radiation,direct_normal_irradiance";
  const char* dayVars =
    "weather_code,temperature_2m_max,temperature_2m_min,"
    "apparent_temperature_max,apparent_temperature_min,"
    "sunrise,sunset,daylight_duration,sunshine_duration,"
    "uv_index_max,uv_index_clear_sky_max,"
    "precipitation_sum,rain_sum,showers_sum,snowfall_sum,"
    "precipitation_hours,precipitation_probability_max,"
    "wind_speed_10m_max,wind_gusts_10m_max,wind_direction_10m_dominant,"
    "shortwave_radiation_sum,et0_fao_evapotranspiration";
  String url = "https://api.open-meteo.com/v1/forecast?latitude=" + cfgLat +
               "&longitude=" + cfgLon +
               "&current=" + String(curVars) +
               "&daily="   + String(dayVars) +
               "&timezone=" + urlEncode(cfgTz) +
               "&forecast_days=7";
  WiFiClientSecure client; client.setInsecure(); client.setTimeout(HTTP_TIMEOUT_MS);
  HTTPClient https; https.setTimeout(HTTP_TIMEOUT_MS); https.setConnectTimeout(HTTP_TIMEOUT_MS);
  https.setReuse(false);
  if (!https.begin(client, url)) { lastWxError = "begin fail"; return false; }
  int code = https.GET();
  if (code != HTTP_CODE_OK) { https.end(); lastWxError = "http " + String(code); return false; }
  String body = https.getString();
  https.end();
  if (body.length() < 100) { lastWxError = "short body"; return false; }
  double v;
  if (jsonExtractNumber(body,"temperature_2m",v))           curWx.tempC = v;
  if (jsonExtractNumber(body,"relative_humidity_2m",v))     curWx.humidity = v;
  if (jsonExtractNumber(body,"apparent_temperature",v))     curWx.feelsC = v;
  if (jsonExtractNumber(body,"dew_point_2m",v))             curWx.dewC = v;
  if (jsonExtractNumber(body,"wet_bulb_temperature_2m",v))  curWx.wetBulb = v;
  if (jsonExtractNumber(body,"is_day",v))                   curWx.isDay = (int)v;
  if (jsonExtractNumber(body,"precipitation",v))            curWx.precip = v;
  if (jsonExtractNumber(body,"rain",v))                     curWx.rain = v;
  if (jsonExtractNumber(body,"showers",v))                  curWx.showers = v;
  if (jsonExtractNumber(body,"snowfall",v))                 curWx.snowfall = v;
  if (jsonExtractNumber(body,"weather_code",v))             curWx.code = (int)v;
  if (jsonExtractNumber(body,"cloud_cover",v))              curWx.cloud = v;
  if (jsonExtractNumber(body,"cloud_cover_low",v))          curWx.cloudLow = v;
  if (jsonExtractNumber(body,"cloud_cover_mid",v))          curWx.cloudMid = v;
  if (jsonExtractNumber(body,"cloud_cover_high",v))         curWx.cloudHigh = v;
  if (jsonExtractNumber(body,"pressure_msl",v))             curWx.pressureMsl = v;
  if (jsonExtractNumber(body,"surface_pressure",v))         curWx.pressureSurf = v;
  if (jsonExtractNumber(body,"wind_speed_10m",v))           curWx.windKph = v;
  if (jsonExtractNumber(body,"wind_direction_10m",v))       curWx.windDir = v;
  if (jsonExtractNumber(body,"wind_gusts_10m",v))           curWx.windGust = v;
  if (jsonExtractNumber(body,"soil_temperature_0cm",v))     curWx.soil0 = v;
  if (jsonExtractNumber(body,"soil_temperature_6cm",v))     curWx.soil6 = v;
  if (jsonExtractNumber(body,"soil_temperature_18cm",v))    curWx.soil18 = v;
  if (jsonExtractNumber(body,"soil_temperature_54cm",v))    curWx.soil54 = v;
  if (jsonExtractNumber(body,"soil_moisture_0_to_1cm",v))   curWx.soilMoist0 = v;
  if (jsonExtractNumber(body,"soil_moisture_1_to_3cm",v))   curWx.soilMoist3 = v;
  if (jsonExtractNumber(body,"soil_moisture_3_to_9cm",v))   curWx.soilMoist9 = v;
  if (jsonExtractNumber(body,"soil_moisture_9_to_27cm",v))  curWx.soilMoist27 = v;
  if (jsonExtractNumber(body,"visibility",v))              curWx.visibilityM = v;
  if (jsonExtractNumber(body,"et0_fao_evapotranspiration",v)) curWx.et0 = v;
  if (jsonExtractNumber(body,"vapour_pressure_deficit",v))  curWx.vpd = v;
  if (jsonExtractNumber(body,"uv_index",v))                 curWx.uvIndex = v;
  if (jsonExtractNumber(body,"uv_index_clear_sky",v))       curWx.uvClear = v;
  if (jsonExtractNumber(body,"freezing_level_height",v))    curWx.freezingLevel = v;
  if (jsonExtractNumber(body,"shortwave_radiation",v))      curWx.shortwave = v;
  if (jsonExtractNumber(body,"direct_radiation",v))         curWx.direct = v;
  if (jsonExtractNumber(body,"diffuse_radiation",v))        curWx.diffuse = v;
  if (jsonExtractNumber(body,"direct_normal_irradiance",v)) curWx.dni = v;
  curWx.condition = weatherCodeToText(curWx.code);
  curWx.valid = (curWx.code >= 0 && !isnan(curWx.tempC));
  dayWx.codeDaily = -1;
  jsonNthInt(body,"weather_code",0,dayWx.codeDaily);
  v = NAN; if (jsonNthFloat(body,"temperature_2m_max",0,v)) dayWx.tMax = v;
  v = NAN; if (jsonNthFloat(body,"temperature_2m_min",0,v)) dayWx.tMin = v;
  v = NAN; if (jsonNthFloat(body,"apparent_temperature_max",0,v)) dayWx.aMax = v;
  v = NAN; if (jsonNthFloat(body,"apparent_temperature_min",0,v)) dayWx.aMin = v;
  v = NAN; if (jsonNthFloat(body,"daylight_duration",0,v)) dayWx.daylightSec = v;
  v = NAN; if (jsonNthFloat(body,"sunshine_duration",0,v)) dayWx.sunshineSec = v;
  v = NAN; if (jsonNthFloat(body,"uv_index_max",0,v)) dayWx.uvMax = v;
  v = NAN; if (jsonNthFloat(body,"uv_index_clear_sky_max",0,v)) dayWx.uvClearMax = v;
  v = NAN; if (jsonNthFloat(body,"precipitation_sum",0,v)) dayWx.precipSum = v;
  v = NAN; if (jsonNthFloat(body,"rain_sum",0,v)) dayWx.rainSum = v;
  v = NAN; if (jsonNthFloat(body,"showers_sum",0,v)) dayWx.showersSum = v;
  v = NAN; if (jsonNthFloat(body,"snowfall_sum",0,v)) dayWx.snowSum = v;
  dayWx.precipHours   = -1; jsonNthInt(body,"precipitation_hours",0,dayWx.precipHours);
  dayWx.precipProbMax = -1; jsonNthInt(body,"precipitation_probability_max",0,dayWx.precipProbMax);
  v = NAN; if (jsonNthFloat(body,"wind_speed_10m_max",0,v)) dayWx.windMax = v;
  v = NAN; if (jsonNthFloat(body,"wind_gusts_10m_max",0,v)) dayWx.gustMax = v;
  v = NAN; if (jsonNthFloat(body,"wind_direction_10m_dominant",0,v)) dayWx.windDirDom = v;
  v = NAN; if (jsonNthFloat(body,"shortwave_radiation_sum",0,v)) dayWx.radiationSum = v;
  v = NAN; if (jsonNthFloat(body,"et0_fao_evapotranspiration",0,v)) dayWx.et0Daily = v;
  String iso;
  if (jsonNthStr(body,"sunrise",0,iso)) { int t = iso.indexOf('T'); if (t>0) dayWx.sunrise = iso.substring(t+1); }
  if (jsonNthStr(body,"sunset",0,iso))  { int t = iso.indexOf('T'); if (t>0) dayWx.sunset  = iso.substring(t+1); }
  dayWx.conditionDaily = weatherCodeToText(dayWx.codeDaily);
  dayWx.valid = (dayWx.codeDaily >= 0 && !isnan(dayWx.tMax));
  weekWx.count = 0;
  jsonArrayCount(body, "time", weekWx.count);
  if (weekWx.count > 7) weekWx.count = 7;
  for (int i = 0; i < weekWx.count; i++) {
    weekWx.date[i] = "";
    jsonNthStr(body, "time", i, weekWx.date[i]);
    weekWx.code[i] = -1;
    jsonNthInt(body, "weather_code", i, weekWx.code[i]);
    weekWx.cond[i] = weatherCodeToText(weekWx.code[i]);
    v = NAN; if (jsonNthFloat(body, "temperature_2m_max", i, v)) weekWx.tMax[i] = v;
    v = NAN; if (jsonNthFloat(body, "temperature_2m_min", i, v)) weekWx.tMin[i] = v;
    v = NAN; if (jsonNthFloat(body, "apparent_temperature_max", i, v)) weekWx.aMax[i] = v;
    v = NAN; if (jsonNthFloat(body, "apparent_temperature_min", i, v)) weekWx.aMin[i] = v;
    v = NAN; if (jsonNthFloat(body, "precipitation_sum", i, v)) weekWx.precipSum[i] = v;
    v = NAN; if (jsonNthFloat(body, "rain_sum", i, v)) weekWx.rainSum[i] = v;
    v = NAN; if (jsonNthFloat(body, "snowfall_sum", i, v)) weekWx.snowSum[i] = v;
    weekWx.precipProb[i] = -1;
    jsonNthInt(body, "precipitation_probability_max", i, weekWx.precipProb[i]);
    v = NAN; if (jsonNthFloat(body, "wind_speed_10m_max", i, v)) weekWx.windMax[i] = v;
    v = NAN; if (jsonNthFloat(body, "wind_gusts_10m_max", i, v)) weekWx.gustMax[i] = v;
    v = NAN; if (jsonNthFloat(body, "uv_index_max", i, v)) weekWx.uvMax[i] = v;
    if (jsonNthStr(body, "sunrise", i, iso)) { int t = iso.indexOf('T'); if (t>0) weekWx.sunrise[i] = iso.substring(t+1); }
    if (jsonNthStr(body, "sunset", i, iso))  { int t = iso.indexOf('T'); if (t>0) weekWx.sunset[i]  = iso.substring(t+1); }
  }
  weekWx.valid = (weekWx.count > 0);
  lastWxError = "";
  return true;
}

// ---------------- AIR QUALITY ----------------
String aqiEuCat(int a) {
  if (a < 0) return "-";
  if (a <= 20) return "Good";
  if (a <= 40) return "Fair";
  if (a <= 60) return "Moderate";
  if (a <= 80) return "Poor";
  if (a <= 100) return "Very Poor";
  return "Extremely Poor";
}
String aqiUsCat(int a) {
  if (a < 0) return "-";
  if (a <= 50) return "Good";
  if (a <= 100) return "Moderate";
  if (a <= 150) return "Unhealthy (Sensitive)";
  if (a <= 200) return "Unhealthy";
  if (a <= 300) return "Very Unhealthy";
  return "Hazardous";
}
bool fetchAirQuality() {
  if (WiFi.status() != WL_CONNECTED) return false;
  if (ESP.getFreeHeap() < MIN_FREE_HEAP) { lastAqError = "low heap"; return false; }
  String url = "https://air-quality-api.open-meteo.com/v1/air-quality?latitude=" + cfgLat +
               "&longitude=" + cfgLon +
               "&current=pm10,pm2_5,pm1_0,carbon_monoxide,nitrogen_dioxide,"
               "sulphur_dioxide,ozone,ammonia,dust,aerosol_optical_depth,uv_index,"
               "alder_pollen,birch_pollen,grass_pollen,mugwort_pollen,olive_pollen,ragweed_pollen,"
               "european_aqi,us_aqi"
               "&timezone=" + urlEncode(cfgTz);
  WiFiClientSecure client; client.setInsecure(); client.setTimeout(HTTP_TIMEOUT_MS);
  HTTPClient https; https.setTimeout(HTTP_TIMEOUT_MS); https.setConnectTimeout(HTTP_TIMEOUT_MS);
  https.setReuse(false);
  if (!https.begin(client, url)) { lastAqError = "begin fail"; return false; }
  int code = https.GET();
  if (code != HTTP_CODE_OK) { https.end(); lastAqError = "http " + String(code); return false; }
  String body = https.getString();
  https.end();
  if (body.length() < 30) { lastAqError = "short body"; return false; }
  double v;
  if (jsonExtractNumber(body,"pm10",v))              airWx.pm10 = v;
  if (jsonExtractNumber(body,"pm2_5",v))             airWx.pm25 = v;
  if (jsonExtractNumber(body,"pm1_0",v))             airWx.pm1  = v;
  if (jsonExtractNumber(body,"carbon_monoxide",v))   airWx.co   = v;
  if (jsonExtractNumber(body,"nitrogen_dioxide",v))  airWx.no2  = v;
  if (jsonExtractNumber(body,"sulphur_dioxide",v))   airWx.so2  = v;
  if (jsonExtractNumber(body,"ozone",v))             airWx.o3   = v;
  if (jsonExtractNumber(body,"ammonia",v))           airWx.nh3  = v;
  if (jsonExtractNumber(body,"dust",v))              airWx.dust = v;
  if (jsonExtractNumber(body,"aerosol_optical_depth",v)) airWx.aod = v;
  if (jsonExtractNumber(body,"uv_index",v))          airWx.uvAi = v;
  if (jsonExtractNumber(body,"alder_pollen",v))      airWx.alderP = v;
  if (jsonExtractNumber(body,"birch_pollen",v))      airWx.birchP = v;
  if (jsonExtractNumber(body,"grass_pollen",v))      airWx.grassP = v;
  if (jsonExtractNumber(body,"mugwort_pollen",v))    airWx.mugwortP = v;
  if (jsonExtractNumber(body,"olive_pollen",v))      airWx.oliveP = v;
  if (jsonExtractNumber(body,"ragweed_pollen",v))    airWx.ragweedP = v;
  if (jsonExtractNumber(body,"european_aqi",v))      airWx.aqiEu = (int)v;
  if (jsonExtractNumber(body,"us_aqi",v))            airWx.aqiUs = (int)v;
  airWx.aqiEuCat = aqiEuCat(airWx.aqiEu);
  airWx.aqiUsCat = aqiUsCat(airWx.aqiUs);
  airWx.valid = (airWx.aqiEu >= 0 || airWx.aqiUs >= 0 ||
                 !isnan(airWx.pm10) || !isnan(airWx.pm25) || !isnan(airWx.o3));
  lastAqError = "";
  return true;
}

// ---------------- TIME / FORMAT ----------------
void applyTimezone() {
  configTzTime(cfgTz.c_str(), "pool.ntp.org", "time.nist.gov", "time.google.com");
}
String formatLocalTime(time_t t, const char* fmt) {
  if (t < 1600000000) return String("--");
  struct tm tmBuf; localtime_r(&t, &tmBuf);
  char out[32]; strftime(out, sizeof(out), fmt, &tmBuf);
  return String(out);
}
String fmtFloat(float v, int dp) { return isnan(v) ? String("--") : String(v, dp); }
String fmtJsonFloat(float v, int dp) { return isnan(v) ? String("null") : String(v, dp); }

// ---------------- WEB ----------------
void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    if (!staConnected && cfgSsid.length() == 0) {
      server.sendHeader("Location", "/settings", true);
      server.send(302, "text/plain", ""); return;
    }
    String b; b.reserve(12000);
    b += F("<div class='card'><h2>System</h2><table>");
    b += "<tr><td>Firmware</td><td>" + String(FW_VERSION) + "</td></tr>";
    b += "<tr><td>STA SSID</td><td>" + htmlEscape(cfgSsid.length()?cfgSsid:String("(none)")) + "</td></tr>";
    b += "<tr><td>STA IP</td><td>"   + (staConnected ? WiFi.localIP().toString() : String("disconnected")) + "</td></tr>";
    b += "<tr><td>RSSI</td><td>"     + (staConnected ? String(WiFi.RSSI()) + " dBm" : String("-")) + "</td></tr>";
    b += "<tr><td>AP</td><td>"       + String(AP_SSID) + " @ " + WiFi.softAPIP().toString() + "</td></tr>";
    b += "<tr><td>Uptime</td><td>"   + String((millis()-bootMs)/1000) + " s</td></tr>";
    b += "<tr><td>Free heap</td><td>"+ String(ESP.getFreeHeap()) + " bytes</td></tr>";
    time_t now = time(nullptr);
    b += "<tr><td>Time</td><td id='clock'>" + formatLocalTime(now, "%H:%M:%S") + "</td></tr>";
    b += "<tr><td>Date</td><td id='date'>"  + formatLocalTime(now, "%Y-%m-%d") + "</td></tr>";
    b += "<tr><td>Timezone</td><td>" + htmlEscape(cfgTz) + "</td></tr>";
    b += "<tr><td>Lat, Lon</td><td>" + htmlEscape(cfgLat) + ", " + htmlEscape(cfgLon) + "</td></tr>";
    if (lastWxError.length()) b += "<tr><td>WX error</td><td>" + htmlEscape(lastWxError) + "</td></tr>";
    if (lastAqError.length()) b += "<tr><td>AQ error</td><td>" + htmlEscape(lastAqError) + "</td></tr>";
    if (lastWeatherMs > 0) {
      unsigned long ageS = (millis() - lastWeatherMs) / 1000;
      String ageTxt = String(ageS) + " s ago";
      if (ageS > (2 * WEATHER_REFRESH_MS / 1000)) ageTxt += " (STALE)";
      b += "<tr><td>WX updated</td><td>" + ageTxt + "</td></tr>";
    }
    if (lastAirMs > 0) {
      unsigned long ageS2 = (millis() - lastAirMs) / 1000;
      String ageTxt2 = String(ageS2) + " s ago";
      if (ageS2 > (2 * AIR_REFRESH_MS / 1000)) ageTxt2 += " (STALE)";
      b += "<tr><td>AQ updated</td><td>" + ageTxt2 + "</td></tr>";
    }
    b += F("</table></div>");
    if (curWx.valid) {
      b += F("<div class='card'><h2>Current Weather</h2><table>");
      b += "<tr><td>Condition</td><td>" + htmlEscape(curWx.condition) + " (code " + String(curWx.code) + ")</td></tr>";
      b += "<tr><td>Day / Night</td><td>" + String(curWx.isDay==1?"Day":(curWx.isDay==0?"Night":"-")) + "</td></tr>";
      b += "<tr><td>Temperature</td><td>"   + fmtFloat(curWx.tempC,1) + " &deg;C</td></tr>";
      b += "<tr><td>Feels like</td><td>"    + fmtFloat(curWx.feelsC,1) + " &deg;C</td></tr>";
      b += "<tr><td>Dew point</td><td>"     + fmtFloat(curWx.dewC,1) + " &deg;C</td></tr>";
      b += "<tr><td>Wet bulb</td><td>"      + fmtFloat(curWx.wetBulb,1) + " &deg;C</td></tr>";
      b += "<tr><td>Humidity</td><td>"      + fmtFloat(curWx.humidity,0) + " %</td></tr>";
      b += "<tr><td>VPD</td><td>"           + fmtFloat(curWx.vpd,2) + " kPa</td></tr>";
      b += "<tr><td>Cloud cover</td><td>"   + fmtFloat(curWx.cloud,0) + " % (low " + fmtFloat(curWx.cloudLow,0) + ", mid " + fmtFloat(curWx.cloudMid,0) + ", high " + fmtFloat(curWx.cloudHigh,0) + ")</td></tr>";
      b += "<tr><td>Precipitation</td><td>" + fmtFloat(curWx.precip,2) + " mm</td></tr>";
      b += "<tr><td>Rain</td><td>"          + fmtFloat(curWx.rain,2) + " mm</td></tr>";
      b += "<tr><td>Showers</td><td>"       + fmtFloat(curWx.showers,2) + " mm</td></tr>";
      b += "<tr><td>Snowfall</td><td>"      + fmtFloat(curWx.snowfall,2) + " cm</td></tr>";
      b += "<tr><td>Pressure (MSL)</td><td>"     + fmtFloat(curWx.pressureMsl,1) + " hPa</td></tr>";
      b += "<tr><td>Pressure (surface)</td><td>" + fmtFloat(curWx.pressureSurf,1) + " hPa</td></tr>";
      b += "<tr><td>Wind speed</td><td>"     + fmtFloat(curWx.windKph,1) + " km/h</td></tr>";
      b += "<tr><td>Wind gust</td><td>"      + fmtFloat(curWx.windGust,1) + " km/h</td></tr>";
      b += "<tr><td>Wind direction</td><td>" + fmtFloat(curWx.windDir,0) + " &deg;</td></tr>";
      b += "<tr><td>Visibility</td><td>"     + fmtFloat(curWx.visibilityM,0) + " m</td></tr>";
      b += "<tr><td>UV index</td><td>"       + fmtFloat(curWx.uvIndex,1) + " (clear " + fmtFloat(curWx.uvClear,1) + ")</td></tr>";
      b += "<tr><td>Freezing level</td><td>" + fmtFloat(curWx.freezingLevel,0) + " m</td></tr>";
      b += "<tr><td>ET0</td><td>"            + fmtFloat(curWx.et0,2) + " mm</td></tr>";
      b += "<tr><td>Shortwave rad.</td><td>" + fmtFloat(curWx.shortwave,1) + " W/m&sup2;</td></tr>";
      b += "<tr><td>Direct rad.</td><td>"    + fmtFloat(curWx.direct,1) + " W/m&sup2;</td></tr>";
      b += "<tr><td>Diffuse rad.</td><td>"   + fmtFloat(curWx.diffuse,1) + " W/m&sup2;</td></tr>";
      b += "<tr><td>DNI</td><td>"            + fmtFloat(curWx.dni,1) + " W/m&sup2;</td></tr>";
      b += "<tr><td>Soil temp 0/6/18/54 cm</td><td>" +
           fmtFloat(curWx.soil0,1) + " / " + fmtFloat(curWx.soil6,1) + " / " +
           fmtFloat(curWx.soil18,1) + " / " + fmtFloat(curWx.soil54,1) + " &deg;C</td></tr>";
      b += "<tr><td>Soil moist 0-1/1-3/3-9/9-27 cm</td><td>" +
           fmtFloat(curWx.soilMoist0,3) + " / " + fmtFloat(curWx.soilMoist3,3) + " / " +
           fmtFloat(curWx.soilMoist9,3) + " / " + fmtFloat(curWx.soilMoist27,3) + " m&sup3;/m&sup3;</td></tr>";
      b += F("</table></div>");
    }

    if (dayWx.valid) {
      b += F("<div class='card'><h2>Today</h2><table>");
      b += "<tr><td>Condition</td><td>" + htmlEscape(dayWx.conditionDaily) + "</td></tr>";
      b += "<tr><td>Temp max / min</td><td>"  + fmtFloat(dayWx.tMax,1) + " / " + fmtFloat(dayWx.tMin,1) + " &deg;C</td></tr>";
      b += "<tr><td>Feels max / min</td><td>" + fmtFloat(dayWx.aMax,1) + " / " + fmtFloat(dayWx.aMin,1) + " &deg;C</td></tr>";
      b += "<tr><td>Sunrise / Sunset</td><td>" + htmlEscape(dayWx.sunrise) + " / " + htmlEscape(dayWx.sunset) + "</td></tr>";
      b += "<tr><td>Daylight</td><td>" + fmtFloat(dayWx.daylightSec/3600.0f,2) + " h</td></tr>";
      b += "<tr><td>Sunshine</td><td>" + fmtFloat(dayWx.sunshineSec/3600.0f,2) + " h</td></tr>";
      b += "<tr><td>UV max</td><td>"   + fmtFloat(dayWx.uvMax,1) + " (clear " + fmtFloat(dayWx.uvClearMax,1) + ")</td></tr>";
      b += "<tr><td>Precip sum</td><td>"  + fmtFloat(dayWx.precipSum,2) + " mm</td></tr>";
      b += "<tr><td>Rain sum</td><td>"    + fmtFloat(dayWx.rainSum,2) + " mm</td></tr>";
      b += "<tr><td>Showers sum</td><td>" + fmtFloat(dayWx.showersSum,2) + " mm</td></tr>";
      b += "<tr><td>Snow sum</td><td>"    + fmtFloat(dayWx.snowSum,2) + " cm</td></tr>";
      b += "<tr><td>Precip hours</td><td>"       + (dayWx.precipHours>=0?String(dayWx.precipHours):String("--")) + " h</td></tr>";
      b += "<tr><td>Precip probability</td><td>" + (dayWx.precipProbMax>=0?String(dayWx.precipProbMax):String("--")) + " %</td></tr>";
      b += "<tr><td>Wind max / gust</td><td>" + fmtFloat(dayWx.windMax,1) + " / " + fmtFloat(dayWx.gustMax,1) + " km/h</td></tr>";
      b += "<tr><td>Wind dir (dom)</td><td>" + fmtFloat(dayWx.windDirDom,0) + " &deg;</td></tr>";
      b += "<tr><td>Radiation sum</td><td>"  + fmtFloat(dayWx.radiationSum,2) + " MJ/m&sup2;</td></tr>";
      b += "<tr><td>ET0 daily</td><td>"      + fmtFloat(dayWx.et0Daily,2) + " mm</td></tr>";
      b += F("</table></div>");
    }
    if (weekWx.valid) {
      b += F("<div class='card wide'><h2>7-Day Forecast</h2><table>");
      b += F("<tr><th>Date</th><th>Cond</th><th>Max/Min</th><th>Precip</th><th>P%</th><th>Wind</th><th>UV</th></tr>");
      for (int i = 0; i < weekWx.count; i++) {
        b += "<tr>";
        b += "<td>" + htmlEscape(weekWx.date[i]) + "</td>";
        b += "<td>" + htmlEscape(weekWx.cond[i]) + "</td>";
        b += "<td>" + fmtFloat(weekWx.tMax[i],0) + "/" + fmtFloat(weekWx.tMin[i],0) + "&deg;</td>";
        b += "<td>" + fmtFloat(weekWx.precipSum[i],1) + "mm</td>";
        b += "<td>" + (weekWx.precipProb[i] >= 0 ? String(weekWx.precipProb[i]) + "%" : String("--")) + "</td>";
        b += "<td>" + fmtFloat(weekWx.windMax[i],0) + "km/h</td>";
        b += "<td>" + fmtFloat(weekWx.uvMax[i],1) + "</td>";
        b += "</tr>";
      }
      b += F("</table></div>");
    }
    if (airWx.valid) {
      b += F("<div class='card'><h2>Air Quality</h2><table>");
      b += "<tr><td>EU AQI</td><td>" + String(airWx.aqiEu) + " (" + htmlEscape(airWx.aqiEuCat) + ")</td></tr>";
      b += "<tr><td>US AQI</td><td>" + String(airWx.aqiUs) + " (" + htmlEscape(airWx.aqiUsCat) + ")</td></tr>";
      b += "<tr><td>PM1.0</td><td>" + fmtFloat(airWx.pm1,1)  + " &micro;g/m&sup3;</td></tr>";
      b += "<tr><td>PM2.5</td><td>" + fmtFloat(airWx.pm25,1) + " &micro;g/m&sup3;</td></tr>";
      b += "<tr><td>PM10</td><td>"  + fmtFloat(airWx.pm10,1) + " &micro;g/m&sup3;</td></tr>";
      b += "<tr><td>CO</td><td>"    + fmtFloat(airWx.co,1)   + " &micro;g/m&sup3;</td></tr>";
      b += "<tr><td>NO&sub2;</td><td>" + fmtFloat(airWx.no2,1) + " &micro;g/m&sup3;</td></tr>";
      b += "<tr><td>SO&sub2;</td><td>" + fmtFloat(airWx.so2,1) + " &micro;g/m&sup3;</td></tr>";
      b += "<tr><td>O&sub3;</td><td>"  + fmtFloat(airWx.o3,1)  + " &micro;g/m&sup3;</td></tr>";
      b += "<tr><td>NH&sub3;</td><td>" + fmtFloat(airWx.nh3,1) + " &micro;g/m&sup3;</td></tr>";
      b += "<tr><td>Dust</td><td>"  + fmtFloat(airWx.dust,1) + " &micro;g/m&sup3;</td></tr>";
      b += "<tr><td>Aerosol OD</td><td>" + fmtFloat(airWx.aod,3) + "</td></tr>";
      b += "<tr><td>UV (AQ)</td><td>" + fmtFloat(airWx.uvAi,1) + "</td></tr>";
      b += "<tr><td>Pollen (alder/birch/grass)</td><td>" +
           fmtFloat(airWx.alderP,1) + " / " + fmtFloat(airWx.birchP,1) + " / " + fmtFloat(airWx.grassP,1) + "</td></tr>";
      b += "<tr><td>Pollen (mugwort/olive/ragweed)</td><td>" +
           fmtFloat(airWx.mugwortP,1) + " / " + fmtFloat(airWx.oliveP,1) + " / " + fmtFloat(airWx.ragweedP,1) + "</td></tr>";
      b += F("</table></div>");
    }
    b += F("<script>"
           "(function(){"
           "var fails=0;"
           "setInterval(function(){"
           "fetch('/api/status').then(function(r){return r.json();}).then(function(j){"
           "fails=0;"
           "document.getElementById('clock').textContent=j.time;"
           "document.getElementById('date').textContent=j.date;"
           "}).catch(function(){"
           "fails++;"
           "if(fails>3){"
           "document.getElementById('clock').textContent='--:--:--';"
           "document.getElementById('date').textContent='----/--/--';"
           "}"
           "});"
           "},1000);"
           "})();"
           "</script>");
    server.send(200, "text/html", htmlPage(b, "Dashboard"));
  });
  server.on("/settings", HTTP_GET, []() {
    String b;
    b += F("<div class='card'><h2>WiFi Settings</h2><form method='POST' action='/save'>");
    b += "<label>SSID</label><input name='ssid' maxlength='32' value='" + htmlEscape(cfgSsid) + "' required>";
    b += "<label>Password</label><input name='pass' type='password' maxlength='64' value='" + htmlEscape(cfgPass) + "'>";
    b += "<h3>Weather Location</h3>";
    b += "<label>Latitude</label><input name='lat' value='" + htmlEscape(cfgLat) + "' required>";
    b += "<label>Longitude</label><input name='lon' value='" + htmlEscape(cfgLon) + "' required>";
    b += "<label>Timezone</label><input name='tz' value='" + htmlEscape(cfgTz) + "' required>";
    b += F("<button type='submit'>Save &amp; Reconnect</button></form></div>");
    b += F("<div class='card'><h2>Danger Zone</h2>"
           "<form method='POST' action='/factory' onsubmit=\"return confirm('Factory reset?');\">"
           "<button class='danger' type='submit'>Factory Reset</button></form></div>");
    server.send(200, "text/html", htmlPage(b, "Settings"));
  });
  server.on("/save", HTTP_POST, []() {
    String s = server.arg("ssid"); s.trim();
    String p = server.arg("pass"); p.trim();
    String la= server.arg("lat");  la.trim();
    String lo= server.arg("lon");  lo.trim();
    String tz= server.arg("tz");   tz.trim();
    if (s.length() == 0) {
      server.send(400, "text/html", htmlPage("<p>SSID required.</p><a href='/settings'>Back</a>", "Error")); return;
    }
    cfgSsid=s; cfgPass=p;
    if (la.length()) {
      char* endp = nullptr;
      double test = strtod(la.c_str(), &endp);
      if (endp != la.c_str() && *endp == '\0' && test >= -90.0 && test <= 90.0) cfgLat = la;
    }
    if (lo.length()) {
      char* endp = nullptr;
      double test = strtod(lo.c_str(), &endp);
      if (endp != lo.c_str() && *endp == '\0' && test >= -180.0 && test <= 180.0) cfgLon = lo;
    }
    if (tz.length()) cfgTz=tz;
    saveConfig(); applyTimezone();
    server.send(200, "text/html",
      htmlPage("<p>Saved. Reconnecting...</p><script>setTimeout(()=>location='/',4000);</script>", "Saved"));
    delay(400);
    WiFi.disconnect(false, false);
    staConnected=false; portalMode=true;
    startSTA();
    if (staConnected) { fetchWeather(); fetchAirQuality(); lastWeatherMs=millis(); lastAirMs=millis(); }
  });
  server.on("/factory", HTTP_POST, []() {
    server.send(200, "text/html",
      htmlPage("<p>Factory reset. Rebooting...</p><script>setTimeout(()=>location='/',6000);</script>", "Reset"));
    delay(500); clearConfig(); delay(300); ESP.restart();
  });
  server.on("/api/status", HTTP_GET, []() { server.send(200, "application/json", makeStatusJson()); });
  server.on("/generate_204", HTTP_GET, [](){ server.sendHeader("Location","/settings",true); server.send(302,"text/plain",""); });
  server.on("/gen_204",      HTTP_GET, [](){ server.sendHeader("Location","/settings",true); server.send(302,"text/plain",""); });
  server.on("/hotspot-detect.html", HTTP_GET, [](){ server.send(200,"text/html",htmlPage("<p>Captive portal</p>","Portal")); });
  server.on("/connecttest.txt",     HTTP_GET, [](){ server.sendHeader("Location","/settings",true); server.send(302,"text/plain",""); });
  server.on("/ncsi.txt",            HTTP_GET, [](){ server.sendHeader("Location","/settings",true); server.send(302,"text/plain",""); });
  server.onNotFound([]() {
    if (server.uri().startsWith("/api/")) { server.send(404,"application/json","{\"error\":\"not_found\"}"); return; }
    server.sendHeader("Location","/settings",true); server.send(302,"text/plain","");
  });
}
String makeStatusJson() {
  time_t now = time(nullptr);
  String j; j.reserve(512);
  j += "{";
  j += "\"fw\":\""     + String(FW_VERSION) + "\",";
  j += "\"ssid\":\""   + jsonEscape(cfgSsid) + "\",";
  j += "\"sta\":\""    + (staConnected ? WiFi.localIP().toString() : String("")) + "\",";
  j += "\"rssi\":"     + String(staConnected ? WiFi.RSSI() : 0) + ",";
  j += "\"uptime\":"   + String((millis()-bootMs)/1000) + ",";
  j += "\"heap\":"     + String(ESP.getFreeHeap()) + ",";
  j += "\"time\":\""   + formatLocalTime(now, "%H:%M:%S") + "\",";
  j += "\"date\":\""   + formatLocalTime(now, "%Y-%m-%d") + "\",";
  j += "\"tzone\":\""  + jsonEscape(cfgTz) + "\",";
  j += "\"wx_valid\":" + String(curWx.valid ? "true" : "false") + ",";
  j += "\"temp\":"     + fmtJsonFloat(curWx.tempC,1) + ",";
  j += "\"humidity\":" + fmtJsonFloat(curWx.humidity,0) + ",";
  j += "\"wind\":"     + fmtJsonFloat(curWx.windKph,1) + ",";
  j += "\"pressure\":" + fmtJsonFloat(curWx.pressureMsl,1) + ",";
  j += "\"condition\":\"" + jsonEscape(curWx.condition) + "\",";
  j += "\"aq_valid\":" + String(airWx.valid ? "true" : "false") + ",";
  j += "\"aqi_eu\":"   + String(airWx.aqiEu) + ",";
  j += "\"aqi_us\":"   + String(airWx.aqiUs);
  j += "}";
  return j;
}
String htmlEscape(const String& s) {
  String o; o.reserve(s.length()+8);
  for (size_t i=0; i<s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '&': o+=F("&amp;"); break;
      case '<': o+=F("&lt;");  break;
      case '>': o+=F("&gt;");  break;
      case '"': o+=F("&quot;");break;
      case '\'':o+=F("&#39;"); break;
      default:  o+=c;
    }
  }
  return o;
}
String jsonEscape(const String& s) {
  String o; o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '"':  o += F("\\\""); break;
      case '\\': o += F("\\\\"); break;
      case '\n': o += F("\\n");  break;
      case '\r': o += F("\\r");  break;
      case '\t': o += F("\\t");  break;
      default:
        if ((unsigned char)c < 0x20) {
          char buf[8];
          snprintf(buf, sizeof(buf), "\\u%04x", c);
          o += buf;
        } else {
          o += c;
        }
    }
  }
  return o;
}
String urlEncode(const String& s) {
  String o; o.reserve(s.length()*2);
  const char* h = "0123456789ABCDEF";
  for (size_t i=0; i<s.length(); i++) {
    char c = s[i];
    if (isalnum((unsigned char)c)||c=='-'||c=='_'||c=='.'||c=='~') o += c;
    else if (c==' ') o += '+';
    else { o += '%'; o += h[(c>>4)&0xF]; o += h[c&0xF]; }
  }
  return o;
}
String htmlPage(const String& body, const String& title) {
  String p; p.reserve(body.length()+1800);
  p += F("<!DOCTYPE html><html><head><meta charset='utf-8'>");
  p += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  p += "<title>" + htmlEscape(title) + "</title>";
  p += F("<style>"
         "html{-webkit-text-size-adjust:100%}"
         "body{font-family:system-ui,Arial,sans-serif;margin:0;padding:16px;background:#0f172a;color:#e2e8f0}"
         "h1{font-size:1.4rem;margin:0 0 12px}"
         "h2{font-size:1.1rem;margin:0 0 8px;color:#38bdf8}"
         "h3{font-size:1rem;margin:14px 0 6px;color:#a78bfa}"
         ".card{background:#1e293b;border-radius:10px;padding:14px;margin-bottom:14px;box-shadow:0 2px 6px rgba(0,0,0,.4);overflow-x:auto}"
         "table{width:100%;border-collapse:collapse;font-size:.9rem}"
         "th,td{padding:5px 4px;border-bottom:1px solid #334155;vertical-align:top;text-align:left}"
         "th{color:#94a3b8;font-weight:600}"
         "td:first-child,th:first-child{color:#94a3b8;width:42%}"
         ".card.wide table{min-width:640px}"
         "input{width:100%;padding:10px;margin:4px 0 10px;border-radius:6px;border:1px solid #334155;background:#0f172a;color:#e2e8f0;box-sizing:border-box}"
         "button{background:#0ea5e9;color:#fff;border:0;border-radius:6px;padding:10px 16px;font-weight:600;cursor:pointer;width:100%}"
         "button.danger{background:#dc2626}"
         ".nav{margin-bottom:12px}.nav a{color:#38bdf8;margin-right:12px;text-decoration:none;font-weight:600}"
         "@media (min-width:900px){"
         "body{max-width:900px;margin:0 auto;padding:24px}"
         "td:first-child,th:first-child{width:30%}"
         "}"
         "@media (prefers-color-scheme:light){"
         "body{background:#f1f5f9;color:#0f172a}"
         ".card{background:#ffffff;box-shadow:0 2px 6px rgba(0,0,0,.08)}"
         "th,td{border-bottom:1px solid #e2e8f0}"
         "td:first-child,th:first-child{color:#64748b}"
         "h2{color:#0369a1}h3{color:#7c3aed}"
         ".nav a{color:#0369a1}"
         "input{background:#f8fafc;color:#0f172a;border:1px solid #cbd5e1}"
         "}"
         "</style></head><body>");
  p += F("<div class='nav'><a href='/'>Dashboard</a><a href='/settings'>Settings</a></div>");
  p += "<h1>" + htmlEscape(title) + "</h1>";
  p += body;
  p += F("</body></html>");
  return p;
}

// ---------------- BOOT BUTTON ----------------
void handleBootButton() {
  if (millis() - bootMs < 2000) return;
  if (factoryDone) return;
  int level = digitalRead(BOOT_BTN_PIN);
  if (level == LOW) {
    if (!btnHeld) {
      delay(BTN_DEBOUNCE_MS);
      if (digitalRead(BOOT_BTN_PIN) == LOW) { btnHeld = true; btnPressStart = millis(); }
    } else {
      unsigned long held = millis() - btnPressStart;
      static unsigned long lastLog = 0;
      if (millis() - lastLog > 1000) {
        lastLog = millis();
        Serial.printf("[BTN] %lus/%lus\n", held/1000, (unsigned long)(FACTORY_HOLD_MS/1000));
      }
      if (held >= FACTORY_HOLD_MS) {
        factoryDone = true;
        WiFi.softAPdisconnect(true);
        delay(200); clearConfig(); delay(300); ESP.restart();
      }
    }
  } else {
    if (btnHeld) { btnHeld = false; btnPressStart = 0; }
  }
}
