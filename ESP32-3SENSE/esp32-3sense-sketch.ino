/*******************************************************
 * ESP32 Plant Monitor
 * Sensors: Capacitive Soil v2.0 + SHT31 + BH1750
 * Features: Web dashboard, settings, calibration, NVS persistence, captive portal
 *******************************************************/

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <BH1750.h>
#include <ArduinoJson.h>
#include <DNSServer.h>

// ---------- Configuration ----------
#define SOIL_PIN        34        
#define I2C_SDA         21
#define I2C_SCL         22
#define I2C_FREQ        100000    

#define SAMPLE_INTERVAL_MS  2000  
#define SOIL_SAMPLES        16    
#define AP_SSID_PREFIX      "PlantMonitor"
#define BOOT_BTN_PIN        0
#define BOOT_BTN_HOLD_MS    5000

// ---------- Globals ----------
WebServer server(80);
Preferences prefs;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
BH1750 bh1750(0x23);
SemaphoreHandle_t i2cMutex;
bool sht31OK = false;
bool bh1750OK = false;
struct Settings {
  char sta_ssid[33]   = "";
  char sta_pass[65]   = "";
  char ap_ssid[33]    = "";
  char ap_pass[65]    = "";
  uint16_t soilDryMv  = 2500;   
  uint16_t soilWetMv  = 1200;   
  int8_t   moistureLow  = 30;   
  int8_t   moistureHigh = 70;   
  float    tempLow    = 15.0f;
  float    tempHigh   = 30.0f;
  float    humLow     = 30.0f;
  float    humHigh    = 70.0f;
  uint16_t luxLow     = 200;
  uint16_t luxHigh    = 20000;
  uint16_t sampleMs   = SAMPLE_INTERVAL_MS;
} cfg;
struct Readings {
  float soilPct   = NAN;
  uint16_t soilMv = 0;
  float tempC     = NAN;
  float humPct    = NAN;
  float lux       = NAN;
  uint32_t lastMs = 0;
  bool  valid     = false;
} r;
unsigned long lastSample = 0;
bool bootBtnArmed = false;
unsigned long bootBtnPressedAt = 0;
DNSServer dnsServer;
bool apModeActive = false;
bool i2cLock(uint32_t ms = 200) {
  if (i2cMutex == nullptr) return false;
  return xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(ms)) == pdTRUE;
}
void i2cUnlock() { xSemaphoreGive(i2cMutex); }
uint16_t readSoilMillivolts() {
  uint16_t buf[SOIL_SAMPLES];
  for (int i = 0; i < SOIL_SAMPLES; i++) {
    buf[i] = (uint16_t)analogReadMilliVolts(SOIL_PIN);
    delayMicroseconds(500);
  }
  for (int i = 1; i < SOIL_SAMPLES; i++) {
    uint16_t k = buf[i]; int j = i - 1;
    while (j >= 0 && buf[j] > k) { buf[j+1] = buf[j]; j--; }
    buf[j+1] = k;
  }
  return buf[SOIL_SAMPLES / 2]; 
}
float mvToPercent(uint16_t mv) {
  int32_t dry = cfg.soilDryMv;
  int32_t wet = cfg.soilWetMv;
  if (dry == wet) return NAN;                  
  float pct = (float)(dry - (int32_t)mv) * 100.0f / (float)(dry - wet);
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

void sampleAll() {
  r.soilMv  = readSoilMillivolts();
  r.soilPct = mvToPercent(r.soilMv);
  if (sht31OK && i2cLock()) {
    float t = sht31.readTemperature();
    float h = sht31.readHumidity();
    r.tempC = isnan(t) ? NAN : t;
    r.humPct = isnan(h) ? NAN : h;
    i2cUnlock();
  } else {
    r.tempC = NAN;
    r.humPct = NAN;
  }
  if (bh1750OK && i2cLock()) {
    float l = bh1750.readLightLevel();
    r.lux = (isnan(l) || l < 0) ? NAN : l;
    i2cUnlock();
  } else {
    r.lux = NAN;
  }
  r.lastMs = millis();
  r.valid  = !isnan(r.soilPct);
}
void loadSettings() {
  prefs.begin("plant", true);
  prefs.getString("sta_ssid", "").toCharArray(cfg.sta_ssid, sizeof(cfg.sta_ssid));
  prefs.getString("sta_pass", "").toCharArray(cfg.sta_pass, sizeof(cfg.sta_pass));
  prefs.getString("ap_ssid",  "").toCharArray(cfg.ap_ssid,  sizeof(cfg.ap_ssid));
  prefs.getString("ap_pass",  "").toCharArray(cfg.ap_pass,  sizeof(cfg.ap_pass));
  cfg.soilDryMv    = prefs.getUShort("soilDry", 2500);
  cfg.soilWetMv    = prefs.getUShort("soilWet", 1200);
  cfg.moistureLow  = prefs.getChar  ("mLow", 30);
  cfg.moistureHigh = prefs.getChar  ("mHigh", 70);
  cfg.tempLow      = prefs.getFloat ("tLow", 15.0f);
  cfg.tempHigh     = prefs.getFloat ("tHigh", 30.0f);
  cfg.humLow       = prefs.getFloat ("hLow", 30.0f);
  cfg.humHigh      = prefs.getFloat ("hHigh", 70.0f);
  cfg.luxLow       = prefs.getUShort("lLow", 200);
  cfg.luxHigh      = prefs.getUShort("lHigh", 20000);
  cfg.sampleMs     = prefs.getUShort("sMs", SAMPLE_INTERVAL_MS);
  prefs.end();
  if (cfg.sampleMs < 500)   cfg.sampleMs = 500;
  if (cfg.sampleMs > 60000) cfg.sampleMs = 60000;
  if (cfg.soilDryMv < 100 || cfg.soilDryMv > 3300) cfg.soilDryMv = 2500;
  if (cfg.soilWetMv < 100 || cfg.soilWetMv > 3300) cfg.soilWetMv = 1200;
  if (cfg.soilDryMv <= cfg.soilWetMv + 50) {
    int32_t newDry = (int32_t)cfg.soilWetMv + 100;
    if (newDry > 3300) {
      newDry = 3300;
      cfg.soilWetMv = (uint16_t)3200;
    }
    cfg.soilDryMv = (uint16_t)newDry;
  }
  if (cfg.moistureLow  < 0 || cfg.moistureLow  > 100) cfg.moistureLow  = 30;
  if (cfg.moistureHigh < 0 || cfg.moistureHigh > 100) cfg.moistureHigh = 70;
  if (cfg.moistureLow >= cfg.moistureHigh) {
    if (cfg.moistureLow >= 100) cfg.moistureLow = 99;
    cfg.moistureHigh = (int8_t)(cfg.moistureLow + 1);
  }
  if (isnan(cfg.tempLow)  || isinf(cfg.tempLow))  cfg.tempLow  = 15.0f;
  if (isnan(cfg.tempHigh) || isinf(cfg.tempHigh)) cfg.tempHigh = 30.0f;
  if (isnan(cfg.humLow)   || isinf(cfg.humLow))   cfg.humLow   = 30.0f;
  if (isnan(cfg.humHigh)  || isinf(cfg.humHigh))  cfg.humHigh  = 70.0f;
  if (cfg.tempLow >= cfg.tempHigh) cfg.tempHigh = cfg.tempLow + 1;
  if (cfg.humLow  >= cfg.humHigh)  cfg.humHigh  = cfg.humLow  + 1;
  if (cfg.luxLow >= cfg.luxHigh) {
    if (cfg.luxLow >= 65535) cfg.luxLow = 65534;
    cfg.luxHigh = (uint16_t)(cfg.luxLow + 1);
  }
  if (strlen(cfg.ap_ssid) == 0) {
    snprintf(cfg.ap_ssid, sizeof(cfg.ap_ssid), "%s-%04X",
             AP_SSID_PREFIX, (uint16_t)(ESP.getEfuseMac() & 0xFFFF));
  }
}
void saveSettings() {
  prefs.begin("plant", false);
  prefs.putString("sta_ssid", cfg.sta_ssid);
  prefs.putString("sta_pass", cfg.sta_pass);
  prefs.putString("ap_ssid",  cfg.ap_ssid);
  prefs.putString("ap_pass",  cfg.ap_pass);
  prefs.putUShort("soilDry",  cfg.soilDryMv);
  prefs.putUShort("soilWet",  cfg.soilWetMv);
  prefs.putChar  ("mLow",  cfg.moistureLow);
  prefs.putChar  ("mHigh", cfg.moistureHigh);
  prefs.putFloat ("tLow",  cfg.tempLow);
  prefs.putFloat ("tHigh", cfg.tempHigh);
  prefs.putFloat ("hLow",  cfg.humLow);
  prefs.putFloat ("hHigh", cfg.humHigh);
  prefs.putUShort("lLow",  cfg.luxLow);
  prefs.putUShort("lHigh", cfg.luxHigh);
  prefs.putUShort("sMs",   cfg.sampleMs);
  prefs.end();
}

// ---------- Web UI ----------
const char PAGE_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Plant Monitor</title>
<style>
 body{font-family:system-ui,Arial,sans-serif;margin:0;background:#111;color:#eee}
 .wrap{max-width:820px;margin:0 auto;padding:16px}
 h1{font-size:1.4rem;margin:8px 0 16px}
 .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px}
 .card{background:#1c1c1c;border:1px solid #333;border-radius:12px;padding:16px}
 .val{font-size:2rem;font-weight:700}
 .lbl{opacity:.7;font-size:.85rem}
 .ok{color:#4ade80}.warn{color:#facc15}.bad{color:#f87171}.na{color:#888}
 .bar{height:8px;background:#333;border-radius:4px;overflow:hidden;margin-top:8px}
 .bar>i{display:block;height:100%;background:#4ade80}
 details{margin-top:16px;background:#1c1c1c;border:1px solid #333;border-radius:12px;padding:12px}
 summary{cursor:pointer;font-weight:600}
 label{display:block;margin:10px 0 4px;font-size:.85rem;opacity:.8}
 input,select{width:100%;padding:8px;background:#111;color:#eee;border:1px solid #444;border-radius:6px}
 button{margin-top:12px;padding:10px 16px;background:#2563eb;color:#fff;border:0;border-radius:8px;cursor:pointer}
 button.ghost{background:#333}
 .row{display:flex;gap:8px;flex-wrap:wrap}
 .row>*{flex:1;min-width:120px}
</style></head><body><div class="wrap">
<h1>🌱 Plant Monitor</h1>
<div class="grid" id="cards"></div>

<details><summary>⚙️ Settings</summary>
 <form id="cfg">
  <label>Soil calibration — DRY reading (mV, in air)</label>
  <div class="row"><input name="soilDry" type="number" min="100" max="3300">
   <button type="button" class="ghost" onclick="snap('soilDry')">Use current</button></div>

  <label>Soil calibration — WET reading (mV, in water)</label>
  <div class="row"><input name="soilWet" type="number" min="100" max="3300">
   <button type="button" class="ghost" onclick="snap('soilWet')">Use current</button></div>

  <div class="row">
   <div><label>Moisture low (%)</label><input name="moistureLow" type="number" min="0" max="100"></div>
   <div><label>Moisture high (%)</label><input name="moistureHigh" type="number" min="0" max="100"></div>
  </div>
  <div class="row">
   <div><label>Temp low (°C)</label><input name="tempLow" type="number" step="0.1"></div>
   <div><label>Temp high (°C)</label><input name="tempHigh" type="number" step="0.1"></div>
  </div>
  <div class="row">
   <div><label>Humidity low (%)</label><input name="humLow" type="number" step="0.1"></div>
   <div><label>Humidity high (%)</label><input name="humHigh" type="number" step="0.1"></div>
  </div>
  <div class="row">
   <div><label>Light low (lux)</label><input name="luxLow" type="number"></div>
   <div><label>Light high (lux)</label><input name="luxHigh" type="number"></div>
  </div>
  <div class="row">
   <div><label>Sample interval (ms)</label><input name="sampleMs" type="number" min="500" max="60000"></div>
  </div>

  <label>WiFi SSID (leave blank for AP-only mode)</label>
  <input name="sta_ssid" maxlength="32">
  <label>WiFi Password</label>
  <input name="sta_pass" type="password" maxlength="64">
  <label>AP SSID</label><input name="ap_ssid" maxlength="32">
  <label>AP Password (min 8 chars, blank = open)</label>
  <input name="ap_pass" type="password" maxlength="64">

  <div class="row">
   <button type="submit">Save &amp; Reboot</button>
   <button type="button" class="ghost" onclick="if(confirm('Reset all to defaults?'))fetch('/reset',{method:'POST'}).then(()=>location.reload())">Factory Reset</button>
  </div>
 </form>
</details>
</div>

<script>
const fields=["soilDry","soilWet","moistureLow","moistureHigh","tempLow","tempHigh","humLow","humHigh","luxLow","luxHigh","sampleMs","sta_ssid","ap_ssid"];
function cls(v,lo,hi,inv){
  if(v===null||v===undefined||isNaN(v))return "na";
  if(lo!==null&&v<lo)return inv?"warn":"bad";
  if(hi!==null&&v>hi)return inv?"warn":"bad";
  return "ok";
}
function card(lbl,val,sub,c){
  return `<div class="card"><div class="lbl">${lbl}</div><div class="val ${c}">${val}</div><div class="lbl">${sub||''}</div></div>`;
}
function render(d){
  const c=d.cfg, m=d.readings, st=d.status;
  const cards=[];
  cards.push(card("Soil moisture",
     m.soilPct==null?"—":m.soilPct.toFixed(1)+"%",
     m.soilMv+" mV • "+st.soil, cls(m.soilPct,c.moistureLow,c.moistureHigh,false)));
  cards.push(card("Temperature",
     m.tempC==null?"—":m.tempC.toFixed(1)+"°C",
     st.sht31, cls(m.tempC,c.tempLow,c.tempHigh,false)));
  cards.push(card("Humidity",
     m.humPct==null?"—":m.humPct.toFixed(1)+"%",
     st.sht31, cls(m.humPct,c.humLow,c.humHigh,false)));
  cards.push(card("Light",
     m.lux==null?"—":m.lux.toFixed(0)+" lx",
     st.bh1750, cls(m.lux,c.luxLow,c.luxHigh,false)));
  document.getElementById("cards").innerHTML=cards.join("");
}
function fill(c){
  for(const f of fields){
    const el=document.querySelector(`[name="${f}"]`);
    if(!el)continue;
    if(document.activeElement===el)continue;
    if(c[f]!==undefined)el.value=c[f];
  }
}
async function poll(){
  try{
    const j=await (await fetch('/api')).json();
    render(j); fill(j.cfg);
  }catch(e){}
  setTimeout(poll,2000);
}
function snap(name){
  fetch('/api').then(r=>r.json()).then(j=>{
    const el=document.querySelector(`[name="${name}"]`);
    if(el && j && j.readings && j.readings.soilMv !== undefined && j.readings.soilMv !== null)
      el.value=j.readings.soilMv;
  }).catch(()=>{});
}
document.getElementById("cfg").addEventListener("submit",async e=>{
  e.preventDefault();
  const fd=new FormData(e.target), o={};
  fd.forEach((v,k)=>o[k]=v);
  try{
    await fetch('/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(o)});
    alert("Saved. Rebooting…");
  }catch(err){
    alert("Save failed (device may be rebooting).");
  }
});
poll();
</script></body></html>)HTML";

void sendJson() {
  StaticJsonDocument<1024> doc;
  JsonObject cfgO = doc.createNestedObject("cfg");
  cfgO["soilDry"]      = cfg.soilDryMv;
  cfgO["soilWet"]      = cfg.soilWetMv;
  cfgO["moistureLow"]  = cfg.moistureLow;
  cfgO["moistureHigh"] = cfg.moistureHigh;
  cfgO["tempLow"]      = cfg.tempLow;
  cfgO["tempHigh"]     = cfg.tempHigh;
  cfgO["humLow"]       = cfg.humLow;
  cfgO["humHigh"]      = cfg.humHigh;
  cfgO["luxLow"]       = cfg.luxLow;
  cfgO["luxHigh"]      = cfg.luxHigh;
  cfgO["sampleMs"]     = cfg.sampleMs;
  cfgO["sta_ssid"]     = cfg.sta_ssid;
  cfgO["ap_ssid"]      = cfg.ap_ssid;
  JsonObject ro = doc.createNestedObject("readings");
  if (isnan(r.soilPct)) ro["soilPct"] = nullptr; else ro["soilPct"] = r.soilPct;
  ro["soilMv"] = r.soilMv;
  if (isnan(r.tempC)) ro["tempC"] = nullptr; else ro["tempC"] = r.tempC;
  if (isnan(r.humPct)) ro["humPct"] = nullptr; else ro["humPct"] = r.humPct;
  if (isnan(r.lux)) ro["lux"] = nullptr; else ro["lux"] = r.lux;
  JsonObject so = doc.createNestedObject("status");
  so["sht31"]  = sht31OK  ? "OK" : "not found";
  so["bh1750"] = bh1750OK ? "OK" : "not found";
  so["soil"]   = r.valid  ? "OK" : "uncalibrated";
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}
void handleRoot() { server.send_P(200, "text/html", PAGE_HTML); }
void handleCaptiveRedirect() {
  String url = "http://" + WiFi.softAPIP().toString() + "/";
  server.sendHeader("Location", url, true);
  server.send(302, "text/plain", "");
}
void handleGenerate204() {
  String url = "http://" + WiFi.softAPIP().toString() + "/";
  server.sendHeader("Location", url, true);
  server.send(302, "text/plain", "");
}
void handleAppleProbe() {
  String url = "http://" + WiFi.softAPIP().toString() + "/";
  server.sendHeader("Location", url, true);
  server.send(302, "text/plain", "");
}
void handleWindowsProbe() {
  String url = "http://" + WiFi.softAPIP().toString() + "/";
  server.sendHeader("Location", url, true);
  server.send(302, "text/plain", "");
}
void handleSave() {
  if (!server.hasArg("plain")) { server.send(400, "text/plain", "no body"); return; }
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) { server.send(400, "text/plain", "bad json"); return; }
  auto setSoil = [&](const char* k, uint16_t &v){
    if (doc.containsKey(k)) {
      long x = doc[k].as<long>();
      v = (uint16_t)constrain(x, 100, 3300);
    }
  };
  auto setLux = [&](const char* k, uint16_t &v){
    if (doc.containsKey(k)) {
      long x = doc[k].as<long>();
      v = (uint16_t)constrain(x, 0, 65535);
    }
  };
  auto setPct = [&](const char* k, int8_t &v){
    if (doc.containsKey(k)) {
      long x = doc[k].as<long>();
      v = (int8_t)constrain(x, 0, 100);
    }
  };
  auto setF = [&](const char* k, float &v){
    if (doc.containsKey(k)) {
      if (doc[k].isNull()) return;
      if (doc[k].is<const char*>() && strlen(doc[k].as<const char*>()) == 0) return;
      float x = doc[k].as<float>();
      if (isnan(x) || isinf(x)) return;
      v = x;
    }
  };
  auto setS = [&](const char* k, char* dst, size_t n, bool skipEmpty){
    if (doc.containsKey(k)) {
      String s = doc[k].as<String>();
      if (skipEmpty && s.length() == 0) return;
      s.toCharArray(dst, n);
    }
  };
  setSoil("soilDry", cfg.soilDryMv);
  setSoil("soilWet", cfg.soilWetMv);
  setPct ("moistureLow",  cfg.moistureLow);
  setPct ("moistureHigh", cfg.moistureHigh);
  setF  ("tempLow",  cfg.tempLow);
  setF  ("tempHigh", cfg.tempHigh);
  setF  ("humLow",   cfg.humLow);
  setF  ("humHigh",  cfg.humHigh);
  setLux("luxLow",  cfg.luxLow);
  setLux("luxHigh", cfg.luxHigh);
  if (doc.containsKey("sampleMs")) {
    long x = doc["sampleMs"].as<long>();
    cfg.sampleMs = (uint16_t)constrain(x, 500, 60000);
  }
  setS("sta_ssid", cfg.sta_ssid, sizeof(cfg.sta_ssid), false);
  setS("sta_pass", cfg.sta_pass, sizeof(cfg.sta_pass), true);
  setS("ap_ssid",  cfg.ap_ssid,  sizeof(cfg.ap_ssid),  false);
  setS("ap_pass",  cfg.ap_pass,  sizeof(cfg.ap_pass),  true);
  if (cfg.soilDryMv <= cfg.soilWetMv + 50) {
    int32_t newDry = (int32_t)cfg.soilWetMv + 100;
    if (newDry > 3300) {
      newDry = 3300;
      cfg.soilWetMv = (uint16_t)3200;
    }
    cfg.soilDryMv = (uint16_t)newDry;
  }
  if (cfg.moistureLow >= cfg.moistureHigh) {
    if (cfg.moistureLow >= 100) cfg.moistureLow = 99;
    cfg.moistureHigh = (int8_t)(cfg.moistureLow + 1);
  }
  saveSettings();
  server.send(200, "text/plain", "ok");
  delay(300);
  ESP.restart();
}
void performFactoryReset() {
  prefs.begin("plant", false);
  prefs.clear();
  prefs.end();
  delay(100);
  ESP.restart();
}
void handleReset() {
  server.send(200, "text/plain", "ok");
  delay(300);
  performFactoryReset();
}
void checkBootButton() {
  bool pressed = (digitalRead(BOOT_BTN_PIN) == LOW);
  if (!bootBtnArmed) {
    if (!pressed) bootBtnArmed = true;
    return;
  }
  if (pressed) {
    if (bootBtnPressedAt == 0) {
      bootBtnPressedAt = millis();
    } else if (millis() - bootBtnPressedAt >= BOOT_BTN_HOLD_MS) {
      delay(50);
      performFactoryReset();
    }
  } else {
    if (bootBtnPressedAt != 0) {
      bootBtnPressedAt = 0;
    }
  }
}
void startNetwork() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  if (strlen(cfg.sta_ssid) > 0) {
    WiFi.begin(cfg.sta_ssid, cfg.sta_pass);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) delay(200);
  }
  if (WiFi.status() == WL_CONNECTED) {
    apModeActive = false;
    return;
  }
  WiFi.mode(WIFI_AP);
  if (strlen(cfg.ap_ssid) == 0) {
    snprintf(cfg.ap_ssid, sizeof(cfg.ap_ssid), "%s-%04X",
             AP_SSID_PREFIX, (uint16_t)(ESP.getEfuseMac() & 0xFFFF));
  }
  if (strlen(cfg.ap_pass) >= 8)
    WiFi.softAP(cfg.ap_ssid, cfg.ap_pass);
  else
    WiFi.softAP(cfg.ap_ssid);
  apModeActive = true;
  dnsServer.start(53, "*", WiFi.softAPIP());
}
void setup() {
  delay(300);
  i2cMutex = xSemaphoreCreateMutex();
  pinMode(BOOT_BTN_PIN, INPUT_PULLUP);
  analogSetPinAttenuation(SOIL_PIN, ADC_11db);
  loadSettings();
  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  Wire.setTimeOut(50);
  if (sht31.begin(0x44) && !isnan(sht31.readTemperature())) {
    sht31OK = true;
  } else if (sht31.begin(0x45) && !isnan(sht31.readTemperature())) {
    sht31OK = true;
  } else {
  }
  if (bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire)) {
    bh1750OK = true;
  } else if (bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x5C, &Wire)) {
    bh1750OK = true;
  } else {
  }
  startNetwork();
  server.on("/",       HTTP_GET,  handleRoot);
  server.on("/api",    HTTP_GET,  sendJson);
  server.on("/save",   HTTP_POST, handleSave);
  server.on("/reset",  HTTP_POST, handleReset);
  server.on("/generate_204",        HTTP_GET, handleGenerate204);
  server.on("/gen_204",             HTTP_GET, handleGenerate204);
  server.on("/hotspot-detect.html", HTTP_GET, handleAppleProbe);
  server.on("/library/test/success.html", HTTP_GET, handleAppleProbe);
  server.on("/connecttest.txt",     HTTP_GET, handleWindowsProbe);
  server.on("/ncsi.txt",            HTTP_GET, handleWindowsProbe);
  server.on("/redirect",            HTTP_GET, handleCaptiveRedirect);
  server.on("/canonical.html",      HTTP_GET, handleCaptiveRedirect);
  server.on("/success.txt",         HTTP_GET, handleCaptiveRedirect);
  server.onNotFound([](){
    if (apModeActive) {
      String url = "http://" + WiFi.softAPIP().toString() + "/";
      server.sendHeader("Location", url, true);
      server.send(302, "text/plain", "");
    } else {
      server.send(404, "text/plain", "not found");
    }
  });
  server.begin();
  sampleAll(); 
}
void loop() {
  server.handleClient();
  if (apModeActive) dnsServer.processNextRequest();
  checkBootButton();
  unsigned long now = millis();
  if (now - lastSample >= cfg.sampleMs) {
    lastSample = now;
    sampleAll();
  }
}
