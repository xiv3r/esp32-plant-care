/*
===============================================================================
 *  ESP32 Plant Care — DHT22 + Soil Moisture Sensor
 *  Author: Raff Alds
 *  Github: https://www.github.com/xiv3r
 *  License: MIT
===============================================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <DHT.h>
#include <DNSServer.h>

// -----------------------------
// Pin definitions
// -----------------------------
#define SOIL_PIN   34         
#define DHT_PIN    4
#define DHT_TYPE   DHT22

#define RELAY_PIN  26
#define RELAY_ON   HIGH
#define RELAY_OFF  LOW

// -----------------------------
// AP
// -----------------------------
#define AP_SSID      "ESP32_Plant_Care"
#define AP_PASSWORD  "12345678"

// -----------------------------
// Settings
// -----------------------------
struct Config {
  int32_t  soilDry;          
  int32_t  soilHyst;         
  float    tempLimit;       
  uint32_t readIntervalMs;   
};

Config cfg = { 2500, 150, 35.0f, 2500 };   

#define DRY_MIN   600
#define DRY_MAX   3400
#define HYST_MIN  20
#define HYST_MAX  500
#define TEMP_MIN  10.0f
#define TEMP_MAX  80.0f
#define INT_MIN   2000u      
#define INT_MAX   60000u

// -----------------------------
// State
// -----------------------------
DHT dht(DHT_PIN, DHT_TYPE);
WebServer server(80);
Preferences prefs;
DNSServer dnsServer;

bool     motorStatus  = false;
int      lastSoil     = -1;         
float    lastTemp     = NAN;
float    lastHum      = NAN;
bool     faultSoil    = false;
bool     faultDht     = false;
uint8_t  overrideMode = 0;         
uint32_t lastReadMs   = 0;
bool     apMode       = true;

static char txBuf[512];             

// -----------------------------
// Dashboard
// -----------------------------
static const char INDEX_HTML[] PROGMEM = R"rawliteral(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Plant Guardian</title><style>
:root{--bg:#0f172a;--card:#1e293b;--txt:#e2e8f0;--mut:#94a3b8;--ok:#22c55e;--warn:#f59e0b;--bad:#ef4444;--btn:#334155}
*{box-sizing:border-box;margin:0;font-family:system-ui,Segoe UI,Roboto,sans-serif}
body{background:var(--bg);color:var(--txt);padding:16px;max-width:760px;margin:auto}
h1{font-size:20px;display:flex;align-items:center;gap:8px}
#dot{width:10px;height:10px;border-radius:50%;background:var(--bad);display:inline-block}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px;margin:14px 0}
.card{background:var(--card);border-radius:12px;padding:14px;margin-top:10px}
.grid .card{margin-top:0}
.card h2{font-size:12px;color:var(--mut);text-transform:uppercase;letter-spacing:.06em}
.big{font-size:26px;font-weight:700;margin-top:6px}
.bar{height:8px;background:#0b1220;border-radius:6px;margin-top:8px;overflow:hidden}
.bar i{display:block;height:100%;width:0;background:var(--ok)}
.banner{display:none;background:var(--bad);color:#fff;border-radius:10px;padding:10px;margin:8px 0;font-size:14px}
.warn{background:var(--warn);color:#221}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:8px 0}
button{background:var(--btn);color:var(--txt);border:0;border-radius:8px;padding:9px 14px;font-size:14px;cursor:pointer}
button.on{background:var(--ok);color:#04120a;font-weight:700}
label{font-size:13px;color:var(--mut)}
input{background:#0b1220;border:1px solid #334155;color:var(--txt);border-radius:8px;padding:8px;width:110px;font-size:14px}
#msg{font-size:13px;min-height:16px}
small{color:var(--mut)}
.pill{padding:4px 10px;border-radius:999px;font-weight:700;font-size:13px;background:#475569}
</style></head><body>
<h1>&#127793; Plant Guardian <span id="dot"></span> <small id="net">connecting&hellip;</small></h1>
<div class="banner" id="bSoil">&#9888; Soil sensor fault (reading out of range) &mdash; pump locked OFF</div>
<div class="banner warn" id="bDht">&#9888; DHT22 read failed &mdash; running on soil-only logic this cycle</div>
<div class="banner warn" id="bMan">&#9888; Manual override active</div>
<div class="grid">
<div class="card"><h2>Soil (raw, higher=drier)</h2><div class="big" id="soilV">&mdash;</div><div class="bar"><i id="soilB"></i></div></div>
<div class="card"><h2>Temperature</h2><div class="big" id="tempV">&mdash;</div></div>
<div class="card"><h2>Humidity</h2><div class="big" id="humV">&mdash;</div></div>
<div class="card"><h2>Pump</h2><div class="big"><span class="pill" id="pumpV">&mdash;</span></div></div>
</div>
<div class="card"><h2>Pump mode</h2>
<div class="row"><button id="ov0" onclick="ovr(0)">Auto</button><button id="ov1" onclick="ovr(1)">Force OFF</button><button id="ov2" onclick="ovr(2)">Force ON</button></div>
<small>Force ON is blocked only by a soil-sensor fault. Mode is held in RAM and resets to Auto on reboot.</small></div>
<div class="card"><h2>Settings (saved to flash)</h2>
<form onsubmit="return false">
<div class="row"><label>Dry threshold <input id="iDry" type="number" min="600" max="3400"></label>
<label>Hysteresis <input id="iHyst" type="number" min="20" max="500"></label></div>
<div class="row"><label>Heat trigger &deg;C <input id="iTemp" type="number" step="0.5" min="10" max="80"></label>
<label>Read interval ms <input id="iInt" type="number" step="100" min="2000" max="60000"></label></div>
<div class="row"><button onclick="save()">Save &amp; apply</button><span id="msg"></span></div>
</form>
<small>ON when soil &gt; dry+hyst (or hot &amp; soil &gt; dry). OFF when soil &lt; dry&minus;hyst. Between: keeps state.</small></div>
<div class="card"><h2>System &mdash; heap fragmentation monitor</h2>
<div class="grid" style="margin:8px 0 0"><div><small>Free heap</small><div id="heapV">&mdash;</div></div>
<div><small>Min heap (watermark)</small><div id="heapMinV">&mdash;</div></div>
<div><small>Largest free block</small><div id="heapMaxV">&mdash;</div></div></div>
<div class="grid" style="margin-top:8px"><div><small>WiFi RSSI</small><div id="rssiV">&mdash;</div></div>
<div><small>IP</small><div id="ipV">&mdash;</div></div>
<div><small>Uptime</small><div id="upV">&mdash;</div></div></div>
<small>Healthy = values flat over days. Largest free block shrinking while free heap stays high = fragmentation.</small></div>
<script>
const $=i=>document.getElementById(i);
let dirty=false;
['iDry','iHyst','iTemp','iInt'].forEach(k=>$(k).addEventListener('input',()=>dirty=true));
const fmtK=v=>(v/1024).toFixed(1)+' KB';
function fmtUp(s){const d=Math.floor(s/86400),h=Math.floor(s%86400/3600),m=Math.floor(s%3600/60);return(d?d+'d ':'')+h+'h '+m+'m';}
function render(j){
 $('dot').style.background='var(--ok)';$('net').textContent='live';
 $('soilV').textContent=j.soil<0?'\u2014':j.soil;
 $('soilB').style.width=j.soil<0?0:Math.min(100,j.soil/40.95)+'%';
 $('soilB').style.background=j.soil>j.soilDry+j.soilHyst?'var(--bad)':(j.soil<j.soilDry-j.soilHyst?'var(--ok)':'var(--warn)');
 $('tempV').textContent=j.temp==null?'\u2014':j.temp.toFixed(1)+' \u00B0C';
 $('humV').textContent=j.hum==null?'\u2014':j.hum.toFixed(1)+' %';
 const p=$('pumpV');p.textContent=j.motor?'RUNNING':'OFF';
 p.style.background=j.motor?'var(--ok)':'#475569';p.style.color=j.motor?'#04120a':'#e2e8f0';
 $('bSoil').style.display=j.faultSoil?'block':'none';
 $('bDht').style.display=j.faultDht?'block':'none';
 $('bMan').style.display=j.ovr?'block':'none';
 [0,1,2].forEach(m=>$('ov'+m).classList.toggle('on',j.ovr===m));
 $('heapV').textContent=fmtK(j.heap);$('heapMinV').textContent=fmtK(j.heapMin);$('heapMaxV').textContent=fmtK(j.heapMax);
 $('rssiV').textContent=j.rssi?j.rssi+' dBm':'\u2014';
 $('ipV').textContent=j.ip;$('upV').textContent=fmtUp(j.up);
 if(!dirty){$('iDry').value=j.soilDry;$('iHyst').value=j.soilHyst;$('iTemp').value=j.tempLimit;$('iInt').value=j.interval;}
}
async function tick(){try{const r=await fetch('/api/status');render(await r.json());}
catch(e){$('dot').style.background='var(--bad)';$('net').textContent='connection lost - retrying';}}
async function save(){try{
 const b=new URLSearchParams({soilDry:$('iDry').value,soilHyst:$('iHyst').value,tempLimit:$('iTemp').value,interval:$('iInt').value});
 const r=await fetch('/api/settings',{method:'POST',body:b});const j=await r.json();
 $('msg').style.color=r.ok?'var(--ok)':'var(--bad)';
 $('msg').textContent=r.ok?'Saved \u2713':('Rejected: '+j.err);
 if(r.ok)dirty=false;tick();}catch(e){$('msg').textContent='Network error';}}
async function ovr(m){try{await fetch('/api/override',{method:'POST',body:new URLSearchParams({mode:m})});tick();}catch(e){}}
tick();setInterval(tick,2000);
</script></body></html>)rawliteral";

// -----------------------------
// Validation
// -----------------------------
static bool validateConfig(const Config& c, char* err, size_t errlen) {
  if (c.soilDry < DRY_MIN || c.soilDry > DRY_MAX) {
    snprintf(err, errlen, "soilDry must be %d..%d", DRY_MIN, DRY_MAX); return false;
  }
  if (c.soilHyst < HYST_MIN || c.soilHyst > HYST_MAX) {
    snprintf(err, errlen, "soilHyst must be %d..%d", HYST_MIN, HYST_MAX); return false;
  }
  if (c.soilDry - c.soilHyst <= 100 || c.soilDry + c.soilHyst >= 3950) {
    snprintf(err, errlen, "dry/hyst band must stay inside 100..3950"); return false;
  }
  if (!isfinite(c.tempLimit)) {
    snprintf(err, errlen, "tempLimit must be a finite number"); return false;
  }
  if (c.tempLimit < TEMP_MIN || c.tempLimit > TEMP_MAX) {
    snprintf(err, errlen, "tempLimit must be 10..80"); return false;
  }
  if (c.readIntervalMs < INT_MIN || c.readIntervalMs > INT_MAX) {
    snprintf(err, errlen, "interval must be 2000..60000 ms"); return false;
  }
  return true;
}

// -----------------------------
// NVS persistence
// -----------------------------
void loadCfg() {
  const Config dflt = { 2500, 150, 35.0f, 2500 };
  prefs.begin("plant", true);
  cfg.soilDry        = prefs.getInt("soilDry",  dflt.soilDry);
  cfg.soilHyst       = prefs.getInt("soilHyst", dflt.soilHyst);
  cfg.tempLimit      = prefs.getFloat("tempLim", dflt.tempLimit);
  cfg.readIntervalMs = prefs.getUInt("interval", dflt.readIntervalMs);
  prefs.end();
  char err[96];
  if (!validateConfig(cfg, err, sizeof(err))) {
    cfg = dflt;
  }
}

void saveCfg() {
  prefs.begin("plant", false);
  prefs.putInt("soilDry",  cfg.soilDry);
  prefs.putInt("soilHyst", cfg.soilHyst);
  prefs.putFloat("tempLim", cfg.tempLimit);
  prefs.putUInt("interval", cfg.readIntervalMs);
  prefs.end();
}

// -----------------------------
// Pump control
// -----------------------------
void applyControl() {
  bool wantOn;
  if (lastSoil < 0 || faultSoil) {
    wantOn = false;
  } else if (overrideMode == 1) {
    wantOn = false;
  } else if (overrideMode == 2) {
    wantOn = true;
  } else {
    bool soilDryNow = (lastSoil > cfg.soilDry + cfg.soilHyst);
    bool soilWetNow = (lastSoil < cfg.soilDry - cfg.soilHyst);
    bool hotDay     = (!faultDht && lastTemp > cfg.tempLimit);
    if      (soilDryNow || (hotDay && lastSoil > cfg.soilDry)) wantOn = true;
    else if (soilWetNow)                                       wantOn = false;
    else                                                       wantOn = motorStatus;
  }
  if (wantOn != motorStatus) {
    motorStatus = wantOn;
    digitalWrite(RELAY_PIN, wantOn ? RELAY_ON : RELAY_OFF);
  }
}

void sampleSensors() {
  lastReadMs = millis();
  lastSoil   = analogRead(SOIL_PIN);
  faultSoil  = (lastSoil < 50 || lastSoil > 4000);

  float t = dht.readTemperature();
  float h = dht.readHumidity();
  faultDht = (isnan(t) || isnan(h) || isinf(t) || isinf(h));
  if (!faultDht) { lastTemp = t; lastHum = h; }

  applyControl();
}

// -----------------------------
// HTTP handlers
// -----------------------------
void handleRoot() {
  server.sendHeader(F("Captive-Portal"), F("http://192.168.4.1/"));
  server.sendHeader(F("Cache-Control"),   F("no-cache"));
  server.send_P(200, PSTR("text/html"), INDEX_HTML);
}

void handleStatus() {
  char tBuf[12], hBuf[12], ip[16];
  if (isnan(lastTemp)) strcpy(tBuf, "null");
  else snprintf(tBuf, sizeof(tBuf), "%.1f", (double)lastTemp);
  if (isnan(lastHum))  strcpy(hBuf, "null");
  else snprintf(hBuf, sizeof(hBuf), "%.1f", (double)lastHum);

  IPAddress a = WiFi.softAPIP();
  snprintf(ip, sizeof(ip), "%u.%u.%u.%u", a[0], a[1], a[2], a[3]);

  snprintf(txBuf, sizeof(txBuf),
    "{\"soil\":%d,\"temp\":%s,\"hum\":%s,\"motor\":%d,\"ovr\":%u,"
    "\"faultSoil\":%d,\"faultDht\":%d,"
    "\"soilDry\":%d,\"soilHyst\":%d,\"tempLimit\":%.1f,\"interval\":%u,"
    "\"rssi\":%d,\"ip\":\"%s\",\"heap\":%u,\"heapMin\":%u,\"heapMax\":%u,\"up\":%lu}",
    lastSoil, tBuf, hBuf, motorStatus ? 1 : 0, (unsigned)overrideMode,
    faultSoil ? 1 : 0, faultDht ? 1 : 0,
    (int)cfg.soilDry, (int)cfg.soilHyst, (double)cfg.tempLimit, (unsigned)cfg.readIntervalMs,
    0, ip,
    (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
    (unsigned)ESP.getMaxAllocHeap(),
    (unsigned long)(millis() / 1000UL));
  server.sendHeader(F("Cache-Control"), F("no-cache"));
  server.send(200, "application/json", txBuf);
}

static bool parseLongField(const String& s, long& out) {
  if (s.length() == 0) return false;
  char* end = nullptr;
  out = strtol(s.c_str(), &end, 10);
  return (end != s.c_str() && *end == '\0');
}
static bool parseFloatField(const String& s, float& out) {
  if (s.length() == 0) return false;
  char* end = nullptr;
  out = strtof(s.c_str(), &end);
  return (end != s.c_str() && *end == '\0');
}

static void sendErr(int code, const char* msg) {
  snprintf(txBuf, sizeof(txBuf), "{\"ok\":false,\"err\":\"%s\"}", msg);
  server.send(code, "application/json", txBuf);
}

void handleSettings() {
  Config c   = cfg;
  char err[96] = "";
  long l; float f;

  if (!server.hasArg("soilDry"))           strcpy(err, "missing soilDry");
  else if (!parseLongField(server.arg("soilDry"), l)) strcpy(err, "soilDry not a number");
  else c.soilDry = l;

  if (!err[0]) {
    if (!server.hasArg("soilHyst"))        strcpy(err, "missing soilHyst");
    else if (!parseLongField(server.arg("soilHyst"), l)) strcpy(err, "soilHyst not a number");
    else c.soilHyst = l;
  }
  if (!err[0]) {
    if (!server.hasArg("tempLimit"))       strcpy(err, "missing tempLimit");
    else if (!parseFloatField(server.arg("tempLimit"), f)) strcpy(err, "tempLimit not a number");
    else c.tempLimit = f;
  }
  if (!err[0]) {
    if (!server.hasArg("interval"))        strcpy(err, "missing interval");
    else if (!parseLongField(server.arg("interval"), l)) strcpy(err, "interval not a number");
    else c.readIntervalMs = (uint32_t)l;
  }

  if (!err[0]) validateConfig(c, err, sizeof(err));

  if (err[0]) { sendErr(400, err); return; }

  cfg = c;
  saveCfg();
  applyControl();
  snprintf(txBuf, sizeof(txBuf), "{\"ok\":true}");
  server.send(200, "application/json", txBuf);
}

void handleOverride() {
  long m;
  if (!server.hasArg("mode") || !parseLongField(server.arg("mode"), m) || m < 0 || m > 2) {
    sendErr(400, "mode must be 0,1,2"); return;
  }
  overrideMode = (uint8_t)m;
  applyControl();
  snprintf(txBuf, sizeof(txBuf), "{\"ok\":true,\"ovr\":%u}", (unsigned)overrideMode);
  server.send(200, "application/json", txBuf);
}

void handleNotFound() {
  server.sendHeader(F("Location"), F("/"), true);
  server.send(302, F("text/plain"), "");
}

void handleFavicon() {
  server.send(204, F("text/plain"), "");
}

void handlePortalRedirect() {
  server.sendHeader(F("Location"), F("/"), true);
  server.send(302, F("text/plain"), "");
}

// -----------------------------
// WiFi AP
// -----------------------------
void wifiInit() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  apMode = true;
  MDNS.begin("plant");
  dnsServer.start(53, "*", WiFi.softAPIP());
}

// -----------------------------
// Setup / loop
// -----------------------------
void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetPinAttenuation(SOIL_PIN, ADC_11db);

  dht.begin();
  loadCfg();
  delay(2000);

  wifiInit();

  server.on("/",             HTTP_GET,  handleRoot);
  server.on("/favicon.ico",  HTTP_GET,  handleFavicon);
  server.on("/api/status",   HTTP_GET,  handleStatus);
  server.on("/api/settings", HTTP_POST, handleSettings);
  server.on("/api/override", HTTP_POST, handleOverride);

  server.on("/generate_204",              HTTP_GET, handlePortalRedirect);
  server.on("/gen_204",                   HTTP_GET, handlePortalRedirect);
  server.on("/hotspot-detect.html",       HTTP_GET, handlePortalRedirect);
  server.on("/library/test/success.html", HTTP_GET, handlePortalRedirect);
  server.on("/ncsi.txt",                  HTTP_GET, handlePortalRedirect);
  server.on("/connecttest.txt",           HTTP_GET, handlePortalRedirect);
  server.on("/redirect",                  HTTP_GET, handlePortalRedirect);
  server.on("/canonical.html",            HTTP_GET, handlePortalRedirect);
  server.on("/success.txt",               HTTP_GET, handlePortalRedirect);
  server.on("/fwlink",                    HTTP_GET, handlePortalRedirect);

  server.onNotFound(handleNotFound);
  server.begin();

  lastReadMs = millis() - cfg.readIntervalMs;
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  if ((uint32_t)(millis() - lastReadMs) >= cfg.readIntervalMs) {
    sampleSensors();
  }
}
