#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <DHT.h>

// ============================================================
// PIN & TYPE CONFIG
// ============================================================
#define SOIL_PIN   34
#define DHT_PIN    4
#define DHT_TYPE   DHT22
#define RELAY_PIN  26
#define RELAY_ON   HIGH
#define RELAY_OFF  LOW

// ============================================================
// STATIC CONFIG
// ============================================================
static const char* WIFI_SSID = "ESP32_DHT22_SOIL_SENSORS";
static const char* WIFI_PASS = "12345678";
static IPAddress LOCAL_IP(192, 168, 1, 50);
static IPAddress GATEWAY  (192, 168, 1, 1);
static IPAddress SUBNET   (255, 255, 255, 0);
static IPAddress DNS1     (192, 168, 1, 1);

// ============================================================
// RUNTIME SETTINGS
// ============================================================
struct Settings {
  int   soilDry;
  int   soilHyst;
  float tempLimit;
  bool  enabled;
};
static Settings cfg = { 2500, 150, 35.0f, true };

// ============================================================
// LIVE STATE
// ============================================================
struct LiveState {
  int   soilValue;
  float temperature;
  float humidity;
  bool  motorStatus;
  bool  dhtOk;
  bool  soilOk;
  uint32_t uptimeSec;
};
static LiveState state = { 0, 0.0f, 0.0f, false, false, false, 0 };

// ============================================================
// COMPONENTS
// ============================================================
DHT dht(DHT_PIN, DHT_TYPE);
AsyncWebServer server(80);
Preferences    prefs;

// ============================================================
// FIXED-SIZE BUFFERS
// ============================================================
static char jsonBuf[512];
static char bodyBuf[512];
static char nvsNs[]  = "irrig";

static volatile uint32_t settingsLockUntil = 0;
static const uint32_t SETTINGS_LOCK_MS = 5000;

// ============================================================
// SETTINGS LOAD / SAVE (NVS)
// ============================================================
void loadSettings()
{
  prefs.begin(nvsNs, true);
  cfg.soilDry   = prefs.getInt  ("soilDry",  2500);
  cfg.soilHyst  = prefs.getInt  ("soilHyst", 150);
  cfg.tempLimit = prefs.getFloat("tempLimit", 35.0f);
  cfg.enabled   = prefs.getBool ("enabled",  true);
  prefs.end();
}
void saveSettings()
{
  prefs.begin(nvsNs, false);
  prefs.putInt  ("soilDry",   cfg.soilDry);
  prefs.putInt  ("soilHyst",  cfg.soilHyst);
  prefs.putFloat("tempLimit", cfg.tempLimit);
  prefs.putBool ("enabled",   cfg.enabled);
  prefs.end();
}

// ============================================================
// SENSOR READ + CONTROL LOGIC
// ============================================================
void readSensors()
{
  int soilValue = analogRead(SOIL_PIN);
  state.soilValue = soilValue;
  if (soilValue < 50 || soilValue > 4000) {
    state.soilOk = false;
    digitalWrite(RELAY_PIN, RELAY_OFF);
    state.motorStatus = false;
    return;
  }
  state.soilOk = true;
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) {
    state.dhtOk = false;
    digitalWrite(RELAY_PIN, RELAY_OFF);
    state.motorStatus = false;
    return;
  }
  state.dhtOk = true;
  state.temperature = t;
  state.humidity    = h;
  if (!cfg.enabled) {
    digitalWrite(RELAY_PIN, RELAY_OFF);
    state.motorStatus = false;
    return;
  }
  bool soilDryNow = (soilValue > cfg.soilDry + cfg.soilHyst);
  bool soilWetNow = (soilValue < cfg.soilDry - cfg.soilHyst);
  bool hotDay     = (t > cfg.tempLimit);
  if (soilDryNow || (hotDay && soilValue > cfg.soilDry)) {
    digitalWrite(RELAY_PIN, RELAY_ON);
    state.motorStatus = true;
  } else if (soilWetNow) {
    digitalWrite(RELAY_PIN, RELAY_OFF);
    state.motorStatus = false;
  }
}

// ============================================================
// JSON SERIALIZER
// ============================================================
const char* buildStatusJson()
{
  snprintf(jsonBuf, sizeof(jsonBuf),
    "{\"soil\":%d,\"temp\":%.1f,\"hum\":%.1f,"
    "\"motor\":%s,\"dhtOk\":%s,\"soilOk\":%s,"
    "\"uptime\":%lu,\"heap\":%lu,"
    "\"soilDry\":%d,\"soilHyst\":%d,\"tempLimit\":%.1f,\"enabled\":%s}",
    state.soilValue,
    state.temperature,
    state.humidity,
    state.motorStatus ? "true" : "false",
    state.dhtOk       ? "true" : "false",
    state.soilOk      ? "true" : "false",
    (unsigned long)state.uptimeSec,
    (unsigned long)ESP.getFreeHeap(),
    cfg.soilDry,
    cfg.soilHyst,
    cfg.tempLimit,
    cfg.enabled ? "true" : "false"
  );
  return jsonBuf;
}

// ============================================================
// HTML PAGE
// ============================================================
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Smart Irrigation</title>
<style>
:root{--bg:#0f172a;--card:#1e293b;--accent:#38bdf8;--ok:#22c55e;--warn:#f59e0b;--off:#64748b}
*{box-sizing:border-box;font-family:system-ui,sans-serif}
body{margin:0;background:var(--bg);color:#e2e8f0;padding:16px}
h1{font-size:1.5rem;margin:0 0 16px;color:var(--accent)}
.grid{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(140px,1fr))}
.card{background:var(--card);border-radius:12px;padding:14px}
.label{font-size:.75rem;color:#94a3b8;text-transform:uppercase;letter-spacing:.05em}
.value{font-size:1.6rem;font-weight:600;margin-top:4px}
.value.ok{color:var(--ok)}.value.warn{color:var(--warn)}.value.off{color:var(--off)}
form{margin-top:20px;background:var(--card);padding:16px;border-radius:12px}
.row{display:flex;justify-content:space-between;align-items:center;margin:10px 0}
input[type=number]{width:110px;padding:6px 8px;background:#0f172a;color:#e2e8f0;
  border:1px solid #334155;border-radius:8px;font-size:1rem}
input[type=checkbox]{width:22px;height:22px}
button{margin-top:12px;width:100%;padding:12px;background:var(--accent);color:#0f172a;
  border:none;border-radius:8px;font-weight:700;font-size:1rem;cursor:pointer}
button:hover{filter:brightness(1.1)}
.footer{margin-top:16px;font-size:.75rem;color:#64748b;text-align:center}
</style></head><body>
<h1>💧 Smart Irrigation</h1>
<div class="grid">
  <div class="card"><div class="label">Soil</div><div class="value" id="soil">—</div></div>
  <div class="card"><div class="label">Temp</div><div class="value" id="temp">—</div></div>
  <div class="card"><div class="label">Humidity</div><div class="value" id="hum">—</div></div>
  <div class="card"><div class="label">Pump</div><div class="value" id="motor">—</div></div>
</div>
<form id="cfg">
  <div class="row"><span>Soil dry threshold</span>
    <input type="number" id="soilDry" min="0" max="4095"></div>
  <div class="row"><span>Hysteresis</span>
    <input type="number" id="soilHyst" min="0" max="500"></div>
  <div class="row"><span>Temp limit (°C)</span>
    <input type="number" id="tempLimit" min="0" max="60" step="0.5"></div>
  <div class="row"><span>System enabled</span>
    <input type="checkbox" id="enabled"></div>
  <button type="submit">Save Settings</button>
</form>
<div class="footer" id="footer">—</div>
<script>
function q(id){return document.getElementById(id)}
async function refresh(){
  try{
    const r=await fetch('/api/status');const d=await r.json();
    q('soil').textContent=d.soil;
    q('soil').className='value '+(d.soilOk?(d.soil>d.soilDry+d.soilHyst?'warn':'ok'):'off');
    q('temp').textContent=d.temp.toFixed(1)+'°C';
    q('hum').textContent=d.hum.toFixed(0)+'%';
    q('motor').textContent=d.motor?'ON':'OFF';
    q('motor').className='value '+(d.motor?'warn':'ok');
    q('footer').textContent='Uptime '+d.uptime+'s · Heap '+d.heap+'B';
    if(!q('cfg').dataset.touched){
      q('soilDry').value=d.soilDry;q('soilHyst').value=d.soilHyst;
      q('tempLimit').value=d.tempLimit;q('enabled').checked=d.enabled;
    }
  }catch(e){q('footer').textContent='disconnected'}
}
document.querySelectorAll('#cfg input').forEach(i=>i.addEventListener('input',
  ()=>q('cfg').dataset.touched='1'));
q('cfg').addEventListener('submit',async e=>{
  e.preventDefault();
  const body=new URLSearchParams({
    soilDry:q('soilDry').value,soilHyst:q('soilHyst').value,
    tempLimit:q('tempLimit').value,enabled:q('enabled').checked?'1':'0'
  });
  await fetch('/api/settings',{method:'POST',body});
  q('cfg').dataset.touched='';
  refresh();
});
setInterval(refresh,2000);refresh();
</script></body></html>
)HTML";

// ============================================================
// ROUTE HANDLERS
// ============================================================
void handleRoot(AsyncWebServerRequest *req)
{
  req->send_P(200, "text/html", INDEX_HTML);
}
void handleStatus(AsyncWebServerRequest *req)
{
  req->send(200, "application/json", buildStatusJson());
}
void urlDecode(const char* src, char* dst, size_t dstSize)
{
  size_t i = 0, j = 0;
  while (src[i] && j < dstSize - 1) {
    if (src[i] == '%' && src[i+1] && src[i+2]) {
      char h[3] = { src[i+1], src[i+2], 0 };
      dst[j++] = (char)strtol(h, nullptr, 16);
      i += 3;
    } else if (src[i] == '+') {
      dst[j++] = ' '; i++;
    } else {
      dst[j++] = src[i++];
    }
  }
  dst[j] = 0;
}
bool parseSettings(const char* body, Settings& out)
{
  char local[512];
  strncpy(local, body, sizeof(local) - 1);
  local[sizeof(local) - 1] = 0;
  char* save = nullptr;
  char* tok = strtok_r(local, "&", &save);
  while (tok) {
    char* eq = strchr(tok, '=');
    if (eq) {
      *eq = 0;
      const char* key = tok;
      const char* val = eq + 1;
      char decodedVal[64];
      urlDecode(val, decodedVal, sizeof(decodedVal));
      if      (!strcmp(key, "soilDry"))   out.soilDry   = atoi(decodedVal);
      else if (!strcmp(key, "soilHyst"))  out.soilHyst  = atoi(decodedVal);
      else if (!strcmp(key, "tempLimit")) out.tempLimit = atof(decodedVal);
      else if (!strcmp(key, "enabled"))   out.enabled   = (decodedVal[0] == '1');
    }
    tok = strtok_r(nullptr, "&", &save);
  }
  return true;
}
void handleSettings(AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total)
{
  uint32_t now = millis();
  if (index == 0) {
    if ((int32_t)(now - settingsLockUntil) < 0) {
      req->send(429, "application/json", "{\"error\":\"busy\"}");
      return;
    }
    settingsLockUntil = now + SETTINGS_LOCK_MS;
    bodyBuf[0] = 0;
  }
  if (index + len >= sizeof(bodyBuf) - 1) {
    if (index + len == total) {
      settingsLockUntil = 0;
      req->send(413, "application/json", "{\"error\":\"body too large\"}");
    }
    return;
  }
  memcpy(bodyBuf + index, data, len);
  if (index + len == total) {
    bodyBuf[total] = 0;
    Settings newCfg = cfg;
    parseSettings(bodyBuf, newCfg);
    if (newCfg.soilDry   < 0)     newCfg.soilDry   = 0;
    if (newCfg.soilDry   > 4095)  newCfg.soilDry   = 4095;
    if (newCfg.soilHyst  < 0)     newCfg.soilHyst  = 0;
    if (newCfg.soilHyst  > 500)   newCfg.soilHyst  = 500;
    if (newCfg.tempLimit < 0)     newCfg.tempLimit = 0;
    if (newCfg.tempLimit > 60)    newCfg.tempLimit = 60;
    cfg = newCfg;
    saveSettings();
    settingsLockUntil = 0;
    req->send(200, "application/json", "{\"ok\":true}");
  }
}

// ============================================================
// WIFI CONNECT
// ============================================================
void connectWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.config(LOCAL_IP, GATEWAY, SUBNET, DNS1);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(250);
  }
}

// ============================================================
// SETUP
// ============================================================
void setup()
{
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);
  dht.begin();
  loadSettings();
  connectWiFi();
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/settings", HTTP_POST,
    [](AsyncWebServerRequest *req) {
      if (req->contentLength() == 0) {
        req->send(400, "application/json", "{\"error\":\"empty body\"}");
      }
    },
    nullptr,
    handleSettings);
  server.onNotFound([](AsyncWebServerRequest *req) {
    req->send(404, "text/plain", "Not Found");
  });
  server.begin();
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop()
{
  static uint32_t lastRead = 0;
  static uint32_t lastUptime = 0;
  uint32_t now = millis();
  if (now - lastRead >= 2500) {
    lastRead = now;
    readSensors();
  }
  if (now - lastUptime >= 1000) {
    lastUptime = now;
    state.uptimeSec++;
  }
  static uint32_t lastHealthCheck = 0;
  if (now - lastHealthCheck >= 60000) {
    lastHealthCheck = now;
    if (ESP.getFreeHeap() < 20000) {
      ESP.restart();
    }
  }
}
