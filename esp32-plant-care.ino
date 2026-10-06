#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DHT.h>

// -----------------------------
// Pin definitions
// -----------------------------
#define SOIL_PIN 34
#define DHT_PIN 4
#define DHT_TYPE DHT22

#define RELAY_PIN 26
#define RELAY_ON  HIGH
#define RELAY_OFF LOW

// -----------------------------
// Access Point credentials
// -----------------------------
const char* AP_SSID = "ESP32_Plant_Care";
const char* AP_PASS = "12345678";

// -----------------------------
// Components
// -----------------------------
DHT dht(DHT_PIN, DHT_TYPE);
WebServer server(80);
Preferences prefs;

// -----------------------------
// Thresholds
// -----------------------------
int   soilDry   = 2500;
int   soilHyst  = 150;
float tempLimit = 35.0;

// -----------------------------
// State
// -----------------------------
bool  motorStatus = false;
int   soilValue   = 0;
float temperature = NAN;
float humidity    = NAN;
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL = 2500;

// -----------------------------
// Static HTML
// -----------------------------
const char HTML_HEAD[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Garden</title>
<style>
body{font-family:sans-serif;max-width:480px;margin:20px auto;padding:10px}
.card{background:#f4f4f4;padding:12px;margin:8px 0;border-radius:8px}
.on{color:#c00;font-weight:bold}.off{color:#080;font-weight:bold}
input{width:80px}
</style></head><body>
<h2>🌱 Garden Dashboard</h2>
<div class='card' id='sensors'>Loading…</div>
<div class='card'>
<h3>Settings</h3>
<label>Soil Dry: <input id='sd' type='number'></label><br>
<label>Hysteresis: <input id='sh' type='number'></label><br>
<label>Temp Limit: <input id='tl' type='number' step='0.1'></label><br>
<button id='saveBtn' onclick='save()' disabled>Save</button>
</div>
<script>
const $ = id => document.getElementById(id);
function refresh(){
 fetch('/api/state').then(r=>r.json()).then(d=>{
  const s = d.soil||0, t = d.temp||0, h = d.hum||0;
  $('sensors').innerHTML =
   'Soil: '+s+' ('+((s/4095*100).toFixed(0))+'%)<br>'+
   'Temp: '+t.toFixed(1)+' °C<br>'+
   'Hum: '+h.toFixed(1)+' %<br>'+
   'Pump: <span class="'+(d.motor?'on':'off')+'">'+(d.motor?'ON':'OFF')+'</span>';
 }).catch(()=>{ $('sensors').textContent='Connection error'; });
}
function loadCfg(){
 fetch('/api/config').then(r=>r.json()).then(d=>{
  $('sd').value=d.soilDry;
  $('sh').value=d.soilHyst;
  $('tl').value=d.tempLimit;
  $('saveBtn').disabled = false;
 }).catch(()=>{
  $('saveBtn').disabled = true;
 });
}
function save(){
 fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify({soilDry:+$('sd').value,soilHyst:+$('sh').value,tempLimit:+$('tl').value})})
 .then(r=>r.ok?alert('Saved'):alert('Error '+r.status))
 .catch(()=>alert('Network error'));
}
setInterval(refresh,3000); refresh(); loadCfg();
</script></body></html>
)HTML";

// -----------------------------
// Averaged soil read
// -----------------------------
int readSoilAvg()
{
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(SOIL_PIN);
    delayMicroseconds(200);
  }
  return sum / 8;
}

// -----------------------------
// Read sensors and control pump
// -----------------------------
void readSensors()
{
  int raw = readSoilAvg();
  soilValue = raw;

  if (raw < 50 || raw >= 4095) {
    digitalWrite(RELAY_PIN, RELAY_OFF);
    motorStatus = false;
    return;
  }

  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) temperature = t;
  if (!isnan(h)) humidity = h;
  if (isnan(t) || isnan(h)) return;

  bool soilDryNow = (raw > soilDry + soilHyst);
  bool soilWetNow = (raw < soilDry - soilHyst);
  bool hotDay     = (t > tempLimit);

  if (soilDryNow || (hotDay && raw > soilDry)) {
    digitalWrite(RELAY_PIN, RELAY_ON);
    motorStatus = true;
  } else if (soilWetNow) {
    digitalWrite(RELAY_PIN, RELAY_OFF);
    motorStatus = false;
  }
}

// -----------------------------
// Web handlers
// -----------------------------
void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", HTML_HEAD);
}

void handleState() {
  static char buf[160];
  snprintf(buf, sizeof(buf),
    "{\"soil\":%d,\"temp\":%.1f,\"hum\":%.1f,\"motor\":%s}",
    soilValue,
    isnan(temperature) ? 0.0 : temperature,
    isnan(humidity)    ? 0.0 : humidity,
    motorStatus ? "true" : "false");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

void handleGetConfig() {
  static char buf[128];
  snprintf(buf, sizeof(buf),
    "{\"soilDry\":%d,\"soilHyst\":%d,\"tempLimit\":%.1f}",
    soilDry, soilHyst, tempLimit);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

void handlePostConfig() {
  if (!server.hasArg("plain")) { server.send(400, "text/plain", "no body"); return; }
  String body = server.arg("plain");

  auto extractInt = [&](const char* key, int def) -> int {
    int i = body.indexOf(key);
    if (i < 0) return def;
    int c = body.indexOf(':', i); if (c < 0) return def;
    int e = body.indexOf(',', c); if (e < 0) e = body.indexOf('}', c);
    if (e < 0) e = body.length();
    return body.substring(c + 1, e).toInt();
  };
  auto extractFloat = [&](const char* key, float def) -> float {
    int i = body.indexOf(key);
    if (i < 0) return def;
    int c = body.indexOf(':', i); if (c < 0) return def;
    int e = body.indexOf(',', c); if (e < 0) e = body.indexOf('}', c);
    if (e < 0) e = body.length();
    return body.substring(c + 1, e).toFloat();
  };

  int   newSoilDry  = constrain(extractInt  ("\"soilDry\"",   soilDry),   100, 4000);
  int   newSoilHyst = constrain(extractInt  ("\"soilHyst\"",  soilHyst),  10,  500);
  float newTempLim  = constrain(extractFloat("\"tempLimit\"", tempLimit), 0.0f, 60.0f);

  bool allZero = (newSoilDry == 100 && newSoilHyst == 10 && newTempLim == 0.0f);
  if (allZero) {
    server.send(400, "application/json", "{\"error\":\"invalid config\"}");
    return;
  }

  soilDry   = newSoilDry;
  soilHyst  = newSoilHyst;
  tempLimit = newTempLim;

  prefs.begin("garden", false);
  prefs.putInt("soilDry", soilDry);
  prefs.putInt("soilHyst", soilHyst);
  prefs.putFloat("tempLimit", tempLimit);
  prefs.end();

  server.send(200, "application/json", "{\"ok\":true}");
}

void handleNotFound() {
  server.send_P(404, "text/plain", "Not found");
}

// -----------------------------
// Load config from NVS
// -----------------------------
void loadConfig() {
  prefs.begin("garden", true);
  soilDry   = prefs.getInt("soilDry", 2500);
  soilHyst  = prefs.getInt("soilHyst", 150);
  tempLimit = prefs.getFloat("tempLimit", 35.0);
  prefs.end();
}

// -----------------------------
// Setup
// -----------------------------
void setup()
{
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);

  analogReadResolution(12);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  analogSetPinAttenuation(SOIL_PIN, ADC_ATTEN_DB_12);
#else
  analogSetPinAttenuation(SOIL_PIN, ADC_11db);
#endif

  dht.begin();
  loadConfig();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/",             HTTP_GET,  handleRoot);
  server.on("/api/state",    HTTP_GET,  handleState);
  server.on("/api/config",   HTTP_GET,  handleGetConfig);
  server.on("/api/config",   HTTP_POST, handlePostConfig);
  server.onNotFound(handleNotFound);

  server.begin();

  delay(2000);
}

// -----------------------------
// Main loop
// -----------------------------
void loop()
{
  server.handleClient();

  unsigned long now = millis();
  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    readSensors();
  }
}
