/*
 * ESP32 Unified Garden Control System
 * ========================================================================
 * Sensors : Capacitive soil moisture (GPIO34) + SHT31 (I2C 21/22)
 * Actuator: Relay GPIO25 (auto + manual override)
 * Network : WiFi AP + STA, captive portal, auto-reconnect
 * Reset   : GPIO0 5s hold (post-boot only; held-at-boot blocks boot)
 *
 * REQUIRED LIBRARIES (Library Manager):
 *   - "Adafruit SHT31 Library"  (by Adafruit)  -> pulls BusIO
 *   - "ArduinoJson"             (by Benoit Blanchon) — MUST be v6.x  (NOT v7)
 * Board: "ESP32 Dev Module"  (any ESP32 core 2.x/3.x)
 *
 * WIRING:
 *   Soil signal  -> GPIO34   (ADC1_CH6, input only)
 *   Soil VCC     -> 3V3      Soil GND -> GND
 *   SHT31 SDA    -> GPIO21   SHT31 SCL -> GPIO22   VCC 3V3   GND GND
 *   Relay IN     -> GPIO25   Relay VCC/GND per module (5V for most)
 *   Boot button  -> GPIO0 (onboard; do NOT hold during power-on)
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <ArduinoJson.h>

// ================= PINS =================
#define PIN_SOIL        34
#define PIN_RELAY       25
#define PIN_BOOT_BTN     0
#define PIN_SDA         21
#define PIN_SCL         22

// ================= CONSTANTS =================
#define AP_SSID              "ESP32_SoilM_SHT31"
#define AP_PASSWORD          "12345678"
#define DNS_PORT             53
#define FACTORY_HOLD_MS      5000
#define SENSOR_INTERVAL_MS   2000
#define SHT_RETRY_MS         10000
#define SHT_STALE_MS         30000
#define WIFI_CONNECT_TIMEOUT 15000
#define WIFI_RETRY_MS        30000
#define MANUAL_OVERRIDE_MS   (5UL * 60UL * 1000UL)
#define WIFI_HEALTH_MS       (30UL * 60UL * 1000UL)

// ================= GLOBALS =================
WebServer  server(80);
DNSServer  dnsServer;
Preferences prefs;
Adafruit_SHT31 sht31 = Adafruit_SHT31();

struct Settings {
    String staSSID          = "";
    String staPass          = "";
    bool   useSTA           = false;
    int    soilDryRaw       = 3200;
    int    soilWetRaw       = 1200;
    int    soilThresholdPct = 30;
    float  tempHighC        = 30.0f;
    float  tempLowC         = 10.0f;
    float  humHighPct       = 80.0f;
    float  humLowPct        = 30.0f;
    int    relayMode        = 0;
    bool   relayActiveLow   = true;
    String deviceName       = "ESP32-Garden";
} cfg;

// Runtime
float soilPct = 0.0f;
int   soilRaw = 0;
bool  soilFault = false;
float temperature = NAN;
float humidity    = NAN;
unsigned long shtLastValidMs = 0;
unsigned long shtLastRetryMs = 0;
bool  shtOK = false;
bool  relayState = false;
bool  tempTrigLatched = false;
bool  humTrigLatched  = false;

unsigned long lastSensorRead = 0;
unsigned long lastWifiRetry  = 0;
bool  wifiWantConnect        = false;
unsigned long lastWifiHealthCheck = 0;

// Manual override
bool          manualOverride      = false;
bool          manualRelayWanted   = false;
unsigned long manualOverrideStart = 0;

// Boot button
unsigned long bootBtnPressStart = 0;
bool          bootBtnPressed    = false;
unsigned long lastShtErrLog = 0;

// ================= FORWARD =================
void loadSettings();
void saveSettings();
void setupWiFi();
void setupWebServer();
void setupCaptivePortal();
void readSensors();
void updateRelay();
void driveRelay(bool on);
void handleBootButton();
void factoryReset();
String getStatusJSON();
String getSettingsJSON();
void applySettingsFromJSON(JsonDocument &doc);

// ================= HTML DASHBOARD =================
const char DASHBOARD_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32 Garden Dashboard</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;
       background:#0f172a;color:#e2e8f0;min-height:100vh;padding:16px}
  .container{max-width:900px;margin:0 auto}
  h1{font-size:1.6rem;margin-bottom:8px;color:#38bdf8}
  .sub{color:#94a3b8;font-size:.85rem;margin-bottom:20px}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:14px;margin-bottom:20px}
  .card{background:#1e293b;border-radius:12px;padding:16px;border:1px solid #334155}
  .card h3{font-size:.8rem;color:#94a3b8;text-transform:uppercase;letter-spacing:.5px;margin-bottom:8px}
  .value{font-size:2rem;font-weight:700;color:#38bdf8}
  .unit{font-size:.9rem;color:#64748b;font-weight:400}
  .status-dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px}
  .dot-on{background:#22c55e;box-shadow:0 0 8px #22c55e}
  .dot-off{background:#64748b}
  .row{display:flex;justify-content:space-between;align-items:center;padding:6px 0;font-size:.9rem}
  .row span:last-child{color:#38bdf8;font-weight:600}
  .section{background:#1e293b;border-radius:12px;padding:18px;margin-bottom:16px;border:1px solid #334155}
  .section h2{font-size:1.1rem;margin-bottom:14px;color:#f1f5f9}
  label{display:block;font-size:.8rem;color:#94a3b8;margin:10px 0 4px}
  input,select{width:100%;padding:9px 12px;border-radius:8px;border:1px solid #475569;
       background:#0f172a;color:#e2e8f0;font-size:.9rem;outline:none}
  input:focus,select:focus{border-color:#38bdf8}
  button{background:#38bdf8;color:#0f172a;border:none;padding:11px 20px;border-radius:8px;
       font-weight:700;font-size:.9rem;cursor:pointer;margin-top:14px;width:100%;transition:.2s}
  button:hover{background:#0ea5e9}
  button.danger{background:#ef4444;color:#fff}
  button.danger:hover{background:#dc2626}
  button.warn{background:#f59e0b;color:#0f172a}
  button.warn:hover{background:#d97706}
  .btn-row{display:flex;gap:10px;flex-wrap:wrap}
  .btn-row button{flex:1;min-width:140px}
  .toast{position:fixed;bottom:20px;left:50%;transform:translateX(-50%);
       background:#22c55e;color:#fff;padding:12px 24px;border-radius:8px;
       font-weight:600;opacity:0;transition:.3s;z-index:999}
  .toast.show{opacity:1}
  .toast.err{background:#ef4444}
  .badge{font-size:.7rem;padding:2px 8px;border-radius:20px;background:#334155;color:#94a3b8;margin-left:8px}
  .badge.on{background:#22c55e33;color:#22c55e}
  .badge.off{background:#64748b33;color:#94a3b8}
  .badge.err{background:#ef444433;color:#ef4444}
  .badge.warn{background:#f59e0b33;color:#f59e0b}
  .tabs{display:flex;gap:6px;margin-bottom:16px;flex-wrap:wrap}
  .tab{padding:8px 16px;border-radius:8px;background:#1e293b;border:1px solid #334155;
       color:#94a3b8;cursor:pointer;font-size:.85rem;font-weight:600;transition:.2s}
  .tab.active{background:#38bdf8;color:#0f172a;border-color:#38bdf8}
  .tab-content{display:none}
  .tab-content.active{display:block}
</style>
</head>
<body>
<div class="container">
  <h1>🌱 ESP32 Garden Control</h1>
  <p class="sub" id="devname">Loading…</p>

  <div class="grid">
    <div class="card"><h3>Soil Moisture</h3>
      <div class="value"><span id="soil">--</span><span class="unit"> %</span></div>
      <div class="row"><span>Raw ADC</span><span id="soilRaw">--</span></div>
    </div>
    <div class="card"><h3>Temperature</h3>
      <div class="value"><span id="temp">--</span><span class="unit"> °C</span></div>
    </div>
    <div class="card"><h3>Humidity</h3>
      <div class="value"><span id="hum">--</span><span class="unit"> %</span></div>
    </div>
    <div class="card"><h3>Relay</h3>
      <div class="value" style="font-size:1.5rem">
        <span class="status-dot" id="relayDot"></span><span id="relayTxt">--</span>
      </div>
      <div class="row"><span>Manual Override</span><span id="ovrTxt">—</span></div>
    </div>
  </div>

  <div class="section">
    <h2>System Status</h2>
    <div class="row"><span>WiFi Mode</span><span id="wifiMode">--</span></div>
    <div class="row"><span>AP IP</span><span id="apIp">--</span></div>
    <div class="row"><span>STA IP</span><span id="staIp">--</span></div>
    <div class="row"><span>STA Connected</span><span id="staConn">--</span></div>
    <div class="row"><span>SHT31</span><span id="shtOK">--</span></div>
    <div class="row"><span>Uptime</span><span id="uptime">--</span></div>
    <div class="row"><span>Free Heap</span><span id="heap">--</span></div>
  </div>

  <div class="tabs">
    <div class="tab active" data-tab="wifi">WiFi</div>
    <div class="tab" data-tab="thresholds">Thresholds</div>
    <div class="tab" data-tab="relay">Relay</div>
    <div class="tab" data-tab="system">System</div>
  </div>

  <div class="tab-content active" id="tab-wifi">
    <div class="section">
      <h2>WiFi Station Settings</h2>
      <label>SSID</label>
      <input type="text" id="staSSID" maxlength="32" placeholder="Network name">
      <label>Password (leave blank to keep current)</label>
      <input type="password" id="staPass" maxlength="63" placeholder="••••••••">
      <label>Use STA Mode</label>
      <select id="useSTA">
        <option value="0">No (AP only)</option>
        <option value="1">Yes (AP + STA)</option>
      </select>
      <div class="btn-row"><button onclick="saveWifi()">Save & Reconnect</button></div>
    </div>
  </div>

  <div class="tab-content" id="tab-thresholds">
    <div class="section">
      <h2>Soil Moisture Thresholds</h2>
      <label>Dry Calibration (raw ADC, higher = drier)</label>
      <input type="number" id="soilDryRaw" min="0" max="4095">
      <label>Wet Calibration (raw ADC, lower = wetter)</label>
      <input type="number" id="soilWetRaw" min="0" max="4095">
      <label>Relay triggers below this moisture (%)</label>
      <input type="number" id="soilThresholdPct" min="0" max="100">
    </div>
    <div class="section">
      <h2>SHT31 Thresholds</h2>
      <label>Temperature High (°C)</label><input type="number" step="0.1" id="tempHighC">
      <label>Temperature Low (°C)</label>  <input type="number" step="0.1" id="tempLowC">
      <label>Humidity High (%)</label>     <input type="number" step="0.1" id="humHighPct">
      <label>Humidity Low (%)</label>      <input type="number" step="0.1" id="humLowPct">
    </div>
    <button onclick="saveThresholds()">Save Thresholds</button>
  </div>

  <div class="tab-content" id="tab-relay">
    <div class="section">
      <h2>Relay Logic</h2>
      <label>Trigger Mode</label>
      <select id="relayMode">
        <option value="0">Soil moisture only</option>
        <option value="1">Temperature only</option>
        <option value="2">Humidity only</option>
        <option value="3">Soil OR Temp OR Humidity</option>
        <option value="4">Soil AND Temp AND Humidity</option>
      </select>
      <label>Relay Active Level</label>
      <select id="relayActiveLow">
        <option value="1">Active LOW (most modules)</option>
        <option value="0">Active HIGH</option>
      </select>
      <div class="row" style="margin-top:16px"><span>Current State</span><span id="relayManual">--</span></div>
      <div class="btn-row">
        <button class="warn" onclick="manualToggle()">Manual Toggle (5 min)</button>
        <button onclick="clearOverride()">Clear Override</button>
      </div>
    </div>
    <button onclick="saveRelay()">Save Relay Settings</button>
  </div>

  <div class="tab-content" id="tab-system">
    <div class="section">
      <h2>Device</h2>
      <label>Device Name</label><input type="text" id="deviceName">
      <button onclick="saveDevice()">Save Device Name</button>
    </div>
    <div class="section">
      <h2>Danger Zone</h2>
      <p style="font-size:.85rem;color:#94a3b8;margin-bottom:10px">
        Erase all settings and reboot the device.
      </p>
      <button class="danger" onclick="factoryReset()">Factory Reset</button>
    </div>
  </div>
</div>
<div class="toast" id="toast"></div>

<script>
function $(id){return document.getElementById(id)}
function toast(msg, err){
  var t=$('toast'); t.textContent=msg;
  t.className='toast show'+(err?' err':'');
  clearTimeout(t._tid); t._tid=setTimeout(function(){t.className='toast'},2500);
}
function fmt(v,d){return (v===null||v===undefined||isNaN(v))?'--':Number(v).toFixed(d===undefined?1:d)}

document.querySelectorAll('.tab').forEach(function(tab){
  tab.addEventListener('click',function(){
    document.querySelectorAll('.tab').forEach(function(t){t.classList.remove('active')});
    document.querySelectorAll('.tab-content').forEach(function(c){c.classList.remove('active')});
    tab.classList.add('active');
    $('tab-'+tab.dataset.tab).classList.add('active');
  });
});

function refresh(){
  fetch('/api/status').then(function(r){return r.json()}).then(function(d){
    $('devname').textContent = d.deviceName + ' — ' + d.apIp;
    $('soil').textContent = fmt(d.soilPct,1);
    $('soilRaw').textContent = d.soilRaw;
    $('temp').textContent = d.shtFresh ? fmt(d.temperature,1) : '--';
    $('hum').textContent  = d.shtFresh ? fmt(d.humidity,1)   : '--';
    $('relayDot').className='status-dot '+(d.relayState?'dot-on':'dot-off');
    $('relayTxt').textContent  = d.relayState?'ON':'OFF';
    $('relayManual').textContent = d.relayState?'ON':'OFF';
    $('ovrTxt').textContent = d.manualOverride ? ('YES ('+d.manualRemainS+'s)') : 'no';
    $('wifiMode').textContent = 'AP + STA';
    $('apIp').textContent   = d.apIp;
    $('staIp').textContent  = d.staIp || '--';
    $('staConn').innerHTML  = d.staConnected ? '<span class="badge on">Connected</span>'
                                             : '<span class="badge off">Disconnected</span>';
    $('shtOK').innerHTML    = d.shtOK ? '<span class="badge on">OK</span>'
                                      : '<span class="badge err">Error</span>';
    $('uptime').textContent = d.uptime;
    $('heap').textContent   = d.freeHeap + ' bytes';
  }).catch(function(){});
}
setInterval(refresh, 2000); refresh();

function loadSettings(retry){
  retry = retry || 0;
  fetch('/api/settings').then(function(r){return r.json()}).then(function(d){
    $('staSSID').value = d.staSSID || '';
    $('staPass').value = '';
    $('useSTA').value  = d.useSTA ? '1' : '0';
    $('soilDryRaw').value = d.soilDryRaw;
    $('soilWetRaw').value = d.soilWetRaw;
    $('soilThresholdPct').value = d.soilThresholdPct;
    $('tempHighC').value = d.tempHighC;
    $('tempLowC').value  = d.tempLowC;
    $('humHighPct').value= d.humHighPct;
    $('humLowPct').value = d.humLowPct;
    $('relayMode').value = d.relayMode;
    $('relayActiveLow').value = d.relayActiveLow ? '1' : '0';
    $('deviceName').value = d.deviceName || '';
  }).catch(function(){
    if(retry < 3) setTimeout(function(){ loadSettings(retry+1); }, 1000);
    else toast('Failed to load settings', true);
  });
}
loadSettings();

function postJSON(url, body, cb){
  fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},
             body:JSON.stringify(body)})
   .then(function(r){return r.json()})
   .then(function(d){ if(cb)cb(d); })
   .catch(function(e){ toast('Error: '+e, true); });
}

function saveWifi(){
  var body = {
    staSSID: $('staSSID').value,
    useSTA:  $('useSTA').value === '1'
  };
  var p = $('staPass').value;
  if(p.length > 0) body.staPass = p;
  postJSON('/api/settings', body, function(){
    toast('WiFi saved — reconnecting…');
    $('staPass').value='';
    setTimeout(refresh, 3000);
  });
}
function numOrNull(el){
  var v = el.value.trim();
  if(v === '') return null;
  var n = Number(v);
  return isNaN(n) ? null : n;
}
function saveThresholds(){
  var body = {}, v;
  if((v=numOrNull($('soilDryRaw')))       !== null) body.soilDryRaw = v;
  if((v=numOrNull($('soilWetRaw')))       !== null) body.soilWetRaw = v;
  if((v=numOrNull($('soilThresholdPct'))) !== null) body.soilThresholdPct = v;
  if((v=numOrNull($('tempHighC')))        !== null) body.tempHighC = v;
  if((v=numOrNull($('tempLowC')))         !== null) body.tempLowC = v;
  if((v=numOrNull($('humHighPct')))       !== null) body.humHighPct = v;
  if((v=numOrNull($('humLowPct')))        !== null) body.humLowPct = v;
  if(Object.keys(body).length === 0){ toast('Nothing to save', true); return; }
  postJSON('/api/settings', body, function(){ toast('Thresholds saved'); });
}
function saveRelay(){
  var body = { relayActiveLow: $('relayActiveLow').value === '1' };
  var m = numOrNull($('relayMode'));
  if(m !== null) body.relayMode = m;
  postJSON('/api/settings', body, function(){ toast('Relay settings saved'); });
}
function saveDevice(){
  postJSON('/api/settings', {deviceName: $('deviceName').value},
           function(){ toast('Device name saved'); });
}
function manualToggle(){
  fetch('/api/relay/toggle',{method:'POST'})
   .then(function(r){return r.json()})
   .then(function(d){ toast('Manual override: '+(d.relayState?'ON':'OFF'));
                      refresh(); });
}
function clearOverride(){
  fetch('/api/relay/clear',{method:'POST'})
   .then(function(){ toast('Override cleared'); refresh(); });
}
function factoryReset(){
  if(!confirm('Factory reset erases ALL settings and reboots. Continue?')) return;
  fetch('/api/factory-reset',{method:'POST'}).catch(function(){});
  toast('Factory reset in progress…');
}
</script>
</body>
</html>
)HTML";

// ================= SETTINGS =================
void loadSettings(){
    prefs.begin("cfg", true);
    cfg.staSSID           = prefs.getString("staSSID", "");
    cfg.staPass           = prefs.getString("staPass", "");
    cfg.useSTA            = prefs.getBool("useSTA", false);
    cfg.soilDryRaw        = prefs.getInt("soilDryRaw", 3200);
    cfg.soilWetRaw        = prefs.getInt("soilWetRaw", 1200);
    cfg.soilThresholdPct  = prefs.getInt("soilThr", 30);
    cfg.tempHighC         = prefs.getFloat("tHigh", 30.0f);
    cfg.tempLowC          = prefs.getFloat("tLow", 10.0f);
    cfg.humHighPct        = prefs.getFloat("hHigh", 80.0f);
    cfg.humLowPct         = prefs.getFloat("hLow", 30.0f);
    cfg.relayMode         = prefs.getInt("relayMode", 0);
    cfg.relayActiveLow    = prefs.getBool("rActLow", true);
    cfg.deviceName        = prefs.getString("devName", "ESP32-Garden");
    prefs.end();

    if (cfg.soilDryRaw < 0)    cfg.soilDryRaw = 0;
    if (cfg.soilDryRaw > 4095) cfg.soilDryRaw = 4095;
    if (cfg.soilWetRaw < 0)    cfg.soilWetRaw = 0;
    if (cfg.soilWetRaw > 4095) cfg.soilWetRaw = 4095;
    if (cfg.soilDryRaw <= cfg.soilWetRaw){
        if (cfg.soilDryRaw + 100 <= 4095) {
            cfg.soilDryRaw = cfg.soilWetRaw + 100;
        } else {
            cfg.soilDryRaw = 4095;
            cfg.soilWetRaw = (4095 - 100 >= 0) ? 3995 : 0;
        }
    }
    if (cfg.tempHighC <= cfg.tempLowC){
        cfg.tempHighC = cfg.tempLowC + 1.0f;
        if (cfg.tempHighC > 125.0f){ cfg.tempHighC = 125.0f; cfg.tempLowC = 124.0f; }
    }
    if (cfg.humHighPct <= cfg.humLowPct){
        cfg.humHighPct = cfg.humLowPct + 1.0f;
        if (cfg.humHighPct > 100.0f){ cfg.humHighPct = 100.0f; cfg.humLowPct = 99.0f; }
    }
    if (cfg.soilThresholdPct < 0)   cfg.soilThresholdPct = 0;
    if (cfg.soilThresholdPct > 100) cfg.soilThresholdPct = 100;
    if (cfg.relayMode < 0) cfg.relayMode = 0;
    if (cfg.relayMode > 4) cfg.relayMode = 4;
    if (cfg.deviceName.length()==0) cfg.deviceName = "ESP32-Garden";
}

void saveSettings(){
    prefs.begin("cfg", false);
    prefs.putString("staSSID", cfg.staSSID);
    prefs.putString("staPass", cfg.staPass);
    prefs.putBool("useSTA", cfg.useSTA);
    prefs.putInt("soilDryRaw", cfg.soilDryRaw);
    prefs.putInt("soilWetRaw", cfg.soilWetRaw);
    prefs.putInt("soilThr", cfg.soilThresholdPct);
    prefs.putFloat("tHigh", cfg.tempHighC);
    prefs.putFloat("tLow", cfg.tempLowC);
    prefs.putFloat("hHigh", cfg.humHighPct);
    prefs.putFloat("hLow", cfg.humLowPct);
    prefs.putInt("relayMode", cfg.relayMode);
    prefs.putBool("rActLow", cfg.relayActiveLow);
    prefs.putString("devName", cfg.deviceName);
    prefs.end();
}

// ================= RESET =================
void factoryReset(){
    driveRelay(false);
    prefs.begin("cfg", false);
    prefs.clear();
    prefs.end();
    delay(300);
    ESP.restart();
}

// ================= BOOT BUTTON =================
void handleBootButton(){
    if (digitalRead(PIN_BOOT_BTN) == LOW){
        if (!bootBtnPressed){ bootBtnPressed = true; bootBtnPressStart = millis(); }
        else if (millis() - bootBtnPressStart >= FACTORY_HOLD_MS) factoryReset();
    } else bootBtnPressed = false;
}

// ================= RELAY =================
void driveRelay(bool on){
    relayState = on;
    int lvl = cfg.relayActiveLow ? (on ? LOW : HIGH) : (on ? HIGH : LOW);
    digitalWrite(PIN_RELAY, lvl);
}

void updateRelay(){
    if (manualOverride){
        if (millis() - manualOverrideStart > MANUAL_OVERRIDE_MS){
            manualOverride = false;
        } else {
            driveRelay(manualRelayWanted);
            return;
        }
    }

    if (soilFault){
        driveRelay(false);
        return;
    }

    const float SOIL_HYST_PCT = 3.0f;
    const float T_HYST  = 0.5f;
    const float H_HYST  = 2.0f;

    bool soilTrig;
    if (relayState)
        soilTrig = (soilPct < (float)cfg.soilThresholdPct + SOIL_HYST_PCT);
    else
        soilTrig = (soilPct < (float)cfg.soilThresholdPct - SOIL_HYST_PCT);

    bool tempTrig = false, humTrig = false;
    bool shtFresh = shtOK && shtLastValidMs != 0 && (millis() - shtLastValidMs < SHT_STALE_MS);
    if (shtFresh){
        if (!isnan(temperature)){
            if (tempTrigLatched)
                tempTrig = (temperature > cfg.tempHighC - T_HYST) || (temperature < cfg.tempLowC + T_HYST);
            else
                tempTrig = (temperature > cfg.tempHighC + T_HYST) || (temperature < cfg.tempLowC - T_HYST);
            tempTrigLatched = tempTrig;
        }
        if (!isnan(humidity)){
            if (humTrigLatched)
                humTrig = (humidity > cfg.humHighPct - H_HYST) || (humidity < cfg.humLowPct + H_HYST);
            else
                humTrig = (humidity > cfg.humHighPct + H_HYST) || (humidity < cfg.humLowPct - H_HYST);
            humTrigLatched = humTrig;
        }
    }

    bool on = false;
    switch (cfg.relayMode){
        case 0: on = soilTrig; break;
        case 1: on = tempTrig; break;
        case 2: on = humTrig;  break;
        case 3: on = soilTrig || tempTrig || humTrig; break;
        case 4:
            on = soilTrig
                 && (shtFresh ? tempTrig : true)
                 && (shtFresh ? humTrig  : true);
            break;
    }
    driveRelay(on);
}

// ================= WIFI =================
void setupWiFi(){
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP_STA);

    uint8_t mac[6]; WiFi.macAddress(mac);
    char apName[32];
    snprintf(apName, sizeof(apName), "%s-%02X%02X%02X",
             AP_SSID, mac[3], mac[4], mac[5]);
    WiFi.softAP(apName, AP_PASSWORD);

    wifiWantConnect = cfg.useSTA && cfg.staSSID.length() > 0;
    if (wifiWantConnect){
        WiFi.begin(cfg.staSSID.c_str(), cfg.staPass.c_str());
        lastWifiRetry = millis();
    }
}

void checkWiFi(){
    if (!wifiWantConnect) return;
    if (WiFi.status() != WL_CONNECTED){
        if (millis() - lastWifiRetry >= WIFI_RETRY_MS){
            lastWifiRetry = millis();
            WiFi.disconnect(false, false);
            delay(50);
            WiFi.begin(cfg.staSSID.c_str(), cfg.staPass.c_str());
        }
        return;
    }
    if (millis() - lastWifiHealthCheck >= WIFI_HEALTH_MS){
        lastWifiHealthCheck = millis();
        IPAddress gw = WiFi.gatewayIP();
        if (gw == IPAddress(0,0,0,0)) return;
        bool alive = false;
        {
            WiFiClient probe;
            probe.setTimeout(3);
            alive = probe.connect(gw, 80, 3000);
            probe.stop();
        }
        if (!alive){
            WiFiClient probe2;
            probe2.setTimeout(3);
            alive = probe2.connect(gw, 443, 3000);
            probe2.stop();
        }
        if (!alive){
            WiFi.disconnect(false, false);
            delay(100);
            WiFi.begin(cfg.staSSID.c_str(), cfg.staPass.c_str());
            lastWifiRetry = millis();
        }
    }
}

void restartSTA(){
    WiFi.disconnect(false, false);
    delay(100);
    wifiWantConnect = cfg.useSTA && cfg.staSSID.length() > 0;
    if (wifiWantConnect){
        WiFi.begin(cfg.staSSID.c_str(), cfg.staPass.c_str());
        lastWifiRetry = millis();
    }
}

// ================= SENSORS =================
void readSensors(){
    long sum = 0;
    for (int i=0;i<8;i++){ sum += analogRead(PIN_SOIL); delay(3); }
    soilRaw = (int)(sum / 8);

    const int FAULT_MARGIN = 300;
    int hiLimit = cfg.soilDryRaw + FAULT_MARGIN;
    int loLimit = cfg.soilWetRaw - FAULT_MARGIN;
    if (hiLimit > 4094) hiLimit = 4094;
    if (loLimit < 1)    loLimit = 1;
    soilFault = (soilRaw >= hiLimit || soilRaw <= loLimit);

    int range = cfg.soilDryRaw - cfg.soilWetRaw;
    if (range <= 0) range = 1;
    float pct = 100.0f * (float)(cfg.soilDryRaw - soilRaw) / (float)range;
    if (pct < 0) pct = 0; if (pct > 100) pct = 100;
    soilPct = pct;

    bool stale = (shtLastValidMs == 0) || (millis() - shtLastValidMs > SHT_STALE_MS);
    if ((!shtOK || stale) && (millis() - shtLastRetryMs > SHT_RETRY_MS)){
        shtLastRetryMs = millis();
        bool r44 = sht31.begin(0x44);
        bool r45 = r44 ? false : sht31.begin(0x45);
        bool ok  = r44 || r45;
        shtOK = ok;
    }

    if (shtOK){
        float t = sht31.readTemperature();
        float h = sht31.readHumidity();
        bool valid = !isnan(t) && !isnan(h) &&
                     t >= -40.0f && t <= 125.0f &&
                     h >= 0.0f  && h <= 100.1f;
        if (valid){
            temperature     = t;
            humidity        = h;
            shtLastValidMs  = millis();
        } else {
            lastShtErrLog = millis();
        }
    }
}

// ================= JSON =================
String getStatusJSON(){
    StaticJsonDocument<1024> d;
    d["deviceName"]  = cfg.deviceName;
    d["soilPct"]     = soilPct;
    d["soilRaw"]     = soilRaw;
    d["soilFault"]   = soilFault;
    bool fresh = shtOK && shtLastValidMs != 0
                 && (millis() - shtLastValidMs < SHT_STALE_MS);
    d["shtFresh"]    = fresh;
    if (fresh && !isnan(temperature)) d["temperature"] = temperature;
    else                              d["temperature"] = nullptr;
    if (fresh && !isnan(humidity))    d["humidity"]    = humidity;
    else                              d["humidity"]    = nullptr;
    d["relayState"]  = relayState;
    d["shtOK"]       = shtOK;
    d["manualOverride"] = manualOverride;
    unsigned long remain = 0;
    if (manualOverride){
        unsigned long elapsed = millis() - manualOverrideStart;
        remain = (elapsed >= MANUAL_OVERRIDE_MS) ? 0 : (MANUAL_OVERRIDE_MS - elapsed)/1000;
    }
    d["manualRemainS"] = remain;
    d["apIp"]        = WiFi.softAPIP().toString();
    d["staIp"]       = (WiFi.status()==WL_CONNECTED) ? WiFi.localIP().toString() : "";
    d["staConnected"]= (WiFi.status()==WL_CONNECTED);
    d["freeHeap"]    = (uint32_t)ESP.getFreeHeap();
    unsigned long up = millis()/1000;
    char buf[32]; snprintf(buf, sizeof(buf), "%luh %lum %lus", up/3600,(up%3600)/60, up%60);
    d["uptime"] = buf;
    String out; serializeJson(d, out); return out;
}

String getSettingsJSON(){
    StaticJsonDocument<1024> d;
    d["staSSID"]          = cfg.staSSID;
    d["staPass"]          = cfg.staPass.length() ? "********" : "";
    d["useSTA"]           = cfg.useSTA;
    d["soilDryRaw"]       = cfg.soilDryRaw;
    d["soilWetRaw"]       = cfg.soilWetRaw;
    d["soilThresholdPct"] = cfg.soilThresholdPct;
    d["tempHighC"]        = cfg.tempHighC;
    d["tempLowC"]         = cfg.tempLowC;
    d["humHighPct"]       = cfg.humHighPct;
    d["humLowPct"]        = cfg.humLowPct;
    d["relayMode"]        = cfg.relayMode;
    d["relayActiveLow"]   = cfg.relayActiveLow;
    d["deviceName"]       = cfg.deviceName;
    String out; serializeJson(d, out); return out;
}

void applySettingsFromJSON(JsonDocument &d){
    bool wifiChanged = false;

    if (d.containsKey("staSSID")){
        String s = d["staSSID"].as<String>();
        if (s.length() > 32) s = s.substring(0, 32);
        if (s != cfg.staSSID){ cfg.staSSID = s; wifiChanged = true; }
    }
    if (d.containsKey("staPass")){
        String p = d["staPass"].as<String>();
        if (p.length() > 63) p = p.substring(0, 63);
        if (p.length() > 0 && p != "********"){ cfg.staPass = p; wifiChanged = true; }
    }
    if (d.containsKey("useSTA")){
        bool u = d["useSTA"].as<bool>();
        if (u != cfg.useSTA){ cfg.useSTA = u; wifiChanged = true; }
    }
    if (d.containsKey("soilDryRaw")){
        int v = d["soilDryRaw"].as<int>();
        if (v>=0 && v<=4095) cfg.soilDryRaw = v;
    }
    if (d.containsKey("soilWetRaw")){
        int v = d["soilWetRaw"].as<int>();
        if (v>=0 && v<=4095) cfg.soilWetRaw = v;
    }
    if (d.containsKey("soilThresholdPct")){
        int v = d["soilThresholdPct"].as<int>();
        if (v>=0 && v<=100) cfg.soilThresholdPct = v;
    }
    if (d.containsKey("tempHighC"))  { float v = d["tempHighC"].as<float>();  if (!isnan(v) && v > -40.0f && v < 125.0f) cfg.tempHighC  = v; }
    if (d.containsKey("tempLowC"))   { float v = d["tempLowC"].as<float>();   if (!isnan(v) && v > -40.0f && v < 125.0f) cfg.tempLowC   = v; }
    if (d.containsKey("humHighPct")) { float v = d["humHighPct"].as<float>(); if (!isnan(v) && v >= 0.0f && v <= 100.0f) cfg.humHighPct = v; }
    if (d.containsKey("humLowPct"))  { float v = d["humLowPct"].as<float>();  if (!isnan(v) && v >= 0.0f && v <= 100.0f) cfg.humLowPct  = v; }
    if (d.containsKey("relayMode")){
        int v = d["relayMode"].as<int>();
        if (v>=0 && v<=4) cfg.relayMode = v;
    }
    if (d.containsKey("relayActiveLow")){
        bool b = d["relayActiveLow"].as<bool>();
        if (b != cfg.relayActiveLow){
            cfg.relayActiveLow = b;
            driveRelay(relayState);
        }
    }
    if (d.containsKey("deviceName")){
        String n = d["deviceName"].as<String>();
        if (n.length()>0 && n.length()<64) cfg.deviceName = n;
    }

    if (cfg.soilDryRaw <= cfg.soilWetRaw) cfg.soilDryRaw = cfg.soilWetRaw + 100;
    if (cfg.tempHighC <= cfg.tempLowC){
        cfg.tempHighC = cfg.tempLowC + 1.0f;
        if (cfg.tempHighC > 125.0f){ cfg.tempHighC = 125.0f; cfg.tempLowC = 124.0f; }
    }
    if (cfg.humHighPct <= cfg.humLowPct){
        cfg.humHighPct = cfg.humLowPct + 1.0f;
        if (cfg.humHighPct > 100.0f){ cfg.humHighPct = 100.0f; cfg.humLowPct = 99.0f; }
    }
    saveSettings();

    if (wifiChanged){
        restartSTA();
    }
}

// ================= HTTP =================
void handleRoot(){ server.send_P(200, "text/html", DASHBOARD_HTML); }
void handleStatus(){ server.send(200, "application/json", getStatusJSON()); }
void handleGetSettings(){ server.send(200, "application/json", getSettingsJSON()); }

void handlePostSettings(){
    if (!server.hasArg("plain")){
        server.send(400, "application/json", "{\"error\":\"no body\"}");
        return;
    }
    StaticJsonDocument<1024> d;
    if (deserializeJson(d, server.arg("plain"))){
        server.send(400, "application/json", "{\"error\":\"bad json\"}");
        return;
    }
    applySettingsFromJSON(d);
    server.send(200, "application/json", "{\"ok\":true}");
}

void handleRelayToggle(){
    manualOverride    = true;
    manualOverrideStart = millis();
    manualRelayWanted = !relayState;
    driveRelay(manualRelayWanted);
    String out = String("{\"relayState\":") + (relayState?"true":"false") + "}";
    server.send(200, "application/json", out);
}

void handleRelayClear(){
    manualOverride = false;
    updateRelay();
    server.send(200, "application/json", "{\"ok\":true}");
}

void handleFactoryReset(){
    server.send(200, "application/json", "{\"ok\":true}");
    delay(200);
    factoryReset();
}

void handleNotFound(){
    if (server.client().localIP() == WiFi.softAPIP()){
        server.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/", true);
        server.send(302, "text/plain", "");
        return;
    }
    server.send_P(200, "text/html", DASHBOARD_HTML);
}

void setupWebServer(){
    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/status", HTTP_GET, handleStatus);
    server.on("/api/settings", HTTP_GET, handleGetSettings);
    server.on("/api/settings", HTTP_POST, handlePostSettings);
    server.on("/api/relay/toggle", HTTP_POST, handleRelayToggle);
    server.on("/api/relay/clear",  HTTP_POST, handleRelayClear);
    server.on("/api/factory-reset",HTTP_POST, handleFactoryReset);
    server.onNotFound(handleNotFound);
    server.begin();
}

void setupCaptivePortal(){
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
}

// ================= SETUP / LOOP =================
void setup(){
    delay(500);

    pinMode(PIN_BOOT_BTN, INPUT_PULLUP);

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setTimeOut(1000);
    shtOK = sht31.begin(0x44) || sht31.begin(0x45);
    shtLastRetryMs = millis();

    loadSettings();

    pinMode(PIN_RELAY, OUTPUT);
    driveRelay(false);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    setupWiFi();
    setupCaptivePortal();
    setupWebServer();

    readSensors();
    updateRelay();
}

void loop(){
    handleBootButton();
    dnsServer.processNextRequest();
    server.handleClient();
    checkWiFi();

    if (millis() - lastSensorRead >= SENSOR_INTERVAL_MS){
        lastSensorRead = millis();
        readSensors();
        updateRelay();
    }
}
