/*
==================================================================
 * ESP32 Soil Moisture Monitoring + Relay Control + Web Dashboard
 * Hardware:
 *   - Soil Moisture Sensor: GPIO34 (ADC1_CH6, input-only)
 *   - Relay Module:         GPIO25 (Digital Output)
 * Libraries: WiFi, WebServer, EEPROM (all built-in)
==================================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>

// ==================== PIN CONFIGURATION ====================
#define SOIL_MOISTURE_PIN   34
#define RELAY_PIN           25
#define RELAY_ACTIVE_LOW    true

// ==================== EEPROM ====================
#define EEPROM_SIZE         512
#define SETTINGS_ADDR       0
#define SETTINGS_MAGIC      0xAB12

// ==================== WIFI CONFIG EEPROM ====================
#define WIFI_CFG_ADDR       300
#define WIFI_CFG_MAGIC      0xCD34

// ==================== WIFI DEFAULTS ====================
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASS     = "YOUR_WIFI_PASSWORD";
const char* AP_SSID       = "ESP32_Soil_Monitor";
const char* AP_PASS       = "12345678";

// ==================== SETTINGS STRUCT ====================
struct Settings {
  uint16_t magic;
  uint16_t dryValue;
  uint16_t wetValue;
  uint8_t  threshold;
  uint8_t  autoMode;
  uint8_t  relayState;
  uint16_t readInterval;
  uint16_t crc;
};

Settings settings;

// ==================== WIFI CONFIG STRUCT ====================
struct WifiConfig {
  uint16_t magic;
  char     staSSID[33];
  char     staPass[65];
  char     apSSID[33];
  char     apPass[65];
  uint8_t  apEnabled;
  uint8_t  forceAP;
  uint16_t crc;
};

WifiConfig wifiCfg;

// ==================== WIFI STATE MACHINE ====================
enum WifiApplyState : uint8_t {
  WIFI_IDLE = 0,
  WIFI_APPLY_PENDING,
  WIFI_APPLY_CONNECTING,
  WIFI_APPLY_DONE
};

volatile WifiApplyState wifiApplyState = WIFI_IDLE;
unsigned long          wifiApplyStart = 0;

enum ScanState : uint8_t {
  SCAN_IDLE = 0,
  SCAN_RUNNING
};

volatile ScanState scanState = SCAN_IDLE;

// ==================== GLOBALS ====================
WebServer server(80);

volatile int  g_rawMoisture      = 0;
volatile int  g_moisturePercent  = 0;
volatile bool g_relayActive      = false;
portMUX_TYPE  g_mux              = portMUX_INITIALIZER_UNLOCKED;

unsigned long lastReadTime   = 0;
unsigned long lastWiFiCheck  = 0;
bool          apModeActive   = false;

// ==================== FORWARD DECLARATIONS ====================
void loadSettings();
void saveSettings();
uint16_t calcCRC(const Settings& s);
void readSensor();
int  calculateMoisture(int raw);
void setRelayHardware(bool on);
void updateRelay();
void handleRoot();
void handleStatus();
void handleRelay();
void handleAuto();
void handleSettings();
void handleNotFound();
void snapshot(int &rawOut, int &pctOut, bool &relayOut, uint8_t &autoOut, uint8_t &thrOut);
uint16_t calcWifiCRC(const WifiConfig& c);
void loadWifiConfig();
void saveWifiConfig();
void requestWifiApply();
void serviceWifiApply();
String wifiStatusJSON();
void handleWifiPage();
void handleWifiStatus();
void handleWifiScanStart();
void handleWifiScanResults();
void handleWifiSave();
void handleWifiReset();

// ==================== INDEX HTML ====================
const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<meta http-equiv="Cache-Control" content="no-store, no-cache, must-revalidate">
<meta name="color-scheme" content="dark light">
<title>ESP32 Soil Dashboard</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{
  font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
  background:linear-gradient(135deg,#667eea,#764ba2);
  min-height:100vh;display:flex;justify-content:center;align-items:flex-start;
  padding:20px;color:#fff;-webkit-tap-highlight-color:transparent;
}
.container{
  background:rgba(255,255,255,.15);backdrop-filter:blur(10px);
  -webkit-backdrop-filter:blur(10px);border-radius:20px;padding:24px;
  box-shadow:0 8px 32px rgba(0,0,0,.3);width:100%;max-width:720px;
}
h1{text-align:center;margin-bottom:16px;font-size:1.6em}
.card{background:rgba(255,255,255,.2);border-radius:15px;padding:18px;margin-bottom:16px}
.card h2{font-size:1.1em;margin-bottom:12px;border-bottom:1px solid rgba(255,255,255,.3);padding-bottom:8px}
.sensor-value{font-size:3em;font-weight:700;text-align:center;margin:8px 0;font-variant-numeric:tabular-nums}
.sensor-label{text-align:center;opacity:.8;font-size:.9em}
.progress-bar{width:100%;height:26px;background:rgba(0,0,0,.3);
  border-radius:13px;overflow:hidden;margin:14px 0;position:relative}
.progress-fill{height:100%;transition:width .4s ease,background .4s ease;
  display:flex;align-items:center;justify-content:center;
  font-weight:700;font-size:.85em;white-space:nowrap;overflow:hidden}
.relay-status{display:flex;justify-content:center;margin:12px 0}
.relay-indicator{width:70px;height:70px;border-radius:50%;
  display:flex;align-items:center;justify-content:center;
  font-size:1.4em;font-weight:700;transition:all .3s;
  box-shadow:0 4px 15px rgba(0,0,0,.3);user-select:none}
.relay-on{background:#4CAF50;box-shadow:0 0 20px #4CAF50,0 4px 15px rgba(0,0,0,.3)}
.relay-off{background:#f44336;box-shadow:0 0 20px #f44336,0 4px 15px rgba(0,0,0,.3)}
.btn{padding:11px 20px;border:none;border-radius:10px;font-size:.95em;
  font-weight:600;cursor:pointer;transition:transform .15s,opacity .15s;
  color:#fff;margin:4px;touch-action:manipulation}
.btn:hover:not(:disabled){transform:scale(1.05)}
.btn:active:not(:disabled){transform:scale(.95)}
.btn:disabled{opacity:.5;cursor:not-allowed;transform:none}
.btn-primary{background:#2196F3}.btn-success{background:#4CAF50}
.btn-danger{background:#f44336}.btn-warning{background:#ff9800}
.btn-group{display:flex;flex-wrap:wrap;justify-content:center;gap:8px;margin:10px 0}
.settings-grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.setting-item{display:flex;flex-direction:column}
.setting-item label{font-size:.82em;margin-bottom:4px;opacity:.9}
.setting-item input{padding:9px;border:2px solid rgba(255,255,255,.3);
  border-radius:8px;background:rgba(0,0,0,.2);color:#fff;font-size:1em;
  width:100%;font-variant-numeric:tabular-nums}
.setting-item input:focus{outline:none;border-color:#2196F3}
.setting-item input:invalid{border-color:#f44336}
.setting-item input::-webkit-outer-spin-button,
.setting-item input::-webkit-inner-spin-button{-webkit-appearance:none;margin:0}
.setting-item input[type=number]{-moz-appearance:textfield}
.status-badge{display:inline-block;padding:4px 14px;border-radius:20px;
  font-size:.8em;font-weight:700}
.badge-auto{background:#2196F3}.badge-manual{background:#ff9800}
.center{text-align:center}.small{font-size:.85em;opacity:.75}
.footer{text-align:center;font-size:.8em;opacity:.6;margin-top:14px}
.footer a{color:#fff;text-decoration:underline;opacity:.9}
.toast{position:fixed;bottom:20px;left:50%;
  transform:translateX(-50%) translateY(100px);
  background:rgba(0,0,0,.85);color:#fff;padding:12px 24px;
  border-radius:10px;font-size:.9em;font-weight:600;
  box-shadow:0 4px 20px rgba(0,0,0,.4);
  transition:transform .3s ease,opacity .3s ease;
  opacity:0;pointer-events:none;z-index:9999;max-width:90vw;text-align:center}
.toast.show{transform:translateX(-50%) translateY(0);opacity:1}
.toast.error{background:rgba(244,67,54,.95)}
.toast.success{background:rgba(76,175,80,.95)}
.offline-banner{display:none;background:rgba(244,67,54,.9);
  padding:8px 14px;border-radius:8px;text-align:center;
  font-size:.85em;font-weight:600;margin-bottom:12px}
.offline-banner.show{display:block}
@media(max-width:520px){
  .settings-grid{grid-template-columns:1fr}
  h1{font-size:1.3em}.sensor-value{font-size:2.4em}
}
</style>
</head>
<body>
<div class="container">
  <div class="offline-banner" id="offlineBanner">⚠️ ESP32 connection lost. Retrying…</div>
  <h1>🌱 Soil Moisture Dashboard</h1>

  <div class="card">
    <h2>📊 Sensor Reading</h2>
    <div class="sensor-value" id="moistureValue">--%</div>
    <div class="sensor-label">Soil Moisture</div>
    <div class="progress-bar">
      <div class="progress-fill" id="moistureBar" style="width:0%;background:#4CAF50"></div>
    </div>
    <div class="center small">
      Raw ADC: <span id="rawValue">--</span> |
      Threshold: <span id="thresholdValue">--</span>%
    </div>
  </div>

  <div class="card">
    <h2>🔌 Relay / Pump</h2>
    <div class="relay-status">
      <div class="relay-indicator relay-off" id="relayIndicator">OFF</div>
    </div>
    <div class="center">
      <span class="status-badge badge-auto" id="modeBadge">AUTO</span>
    </div>
    <div class="btn-group">
      <button class="btn btn-success" id="btnOn"   type="button">ON</button>
      <button class="btn btn-danger"  id="btnOff"  type="button">OFF</button>
      <button class="btn btn-warning" id="btnAuto" type="button">Toggle Auto</button>
    </div>
  </div>

  <div class="card">
    <h2>⚙️ Settings</h2>
    <div class="settings-grid">
      <div class="setting-item"><label for="dryValue">Dry ADC (air)</label>
        <input type="number" id="dryValue" min="0" max="4095" step="1"
               inputmode="numeric" value="3200" required></div>
      <div class="setting-item"><label for="wetValue">Wet ADC (water)</label>
        <input type="number" id="wetValue" min="0" max="4095" step="1"
               inputmode="numeric" value="1400" required></div>
      <div class="setting-item"><label for="threshold">Threshold (%)</label>
        <input type="number" id="threshold" min="1" max="99" step="1"
               inputmode="numeric" value="30" required></div>
      <div class="setting-item"><label for="readInterval">Read Interval (s)</label>
        <input type="number" id="readInterval" min="2" max="3600" step="1"
               inputmode="numeric" value="5" required></div>
    </div>
    <div class="btn-group">
      <button class="btn btn-primary" id="btnSave"    type="button">💾 Save</button>
      <button class="btn btn-primary" id="btnRefresh" type="button">🔄 Refresh</button>
    </div>
  </div>

  <div class="footer">ESP32 Soil Controller | <span id="ipAddr">--</span> | <a href="/wifi">⚙️ WiFi</a></div>
</div>

<div class="toast" id="toast"></div>

<script>
'use strict';

const FETCH_TIMEOUT  = 5000;
const POLL_MS        = 2000;
const OFFLINE_MS     = 8000;
const MAX_BACKOFF_MS = 30000;

let currentAuto  = 1;
let pollTimer    = null;
let lastFetchOK  = Date.now();
let failCount    = 0;
let getInflight  = null;
let postInflight = false;

const $ = id => document.getElementById(id);
const elMoisture  = $('moistureValue');
const elRaw       = $('rawValue');
const elThreshold = $('thresholdValue');
const elBar       = $('moistureBar');
const elRelayInd  = $('relayIndicator');
const elModeBadge = $('modeBadge');
const elOffline   = $('offlineBanner');
const elToast     = $('toast');
const elIP        = $('ipAddr');
const elDry       = $('dryValue');
const elWet       = $('wetValue');
const elThr       = $('threshold');
const elInt       = $('readInterval');
const btnOn       = $('btnOn');
const btnOff      = $('btnOff');
const btnAuto     = $('btnAuto');
const btnSave     = $('btnSave');
const btnRefresh  = $('btnRefresh');

const lastData = {
  dryValue:null, wetValue:null,
  threshold:null, readInterval:null
};

let toastTimer = null;
function showToast(msg, type) {
  elToast.textContent = msg;
  elToast.className = 'toast show ' + (type || '');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => {
    elToast.className = 'toast ' + (type || '');
  }, 2200);
}

function setBusy(el, busy) { if (el) el.disabled = !!busy; }

async function doGet(url) {
  if (getInflight) { try { getInflight.abort(); } catch(e){} }
  const ctrl = new AbortController();
  getInflight = ctrl;
  const to = setTimeout(() => ctrl.abort(), FETCH_TIMEOUT);
  try {
    const res = await fetch(url, { signal: ctrl.signal, cache:'no-store' });
    clearTimeout(to);
    if (!res.ok) {
      let msg = 'HTTP ' + res.status;
      try { const j = await res.json(); if (j && j.error) msg = j.error; } catch(e){}
      throw new Error(msg);
    }
    const text = await res.text();
    if (!text) throw new Error('Empty response');
    try { return JSON.parse(text); }
    catch(e) { throw new Error('Bad JSON'); }
  } finally {
    clearTimeout(to);
    if (getInflight === ctrl) getInflight = null;
  }
}

async function doPost(url, body) {
  if (postInflight) throw new Error('Another save in progress');
  postInflight = true;
  const ctrl = new AbortController();
  const to = setTimeout(() => ctrl.abort(), FETCH_TIMEOUT);
  try {
    const res = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type':'application/json' },
      body: JSON.stringify(body),
      signal: ctrl.signal,
      cache: 'no-store'
    });
    clearTimeout(to);
    const text = await res.text();
    let data = null;
    try { data = text ? JSON.parse(text) : null; } catch(e){}
    if (!res.ok) {
      const msg = (data && data.error) ? data.error : ('HTTP ' + res.status);
      throw new Error(msg);
    }
    return data;
  } finally {
    clearTimeout(to);
    postInflight = false;
  }
}

function updateUI(d) {
  if (!d || typeof d !== 'object') return;

  const pct = Number.isFinite(Number(d.moisture))
            ? Math.max(0, Math.min(100, Number(d.moisture))) : 0;
  elMoisture.textContent = pct + '%';
  elRaw.textContent      = Number.isFinite(Number(d.raw)) ? d.raw : '--';
  elThreshold.textContent= Number.isFinite(Number(d.threshold)) ? d.threshold : '--';

  elBar.style.width = pct + '%';
  elBar.style.background =
    pct < 30 ? '#f44336' :
    pct < 60 ? '#ff9800' : '#4CAF50';

  elBar.textContent = (pct >= 15) ? (pct + '%') : '';

  const relayOn = (d.relay === 1 || d.relay === true);
  elRelayInd.textContent = relayOn ? 'ON' : 'OFF';
  elRelayInd.className   = 'relay-indicator ' + (relayOn ? 'relay-on' : 'relay-off');

  const auto = (d.auto === 1 || d.auto === true);
  currentAuto = auto ? 1 : 0;
  elModeBadge.textContent = auto ? 'AUTO' : 'MANUAL';
  elModeBadge.className   = 'status-badge ' + (auto ? 'badge-auto' : 'badge-manual');

  const pairs = [[elDry, d.dryValue],[elWet, d.wetValue],
                 [elThr, d.threshold],[elInt, d.readInterval]];
  for (const [el, val] of pairs) {
    if (document.activeElement !== el && Number.isFinite(Number(val))) {
      if (el.value !== String(val)) el.value = val;
    }
  }

  if (Number.isFinite(Number(d.dryValue)))     lastData.dryValue     = Number(d.dryValue);
  if (Number.isFinite(Number(d.wetValue)))     lastData.wetValue     = Number(d.wetValue);
  if (Number.isFinite(Number(d.threshold)))    lastData.threshold    = Number(d.threshold);
  if (Number.isFinite(Number(d.readInterval))) lastData.readInterval = Number(d.readInterval);

  btnOn.disabled = btnOff.disabled = btnAuto.disabled = false;
}

async function fetchStatus(silent) {
  try {
    const d = await doGet('/status');
    if (d) updateUI(d);
    failCount = 0;
    lastFetchOK = Date.now();
    elOffline.classList.remove('show');
  } catch (err) {
    if (err.name === 'AbortError') return;
    failCount++;
    if (!silent) console.warn('Status:', err.message);
    if (Date.now() - lastFetchOK > OFFLINE_MS) {
      elOffline.classList.add('show');
    }
  }
}

function scheduleNextPoll() {
  if (pollTimer) clearTimeout(pollTimer);
  const base = POLL_MS;
  const delay = failCount > 0
    ? Math.min(base * Math.pow(2, failCount - 1), MAX_BACKOFF_MS)
    : base;

  pollTimer = setTimeout(async () => {
    if (!document.hidden) await fetchStatus(true);
    scheduleNextPoll();
  }, delay);
}

document.addEventListener('visibilitychange', () => {
  if (!document.hidden) {
    failCount = 0;
    fetchStatus(true);
  }
});

async function setRelay(state) {
  if (state === 1) {
    if (!confirm('Turn relay ON manually?\n\nThis switches to MANUAL mode.')) return;
  }
  setBusy(btnOn, true); setBusy(btnOff, true);
  try {
    const d = await doGet('/relay?state=' + state);
    if (d) updateUI(d);
    showToast('Relay ' + (state ? 'ON' : 'OFF'), 'success');
  } catch (err) {
    showToast('Relay error: ' + err.message, 'error');
  } finally {
    setBusy(btnOn, false); setBusy(btnOff, false);
  }
}

async function toggleAuto() {
  const next = currentAuto ? 0 : 1;
  setBusy(btnAuto, true);
  try {
    const d = await doGet('/auto?state=' + next);
    if (d) updateUI(d);
    showToast('Mode: ' + (next ? 'AUTO' : 'MANUAL'), 'success');
  } catch (err) {
    showToast('Mode error: ' + err.message, 'error');
  } finally {
    setBusy(btnAuto, false);
  }
}

async function saveSettings() {
  const dv = Number(elDry.value.trim());
  const wv = Number(elWet.value.trim());
  const th = Number(elThr.value.trim());
  const ri = Number(elInt.value.trim());

  if (!Number.isInteger(dv) || !Number.isInteger(wv) ||
      !Number.isInteger(th) || !Number.isInteger(ri)) {
    showToast('Please enter whole numbers only', 'error');
    return;
  }

  if (dv < 0 || dv > 4095) { showToast('Dry ADC must be 0-4095','error'); elDry.focus(); return; }
  if (wv < 0 || wv > 4095) { showToast('Wet ADC must be 0-4095','error'); elWet.focus(); return; }
  if (dv <= wv)            { showToast('Dry ADC must be > Wet ADC','error'); elDry.focus(); return; }
  if (th < 1 || th > 99)   { showToast('Threshold 1-99','error'); elThr.focus(); return; }
  if (ri < 2 || ri > 3600) { showToast('Interval 2-3600','error'); elInt.focus(); return; }

  if (lastData.dryValue !== null &&
      dv === lastData.dryValue && wv === lastData.wetValue &&
      th === lastData.threshold && ri === lastData.readInterval) {
    showToast('No changes to save');
    return;
  }

  setBusy(btnSave, true);
  try {
    const d = await doPost('/settings', {
      dryValue: dv, wetValue: wv, threshold: th, readInterval: ri
    });
    if (d) updateUI(d);
    showToast('Settings saved ✓', 'success');
  } catch (err) {
    showToast('Save failed: ' + err.message, 'error');
  } finally {
    setBusy(btnSave, false);
  }
}

btnOn.addEventListener('click',      () => setRelay(1));
btnOff.addEventListener('click',     () => setRelay(0));
btnAuto.addEventListener('click',    toggleAuto);
btnSave.addEventListener('click',    saveSettings);
btnRefresh.addEventListener('click', () => { failCount = 0; fetchStatus(false); });

[elDry, elWet, elThr, elInt].forEach(el => {
  el.addEventListener('keydown', e => {
    if (e.key === 'Enter') { e.preventDefault(); saveSettings(); }
  });
});

elIP.textContent = location.hostname || 'ESP32';
btnOn.disabled = btnOff.disabled = btnAuto.disabled = true;
fetchStatus(false).then(() => {
  btnOn.disabled = btnOff.disabled = btnAuto.disabled = false;
});
scheduleNextPoll();

window.addEventListener('beforeunload', () => {
  if (pollTimer) clearTimeout(pollTimer);
});
</script>
</body>
</html>
)HTMLPAGE";

// ==================== WIFI HTML ====================
const char WIFI_HTML[] PROGMEM = R"WIFIPAGE(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<meta http-equiv="Cache-Control" content="no-store, no-cache, must-revalidate">
<meta name="color-scheme" content="dark light">
<title>ESP32 WiFi Settings</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{
  font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
  background:linear-gradient(135deg,#667eea,#764ba2);
  min-height:100vh;display:flex;justify-content:center;align-items:flex-start;
  padding:20px;color:#fff;-webkit-tap-highlight-color:transparent;
}
.container{
  background:rgba(255,255,255,.15);backdrop-filter:blur(10px);
  -webkit-backdrop-filter:blur(10px);border-radius:20px;padding:24px;
  box-shadow:0 8px 32px rgba(0,0,0,.3);width:100%;max-width:720px;
}
h1{text-align:center;margin-bottom:16px;font-size:1.6em}
.card{background:rgba(255,255,255,.2);border-radius:15px;padding:18px;margin-bottom:16px}
.card h2{font-size:1.1em;margin-bottom:12px;border-bottom:1px solid rgba(255,255,255,.3);padding-bottom:8px}
.info-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;font-size:.9em}
.info-item{display:flex;flex-direction:column;padding:8px;background:rgba(0,0,0,.15);border-radius:8px}
.info-item .lbl{opacity:.75;font-size:.8em;margin-bottom:3px}
.info-item .val{font-weight:600;word-break:break-all}
.badge{display:inline-block;padding:2px 10px;border-radius:20px;font-size:.8em;font-weight:700}
.badge-ok{background:#4CAF50}
.badge-warn{background:#ff9800}
.badge-off{background:#f44336}
.badge-info{background:#2196F3}
.form-group{display:flex;flex-direction:column;margin-bottom:12px}
.form-group label{font-size:.85em;margin-bottom:5px;opacity:.9}
.form-group input[type=text],
.form-group input[type=password]{
  padding:10px;border:2px solid rgba(255,255,255,.3);
  border-radius:8px;background:rgba(0,0,0,.2);
  color:#fff;font-size:1em;width:100%;font-family:inherit;
}
.form-group input:focus{outline:none;border-color:#2196F3}
.form-row{display:flex;align-items:center;gap:10px;margin-bottom:12px}
.form-row input[type=checkbox]{
  width:22px;height:22px;cursor:pointer;accent-color:#2196F3;
}
.form-row label{cursor:pointer;font-size:.9em}
.btn{padding:11px 20px;border:none;border-radius:10px;font-size:.95em;
  font-weight:600;cursor:pointer;transition:transform .15s,opacity .15s;
  color:#fff;margin:4px;touch-action:manipulation}
.btn:hover:not(:disabled){transform:scale(1.05)}
.btn:active:not(:disabled){transform:scale(.95)}
.btn:disabled{opacity:.5;cursor:not-allowed}
.btn-primary{background:#2196F3}
.btn-success{background:#4CAF50}
.btn-danger{background:#f44336}
.btn-warning{background:#ff9800}
.btn-secondary{background:#546E7A}
.btn-group{display:flex;flex-wrap:wrap;justify-content:center;gap:8px;margin:10px 0}
.net-list{max-height:250px;overflow-y:auto;background:rgba(0,0,0,.15);
  border-radius:10px;padding:8px;margin-top:8px}
.net-item{padding:10px;border-radius:8px;cursor:pointer;
  display:flex;justify-content:space-between;align-items:center;
  transition:background .15s;margin-bottom:4px}
.net-item:hover{background:rgba(255,255,255,.1)}
.net-item.active{background:rgba(33,150,243,.4)}
.net-ssid{font-weight:600;font-size:.95em}
.net-rssi{font-size:.8em;opacity:.85}
.toast{position:fixed;bottom:20px;left:50%;
  transform:translateX(-50%) translateY(100px);
  background:rgba(0,0,0,.85);color:#fff;padding:12px 24px;
  border-radius:10px;font-size:.9em;font-weight:600;
  box-shadow:0 4px 20px rgba(0,0,0,.4);
  transition:transform .3s ease,opacity .3s ease;
  opacity:0;pointer-events:none;z-index:9999;max-width:90vw;text-align:center}
.toast.show{transform:translateX(-50%) translateY(0);opacity:1}
.toast.error{background:rgba(244,67,54,.95)}
.toast.success{background:rgba(76,175,80,.95)}
.footer{text-align:center;font-size:.8em;opacity:.6;margin-top:14px}
.footer a{color:#fff;text-decoration:underline;opacity:.9}
@media(max-width:520px){
  .info-grid{grid-template-columns:1fr}
  h1{font-size:1.3em}
}
</style>
</head>
<body>
<div class="container">
  <h1>📶 WiFi Settings</h1>

  <div class="card">
    <h2>📡 Current Status</h2>
    <div class="info-grid">
      <div class="info-item"><span class="lbl">Mode</span>
        <span class="val"><span class="badge badge-info" id="curMode">--</span></span></div>
      <div class="info-item"><span class="lbl">Connection</span>
        <span class="val"><span class="badge badge-off" id="curConn">--</span></span></div>
      <div class="info-item"><span class="lbl">STA IP</span>
        <span class="val" id="curStaIP">--</span></div>
      <div class="info-item"><span class="lbl">STA Signal</span>
        <span class="val" id="curRSSI">--</span></div>
      <div class="info-item"><span class="lbl">AP IP</span>
        <span class="val" id="curApIP">--</span></div>
      <div class="info-item"><span class="lbl">AP Clients</span>
        <span class="val" id="curClients">--</span></div>
      <div class="info-item"><span class="lbl">STA SSID</span>
        <span class="val" id="curStaSSID">--</span></div>
      <div class="info-item"><span class="lbl">AP SSID</span>
        <span class="val" id="curApSSID">--</span></div>
    </div>
    <div class="btn-group">
      <button class="btn btn-primary" id="btnRefresh" type="button">🔄 Refresh</button>
    </div>
  </div>

  <div class="card">
    <h2>📡 Station (Connect to Router)</h2>
    <div class="form-group">
      <label for="staSSID">Network Name (SSID)</label>
      <input type="text" id="staSSID" maxlength="32" placeholder="Your WiFi network"
             autocomplete="off" autocapitalize="off" spellcheck="false">
    </div>
    <div class="form-group">
      <label for="staPass">Password (leave blank to keep current)</label>
      <input type="password" id="staPass" maxlength="64" placeholder="••••••••"
             autocomplete="new-password">
    </div>
    <div class="form-row">
      <input type="checkbox" id="clearStaPass">
      <label for="clearStaPass">Clear password (open network)</label>
    </div>
    <div class="btn-group">
      <button class="btn btn-secondary" id="btnScan" type="button">🔍 Scan Networks</button>
    </div>
    <div class="net-list" id="netList" style="display:none;"></div>
  </div>

  <div class="card">
    <h2>📻 Access Point (Broadcast Your Own)</h2>
    <div class="form-row">
      <input type="checkbox" id="apEnabled">
      <label for="apEnabled">Enable Access Point</label>
    </div>
    <div class="form-group">
      <label for="apSSID">AP Network Name</label>
      <input type="text" id="apSSID" maxlength="32" placeholder="ESP32_Soil_Monitor"
             autocomplete="off" autocapitalize="off" spellcheck="false">
    </div>
    <div class="form-group">
      <label for="apPass">AP Password (8-64 chars, blank = keep)</label>
      <input type="password" id="apPass" maxlength="64" placeholder="••••••••"
             autocomplete="new-password">
    </div>
    <div class="form-row">
      <input type="checkbox" id="forceAP">
      <label for="forceAP">Force AP Only (disable STA)</label>
    </div>
  </div>

  <div class="card">
    <h2>💾 Actions</h2>
    <div class="btn-group">
      <button class="btn btn-success" id="btnSave"  type="button">💾 Save & Apply</button>
      <button class="btn btn-danger"  id="btnReset" type="button">↺ Reset to Defaults</button>
      <button class="btn btn-warning" id="btnBack"  type="button">← Back to Dashboard</button>
    </div>
  </div>

  <div class="footer">ESP32 Soil Controller | <span id="ipAddr">--</span></div>
</div>

<div class="toast" id="toast"></div>

<script>
'use strict';

const FETCH_TIMEOUT = 8000;

const $ = id => document.getElementById(id);
const elMode     = $('curMode');
const elConn     = $('curConn');
const elStaIP    = $('curStaIP');
const elRSSI     = $('curRSSI');
const elApIP     = $('curApIP');
const elClients  = $('curClients');
const elStaSSID  = $('curStaSSID');
const elApSSID   = $('curApSSID');
const elIP       = $('ipAddr');
const elToast    = $('toast');

const inpStaSSID   = $('staSSID');
const inpStaPass   = $('staPass');
const inpApSSID    = $('apSSID');
const inpApPass    = $('apPass');
const inpApEnab    = $('apEnabled');
const inpForceAP   = $('forceAP');
const inpClearSta  = $('clearStaPass');

const btnRefresh = $('btnRefresh');
const btnScan    = $('btnScan');
const btnSave    = $('btnSave');
const btnReset   = $('btnReset');
const btnBack    = $('btnBack');
const netList    = $('netList');

let toastTimer = null;
function showToast(msg, type) {
  elToast.textContent = msg;
  elToast.className = 'toast show ' + (type || '');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => {
    elToast.className = 'toast ' + (type || '');
  }, 2500);
}

function setBusy(el, busy) { if (el) el.disabled = !!busy; }

async function apiFetch(url, opts) {
  const ctrl = new AbortController();
  const to = setTimeout(() => ctrl.abort(), FETCH_TIMEOUT);
  try {
    const res = await fetch(url, Object.assign({}, opts, {
      signal: ctrl.signal,
      cache: 'no-store'
    }));
    clearTimeout(to);
    const text = await res.text();
    let data = null;
    try { data = text ? JSON.parse(text) : null; } catch(e){}
    if (!res.ok) {
      const msg = (data && data.error) ? data.error : ('HTTP ' + res.status);
      throw new Error(msg);
    }
    return data;
  } finally {
    clearTimeout(to);
  }
}

function updateStatus(d) {
  if (!d) return;
  elMode.textContent = d.mode || '--';
  elMode.className = 'badge badge-info';

  const connected = d.connected === 1;
  elConn.textContent = connected ? 'Connected' : 'Disconnected';
  elConn.className = 'badge ' + (connected ? 'badge-ok' : 'badge-off');

  elStaIP.textContent   = d.staIP || '--';
  elRSSI.textContent    = d.rssi ? (d.rssi + ' dBm') : '--';
  elApIP.textContent    = d.apIP || '--';
  elClients.textContent = d.apClients !== undefined ? d.apClients : '--';
  elStaSSID.textContent = d.staSSID || '--';
  elApSSID.textContent  = d.apSSID || '--';

  if (document.activeElement !== inpStaSSID) inpStaSSID.value = d.staSSID || '';
  if (document.activeElement !== inpApSSID)  inpApSSID.value  = d.apSSID  || '';
  inpApEnab.checked  = d.apEnabled === 1;
  inpForceAP.checked = d.forceAP === 1;
}

async function refreshStatus() {
  try {
    const d = await apiFetch('/wifi/status');
    updateStatus(d);
  } catch (err) {
    showToast('Status error: ' + err.message, 'error');
  }
}

function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, c => ({
    '&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'
  })[c]);
}

async function scanNetworks() {
  setBusy(btnScan, true);
  netList.style.display = 'block';
  netList.innerHTML = '<div style="padding:10px;text-align:center;">Scanning…</div>';

  try {
    await apiFetch('/wifi/scan');

    let networks = null;
    const deadline = Date.now() + 25000;
    while (Date.now() < deadline) {
      await new Promise(r => setTimeout(r, 700));
      const r = await apiFetch('/wifi/scan/results');
      if (r.state === 'running') continue;
      if (r.state === 'failed') throw new Error('Scan failed');
      if (r.state === 'done') { networks = r.networks; break; }
    }

    if (!networks) throw new Error('Scan timeout');

    if (!Array.isArray(networks) || networks.length === 0) {
      netList.innerHTML = '<div style="padding:10px;text-align:center;opacity:.7;">No networks found</div>';
      return;
    }

    networks.sort((a, b) => (b.rssi || 0) - (a.rssi || 0));

    netList.innerHTML = '';
    for (const n of networks) {
      const div = document.createElement('div');
      div.className = 'net-item';
      const ssid = n.ssid || '(hidden)';
      const rssi = n.rssi || 0;
      const enc  = n.enc === 1 ? '🔒' : '🔓';
      div.innerHTML = '<span class="net-ssid">' + enc + ' ' + escapeHtml(ssid) + '</span>' +
                      '<span class="net-rssi">' + rssi + ' dBm</span>';
      div.addEventListener('click', () => {
        inpStaSSID.value = ssid;
        inpStaPass.value = '';
        inpClearSta.checked = (n.enc === 0);
        inpStaPass.focus();
        document.querySelectorAll('.net-item').forEach(x => x.classList.remove('active'));
        div.classList.add('active');
        showToast('Selected: ' + ssid);
      });
      netList.appendChild(div);
    }
  } catch (err) {
    netList.innerHTML = '<div style="padding:10px;text-align:center;color:#f44336;">Scan failed: ' +
                        escapeHtml(err.message) + '</div>';
  } finally {
    setBusy(btnScan, false);
  }
}

async function saveWifi() {
  const staSSID = inpStaSSID.value.trim();
  const staPass = inpStaPass.value;
  const apSSID  = inpApSSID.value.trim();
  const apPass  = inpApPass.value;
  const clearSta = inpClearSta.checked;

  if (staSSID.length > 32) {
    showToast('STA SSID must be 32 chars or less', 'error'); return;
  }
  if (apSSID.length < 1 || apSSID.length > 32) {
    showToast('AP SSID must be 1-32 chars', 'error'); inpApSSID.focus(); return;
  }
  if (apPass.length > 0 && (apPass.length < 8 || apPass.length > 64)) {
    showToast('AP password must be 8-64 chars', 'error'); inpApPass.focus(); return;
  }
  if (!inpForceAP.checked && !inpApEnab.checked && staSSID.length === 0) {
    showToast('Enable AP or enter STA SSID', 'error'); return;
  }

  setBusy(btnSave, true);
  try {
    const d = await apiFetch('/wifi/save', {
      method: 'POST',
      headers: { 'Content-Type':'application/json' },
      body: JSON.stringify({
        staSSID: staSSID,
        staPass: staPass,
        apSSID:  apSSID,
        apPass:  apPass,
        apEnabled: inpApEnab.checked ? 1 : 0,
        forceAP:   inpForceAP.checked ? 1 : 0,
        clearStaPass: clearSta ? "1" : "0"
      })
    });
    showToast('Saved ✓ Device is reconnecting…', 'success');
    if (d) updateStatus(d);
    inpStaPass.value = '';
    inpApPass.value  = '';
    inpClearSta.checked = false;
    setTimeout(refreshStatus, 3000);
  } catch (err) {
    showToast('Save failed: ' + err.message, 'error');
  } finally {
    setBusy(btnSave, false);
  }
}

async function resetWifi() {
  if (!confirm('Reset WiFi settings to defaults?\n\nDevice will reconnect.')) return;
  setBusy(btnReset, true);
  try {
    const d = await apiFetch('/wifi/reset');
    showToast('Reset ✓ Reconnecting…', 'success');
    if (d) updateStatus(d);
    setTimeout(refreshStatus, 3000);
  } catch (err) {
    showToast('Reset failed: ' + err.message, 'error');
  } finally {
    setBusy(btnReset, false);
  }
}

btnRefresh.addEventListener('click', refreshStatus);
btnScan.addEventListener('click',    scanNetworks);
btnSave.addEventListener('click',    saveWifi);
btnReset.addEventListener('click',   resetWifi);
btnBack.addEventListener('click',    () => { location.href = '/'; });

elIP.textContent = location.hostname || 'ESP32';
refreshStatus();
setInterval(refreshStatus, 5000);
</script>
</body>
</html>
)WIFIPAGE";

// ==================== SETUP ====================
void setup() {
  analogReadResolution(12);
#if defined(ADC_ATTEN_DB_11)
  analogSetAttenuation(ADC_ATTEN_DB_11);
#elif defined(ADC_11db)
  analogSetAttenuation(ADC_11db);
#elif defined(ADC_ATTENUATION_11_DB)
  analogSetAttenuation(ADC_ATTENUATION_11_DB);
#endif
  pinMode(SOIL_MOISTURE_PIN, INPUT);

  pinMode(RELAY_PIN, OUTPUT);
#if RELAY_ACTIVE_LOW
  digitalWrite(RELAY_PIN, HIGH);
#else
  digitalWrite(RELAY_PIN, LOW);
#endif
  setRelayHardware(false);

  EEPROM.begin(EEPROM_SIZE);
  loadSettings();
  loadWifiConfig();

  WiFi.persistent(false);
  WiFi.setSleep(false);

  WiFi.mode(wifiCfg.forceAP ? WIFI_AP :
            (wifiCfg.apEnabled ? WIFI_AP_STA : WIFI_STA));

  if (wifiCfg.forceAP) {
    WiFi.softAP(wifiCfg.apSSID, wifiCfg.apPass);
    apModeActive = true;
  } else {
    if (wifiCfg.apEnabled) {
      WiFi.softAP(wifiCfg.apSSID, wifiCfg.apPass);
    }
    WiFi.begin(wifiCfg.staSSID, wifiCfg.staPass);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
      delay(300);
      yield();
    }

    apModeActive = (WiFi.status() != WL_CONNECTED) && wifiCfg.apEnabled;
  }

  server.on("/",                  HTTP_GET,  handleRoot);
  server.on("/status",            HTTP_GET,  handleStatus);
  server.on("/relay",             HTTP_GET,  handleRelay);
  server.on("/auto",              HTTP_GET,  handleAuto);
  server.on("/settings",          HTTP_POST, handleSettings);
  server.on("/wifi",              HTTP_GET,  handleWifiPage);
  server.on("/wifi/status",       HTTP_GET,  handleWifiStatus);
  server.on("/wifi/scan",         HTTP_GET,  handleWifiScanStart);
  server.on("/wifi/scan/results", HTTP_GET,  handleWifiScanResults);
  server.on("/wifi/save",         HTTP_POST, handleWifiSave);
  server.on("/wifi/reset",        HTTP_GET,  handleWifiReset);
  server.onNotFound(handleNotFound);
  server.begin();

  readSensor();
  updateRelay();
  lastReadTime = millis();
}

// ==================== LOOP ====================
void loop() {
  server.handleClient();
  serviceWifiApply();

  unsigned long now = millis();

  uint16_t interval = settings.readInterval;
  if (interval < 2) interval = 2;
  if (now - lastReadTime >= (unsigned long)interval * 1000UL) {
    lastReadTime = now;
    readSensor();
    updateRelay();
  }

  if (!apModeActive && !wifiCfg.forceAP && wifiApplyState == WIFI_IDLE &&
      (now - lastWiFiCheck >= 15000UL)) {
    lastWiFiCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.disconnect();
      WiFi.begin(wifiCfg.staSSID, wifiCfg.staPass);
    }
  }

  delay(5);
  yield();
}

// ==================== SENSOR ====================
void readSensor() {
  const int SAMPLES = 16;
  const int DISCARD = 4;
  long sum = 0;
  int count = 0;
  for (int i = 0; i < SAMPLES; i++) {
    int v = analogRead(SOIL_MOISTURE_PIN);
    if (i >= DISCARD) { sum += v; count++; }
    delayMicroseconds(200);
  }
  int raw = count ? (int)(sum / count) : 0;

  int pct = calculateMoisture(raw);

  static int  lastPct    = -1;
  static int  stableRuns = 0;

  if (lastPct >= 0 && abs(pct - lastPct) > 40) {
    stableRuns++;
    if (stableRuns < 3) return;
  } else {
    stableRuns = 0;
  }
  lastPct = pct;

  portENTER_CRITICAL(&g_mux);
  g_rawMoisture     = raw;
  g_moisturePercent = pct;
  portEXIT_CRITICAL(&g_mux);
}

int calculateMoisture(int raw) {
  int dry = settings.dryValue;
  int wet = settings.wetValue;

  if (dry <= wet) return 0;

  if (raw >= dry) return 0;
  if (raw <= wet) return 100;

  float pct = 100.0f * (float)(dry - raw) / (float)(dry - wet);

  if (pct < 0.0f)   pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return (int)(pct + 0.5f);
}

void setRelayHardware(bool on) {
#if RELAY_ACTIVE_LOW
  digitalWrite(RELAY_PIN, on ? LOW : HIGH);
#else
  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
#endif
}

void updateRelay() {
  static unsigned long relayOnSince = 0;
  static bool          maxTrip      = false;

  bool newState;

  if (settings.autoMode == 1) {
    int pct = g_moisturePercent;
    newState = (pct < settings.threshold);

    if (newState && !g_relayActive) {
      relayOnSince = millis();
      maxTrip = false;
    }
    if (newState && (millis() - relayOnSince > 300000UL)) {
      newState = false;
      maxTrip = true;
    }
  } else {
    newState = (settings.relayState == 1);
    maxTrip = false;
  }

  if (newState != g_relayActive) {
    setRelayHardware(newState);
    g_relayActive = newState;
  }
}

// ==================== SETTINGS ====================
uint16_t calcCRC(const Settings& s) {
  const uint8_t* p = (const uint8_t*)&s;
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < sizeof(Settings) - sizeof(uint16_t); i++) {
    c ^= p[i];
    for (int j = 0; j < 8; j++)
      c = (c & 1) ? (c >> 1) ^ 0xA001 : (c >> 1);
  }
  return c;
}

void loadSettings() {
  EEPROM.get(SETTINGS_ADDR, settings);

  bool valid = (settings.magic == SETTINGS_MAGIC)
            && (settings.dryValue <= 4095)
            && (settings.wetValue <= 4095)
            && (settings.dryValue > settings.wetValue)
            && (settings.threshold >= 1 && settings.threshold <= 99)
            && (settings.readInterval >= 2 && settings.readInterval <= 3600)
            && (settings.autoMode <= 1)
            && (settings.relayState <= 1)
            && (settings.crc == calcCRC(settings));

  if (!valid) {
    settings.magic        = SETTINGS_MAGIC;
    settings.dryValue     = 3200;
    settings.wetValue     = 1400;
    settings.threshold    = 30;
    settings.autoMode     = 1;
    settings.relayState   = 0;
    settings.readInterval = 5;
    settings.crc          = calcCRC(settings);
    saveSettings();
  }
}

void saveSettings() {
  settings.magic = SETTINGS_MAGIC;
  settings.crc   = calcCRC(settings);
  EEPROM.put(SETTINGS_ADDR, settings);
  EEPROM.commit();
}

// ==================== WIFI CONFIG ====================
uint16_t calcWifiCRC(const WifiConfig& c) {
  const uint8_t* p = (const uint8_t*)&c;
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < sizeof(WifiConfig) - sizeof(uint16_t); i++) {
    crc ^= p[i];
    for (int j = 0; j < 8; j++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

void loadWifiConfig() {
  EEPROM.get(WIFI_CFG_ADDR, wifiCfg);

  bool valid = (wifiCfg.magic == WIFI_CFG_MAGIC)
            && (wifiCfg.crc == calcWifiCRC(wifiCfg))
            && (wifiCfg.apEnabled <= 1)
            && (wifiCfg.forceAP <= 1)
            && (strlen(wifiCfg.apSSID) >= 1)
            && (strlen(wifiCfg.apSSID) <= 32)
            && (strlen(wifiCfg.apPass) >= 8)
            && (strlen(wifiCfg.apPass) <= 64)
            && (strlen(wifiCfg.staSSID) <= 32)
            && (strlen(wifiCfg.staPass) <= 64)
            && (strlen(wifiCfg.staSSID) >= 1 || wifiCfg.apEnabled == 1);

  if (!valid) {
    memset(&wifiCfg, 0, sizeof(wifiCfg));
    wifiCfg.magic = WIFI_CFG_MAGIC;
    strncpy(wifiCfg.staSSID, WIFI_SSID, sizeof(wifiCfg.staSSID) - 1);
    strncpy(wifiCfg.staPass, WIFI_PASS, sizeof(wifiCfg.staPass) - 1);
    strncpy(wifiCfg.apSSID,  AP_SSID,   sizeof(wifiCfg.apSSID) - 1);
    strncpy(wifiCfg.apPass,  AP_PASS,   sizeof(wifiCfg.apPass) - 1);
    wifiCfg.apEnabled = 1;
    wifiCfg.forceAP   = 0;
    wifiCfg.crc       = calcWifiCRC(wifiCfg);
    saveWifiConfig();
  }
}

void saveWifiConfig() {
  wifiCfg.magic = WIFI_CFG_MAGIC;
  wifiCfg.crc   = calcWifiCRC(wifiCfg);
  EEPROM.put(WIFI_CFG_ADDR, wifiCfg);
  EEPROM.commit();
}

void requestWifiApply() {
  wifiApplyState = WIFI_APPLY_PENDING;
}

void serviceWifiApply() {
  unsigned long now = millis();

  switch (wifiApplyState) {
    case WIFI_IDLE:
      return;

    case WIFI_APPLY_PENDING:
      WiFi.disconnect(true);
      WiFi.mode(wifiCfg.forceAP ? WIFI_AP :
                (wifiCfg.apEnabled ? WIFI_AP_STA : WIFI_STA));

      if (wifiCfg.forceAP) {
        WiFi.softAP(wifiCfg.apSSID, wifiCfg.apPass);
        apModeActive = true;
        wifiApplyState = WIFI_APPLY_DONE;
      } else {
        if (wifiCfg.apEnabled) {
          WiFi.softAP(wifiCfg.apSSID, wifiCfg.apPass);
        }
        WiFi.begin(wifiCfg.staSSID, wifiCfg.staPass);
        wifiApplyStart = now;
        wifiApplyState = WIFI_APPLY_CONNECTING;
      }
      return;

    case WIFI_APPLY_CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        apModeActive = false;
        wifiApplyState = WIFI_APPLY_DONE;
      } else if (now - wifiApplyStart > 15000UL) {
        apModeActive = wifiCfg.apEnabled;
        wifiApplyState = WIFI_APPLY_DONE;
      }
      return;

    case WIFI_APPLY_DONE:
      wifiApplyState = WIFI_IDLE;
      return;
  }
}

String wifiStatusJSON() {
  String j = "{";
  wifi_mode_t m = WiFi.getMode();
  const char* modeStr =
    (m == WIFI_AP)     ? "AP" :
    (m == WIFI_STA)    ? "STA" :
    (m == WIFI_AP_STA) ? "AP+STA" : "OFF";

  j += "\"mode\":\"" + String(modeStr) + "\",";
  j += "\"staSSID\":\"" + String(wifiCfg.staSSID) + "\",";
  j += "\"apSSID\":\""  + String(wifiCfg.apSSID)  + "\",";
  j += "\"apEnabled\":" + String(wifiCfg.apEnabled) + ",";
  j += "\"forceAP\":"   + String(wifiCfg.forceAP) + ",";
  j += "\"connected\":" + String(WiFi.status() == WL_CONNECTED ? 1 : 0) + ",";
  j += "\"applyState\":" + String((int)wifiApplyState) + ",";

  if (WiFi.status() == WL_CONNECTED) {
    j += "\"staIP\":\"" + WiFi.localIP().toString() + "\",";
    j += "\"rssi\":"     + String(WiFi.RSSI()) + ",";
  } else {
    j += "\"staIP\":\"\",";
    j += "\"rssi\":0,";
  }

  if (m == WIFI_AP || m == WIFI_AP_STA) {
    j += "\"apIP\":\"" + WiFi.softAPIP().toString() + "\",";
    j += "\"apClients\":" + String(WiFi.softAPgetStationNum()) + ",";
  } else {
    j += "\"apIP\":\"\",";
    j += "\"apClients\":0,";
  }

  j += "\"mac\":\"" + WiFi.macAddress() + "\"";
  j += "}";
  return j;
}

// ==================== WIFI HANDLERS ====================
void handleWifiPage() {
  server.send_P(200, "text/html", WIFI_HTML);
}

void handleWifiStatus() {
  server.send(200, "application/json", wifiStatusJSON());
}

void handleWifiScanStart() {
  if (scanState == SCAN_RUNNING) {
    server.send(200, "application/json", "{\"state\":\"running\"}");
    return;
  }

  WiFi.scanDelete();

  wifi_mode_t m = WiFi.getMode();
  if (m == WIFI_AP) {
    WiFi.mode(WIFI_AP_STA);
  }

  int16_t r = WiFi.scanNetworks(true, false);
  if (r == WIFI_SCAN_FAILED) {
    server.send(500, "application/json", "{\"error\":\"scan failed\"}");
    return;
  }

  scanState = SCAN_RUNNING;
  server.send(200, "application/json", "{\"state\":\"running\"}");
}

void handleWifiScanResults() {
  int16_t n = WiFi.scanComplete();

  if (n == WIFI_SCAN_RUNNING) {
    server.send(200, "application/json", "{\"state\":\"running\"}");
    return;
  }
  if (n == WIFI_SCAN_FAILED) {
    scanState = SCAN_IDLE;
    server.send(200, "application/json", "{\"state\":\"failed\"}");
    return;
  }
  if (n < 0) {
    server.send(200, "application/json", "{\"state\":\"idle\"}");
    return;
  }

  String j = "{\"state\":\"done\",\"networks\":[";
  for (int i = 0; i < n; i++) {
    if (i > 0) j += ",";
    String ssid = WiFi.SSID(i);
    ssid.replace("\"", "\\\"");
    j += "{\"ssid\":\"" + ssid + "\",";
    j += "\"rssi\":" + String(WiFi.RSSI(i)) + ",";
    j += "\"enc\":" + String(WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? 0 : 1) + "}";
  }
  j += "]}";
  WiFi.scanDelete();
  scanState = SCAN_IDLE;
  server.send(200, "application/json", j);
}

void handleWifiSave() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"no body\"}");
    return;
  }

  String body = server.arg("plain");

  auto extractStr = [&](const char* key, const char* def) -> String {
    String k = String("\"") + key + "\":\"";
    int i = body.indexOf(k);
    if (i < 0) return String(def);
    i += k.length();
    int end = body.indexOf('"', i);
    if (end < 0) return String(def);
    return body.substring(i, end);
  };

  auto extractInt = [&](const char* key, int def) -> int {
    String k = String("\"") + key + "\":";
    int i = body.indexOf(k);
    if (i < 0) return def;
    i += k.length();
    while (i < (int)body.length() && (body[i]==' '||body[i]=='\t')) i++;
    int v = 0; bool neg = false;
    if (i < (int)body.length() && body[i]=='-'){neg=true;i++;}
    bool found = false;
    while (i < (int)body.length() && isdigit((unsigned char)body[i])) {
      v = v*10 + (body[i]-'0'); i++; found = true;
    }
    return found ? (neg?-v:v) : def;
  };

  String newStaSSID = extractStr("staSSID", wifiCfg.staSSID);
  String newStaPass = extractStr("staPass", "");
  String newApSSID  = extractStr("apSSID",  wifiCfg.apSSID);
  String newApPass  = extractStr("apPass",  "");
  int    apEnabled  = extractInt("apEnabled", wifiCfg.apEnabled);
  int    forceAP    = extractInt("forceAP",   wifiCfg.forceAP);
  String clearStaPw = extractStr("clearStaPass", "0");

  if (newApSSID.length() < 1 || newApSSID.length() > 32) {
    server.send(400, "application/json",
                "{\"error\":\"AP SSID must be 1-32 chars\"}");
    return;
  }
  if (newApPass.length() > 0 &&
      (newApPass.length() < 8 || newApPass.length() > 64)) {
    server.send(400, "application/json",
                "{\"error\":\"AP password must be 8-64 chars or empty\"}");
    return;
  }
  if (newStaSSID.length() > 32) {
    server.send(400, "application/json",
                "{\"error\":\"STA SSID must be 32 chars or less\"}");
    return;
  }
  if (newStaPass.length() > 64) {
    server.send(400, "application/json",
                "{\"error\":\"STA password must be 64 chars or less\"}");
    return;
  }

  bool newApEnabled = (apEnabled == 1);
  bool newForceAP   = (forceAP == 1);

  if (!newForceAP && !newApEnabled && newStaSSID.length() == 0) {
    server.send(400, "application/json",
                "{\"error\":\"Cannot disable both STA and AP - device would be unreachable\"}");
    return;
  }

  strncpy(wifiCfg.staSSID, newStaSSID.c_str(), sizeof(wifiCfg.staSSID) - 1);
  wifiCfg.staSSID[sizeof(wifiCfg.staSSID) - 1] = 0;

  if (clearStaPw == "1") {
    wifiCfg.staPass[0] = 0;
  } else if (newStaPass.length() > 0) {
    strncpy(wifiCfg.staPass, newStaPass.c_str(), sizeof(wifiCfg.staPass) - 1);
    wifiCfg.staPass[sizeof(wifiCfg.staPass) - 1] = 0;
  }

  strncpy(wifiCfg.apSSID, newApSSID.c_str(), sizeof(wifiCfg.apSSID) - 1);
  wifiCfg.apSSID[sizeof(wifiCfg.apSSID) - 1] = 0;

  if (newApPass.length() > 0) {
    strncpy(wifiCfg.apPass, newApPass.c_str(), sizeof(wifiCfg.apPass) - 1);
    wifiCfg.apPass[sizeof(wifiCfg.apPass) - 1] = 0;
  }

  wifiCfg.apEnabled = newApEnabled ? 1 : 0;
  wifiCfg.forceAP   = newForceAP   ? 1 : 0;

  saveWifiConfig();

  String resp = wifiStatusJSON();
  server.send(200, "application/json", resp);

  requestWifiApply();
}

void handleWifiReset() {
  memset(&wifiCfg, 0, sizeof(wifiCfg));
  wifiCfg.magic = WIFI_CFG_MAGIC;
  strncpy(wifiCfg.staSSID, WIFI_SSID, sizeof(wifiCfg.staSSID) - 1);
  strncpy(wifiCfg.staPass, WIFI_PASS, sizeof(wifiCfg.staPass) - 1);
  strncpy(wifiCfg.apSSID,  AP_SSID,   sizeof(wifiCfg.apSSID) - 1);
  strncpy(wifiCfg.apPass,  AP_PASS,   sizeof(wifiCfg.apPass) - 1);
  wifiCfg.apEnabled = 1;
  wifiCfg.forceAP   = 0;
  saveWifiConfig();

  String resp = wifiStatusJSON();
  server.send(200, "application/json", resp);

  requestWifiApply();
}

// ==================== SNAPSHOT ====================
void snapshot(int &rawOut, int &pctOut, bool &relayOut,
              uint8_t &autoOut, uint8_t &thrOut) {
  portENTER_CRITICAL(&g_mux);
  rawOut   = g_rawMoisture;
  pctOut   = g_moisturePercent;
  relayOut = g_relayActive;
  portEXIT_CRITICAL(&g_mux);
  autoOut  = settings.autoMode;
  thrOut   = settings.threshold;
}

// ==================== DASHBOARD HANDLERS ====================
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  int raw, pct; bool relay; uint8_t autoM, thr;
  snapshot(raw, pct, relay, autoM, thr);

  String json = "{";
  json += "\"moisture\":"    + String(pct)                  + ",";
  json += "\"raw\":"         + String(raw)                  + ",";
  json += "\"relay\":"       + String(relay ? 1 : 0)        + ",";
  json += "\"auto\":"        + String(autoM)                + ",";
  json += "\"threshold\":"   + String(thr)                  + ",";
  json += "\"dryValue\":"    + String(settings.dryValue)    + ",";
  json += "\"wetValue\":"    + String(settings.wetValue)    + ",";
  json += "\"readInterval\":"+ String(settings.readInterval);
  json += "}";
  server.send(200, "application/json", json);
}

void handleRelay() {
  if (server.hasArg("state")) {
    int s = server.arg("state").toInt();
    settings.autoMode   = 0;
    settings.relayState = (s == 1) ? 1 : 0;
    saveSettings();

    bool newState = (settings.relayState == 1);
    if (newState != g_relayActive) {
      setRelayHardware(newState);
      g_relayActive = newState;
    }
  }
  handleStatus();
}

void handleAuto() {
  if (server.hasArg("state")) {
    int s = server.arg("state").toInt();
    settings.autoMode = (s == 1) ? 1 : 0;
    if (settings.autoMode == 0) {
      settings.relayState = g_relayActive ? 1 : 0;
    }
    saveSettings();
    updateRelay();
  }
  handleStatus();
}

void handleSettings() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"no body\"}");
    return;
  }

  String body = server.arg("plain");

  auto extract = [&](const char* key, long def)->long {
    String k = String("\"") + key + "\":";
    int i = body.indexOf(k);
    if (i < 0) return def;
    i += k.length();
    while (i < (int)body.length() && (body[i]==' '||body[i]=='\t')) i++;
    long v = 0; bool neg = false;
    if (i < (int)body.length() && body[i]=='-'){neg=true;i++;}
    bool found = false;
    while (i < (int)body.length() && isdigit((unsigned char)body[i])) {
      v = v*10 + (body[i]-'0'); i++; found = true;
    }
    return found ? (neg?-v:v) : def;
  };

  long dv  = extract("dryValue",     settings.dryValue);
  long wv  = extract("wetValue",     settings.wetValue);
  long th  = extract("threshold",    settings.threshold);
  long ri  = extract("readInterval", settings.readInterval);

  if (dv < 0 || dv > 4095 || wv < 0 || wv > 4095 || dv <= wv) {
    server.send(400, "application/json",
                "{\"error\":\"invalid dry/wet (dry must be > wet, 0-4095)\"}");
    return;
  }
  if (th < 1 || th > 99) {
    server.send(400, "application/json",
                "{\"error\":\"threshold must be 1-99\"}");
    return;
  }
  if (ri < 2 || ri > 3600) {
    server.send(400, "application/json",
                "{\"error\":\"readInterval must be 2-3600\"}");
    return;
  }

  settings.dryValue     = (uint16_t)dv;
  settings.wetValue     = (uint16_t)wv;
  settings.threshold    = (uint8_t)th;
  settings.readInterval = (uint16_t)ri;
  saveSettings();

  readSensor();
  updateRelay();
  handleStatus();
}

void handleNotFound() {
  server.send(404, "text/plain", "404 Not Found");
}
