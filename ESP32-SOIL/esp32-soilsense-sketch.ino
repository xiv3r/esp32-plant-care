#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>

#define SOIL_MOISTURE_PIN 34
#define RELAY_PIN         25
#define RELAY_ACTIVE_LOW  0
#define AUTO_HYSTERESIS   3
#define SENSOR_FAULT_RAIL 3

#define EEPROM_SIZE         512
#define EEPROM_MAGIC_NUMBER 0xAB
#define EEPROM_MAGIC_ADDR   0
#define WIFI_SSID_MAX       32
#define WIFI_PASS_MAX       63

#define ADDR_MAGIC              0
#define ADDR_WIFI_SSID          1
#define ADDR_WIFI_PASS          65
#define ADDR_MOISTURE_THRESHOLD 129
#define ADDR_AUTO_MODE          131
#define ADDR_RELAY_STATE        132
#define ADDR_CALIBRATION_DRY    133
#define ADDR_CALIBRATION_WET    135
#define ADDR_SENSOR_MIN         137
#define ADDR_SENSOR_MAX         139

#define DEFAULT_MOISTURE_THRESHOLD 40
#define DEFAULT_CALIBRATION_DRY    4095
#define DEFAULT_CALIBRATION_WET    0
#define DEFAULT_SENSOR_MIN         0
#define DEFAULT_SENSOR_MAX         4095

WebServer server(80);

String wifiSSID = "";
String wifiPassword = "";
bool   apMode = false;

int  moistureThreshold = DEFAULT_MOISTURE_THRESHOLD;
bool autoMode          = true;
bool relayState        = false;

int calibrationDry = DEFAULT_CALIBRATION_DRY;
int calibrationWet = DEFAULT_CALIBRATION_WET;
int sensorMin      = DEFAULT_SENSOR_MIN;
int sensorMax      = DEFAULT_SENSOR_MAX;

unsigned long lastSensorRead = 0;
unsigned long lastWiFiCheck  = 0;
const unsigned long SENSOR_INTERVAL     = 2000;
const unsigned long WIFI_CHECK_INTERVAL = 30000;

int rawMoistureValue   = 0;
int moisturePercent    = 0;
int smoothedMoisture   = 50;
bool sensorFault       = false;
int  sensorRailCount   = 0;

const int SMOOTHING_SAMPLES = 10;
int moistureReadings[SMOOTHING_SAMPLES];
int readingIndex = 0;

enum WifiState { WIFI_STATE_INIT, WIFI_STATE_CONNECTING, WIFI_STATE_CONNECTED, WIFI_STATE_AP };
WifiState wifiState = WIFI_STATE_INIT;
unsigned long wifiConnectStart = 0;
const unsigned long WIFI_CONNECT_TIMEOUT = 15000;

bool pendingRestart = false;
unsigned long pendingRestartAt = 0;

bool eepromDirty = false;
unsigned long eepromDirtyAt = 0;

const char* AP_SSID     = "ESP32_SoilMonitor";
const char* AP_PASSWORD = "12345678";

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32 Soil Monitor</title>
<style>
*{margin:0;padding:0;box-sizing:border-box;font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif}
body{background:linear-gradient(135deg,#1a1a2e 0%,#16213e 100%);min-height:100vh;color:#fff;padding:20px}
.container{max-width:900px;margin:0 auto}
h1{text-align:center;margin-bottom:30px;font-size:2rem;background:linear-gradient(90deg,#00d4ff,#00ff88);-webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text}
.dashboard{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:20px;margin-bottom:20px}
.card{background:rgba(255,255,255,0.05);border-radius:15px;padding:25px;border:1px solid rgba(255,255,255,0.1);backdrop-filter:blur(10px);transition:transform 0.3s ease,box-shadow 0.3s ease}
.card:hover{transform:translateY(-5px);box-shadow:0 10px 30px rgba(0,212,255,0.2)}
.card h2{font-size:1.1rem;margin-bottom:15px;color:#00d4ff;display:flex;align-items:center;gap:10px}
.moisture-value{font-size:3rem;font-weight:bold;text-align:center;margin:20px 0}
.moisture-value.dry{color:#ff6b6b}
.moisture-value.moderate{color:#ffd93d}
.moisture-value.wet{color:#00ff88}
.moisture-bar{width:100%;height:20px;background:rgba(255,255,255,0.1);border-radius:10px;overflow:hidden;margin-top:15px}
.moisture-bar-fill{height:100%;border-radius:10px;transition:width 0.5s ease;background:linear-gradient(90deg,#ff6b6b,#ffd93d,#00ff88)}
.status-badge{display:inline-block;padding:5px 15px;border-radius:20px;font-size:0.85rem;font-weight:600;margin:5px 0}
.status-badge.active{background:rgba(0,255,136,0.2);color:#00ff88;border:1px solid #00ff88}
.status-badge.inactive{background:rgba(255,107,107,0.2);color:#ff6b6b;border:1px solid #ff6b6b}
.status-badge.warn{background:rgba(255,217,61,0.2);color:#ffd93d;border:1px solid #ffd93d}
.btn{padding:12px 24px;border:none;border-radius:10px;cursor:pointer;font-size:1rem;font-weight:600;transition:all 0.3s ease;width:100%;margin-top:10px}
.btn:disabled{cursor:not-allowed}
.btn-primary{background:linear-gradient(135deg,#00d4ff,#0099cc);color:#fff}
.btn-primary:hover:not(:disabled){background:linear-gradient(135deg,#00e5ff,#00aadd);box-shadow:0 5px 20px rgba(0,212,255,0.4)}
.btn-danger{background:linear-gradient(135deg,#ff6b6b,#cc5555);color:#fff}
.btn-danger:hover:not(:disabled){background:linear-gradient(135deg,#ff7b7b,#dd6666);box-shadow:0 5px 20px rgba(255,107,107,0.4)}
.btn-success{background:linear-gradient(135deg,#00ff88,#00cc6a);color:#1a1a2e}
.btn-success:hover:not(:disabled){background:linear-gradient(135deg,#00ff99,#00dd7b);box-shadow:0 5px 20px rgba(0,255,136,0.4)}
.btn-warning{background:linear-gradient(135deg,#ffd93d,#ccad30);color:#1a1a2e}
.btn-warning:hover:not(:disabled){background:linear-gradient(135deg,#ffe94d,#ddbd40);box-shadow:0 5px 20px rgba(255,217,61,0.4)}
.settings-form{display:flex;flex-direction:column;gap:15px}
.form-group{display:flex;flex-direction:column;gap:5px}
.form-group label{font-size:0.9rem;color:#aaa}
.form-group input,.form-group select{padding:12px;border-radius:8px;border:1px solid rgba(255,255,255,0.2);background:rgba(255,255,255,0.05);color:#fff;font-size:1rem}
.form-group input:focus{outline:none;border-color:#00d4ff}
.form-group input[type="range"]{padding:0;height:8px;-webkit-appearance:none;background:rgba(255,255,255,0.1);border-radius:4px}
.form-group input[type="range"]::-webkit-slider-thumb{-webkit-appearance:none;width:20px;height:20px;border-radius:50%;background:#00d4ff;cursor:pointer;box-shadow:0 0 10px rgba(0,212,255,0.5)}
.range-value{text-align:center;font-size:1.5rem;font-weight:bold;color:#00d4ff}
.info-row{display:flex;justify-content:space-between;padding:8px 0;border-bottom:1px solid rgba(255,255,255,0.05)}
.info-row:last-child{border-bottom:none}
.info-label{color:#aaa}
.info-value{font-weight:600}
.toast{position:fixed;bottom:30px;left:50%;transform:translateX(-50%) translateY(100px);background:rgba(0,255,136,0.9);color:#1a1a2e;padding:15px 30px;border-radius:10px;font-weight:600;opacity:0;transition:all 0.3s ease;z-index:1000;max-width:90vw;text-align:center}
.toast.show{transform:translateX(-50%) translateY(0);opacity:1}
.toast.error{background:rgba(255,107,107,0.9);color:#fff}
.tabs{display:flex;gap:10px;margin-bottom:20px;flex-wrap:wrap}
.tab{padding:10px 20px;border-radius:10px;cursor:pointer;background:rgba(255,255,255,0.05);border:1px solid rgba(255,255,255,0.1);transition:all 0.3s ease;font-weight:600}
.tab.active{background:linear-gradient(135deg,#00d4ff,#0099cc);border-color:transparent}
.tab:hover:not(.active){background:rgba(255,255,255,0.1)}
.tab-content{display:none}
.tab-content.active{display:block}
.relay-indicator{width:80px;height:80px;border-radius:50%;margin:20px auto;display:flex;align-items:center;justify-content:center;font-size:2rem;transition:all 0.3s ease}
.relay-indicator.on{background:radial-gradient(circle,rgba(0,255,136,0.3),transparent);box-shadow:0 0 30px rgba(0,255,136,0.5);color:#00ff88}
.relay-indicator.off{background:radial-gradient(circle,rgba(255,107,107,0.2),transparent);box-shadow:0 0 30px rgba(255,107,107,0.3);color:#ff6b6b}
.calibration-info{background:rgba(255,217,61,0.1);border:1px solid rgba(255,217,61,0.3);border-radius:10px;padding:15px;margin-top:15px;font-size:0.9rem}
@media (max-width:600px){h1{font-size:1.5rem}.moisture-value{font-size:2.5rem}.card{padding:20px}}
</style>
</head>
<body>
<div class="container">
<h1>🌱 ESP32 Soil Monitor</h1>
<div class="tabs">
<div class="tab active" onclick="showTab(this,'dashboard')">📊 Dashboard</div>
<div class="tab" onclick="showTab(this,'control')">🎛️ Control</div>
<div class="tab" onclick="showTab(this,'settings')">⚙️ Settings</div>
<div class="tab" onclick="showTab(this,'calibration')">📐 Calibration</div>
</div>

<div id="dashboard" class="tab-content active">
<div class="dashboard">
<div class="card">
<h2>💧 Soil Moisture</h2>
<div class="moisture-value" id="moistureValue">--%</div>
<div class="moisture-bar"><div class="moisture-bar-fill" id="moistureBar" style="width:0%"></div></div>
<div style="margin-top:15px;text-align:center;"><span class="status-badge" id="moistureStatus">Loading...</span></div>
</div>
<div class="card">
<h2>🔌 Relay Status</h2>
<div class="relay-indicator off" id="relayIndicator">⭕</div>
<div style="text-align:center;"><span class="status-badge inactive" id="relayStatus">OFF</span></div>
<div style="margin-top:15px;">
<div class="info-row"><span class="info-label">Mode</span><span class="info-value" id="modeDisplay">Auto</span></div>
<div class="info-row"><span class="info-label">Threshold</span><span class="info-value" id="thresholdDisplay">--%</span></div>
<div class="info-row"><span class="info-label">Hysteresis</span><span class="info-value" id="hysteresisDisplay">--%</span></div>
</div>
</div>
<div class="card">
<h2>📶 System Info</h2>
<div class="info-row"><span class="info-label">WiFi Mode</span><span class="info-value" id="wifiMode">--</span></div>
<div class="info-row"><span class="info-label">IP Address</span><span class="info-value" id="ipAddress">--</span></div>
<div class="info-row"><span class="info-label">Signal</span><span class="info-value" id="signalStrength">--</span></div>
<div class="info-row"><span class="info-label">Uptime</span><span class="info-value" id="uptime">--</span></div>
<div class="info-row"><span class="info-label">Free Heap</span><span class="info-value" id="freeMemory">--</span></div>
<div class="info-row"><span class="info-label">Sensor</span><span class="info-value" id="sensorHealth">--</span></div>
</div>
</div>
</div>

<div id="control" class="tab-content">
<div class="dashboard">
<div class="card">
<h2>🎛️ Relay Control</h2>
<div class="relay-indicator off" id="relayIndicator2">⭕</div>
<div style="text-align:center;margin-bottom:15px;"><span class="status-badge inactive" id="relayStatus2">OFF</span></div>
<button class="btn btn-success" onclick="setRelay(true)">Turn ON</button>
<button class="btn btn-danger" onclick="setRelay(false)" style="margin-top:10px;">Turn OFF</button>
<button class="btn btn-primary" onclick="toggleRelay()" style="margin-top:10px;">Toggle</button>
</div>
<div class="card">
<h2>🤖 Automation Mode</h2>
<div style="text-align:center;margin:20px 0;"><span class="status-badge" id="autoModeStatus">Auto</span></div>
<button class="btn btn-primary" onclick="setAutoMode(true)">Enable Auto</button>
<button class="btn btn-warning" onclick="setAutoMode(false)" style="margin-top:10px;">Enable Manual</button>
<div class="calibration-info" style="margin-top:20px;"><strong>ℹ️</strong> Relay ON when moisture &lt; threshold − hysteresis, OFF when &gt; threshold + hysteresis.</div>
</div>
</div>
</div>

<div id="settings" class="tab-content">
<div class="dashboard">
<div class="card">
<h2>💧 Moisture Threshold</h2>
<div class="range-value" id="thresholdValue">40%</div>
<input type="range" id="thresholdSlider" min="5" max="95" value="40" oninput="document.getElementById('thresholdValue').textContent=this.value+'%'" style="width:100%;margin:20px 0;">
<button class="btn btn-primary" onclick="saveThreshold()">Save Threshold</button>
<div class="calibration-info">Relay activates when moisture drops below threshold (with hysteresis).</div>
</div>
<div class="card">
<h2>📶 WiFi Settings</h2>
<div class="settings-form">
<div class="form-group"><label>WiFi SSID</label><input type="text" id="wifiSSID" placeholder="Enter WiFi name" maxlength="32"></div>
<div class="form-group"><label>WiFi Password</label><input type="password" id="wifiPassword" placeholder="Enter WiFi password" maxlength="63"></div>
<button class="btn btn-primary" onclick="saveWiFi()">Save &amp; Connect</button>
<button class="btn btn-warning" onclick="resetWiFi()">Reset to AP Mode</button>
</div>
</div>
</div>
</div>

<div id="calibration" class="tab-content">
<div class="dashboard">
<div class="card">
<h2>📐 Sensor Calibration</h2>
<p style="color:#aaa;margin-bottom:20px;font-size:0.9rem;">Place sensor in dry air → Save Dry. Place in water → Save Wet.</p>
<div class="info-row"><span class="info-label">Current Raw</span><span class="info-value" id="rawValue">--</span></div>
<div class="info-row"><span class="info-label">Dry Calibration</span><span class="info-value" id="dryCalValue">--</span></div>
<div class="info-row"><span class="info-label">Wet Calibration</span><span class="info-value" id="wetCalValue">--</span></div>
<button class="btn btn-warning" data-calib="1" onclick="saveCalibration('dry')" style="margin-top:15px;">📍 Save as Dry Point</button>
<button class="btn btn-primary" data-calib="1" onclick="saveCalibration('wet')" style="margin-top:10px;">💧 Save as Wet Point</button>
</div>
<div class="card">
<h2>🔧 Advanced Calibration</h2>
<div class="settings-form">
<div class="form-group"><label>Sensor Minimum (Dry)</label><input type="number" id="sensorMin" min="0" max="4095" value="4095"></div>
<div class="form-group"><label>Sensor Maximum (Wet)</label><input type="number" id="sensorMax" min="0" max="4095" value="0"></div>
<button class="btn btn-primary" onclick="saveAdvancedCalibration()">Save Advanced</button>
</div>
</div>
</div>
</div>
</div>

<div class="toast" id="toast">Settings saved!</div>

<script>
let refreshInterval=null;
let isRefreshing=false;

function showTab(el,name){
  document.querySelectorAll('.tab-content').forEach(t=>t.classList.remove('active'));
  document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));
  document.getElementById(name).classList.add('active');
  el.classList.add('active');
}

function showToast(msg,isError){
  const t=document.getElementById('toast');
  t.textContent=msg;
  t.className='toast show'+(isError?' error':'');
  clearTimeout(t._to);
  t._to=setTimeout(()=>t.classList.remove('show'),3000);
}

async function api(path,opts){
  const ctl=new AbortController();
  const tm=setTimeout(()=>ctl.abort(),8000);
  try{
    const r=await fetch(path,Object.assign({},opts,{signal:ctl.signal}));
    clearTimeout(tm);
    let body=null;
    try{body=await r.json();}catch(e){}
    if(!r.ok){
      const err=new Error((body&&body.error)||('HTTP '+r.status));
      err.body=body;
      throw err;
    }
    return body||{};
  }catch(e){
    clearTimeout(tm);
    throw e;
  }
}

async function refreshData(){
  if(isRefreshing)return;
  isRefreshing=true;
  try{
    const d=await api('/api/status');
    updateUI(d);
  }catch(e){}
  finally{isRefreshing=false;}
}

function updateUI(d){
  const mv=document.getElementById('moistureValue');
  if(d.sensorFault){
    mv.textContent='ERR';
    mv.className='moisture-value dry';
  } else {
    mv.textContent=d.moisture+'%';
    const band_lo=d.threshold-d.hysteresis;
    const band_hi=d.threshold+d.hysteresis;
    mv.className='moisture-value '+(d.moisture<band_lo?'dry':d.moisture<band_hi?'moderate':'wet');
  }
  document.getElementById('moistureBar').style.width=(d.sensorFault?0:d.moisture)+'%';

  const ms=document.getElementById('moistureStatus');
  if(d.sensorFault){
    ms.textContent='SENSOR FAULT';
    ms.className='status-badge inactive';
  } else if(d.moisture<(d.threshold-d.hysteresis)){
    ms.textContent='DRY - Needs Water';
    ms.className='status-badge inactive';
  } else if(d.moisture<(d.threshold+d.hysteresis)){
    ms.textContent='MODERATE';
    ms.className='status-badge warn';
  } else {
    ms.textContent='WET - Good';
    ms.className='status-badge active';
  }

  ['relayIndicator','relayIndicator2'].forEach(id=>{
    const el=document.getElementById(id);
    if(el){el.className='relay-indicator '+(d.relay?'on':'off');el.textContent=d.relay?'💡':'⭕';}
  });
  ['relayStatus','relayStatus2'].forEach(id=>{
    const el=document.getElementById(id);
    if(el){el.textContent=d.relay?'ON':'OFF';el.className='status-badge '+(d.relay?'active':'inactive');}
  });

  document.getElementById('modeDisplay').textContent=d.autoMode?'Auto':'Manual';
  const ams=document.getElementById('autoModeStatus');
  ams.textContent=d.autoMode?'Auto Mode':'Manual Mode';
  ams.className='status-badge '+(d.autoMode?'active':'inactive');

  document.getElementById('thresholdDisplay').textContent=d.threshold+'%';
  document.getElementById('hysteresisDisplay').textContent='±'+d.hysteresis+'%';

  const ts=document.getElementById('thresholdSlider');
  if(ts&&!ts.dataset.userEdited){ts.value=d.threshold;document.getElementById('thresholdValue').textContent=d.threshold+'%';}

  document.getElementById('wifiMode').textContent=d.apMode?'Access Point':'Station';
  document.getElementById('ipAddress').textContent=d.ip;
  document.getElementById('signalStrength').textContent=d.apMode?'N/A (AP)':(d.rssi+' dBm');
  document.getElementById('uptime').textContent=formatUptime(d.uptime);
  document.getElementById('freeMemory').textContent=(d.freeHeap/1024).toFixed(1)+' KB';

  const sh=document.getElementById('sensorHealth');
  if(d.sensorFault){sh.textContent='FAULT';sh.style.color='#ff6b6b';}
  else{sh.textContent='OK';sh.style.color='#00ff88';}

  document.querySelectorAll('[data-calib="1"]').forEach(b=>{
    b.disabled=!!d.sensorFault;
    b.style.opacity=d.sensorFault?'0.4':'1';
  });

  document.getElementById('rawValue').textContent=d.rawValue;
  document.getElementById('dryCalValue').textContent=d.calibrationDry;
  document.getElementById('wetCalValue').textContent=d.calibrationWet;

  const ws=document.getElementById('wifiSSID');
  if(ws&&!ws.dataset.userEdited&&d.ssid&&!ws.value) ws.value=d.ssid;
  const sm=document.getElementById('sensorMin');
  if(sm&&!sm.dataset.userEdited) sm.value=d.sensorMin;
  const sx=document.getElementById('sensorMax');
  if(sx&&!sx.dataset.userEdited) sx.value=d.sensorMax;
}

function formatUptime(s){
  const dd=Math.floor(s/86400), hh=Math.floor((s%86400)/3600), mm=Math.floor((s%3600)/60), ss=s%60;
  if(dd>0)return dd+'d '+hh+'h '+mm+'m';
  if(hh>0)return hh+'h '+mm+'m '+ss+'s';
  if(mm>0)return mm+'m '+ss+'s';
  return ss+'s';
}

async function setRelay(state){
  try{
    const d=await api('/api/relay',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({state})});
    if(d.success){showToast('Relay '+(state?'ON':'OFF'));refreshData();}
    else showToast('Failed',true);
  }catch(e){showToast(e.message||'Connection error',true);}
}

async function toggleRelay(){
  try{
    const d=await api('/api/relay/toggle',{method:'POST'});
    if(d.success){showToast('Relay toggled');refreshData();}
  }catch(e){showToast(e.message||'Connection error',true);}
}

async function setAutoMode(enabled){
  try{
    const d=await api('/api/auto',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled})});
    if(d.success){showToast('Auto '+(enabled?'enabled':'disabled'));refreshData();}
  }catch(e){showToast(e.message||'Connection error',true);}
}

async function saveThreshold(){
  const threshold=parseInt(document.getElementById('thresholdSlider').value,10);
  try{
    const d=await api('/api/threshold',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({threshold})});
    if(d.success){showToast('Threshold saved: '+threshold+'%');refreshData();}
  }catch(e){showToast(e.message||'Connection error',true);}
}

async function saveWiFi(){
  const ssid=document.getElementById('wifiSSID').value.trim();
  const password=document.getElementById('wifiPassword').value;
  if(!ssid){showToast('Please enter WiFi SSID',true);return;}
  if(ssid.length>32){showToast('SSID too long (max 32)',true);return;}
  if(password.length>63){showToast('Password too long (max 63)',true);return;}
  if(!confirm('Save WiFi and reboot device?'))return;
  try{
    const d=await api('/api/wifi',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ssid,password})});
    if(d.success){showToast('Saved. Device will reboot...');}
    else showToast('Save failed',true);
  }catch(e){showToast(e.message||'Connection error',true);}
}

async function resetWiFi(){
  if(!confirm('Reset WiFi to Access Point mode?'))return;
  try{
    const d=await api('/api/wifi/reset',{method:'POST'});
    if(d.success)showToast('Rebooting to AP mode...');
  }catch(e){showToast(e.message||'Connection error',true);}
}

async function saveCalibration(type){
  try{
    const d=await api('/api/calibrate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({type})});
    if(d.success){showToast(type==='dry'?'Dry calibration saved':'Wet calibration saved');refreshData();}
    else showToast('Calibration rejected',true);
  }catch(e){showToast(e.message||'Connection error',true);}
}

async function saveAdvancedCalibration(){
  const min=parseInt(document.getElementById('sensorMin').value,10);
  const max=parseInt(document.getElementById('sensorMax').value,10);
  if(isNaN(min)||isNaN(max)||min<0||min>4095||max<0||max>4095){showToast('Values must be 0-4095',true);return;}
  try{
    const d=await api('/api/calibrate/advanced',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({min,max})});
    if(d.success){showToast('Advanced calibration saved');refreshData();}
  }catch(e){showToast(e.message||'Connection error',true);}
}

document.addEventListener('DOMContentLoaded',()=>{
  ['wifiSSID','sensorMin','sensorMax','thresholdSlider'].forEach(id=>{
    const el=document.getElementById(id);
    if(!el)return;
    el.addEventListener('input',()=>{el.dataset.userEdited='1';});
    el.addEventListener('focus',()=>{
      const t=el._editTimer;
      if(t){clearTimeout(t);el._editTimer=null;}
    });
    el.addEventListener('blur',()=>{
      const t=el._editTimer;
      if(t)clearTimeout(t);
      el._editTimer=setTimeout(()=>{delete el.dataset.userEdited;el._editTimer=null;},1500);
    });
  });
  refreshData();
  refreshInterval=setInterval(refreshData,3000);
});

window.addEventListener('beforeunload',()=>{if(refreshInterval)clearInterval(refreshInterval);});
</script>
</body>
</html>
)rawliteral";

void saveStringToEEPROM(int addr, const String& str, uint8_t maxLen) {
  uint8_t len = str.length();
  if (len > maxLen) len = maxLen;
  EEPROM.write(addr, len);
  for (uint8_t i = 0; i < len; i++) EEPROM.write(addr + 1 + i, str[i]);
  EEPROM.write(addr + 1 + len, 0);
}

String readStringFromEEPROM(int addr, uint8_t maxLen) {
  uint8_t len = EEPROM.read(addr);
  if (len > maxLen) return "";
  String s;
  s.reserve(len);
  for (uint8_t i = 0; i < len; i++) {
    char c = EEPROM.read(addr + 1 + i);
    if (c == 0) break;
    s += c;
  }
  return s;
}

void saveIntToEEPROM(int addr, int value) {
  EEPROM.write(addr,     (value >> 8) & 0xFF);
  EEPROM.write(addr + 1,  value       & 0xFF);
}

int readIntFromEEPROM(int addr) {
  return (EEPROM.read(addr) << 8) | EEPROM.read(addr + 1);
}

void saveBoolToEEPROM(int addr, bool value) { EEPROM.write(addr, value ? 1 : 0); }
bool readBoolFromEEPROM(int addr)          { return EEPROM.read(addr) == 1; }

void markEepromDirty() {
  eepromDirty = true;
  eepromDirtyAt = millis() + 500;
}

void serviceEeprom() {
  if (eepromDirty && (long)(millis() - eepromDirtyAt) >= 0) {
    eepromDirty = false;
    EEPROM.commit();
  }
}

void saveSettings() {
  saveStringToEEPROM(ADDR_WIFI_SSID, wifiSSID,     WIFI_SSID_MAX);
  saveStringToEEPROM(ADDR_WIFI_PASS, wifiPassword, WIFI_PASS_MAX);
  saveIntToEEPROM(ADDR_MOISTURE_THRESHOLD, moistureThreshold);
  saveBoolToEEPROM(ADDR_AUTO_MODE,         autoMode);
  saveBoolToEEPROM(ADDR_RELAY_STATE,       relayState);
  saveIntToEEPROM(ADDR_CALIBRATION_DRY,    calibrationDry);
  saveIntToEEPROM(ADDR_CALIBRATION_WET,    calibrationWet);
  saveIntToEEPROM(ADDR_SENSOR_MIN,         sensorMin);
  saveIntToEEPROM(ADDR_SENSOR_MAX,         sensorMax);
  EEPROM.commit();
}

void normalizeCalibration() {
  if (calibrationDry < calibrationWet) {
    int t = calibrationDry;
    calibrationDry = calibrationWet;
    calibrationWet = t;
  }
  if (calibrationDry < 0)    calibrationDry = 0;
  if (calibrationDry > 4095) calibrationDry = 4095;
  if (calibrationWet < 0)    calibrationWet = 0;
  if (calibrationWet > 4095) calibrationWet = 4095;
  if (calibrationDry == calibrationWet) {
    calibrationDry = DEFAULT_CALIBRATION_DRY;
    calibrationWet = DEFAULT_CALIBRATION_WET;
  }
}

void loadSettings() {
  if (EEPROM.read(ADDR_MAGIC) == EEPROM_MAGIC_NUMBER) {
    wifiSSID     = readStringFromEEPROM(ADDR_WIFI_SSID, WIFI_SSID_MAX);
    wifiPassword = readStringFromEEPROM(ADDR_WIFI_PASS, WIFI_PASS_MAX);
    moistureThreshold = readIntFromEEPROM(ADDR_MOISTURE_THRESHOLD);
    autoMode          = readBoolFromEEPROM(ADDR_AUTO_MODE);
    relayState        = readBoolFromEEPROM(ADDR_RELAY_STATE);
    calibrationDry    = readIntFromEEPROM(ADDR_CALIBRATION_DRY);
    calibrationWet    = readIntFromEEPROM(ADDR_CALIBRATION_WET);
    sensorMin         = readIntFromEEPROM(ADDR_SENSOR_MIN);
    sensorMax         = readIntFromEEPROM(ADDR_SENSOR_MAX);

    if (moistureThreshold < 5 || moistureThreshold > 95) moistureThreshold = DEFAULT_MOISTURE_THRESHOLD;
    if (sensorMin < 0 || sensorMin > 4095) sensorMin = DEFAULT_SENSOR_MIN;
    if (sensorMax < 0 || sensorMax > 4095) sensorMax = DEFAULT_SENSOR_MAX;

    String cleanSSID = "";
    for (size_t i = 0; i < wifiSSID.length(); i++)
      if (wifiSSID[i] >= 32 && wifiSSID[i] < 127) cleanSSID += wifiSSID[i];
    wifiSSID = cleanSSID;

    normalizeCalibration();
  } else {
    wifiSSID = "";
    wifiPassword = "";
    moistureThreshold = DEFAULT_MOISTURE_THRESHOLD;
    autoMode          = true;
    relayState        = false;
    calibrationDry    = DEFAULT_CALIBRATION_DRY;
    calibrationWet    = DEFAULT_CALIBRATION_WET;
    sensorMin         = DEFAULT_SENSOR_MIN;
    sensorMax         = DEFAULT_SENSOR_MAX;
    EEPROM.write(ADDR_MAGIC, EEPROM_MAGIC_NUMBER);
    saveSettings();
  }
}

inline void relayWrite(bool on) {
#if RELAY_ACTIVE_LOW
  digitalWrite(RELAY_PIN, on ? LOW : HIGH);
#else
  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
#endif
}

int readMoistureRaw() {
  const int N = 9;
  int samples[N];
  for (int i = 0; i < N; i++) {
    samples[i] = analogRead(SOIL_MOISTURE_PIN);
    delay(3);
  }
  for (int i = 1; i < N; i++) {
    int k = samples[i];
    int j = i - 1;
    while (j >= 0 && samples[j] > k) {
      samples[j + 1] = samples[j];
      j--;
    }
    samples[j + 1] = k;
  }
  return samples[N / 2];
}

int calculateMoisturePercent(int rawValue) {
  int dryValue = calibrationDry;
  int wetValue = calibrationWet;
  if (dryValue < wetValue) {
    int t = dryValue;
    dryValue = wetValue;
    wetValue = t;
  }
  int range = dryValue - wetValue;
  if (range <= 0) return 0;
  if (rawValue > dryValue) rawValue = dryValue;
  if (rawValue < wetValue) rawValue = wetValue;
  long percent = (long)(rawValue - wetValue) * 100L / range;
  if (percent < 0)   percent = 0;
  if (percent > 100) percent = 100;
  return (int)percent;
}

int getSmoothedMoisture() {
  moistureReadings[readingIndex] = moisturePercent;
  readingIndex = (readingIndex + 1) % SMOOTHING_SAMPLES;
  long sum = 0;
  for (int i = 0; i < SMOOTHING_SAMPLES; i++) sum += moistureReadings[i];
  return sum / SMOOTHING_SAMPLES;
}

void updateSensorReadings() {
  rawMoistureValue = readMoistureRaw();

  if (rawMoistureValue <= 5 || rawMoistureValue >= 4090) {
    if (sensorRailCount < 255) sensorRailCount++;
  } else {
    sensorRailCount = 0;
    sensorFault = false;
  }
  if (sensorRailCount >= SENSOR_FAULT_RAIL) sensorFault = true;

  moisturePercent  = calculateMoisturePercent(rawMoistureValue);
  smoothedMoisture = getSmoothedMoisture();
}

void setRelayState(bool state, bool persist) {
  if (state == relayState && !persist) return;
  relayState = state;
  relayWrite(state);
  if (persist) {
    saveBoolToEEPROM(ADDR_RELAY_STATE, relayState);
    markEepromDirty();
  }
}

void handleAutoMode() {
  if (!autoMode || sensorFault) return;

  bool shouldBeOn;
  if (relayState) {
    shouldBeOn = (smoothedMoisture < (moistureThreshold + AUTO_HYSTERESIS));
  } else {
    shouldBeOn = (smoothedMoisture < (moistureThreshold - AUTO_HYSTERESIS));
  }

  if (shouldBeOn != relayState) {
    setRelayState(shouldBeOn, false);
  }
}

void startAPMode() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  apMode = true;
  wifiState = WIFI_STATE_AP;
}

void beginWifiConnect() {
  if (wifiSSID.length() == 0) {
    startAPMode();
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
  wifiState = WIFI_STATE_CONNECTING;
  wifiConnectStart = millis();
}

void serviceWifi() {
  switch (wifiState) {
    case WIFI_STATE_CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        wifiState = WIFI_STATE_CONNECTED;
        apMode = false;
      } else if (millis() - wifiConnectStart > WIFI_CONNECT_TIMEOUT) {
        WiFi.disconnect(true);
        startAPMode();
      }
      break;

    case WIFI_STATE_CONNECTED:
      if (WiFi.status() != WL_CONNECTED) {
        beginWifiConnect();
      }
      break;

    default:
      break;
  }
}

void handleStatus() {
  char buf[640];
  String ipStr = apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  int rssi = apMode ? 0 : WiFi.RSSI();

  String safeSSID;
  safeSSID.reserve(wifiSSID.length() + 8);
  for (size_t i = 0; i < wifiSSID.length(); i++) {
    char c = wifiSSID[i];
    if (c == '"' || c == '\\') {
      safeSSID += '\\';
      safeSSID += c;
    } else {
      safeSSID += c;
    }
  }

  snprintf(buf, sizeof(buf),
    "{\"moisture\":%d,\"rawValue\":%d,\"relay\":%s,\"autoMode\":%s,"
    "\"threshold\":%d,\"hysteresis\":%d,\"apMode\":%s,\"ip\":\"%s\","
    "\"ssid\":\"%s\",\"rssi\":%d,\"uptime\":%lu,\"freeHeap\":%u,"
    "\"sensorFault\":%s,\"calibrationDry\":%d,\"calibrationWet\":%d,"
    "\"sensorMin\":%d,\"sensorMax\":%d}",
    smoothedMoisture, rawMoistureValue,
    relayState ? "true" : "false",
    autoMode   ? "true" : "false",
    moistureThreshold, AUTO_HYSTERESIS,
    apMode ? "true" : "false",
    ipStr.c_str(), safeSSID.c_str(),
    rssi, (unsigned long)(millis() / 1000), (unsigned)ESP.getFreeHeap(),
    sensorFault ? "true" : "false",
    calibrationDry, calibrationWet, sensorMin, sensorMax);

  server.send(200, "application/json", buf);
}

bool jsonGetInt(const String& body, const char* key, int& out) {
  String needle = String("\"") + key + "\"";
  int k = body.indexOf(needle);
  if (k < 0) return false;
  int colon = body.indexOf(':', k + needle.length());
  if (colon < 0) return false;
  int i = colon + 1;
  while (i < (int)body.length() && isspace(body[i])) i++;
  int start = i;
  if (i < (int)body.length() && body[i] == '-') i++;
  while (i < (int)body.length() && isdigit(body[i])) i++;
  if (i == start) return false;
  if (i == start + 1 && body[start] == '-') return false;
  out = body.substring(start, i).toInt();
  return true;
}

bool jsonGetBool(const String& body, const char* key, bool& out) {
  String needle = String("\"") + key + "\"";
  int k = body.indexOf(needle);
  if (k < 0) return false;
  int colon = body.indexOf(':', k + needle.length());
  if (colon < 0) return false;
  int i = colon + 1;
  while (i < (int)body.length() && isspace(body[i])) i++;
  if (body.substring(i, i + 4) == "true")  { out = true;  return true; }
  if (body.substring(i, i + 5) == "false") { out = false; return true; }
  return false;
}

bool jsonGetString(const String& body, const char* key, String& out) {
  String needle = String("\"") + key + "\"";
  int k = body.indexOf(needle);
  if (k < 0) return false;
  int colon = body.indexOf(':', k + needle.length());
  if (colon < 0) return false;
  int q1 = body.indexOf('"', colon + 1);
  if (q1 < 0) return false;
  String val;
  for (int i = q1 + 1; i < (int)body.length(); i++) {
    char c = body[i];
    if (c == '\\' && i + 1 < (int)body.length()) {
      char n = body[i + 1];
      if (n == '"')       { val += '"';  i++; continue; }
      if (n == '\\')      { val += '\\'; i++; continue; }
      if (n == 'n')       { val += '\n'; i++; continue; }
      if (n == 'r')       { val += '\r'; i++; continue; }
      if (n == 't')       { val += '\t'; i++; continue; }
      val += c;
      continue;
    }
    if (c == '"') {
      out = val;
      return true;
    }
    val += c;
  }
  return false;
}

void sendOK() {
  server.send(200, "application/json", "{\"success\":true}");
}

void sendErr(const char* msg) {
  String r = String("{\"success\":false,\"error\":\"") + msg + "\"}";
  server.send(400, "application/json", r);
}

void handleRelay() {
  if (!server.hasArg("plain")) { sendErr("No body"); return; }
  bool st;
  if (!jsonGetBool(server.arg("plain"), "state", st)) { sendErr("No state"); return; }

  if (st != relayState) {
    if (autoMode) {
      autoMode = false;
      saveBoolToEEPROM(ADDR_AUTO_MODE, autoMode);
      markEepromDirty();
    }
    setRelayState(st, true);
  }
  sendOK();
}

void handleRelayToggle() {
  bool st = !relayState;
  if (autoMode) {
    autoMode = false;
    saveBoolToEEPROM(ADDR_AUTO_MODE, autoMode);
    markEepromDirty();
  }
  setRelayState(st, true);
  sendOK();
}

void handleAuto() {
  if (!server.hasArg("plain")) { sendErr("No body"); return; }
  bool en;
  if (!jsonGetBool(server.arg("plain"), "enabled", en)) { sendErr("No enabled"); return; }
  autoMode = en;
  saveBoolToEEPROM(ADDR_AUTO_MODE, autoMode);
  markEepromDirty();
  if (en) handleAutoMode();
  sendOK();
}

void handleThreshold() {
  if (!server.hasArg("plain")) { sendErr("No body"); return; }
  int v;
  if (!jsonGetInt(server.arg("plain"), "threshold", v)) { sendErr("No threshold"); return; }
  if (v < 5)  v = 5;
  if (v > 95) v = 95;
  moistureThreshold = v;
  saveIntToEEPROM(ADDR_MOISTURE_THRESHOLD, moistureThreshold);
  markEepromDirty();
  sendOK();
}

void handleWiFi() {
  if (!server.hasArg("plain")) { sendErr("No body"); return; }
  String ssid, pass;
  if (!jsonGetString(server.arg("plain"), "ssid", ssid)) { sendErr("No ssid"); return; }
  jsonGetString(server.arg("plain"), "password", pass);
  if (ssid.length() == 0 || ssid.length() > WIFI_SSID_MAX) { sendErr("Invalid SSID"); return; }
  if (pass.length() > WIFI_PASS_MAX) { sendErr("Password too long"); return; }

  wifiSSID = ssid;
  wifiPassword = pass;
  saveSettings();

  sendOK();
  pendingRestart = true;
  pendingRestartAt = millis() + 1500;
}

void handleWiFiReset() {
  wifiSSID = "";
  wifiPassword = "";
  saveSettings();
  sendOK();
  pendingRestart = true;
  pendingRestartAt = millis() + 1500;
}

void handleCalibrate() {
  if (!server.hasArg("plain")) { sendErr("No body"); return; }
  if (sensorFault) { sendErr("Sensor fault"); return; }
  String type;
  if (!jsonGetString(server.arg("plain"), "type", type)) { sendErr("No type"); return; }

  if (type == "dry") {
    calibrationDry = rawMoistureValue;
  } else if (type == "wet") {
    calibrationWet = rawMoistureValue;
  } else {
    sendErr("Invalid type");
    return;
  }
  normalizeCalibration();
  sensorMin = calibrationDry;
  sensorMax = calibrationWet;
  saveIntToEEPROM(ADDR_CALIBRATION_DRY, calibrationDry);
  saveIntToEEPROM(ADDR_CALIBRATION_WET, calibrationWet);
  saveIntToEEPROM(ADDR_SENSOR_MIN,      sensorMin);
  saveIntToEEPROM(ADDR_SENSOR_MAX,      sensorMax);
  markEepromDirty();
  sendOK();
}

void handleAdvancedCalibration() {
  if (!server.hasArg("plain")) { sendErr("No body"); return; }
  int mn, mx;
  if (!jsonGetInt(server.arg("plain"), "min", mn)) { sendErr("No min"); return; }
  if (!jsonGetInt(server.arg("plain"), "max", mx)) { sendErr("No max"); return; }
  if (mn < 0 || mn > 4095 || mx < 0 || mx > 4095 || mn == mx) { sendErr("Invalid range"); return; }

  calibrationDry = mn;
  calibrationWet = mx;
  normalizeCalibration();
  sensorMin = calibrationDry;
  sensorMax = calibrationWet;

  saveIntToEEPROM(ADDR_SENSOR_MIN,      sensorMin);
  saveIntToEEPROM(ADDR_SENSOR_MAX,      sensorMax);
  saveIntToEEPROM(ADDR_CALIBRATION_DRY, calibrationDry);
  saveIntToEEPROM(ADDR_CALIBRATION_WET, calibrationWet);
  markEepromDirty();
  sendOK();
}

void handleFavicon() {
  server.send(204, "text/plain", "");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not Found");
}

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
#if RELAY_ACTIVE_LOW
  digitalWrite(RELAY_PIN, HIGH);
#else
  digitalWrite(RELAY_PIN, LOW);
#endif
  pinMode(SOIL_MOISTURE_PIN, INPUT);

  EEPROM.begin(EEPROM_SIZE);
  loadSettings();

  relayWrite(relayState);

  rawMoistureValue = readMoistureRaw();
  moisturePercent  = calculateMoisturePercent(rawMoistureValue);
  for (int i = 0; i < SMOOTHING_SAMPLES; i++) moistureReadings[i] = moisturePercent;
  smoothedMoisture = moisturePercent;

  beginWifiConnect();

  server.on("/",                        HTTP_GET,  []() { server.send_P(200, "text/html", INDEX_HTML); });
  server.on("/favicon.ico",             HTTP_GET,  handleFavicon);
  server.on("/api/status",              HTTP_GET,  handleStatus);
  server.on("/api/relay",               HTTP_POST, handleRelay);
  server.on("/api/relay/toggle",        HTTP_POST, handleRelayToggle);
  server.on("/api/auto",                HTTP_POST, handleAuto);
  server.on("/api/threshold",           HTTP_POST, handleThreshold);
  server.on("/api/wifi",                HTTP_POST, handleWiFi);
  server.on("/api/wifi/reset",          HTTP_POST, handleWiFiReset);
  server.on("/api/calibrate",           HTTP_POST, handleCalibrate);
  server.on("/api/calibrate/advanced",  HTTP_POST, handleAdvancedCalibration);
  server.onNotFound(handleNotFound);

  server.begin();
}

void loop() {
  server.handleClient();
  serviceWifi();
  serviceEeprom();

  unsigned long now = millis();

  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    updateSensorReadings();
    handleAutoMode();
  }

  if (now - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
    lastWiFiCheck = now;
    if (wifiState == WIFI_STATE_CONNECTED && WiFi.status() != WL_CONNECTED) {
      beginWifiConnect();
    }
  }

  if (pendingRestart && (long)(now - pendingRestartAt) >= 0) {
    if (eepromDirty) EEPROM.commit();
    delay(100);
    ESP.restart();
  }

  delay(2);
}
