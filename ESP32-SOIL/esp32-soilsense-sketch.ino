/* 
==========================================
 * ESP32 Soil Moisture Monitor & Relay Control
==========================================
 */

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <esp_task_wdt.h>

// ---------- Pins ----------
#define SOIL_MOISTURE_PIN   34
#define RELAY_PIN           25
#define LED_PIN             2
#define RELAY_ACTIVE_HIGH   1

// ---------- Constants ----------
#define AP_SSID              "ESP32-SoilMonitor"
#define AP_PASSWORD          "configure123"
#define OTA_PASSWORD         "admin123"
#define MDNS_HOSTNAME        "soilmonitor"
#define WDT_TIMEOUT_S        30

#define SENSOR_DRY_VALUE     3200
#define SENSOR_WET_VALUE     1200
#define SENSOR_SAMPLES       10

#define DEFAULT_THRESH_LOW       30.0f
#define DEFAULT_THRESH_HIGH      70.0f
#define DEFAULT_AUTO_MODE        true
#define DEFAULT_WATER_DURATION   30
#define DEFAULT_COOLDOWN         300

#define SENSOR_READ_INTERVAL     2000
#define WIFI_RETRY_INTERVAL      5000
#define WIFI_TIMEOUT             30000
#define AP_TIMEOUT_MS            300000
#define WS_CLEANUP_INTERVAL      1500
#define CONFIG_SAVE_MIN_INTERVAL 2000
#define CONFIG_FLUSH_INTERVAL    5000

// ---------- Globals ----------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences prefs;

bool apMode = false;
bool relayState = false;
bool autoMode = true;
bool isWatering = false;
bool otaInProgress = false;
bool mdnsStarted = false;

volatile bool broadcastPending = false;
volatile bool rebootPending = false;
volatile unsigned long rebootAt = 0;

float currentMoisture = 0.0f;
int rawSensorValue = 0;
float sensorVoltage = 0.0f;

struct Config {
  float threshLow;
  float threshHigh;
  bool autoMode;
  int wateringDuration;
  int cooldownPeriod;
  char wifiSSID[64];
  char wifiPassword[64];
} config;

portMUX_TYPE configMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool configDirty = false;
unsigned long lastConfigSave = 0;
unsigned long lastConfigFlush = 0;

unsigned long lastSensorRead = 0;
unsigned long lastWiFiRetry = 0;
unsigned long lastWsCleanup = 0;
unsigned long lastWateringTime = 0;
unsigned long wateringStartTime = 0;
unsigned long apStartTime = 0;
unsigned long totalWateringTime = 0;
unsigned long wateringCycles = 0;
unsigned long uptimeSec = 0;

// ---------- Forward decls ----------
void loadConfig();
void saveConfig(bool force = false);
void flushConfigIfDirty();
void setupWiFi();
void setupAP();
void setupWebServer();
void setupWebSocket();
void setupOTA();
void setupMDNS();
void readSensor();
float calculateMoisture(int rawValue);
void controlRelay(bool state);
void processAutoIrrigation();
void handleWebSocketMessage(AsyncWebSocketClient *client, void *arg, uint8_t *data, size_t len);
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len);
String getSystemStatusJSON();
void applyConfigFromJSON(JsonDocument &doc);
void checkWiFiConnection();
void checkAPTimeout();
void resetConfig();
void blinkLED(int times, int delayMs);
String getDashboardHTML();
String getUpdateHTML();
void requestBroadcast() { broadcastPending = true; }

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(RELAY_PIN, OUTPUT);
#if RELAY_ACTIVE_HIGH
  digitalWrite(RELAY_PIN, LOW);
#else
  digitalWrite(RELAY_PIN, HIGH);
#endif
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(SOIL_MOISTURE_PIN, INPUT);
  analogReadResolution(12);
#if defined(ADC_ATTEN_DB_11)
  analogSetAttenuation(ADC_ATTEN_DB_11);
#elif defined(ADC_11db)
  analogSetAttenuation(ADC_11db);
#elif defined(ADC_ATTEN_DB_12)
  analogSetAttenuation(ADC_ATTEN_DB_12);
#else
  analogSetAttenuation(ADC_ATTENDB_MAX);
#endif

  prefs.begin("soilmonitor", false);
  loadConfig();

  setupWiFi();
  setupOTA();
  setupMDNS();
  setupWebServer();
  setupWebSocket();

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg = {
    .timeout_ms = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_deinit();
  esp_task_wdt_init(&cfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_err_t werr = esp_task_wdt_add(NULL);
  (void)werr;

  readSensor();
  blinkLED(3, 200);
}

// ==================== LOOP ====================
void loop() {
  unsigned long now = millis();
  esp_task_wdt_reset();

  uptimeSec = now / 1000;

  checkWiFiConnection();
  checkAPTimeout();

  if (now - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = now;
    readSensor();
  }

  if (autoMode && !otaInProgress) processAutoIrrigation();

  static unsigned long lastStatTick = 0;
  if (now - lastStatTick >= 1000) {
    lastStatTick = now;
    if (isWatering) totalWateringTime++;
  }

  if (now - lastWsCleanup >= WS_CLEANUP_INTERVAL) {
    lastWsCleanup = now;
    ws.cleanupClients();
  }

  flushConfigIfDirty();

  if (broadcastPending) {
    broadcastPending = false;
    if (ws.count() > 0) ws.textAll(getSystemStatusJSON());
  }

  static unsigned long lastBroadcast = 0;
  if (now - lastBroadcast >= 2000 && ws.count() > 0) {
    lastBroadcast = now;
    ws.textAll(getSystemStatusJSON());
  }

  if (rebootPending && now >= rebootAt) {
    ESP.restart();
  }

  ArduinoOTA.handle();
  delay(10);
}

// ==================== CONFIG ====================
void loadConfig() {
  portENTER_CRITICAL(&configMux);
  config.threshLow        = prefs.getFloat("tLow", DEFAULT_THRESH_LOW);
  config.threshHigh       = prefs.getFloat("tHigh", DEFAULT_THRESH_HIGH);
  config.autoMode         = prefs.getBool("auto", DEFAULT_AUTO_MODE);
  config.wateringDuration = prefs.getInt("wDur", DEFAULT_WATER_DURATION);
  config.cooldownPeriod   = prefs.getInt("cool", DEFAULT_COOLDOWN);
  String s = prefs.getString("ssid", "");
  String p = prefs.getString("pass", "");
  s.toCharArray(config.wifiSSID, sizeof(config.wifiSSID));
  p.toCharArray(config.wifiPassword, sizeof(config.wifiPassword));
  autoMode = config.autoMode;
  portEXIT_CRITICAL(&configMux);
}

void saveConfig(bool force) {
  unsigned long now = millis();
  if (!force && now - lastConfigSave < CONFIG_SAVE_MIN_INTERVAL) {
    configDirty = true;
    return;
  }
  lastConfigSave = now;
  configDirty = false;

  portENTER_CRITICAL(&configMux);
  float tLow  = config.threshLow;
  float tHigh = config.threshHigh;
  bool  aM    = config.autoMode;
  int   wDur  = config.wateringDuration;
  int   cool  = config.cooldownPeriod;
  char  ssid[64];
  char  pass[64];
  memcpy(ssid, config.wifiSSID, sizeof(ssid));
  memcpy(pass, config.wifiPassword, sizeof(pass));
  portEXIT_CRITICAL(&configMux);

  prefs.putFloat("tLow", tLow);
  prefs.putFloat("tHigh", tHigh);
  prefs.putBool("auto", aM);
  prefs.putInt("wDur", wDur);
  prefs.putInt("cool", cool);
  prefs.putString("ssid", String(ssid));
  prefs.putString("pass", String(pass));
}

void flushConfigIfDirty() {
  unsigned long now = millis();
  if (!configDirty) return;
  if (now - lastConfigFlush < CONFIG_FLUSH_INTERVAL) return;
  lastConfigFlush = now;
  saveConfig(true);
}

void resetConfig() {
  prefs.clear();
  portENTER_CRITICAL(&configMux);
  config.threshLow        = DEFAULT_THRESH_LOW;
  config.threshHigh       = DEFAULT_THRESH_HIGH;
  config.autoMode         = DEFAULT_AUTO_MODE;
  config.wateringDuration = DEFAULT_WATER_DURATION;
  config.cooldownPeriod   = DEFAULT_COOLDOWN;
  config.wifiSSID[0]      = 0;
  config.wifiPassword[0]  = 0;
  autoMode = config.autoMode;
  portEXIT_CRITICAL(&configMux);
  saveConfig(true);
}

// ==================== WIFI ====================
void setupWiFi() {
  if (strlen(config.wifiSSID) > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(config.wifiSSID, config.wifiPassword);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_TIMEOUT) {
      delay(500);
      esp_task_wdt_reset();
    }
    if (WiFi.status() == WL_CONNECTED) {
      apMode = false;
      return;
    }
  }
  setupAP();
}

void setupAP() {
  apMode = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  apStartTime = millis();
}

void checkWiFiConnection() {
  if (apMode) return;
  if (WiFi.status() == WL_CONNECTED) {
    if (!mdnsStarted) {
      setupMDNS();
    }
    return;
  }
  if (millis() - lastWiFiRetry < WIFI_RETRY_INTERVAL) return;
  lastWiFiRetry = millis();
  WiFi.disconnect();
  WiFi.begin(config.wifiSSID, config.wifiPassword);
}

void checkAPTimeout() {
  if (!apMode) return;
  if (strlen(config.wifiSSID) == 0) return;
  if (millis() - apStartTime < AP_TIMEOUT_MS) return;
  apMode = false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(config.wifiSSID, config.wifiPassword);
  apStartTime = millis();
}

void setupMDNS() {
  if (apMode || WiFi.status() != WL_CONNECTED) return;
  if (mdnsStarted) return;
  if (MDNS.begin(MDNS_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    mdnsStarted = true;
  }
}

// ==================== OTA ====================
void setupOTA() {
  ArduinoOTA.setHostname(MDNS_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    otaInProgress = true;
    digitalWrite(LED_PIN, HIGH);
  });
  ArduinoOTA.onEnd([]() {
    otaInProgress = false;
    digitalWrite(LED_PIN, LOW);
  });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    esp_task_wdt_reset();
    (void)p; (void)t;
  });
  ArduinoOTA.onError([](ota_error_t e) {
    otaInProgress = false;
    digitalWrite(LED_PIN, LOW);
    (void)e;
  });
  ArduinoOTA.begin();
}

// ==================== SENSOR ====================
void readSensor() {
  long sum = 0;
  for (int i = 0; i < SENSOR_SAMPLES; i++) {
    sum += analogRead(SOIL_MOISTURE_PIN);
    delayMicroseconds(500);
  }
  rawSensorValue = sum / SENSOR_SAMPLES;
  sensorVoltage = rawSensorValue * 3.3f / 4095.0f;
  currentMoisture = calculateMoisture(rawSensorValue);
}

float calculateMoisture(int raw) {
  int clamped = constrain(raw, SENSOR_WET_VALUE, SENSOR_DRY_VALUE);
  float m = 100.0f * (float)(SENSOR_DRY_VALUE - clamped) /
            (float)(SENSOR_DRY_VALUE - SENSOR_WET_VALUE);
  return constrain(m, 0.0f, 100.0f);
}

// ==================== RELAY ====================
void controlRelay(bool state) {
  bool changed = (relayState != state);
  relayState = state;
#if RELAY_ACTIVE_HIGH
  digitalWrite(RELAY_PIN, state ? HIGH : LOW);
#else
  digitalWrite(RELAY_PIN, state ? LOW : HIGH);
#endif

  if (state) {
    if (!isWatering) {
      wateringStartTime = millis();
      isWatering = true;
      wateringCycles++;
    }
    lastWateringTime = millis();
  } else {
    isWatering = false;
  }
  if (changed) requestBroadcast();
}

void processAutoIrrigation() {
  unsigned long now = millis();

  if (isWatering &&
      (now - wateringStartTime) > (unsigned long)config.wateringDuration * 1000UL) {
    controlRelay(false);
    return;
  }

  bool inCooldown = (lastWateringTime > 0) &&
                    (now - lastWateringTime) <
                    (unsigned long)config.cooldownPeriod * 1000UL;

  if (!isWatering && !inCooldown && currentMoisture < config.threshLow) {
    controlRelay(true);
  } else if (isWatering && currentMoisture >= config.threshHigh) {
    controlRelay(false);
  }
}

// ==================== WEBSOCKET ====================
void setupWebSocket() {
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
}

void onWebSocketEvent(AsyncWebSocket *srv, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  (void)srv;
  switch (type) {
    case WS_EVT_CONNECT:
      requestBroadcast();
      break;
    case WS_EVT_DISCONNECT:
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(client, arg, data, len);
      break;
    default: break;
  }
}

void handleWebSocketMessage(AsyncWebSocketClient *client, void *arg,
                            uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (!(info->final && info->index == 0 && info->len == len &&
        info->opcode == WS_TEXT)) return;

  String message((char*)data, len);

  DynamicJsonDocument doc(768);
  if (deserializeJson(doc, message)) {
    return;
  }

  String cmd = doc["command"] | "";

  if (cmd == "setRelay") {
    if (autoMode) {
      DynamicJsonDocument err(128);
      err["type"] = "error";
      err["message"] = "Auto mode active";
      String s; serializeJson(err, s);
      if (client) client->text(s);
    } else {
      controlRelay(doc["state"] | false);
    }
  }
  else if (cmd == "setAutoMode") {
    bool enabled = doc["enabled"] | false;
    portENTER_CRITICAL(&configMux);
    autoMode = enabled;
    config.autoMode = enabled;
    portEXIT_CRITICAL(&configMux);
    if (!autoMode) {
      controlRelay(false);
    } else {
      wateringStartTime = millis();
      lastWateringTime = 0;
    }
    saveConfig(true);
    requestBroadcast();
  }
  else if (cmd == "updateConfig") {
    applyConfigFromJSON(doc);
    saveConfig();
    requestBroadcast();
  }
  else if (cmd == "getStatus") {
    requestBroadcast();
  }
  else if (cmd == "resetConfig") {
    resetConfig();
    requestBroadcast();
  }
}

// ==================== JSON ====================
String getSystemStatusJSON() {
  DynamicJsonDocument doc(1024);
  doc["type"] = "status";
  doc["uptime"] = uptimeSec;
  doc["freeHeap"] = ESP.getFreeHeap();
  bool staConnected = (WiFi.status() == WL_CONNECTED);
  doc["wifiRSSI"] = apMode ? 0 : (staConnected ? WiFi.RSSI() : 0);
  doc["ipAddress"] = apMode ? WiFi.softAPIP().toString()
                            : WiFi.localIP().toString();
  doc["apMode"] = apMode;
  doc["moisture"] = currentMoisture;
  doc["rawValue"] = rawSensorValue;
  doc["voltage"] = sensorVoltage;
  doc["relayState"] = relayState;
  doc["autoMode"] = autoMode;
  doc["isWatering"] = isWatering;
  doc["otaInProgress"] = otaInProgress;
  doc["totalWateringTime"] = totalWateringTime;
  doc["wateringCycles"] = wateringCycles;
  JsonObject c = doc.createNestedObject("config");
  portENTER_CRITICAL(&configMux);
  c["moistureThresholdLow"]  = config.threshLow;
  c["moistureThresholdHigh"] = config.threshHigh;
  c["autoMode"]              = config.autoMode;
  c["wateringDuration"]      = config.wateringDuration;
  c["cooldownPeriod"]        = config.cooldownPeriod;
  portEXIT_CRITICAL(&configMux);
  String out; serializeJson(doc, out);
  return out;
}

void applyConfigFromJSON(JsonDocument &doc) {
  portENTER_CRITICAL(&configMux);
  if (doc.containsKey("moistureThresholdLow"))
    config.threshLow = constrain(doc["moistureThresholdLow"].as<float>(), 0.0f, 95.0f);
  if (doc.containsKey("moistureThresholdHigh"))
    config.threshHigh = constrain(doc["moistureThresholdHigh"].as<float>(), 5.0f, 100.0f);
  if (doc.containsKey("autoMode")) {
    config.autoMode = doc["autoMode"].as<bool>();
    autoMode = config.autoMode;
  }
  if (doc.containsKey("wateringDuration"))
    config.wateringDuration = constrain(doc["wateringDuration"].as<int>(), 1, 3600);
  if (doc.containsKey("cooldownPeriod"))
    config.cooldownPeriod = constrain(doc["cooldownPeriod"].as<int>(), 0, 86400);

  if (config.threshLow >= config.threshHigh - 5.0f) {
    config.threshHigh = min(100.0f, config.threshLow + 5.0f);
    config.threshLow  = max(0.0f,   config.threshHigh - 5.0f);
  }
  portEXIT_CRITICAL(&configMux);
}

// ==================== WEB SERVER ====================
void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "text/html", getDashboardHTML());
  });
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "application/json", getSystemStatusJSON());
  });
  server.on("/api/relay", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (autoMode) {
      req->send(400, "application/json",
                "{\"success\":false,\"error\":\"Auto mode enabled\"}");
      return;
    }
    if (req->hasParam("state", true)) {
      controlRelay(req->getParam("state", true)->value() == "1");
      req->send(200, "application/json", "{\"success\":true}");
    } else if (req->hasParam("state")) {
      controlRelay(req->getParam("state")->value() == "1");
      req->send(200, "application/json", "{\"success\":true}");
    } else {
      req->send(400, "application/json",
                "{\"success\":false,\"error\":\"Missing state\"}");
    }
  });
  server.on("/api/wifi", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (!req->hasParam("ssid", true)) {
      req->send(400, "application/json",
                "{\"success\":false,\"error\":\"Missing ssid\"}");
      return;
    }
    String ssid = req->getParam("ssid", true)->value();
    String pass = req->hasParam("password", true)
                  ? req->getParam("password", true)->value() : "";
    portENTER_CRITICAL(&configMux);
    ssid.toCharArray(config.wifiSSID, sizeof(config.wifiSSID));
    pass.toCharArray(config.wifiPassword, sizeof(config.wifiPassword));
    portEXIT_CRITICAL(&configMux);
    saveConfig(true);
    req->send(200, "application/json",
              "{\"success\":true,\"message\":\"Saved. Rebooting.\"}");
    rebootPending = true;
    rebootAt = millis() + 1500;
  });

  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "text/html", getUpdateHTML());
  });

  server.on("/update", HTTP_POST,
    [](AsyncWebServerRequest *req) {
      bool ok = !Update.hasError();
      AsyncWebServerResponse *r = req->beginResponse(200, "text/plain",
                                                     ok ? "OK" : "FAIL");
      r->addHeader("Connection", "close");
      req->send(r);
      if (ok) {
        rebootPending = true;
        rebootAt = millis() + 1500;
      }
    },
    [](AsyncWebServerRequest *req, String filename, size_t index,
       uint8_t *data, size_t len, bool final) {
      esp_task_wdt_reset();
      if (!index) {
        size_t sz = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        if (!Update.begin(sz)) Update.printError(Serial);
      }
      if (!Update.hasError()) {
        if (Update.write(data, len) != len) Update.printError(Serial);
      }
      if (final) {
        if (Update.end(true)) { }
        else Update.printError(Serial);
      }
    });

  server.onNotFound([](AsyncWebServerRequest *req) {
    req->send(404, "text/plain", "Not Found");
  });

  server.begin();
}

// ==================== UTILS ====================
void blinkLED(int times, int d) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH); delay(d);
    digitalWrite(LED_PIN, LOW);  delay(d);
  }
}

String getDashboardHTML() {
  return String(R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Soil Monitor</title>
<style>
body{font-family:system-ui,sans-serif;background:#0d1117;color:#e6edf3;margin:0;padding:20px}
.wrap{max-width:900px;margin:auto}
h1{color:#58a6ff;text-align:center}
.card{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:18px;margin:12px 0}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:12px}
.val{font-size:2.2em;text-align:center;color:#58a6ff}
.bar{height:22px;background:#21262d;border-radius:11px;overflow:hidden;margin:10px 0}
.fill{height:100%;transition:.4s;display:flex;align-items:center;justify-content:center;font-size:.85em;color:#000;font-weight:700}
.btn{padding:10px 18px;border:none;border-radius:6px;cursor:pointer;font-weight:700;margin:4px;color:#fff}
.on{background:#238636}.off{background:#da3633}.save{background:#1f6feb}.rst{background:#8957e5}
.btn:disabled{opacity:.4;cursor:not-allowed}
input[type=number],input[type=text],input[type=password]{width:100%;padding:8px;background:#0d1117;border:1px solid #30363d;border-radius:5px;color:#e6edf3;box-sizing:border-box}
label{display:block;margin:8px 0 4px;color:#8b949e;font-size:.85em}
.row{display:flex;justify-content:space-between;padding:6px 0;border-bottom:1px solid #21262d}
.row:last-child{border:0}
.sw{position:relative;display:inline-block;width:50px;height:26px}
.sw input{display:none}
.sw span{position:absolute;inset:0;background:#30363d;border-radius:26px;transition:.3s;cursor:pointer}
.sw span::before{content:"";position:absolute;width:20px;height:20px;left:3px;top:3px;background:#fff;border-radius:50%;transition:.3s}
.sw input:checked+span{background:#238636}
.sw input:checked+span::before{transform:translateX(24px)}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px}
.dot.on{background:#3fb950;box-shadow:0 0 8px #3fb950}
.dot.off{background:#f85149}
</style></head><body>
<div class="wrap"><h1>🌱 ESP32 Soil Monitor</h1>
<div class="grid">
<div class="card"><h3>💧 Moisture</h3>
<div class="val"><span id="m">--</span>%</div>
<div class="bar"><div id="mb" class="fill" style="width:0%"></div></div>
<div class="row"><span>Raw</span><b id="raw">--</b></div>
<div class="row"><span>Voltage</span><b id="v">--</b></div></div>

<div class="card"><h3>⚡ Relay</h3>
<div style="text-align:center;margin:14px 0;font-size:1.2em">
<span id="rd" class="dot off"></span><span id="rt">OFF</span></div>
<div style="text-align:center">
<button class="btn on" id="bon" onclick="relay(1)">ON</button>
<button class="btn off" id="boff" onclick="relay(0)">OFF</button></div>
<div style="text-align:center;margin-top:14px">
Auto <label class="sw"><input type="checkbox" id="at" onchange="auto(this.checked)"><span></span></label></div>
<div id="wi" style="text-align:center;color:#3fb950;margin-top:8px;display:none">💦 Watering</div></div>

<div class="card"><h3>⚙️ Config</h3>
<label>Low Threshold (%)</label><input type="number" id="tl" min="0" max="100">
<label>High Threshold (%)</label><input type="number" id="th" min="0" max="100">
<label>Max Duration (s)</label><input type="number" id="wd" min="1" max="3600">
<label>Cooldown (s)</label><input type="number" id="cd" min="0" max="86400">
<div style="text-align:center;margin-top:12px">
<button class="btn save" onclick="save()">Save</button>
<button class="btn rst" onclick="reset()">Reset</button></div></div>

<div class="card"><h3>📊 System</h3>
<div class="row"><span>IP</span><b id="ip">--</b></div>
<div class="row"><span>RSSI</span><b id="rssi">--</b></div>
<div class="row"><span>Uptime</span><b id="up">--</b></div>
<div class="row"><span>Heap</span><b id="hp">--</b></div>
<div class="row"><span>Cycles</span><b id="cy">--</b></div></div>

<div class="card"><h3>📶 WiFi</h3>
<label>SSID</label><input type="text" id="ws" placeholder="WiFi name">
<label>Password</label><input type="password" id="wp">
<div style="text-align:center;margin-top:12px">
<button class="btn save" onclick="wifi()">Save & Reboot</button></div>
<div style="text-align:center;margin-top:8px;color:#8b949e;font-size:.85em">Mode: <span id="wm">--</span></div></div>
</div>
<div style="text-align:center;color:#8b949e;margin-top:20px;font-size:.8em">
v1.3 | <a href="/update" style="color:#58a6ff">OTA</a></div>
</div>
<script>
let ws,rtmr;
function conn(){ws=new WebSocket('ws://'+location.host+'/ws');
ws.onopen=()=>clearInterval(rtmr);
ws.onclose=()=>rtmr=setInterval(conn,2000);
ws.onmessage=e=>{const d=JSON.parse(e.data);if(d.type==='status')upd(d);};
}
function upd(d){
  document.getElementById('m').textContent=d.moisture.toFixed(1);
  const mb=document.getElementById('mb');
  mb.style.width=d.moisture+'%';mb.textContent=d.moisture.toFixed(0)+'%';
  mb.style.background=d.moisture<30?'#f85149':d.moisture<60?'#d29922':'#3fb950';
  document.getElementById('raw').textContent=d.rawValue;
  document.getElementById('v').textContent=d.voltage.toFixed(2)+'V';
  const rd=document.getElementById('rd');
  rd.className='dot '+(d.relayState?'on':'off');
  document.getElementById('rt').textContent=d.relayState?'ON':'OFF';
  document.getElementById('at').checked=d.autoMode;
  document.getElementById('wi').style.display=d.isWatering?'block':'none';
  document.getElementById('ip').textContent=d.ipAddress;
  document.getElementById('rssi').textContent=d.apMode?'AP':(d.wifiRSSI+'dBm');
  document.getElementById('up').textContent=ft(d.uptime);
  document.getElementById('hp').textContent=(d.freeHeap/1024).toFixed(1)+'KB';
  document.getElementById('cy').textContent=d.wateringCycles;
  document.getElementById('wm').textContent=d.apMode?'AP':'STA';
  if(d.config){
    document.getElementById('tl').value=d.config.moistureThresholdLow;
    document.getElementById('th').value=d.config.moistureThresholdHigh;
    document.getElementById('wd').value=d.config.wateringDuration;
    document.getElementById('cd').value=d.config.cooldownPeriod;
  }
  document.getElementById('bon').disabled=d.autoMode;
  document.getElementById('boff').disabled=d.autoMode;
}
function ft(s){if(s<60)return s+'s';if(s<3600)return Math.floor(s/60)+'m';
  return Math.floor(s/3600)+'h'+Math.floor((s%3600)/60)+'m';}
function cmd(o){if(ws&&ws.readyState===1)ws.send(JSON.stringify(o));}
function relay(s){cmd({command:'setRelay',state:s});}
function auto(e){cmd({command:'setAutoMode',enabled:e});}
function save(){cmd({command:'updateConfig',
  moistureThresholdLow:+document.getElementById('tl').value,
  moistureThresholdHigh:+document.getElementById('th').value,
  wateringDuration:+document.getElementById('wd').value,
  cooldownPeriod:+document.getElementById('cd').value});}
function reset(){if(confirm('Reset defaults?'))cmd({command:'resetConfig'});}
function wifi(){const f=new FormData();
  f.append('ssid',document.getElementById('ws').value);
  f.append('password',document.getElementById('wp').value);
  fetch('/api/wifi',{method:'POST',body:f}).then(r=>r.json()).then(j=>{
    alert(j.message||j.error);setTimeout(()=>location.href='/',6000);});}
conn();
setInterval(()=>{if(!ws||ws.readyState!==1)
  fetch('/api/status').then(r=>r.json()).then(upd);},5000);
</script></body></html>)rawliteral");
}

String getUpdateHTML() {
  return String(R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<title>OTA Update</title>
<style>body{font-family:system-ui;background:#0d1117;color:#e6edf3;text-align:center;padding:40px}
input,button{margin:10px;padding:10px;border-radius:6px;border:1px solid #30363d;background:#161b22;color:#e6edf3}
button{background:#1f6feb;cursor:pointer}
a{color:#58a6ff}</style></head><body>
<h1>📦 OTA Update</h1>
<form method="POST" action="/update" enctype="multipart/form-data">
<input type="file" name="firmware" accept=".bin" required><br>
<button>Upload</button></form>
<p><a href="/">← Back</a></p></body></html>)rawliteral");
}
