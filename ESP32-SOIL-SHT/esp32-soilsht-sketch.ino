// =====================================================================
//  ESP32 Soil Moisture and SHT31 Temperature Humidity
// =====================================================================

#include <Arduino.h>
#include <stdarg.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SHT31.h>
#include <esp_heap_caps.h>
#include <esp_attr.h>

// =====================================================
//  BUILD & HARDWARE DEFINITIONS
// =====================================================
#define FIRMWARE_VERSION "1.0.0"
#define CFG_NAMESPACE "app"

#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
#define PIN_RELAY_SHT 25
#define PIN_RELAY_SOIL 26
#define PIN_BUZZER 27
#define PIN_LED 33
#define PIN_SOIL_ADC 34
#define PIN_BOOT_BUTTON 0

#define SHT31_I2C_ADDR 0x44
#define SSD1306_I2C_ADDR 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

#define LEDC_CHANNEL 0
#define LEDC_FREQ 5000UL
#define LEDC_RES 8U

#define HISTORY_MAX 120
#define HIST_INVALID (-999)

#define CONFIG_MAGIC 0x534F494Cu
#define CONFIG_VERSION 1u

#define DEFAULT_AP_SSID "ESP32_SOIL_SHT"
#define DEFAULT_AP_PASS "12345678"

// =====================================================
//  CONFIGURATION MODEL
// =====================================================
struct Config
{
    uint32_t magic;
    uint16_t version;
    uint16_t reserved0;
    uint32_t writeCounter;

    char apSsid[33];
    char apPass[17];
    uint8_t apChannel;
    uint8_t apMaxClients;
    uint8_t apHidden;

    uint8_t shtEnabled;
    uint8_t shtRelayMode;
    uint8_t shtRelayManual;
    uint8_t shtRelayActiveHigh;
    float shtTempOn;
    float shtTempOff;
    float shtHumOn;
    float shtHumOff;
    uint32_t shtIntervalMs;

    uint8_t soilEnabled;
    uint8_t soilRelayMode;
    uint8_t soilRelayManual;
    uint8_t soilRelayActiveHigh;
    int16_t soilDryPct;
    int16_t soilWetPct;
    uint16_t soilRawDry;
    uint16_t soilRawWet;
    uint32_t soilIntervalMs;

    uint8_t buzzerEnabled;
    uint8_t buzzerMode;
    uint8_t buzzerActiveHigh;
    uint16_t buzzerBeepOnMs;
    uint16_t buzzerBeepOffMs;

    uint8_t ledEnabled;
    uint8_t ledMode;
    uint8_t ledBrightness;
    uint8_t ledInvert;

    uint8_t oledEnabled;
    uint8_t oledContrast;
    uint32_t oledIntervalMs;
    uint8_t oledShowSensors;
    uint8_t oledShowStatus;

    uint32_t crc;
};

// =====================================================
//  JSON BUILDER (static buffer, heap safe)
// =====================================================
struct JsonBuf
{
    char buf[8192];
    size_t len;
    bool overflow;

    void init()
    {
        len = 0;
        overflow = false;
        buf[0] = '\0';
    }

    void append(const char *fmt, ...)
    {
        if (overflow)
        {
            return;
        }
        va_list ap;
        va_start(ap, fmt);
        int n = vsnprintf(buf + len, sizeof(buf) - len, fmt, ap);
        va_end(ap);
        if (n < 0)
        {
            overflow = true;
            return;
        }
        if ((size_t)n >= sizeof(buf) - len)
        {
            overflow = true;
            buf[sizeof(buf) - 1U] = '\0';
            return;
        }
        len += (size_t)n;
    }
};

// =====================================================
//  SCHEDULER TYPES
// =====================================================
typedef void (*TaskFn)(void);

struct Task
{
    TaskFn fn;
    uint64_t nextMs;
    uint32_t intervalMs;
};

// =====================================================
//  SMALL STRING HELPERS (no heap, bounded copies)
// =====================================================
static void copyString(char *dst, size_t cap, const char *src)
{
    size_t i = 0;
    while (src[i] != '\0' && i + 1U < cap)
    {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
}

static void sanitizeCopy(char *dst, size_t cap, const char *src)
{
    size_t o = 0;
    for (size_t i = 0; src[i] != '\0' && o + 1U < cap; i++)
    {
        unsigned char c = (unsigned char)src[i];
        if (c >= 0x20U && c <= 0x7EU)
        {
            dst[o++] = (char)c;
        }
    }
    dst[o] = '\0';
}

// =====================================================
//  RUNTIME OBJECTS
// =====================================================
static Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
static Adafruit_SHT31 sht31;
static WebServer server(80);
static DNSServer dnsServer;
static Preferences prefs;

// =====================================================
//  64-BIT TIME BASE
//  millis() is a 32-bit counter and rolls over every
//  49.7 days.  The helper below extends it into a
//  64-bit millisecond clock so uptime, scheduling and
//  history timestamps stay monotonic for years.
// =====================================================
static uint64_t g_nowMs = 0ULL;
static uint32_t g_millisPrev = 0UL;

void updateClock()
{
    uint32_t m = millis();
    if (m < g_millisPrev)
    {
        g_nowMs += 0x100000000ULL;
    }
    g_millisPrev = m;
    g_nowMs = (g_nowMs & 0xFFFFFFFF00000000ULL) | (uint64_t)m;
}

uint64_t nowMs()
{
    return g_nowMs;
}

// =====================================================
//  GLOBAL STATE
// =====================================================
static Config g_cfg;
static uint8_t g_activeSlot = 0;
static bool g_configFresh = false;
static bool g_configDirty = false;
static uint64_t g_configDirtySince = 0ULL;
static uint64_t g_lastConfigWrite = 0ULL;
static uint32_t g_configWriteCount = 0UL;

static float g_temperature = 0.0f;
static float g_humidity = 0.0f;
static uint16_t g_soilRaw = 0;
static uint8_t g_soilPercent = 0;
static bool g_shtOnline = false;
static bool g_shtBegun = false;
static bool g_soilOnline = false;
static uint32_t g_shtErrorCount = 0UL;

static bool g_relayShtState = false;
static bool g_relaySoilState = false;
static bool g_buzzerState = false;
static bool g_buzzerPhase = false;
static uint64_t g_buzzerNext = 0ULL;
static uint64_t g_buzzerTestUntil = 0ULL;

static uint8_t g_ledDutyApplied = 0xFF;
static uint64_t g_btnHoldMs = 0ULL;

static uint64_t g_shtLastRead = 0ULL;
static uint64_t g_soilLastRead = 0ULL;
static uint8_t g_oledPage = 0;
static bool g_oledBegun = false;
static bool g_oledPowered = false;
static uint64_t g_oledNext = 0ULL;

static uint32_t g_freeHeap = 0UL;
static uint32_t g_minFreeHeap = 0xFFFFFFFFUL;
static uint32_t g_largestBlock = 0UL;
static uint8_t g_fragPercent = 0;
static uint8_t g_wifiClients = 0;
static uint64_t g_nextHeapCheck = 0ULL;

static bool g_pendingReset = false;
static bool g_pendingReboot = false;

RTC_DATA_ATTR static uint32_t g_bootCount = 0UL;

// =====================================================
//  CRC32 (IEEE 802.3, bitwise, no RAM table)
// =====================================================
static uint32_t crc32Compute(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= (uint32_t)data[i];
        for (uint8_t k = 0; k < 8; k++)
        {
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

static bool configValidate(const Config &c)
{
    if (c.magic != CONFIG_MAGIC)
    {
        return false;
    }
    if (c.version != CONFIG_VERSION)
    {
        return false;
    }
    Config tmp;
    memcpy(&tmp, &c, sizeof(Config));
    tmp.crc = 0;
    uint32_t expected = crc32Compute((const uint8_t *)&tmp, sizeof(Config));
    return expected == c.crc;
}

// =====================================================
//  CONFIG DEFAULTS
// =====================================================
static void configDefaults(Config &c)
{
    memset(&c, 0, sizeof(Config));
    c.magic = CONFIG_MAGIC;
    c.version = CONFIG_VERSION;
    c.writeCounter = 0;

    copyString(c.apSsid, sizeof(c.apSsid), DEFAULT_AP_SSID);
    copyString(c.apPass, sizeof(c.apPass), DEFAULT_AP_PASS);
    c.apChannel = 1;
    c.apMaxClients = 4;
    c.apHidden = 0;

    c.shtEnabled = 1;
    c.shtRelayMode = 1;
    c.shtRelayManual = 0;
    c.shtRelayActiveHigh = 1;
    c.shtTempOn = 30.0f;
    c.shtTempOff = 26.0f;
    c.shtHumOn = 80.0f;
    c.shtHumOff = 60.0f;
    c.shtIntervalMs = 2000UL;

    c.soilEnabled = 1;
    c.soilRelayMode = 1;
    c.soilRelayManual = 0;
    c.soilRelayActiveHigh = 1;
    c.soilDryPct = 30;
    c.soilWetPct = 60;
    c.soilRawDry = 3200;
    c.soilRawWet = 1400;
    c.soilIntervalMs = 2000UL;

    c.buzzerEnabled = 0;
    c.buzzerMode = 2;
    c.buzzerActiveHigh = 1;
    c.buzzerBeepOnMs = 200;
    c.buzzerBeepOffMs = 800;

    c.ledEnabled = 1;
    c.ledMode = 2;
    c.ledBrightness = 128;
    c.ledInvert = 0;

    c.oledEnabled = 1;
    c.oledContrast = 128;
    c.oledIntervalMs = 1000UL;
    c.oledShowSensors = 1;
    c.oledShowStatus = 1;
}

// =====================================================
//  NVS PERSISTENCE MANAGER
//  Two alternating slots + CRC32 + magic + version.
//  A power cut during a write only damages the slot
//  being written; the previous slot stays valid.
// =====================================================
static const char *CONFIG_SLOT_KEYS[2] = {"cfgA", "cfgB"};

static bool configReadSlot(uint8_t slot, Config &out)
{
    size_t n = prefs.getBytes(CONFIG_SLOT_KEYS[slot], &out, sizeof(Config));
    if (n != sizeof(Config))
    {
        return false;
    }
    return configValidate(out);
}

static bool configWriteSlot(uint8_t slot, const Config &c)
{
    size_t n = prefs.putBytes(CONFIG_SLOT_KEYS[slot], &c, sizeof(Config));
    if (n != sizeof(Config))
    {
        return false;
    }
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    return true;
#else
    return prefs.commit();
#endif
}

static void configLoad()
{
    Config a;
    Config b;
    bool okA = configReadSlot(0, a);
    bool okB = configReadSlot(1, b);

    if (okA && okB)
    {
        if (a.writeCounter >= b.writeCounter)
        {
            memcpy(&g_cfg, &a, sizeof(Config));
            g_activeSlot = 0;
        }
        else
        {
            memcpy(&g_cfg, &b, sizeof(Config));
            g_activeSlot = 1;
        }
        g_configFresh = false;
    }
    else if (okA)
    {
        memcpy(&g_cfg, &a, sizeof(Config));
        g_activeSlot = 0;
        g_configFresh = false;
    }
    else if (okB)
    {
        memcpy(&g_cfg, &b, sizeof(Config));
        g_activeSlot = 1;
        g_configFresh = false;
    }
    else
    {
        configDefaults(g_cfg);
        g_activeSlot = 0;
        g_configFresh = true;
    }
}

static bool configSave()
{
    Config tmp;
    memcpy(&tmp, &g_cfg, sizeof(Config));
    tmp.writeCounter = g_cfg.writeCounter + 1U;
    tmp.crc = 0;
    tmp.crc = crc32Compute((const uint8_t *)&tmp, sizeof(Config));

    uint8_t target = (uint8_t)(1U - g_activeSlot);
    if (!configWriteSlot(target, tmp))
    {
        return false;
    }
    g_cfg.writeCounter = tmp.writeCounter;
    g_cfg.crc = tmp.crc;
    g_activeSlot = target;
    g_configWriteCount++;
    g_lastConfigWrite = nowMs();
    return true;
}

static void markConfigDirty()
{
    if (!g_configDirty)
    {
        g_configDirty = true;
        g_configDirtySince = nowMs();
    }
}

void taskConfigFlush()
{
    if (!g_configDirty)
    {
        return;
    }
    uint64_t now = nowMs();
    bool settled = (now - g_configDirtySince) >= 1500ULL;
    bool forced = (now - g_configDirtySince) >= 10000ULL;
    bool spacingOk = (now - g_lastConfigWrite) >= 500ULL;
    if ((settled && spacingOk) || forced)
    {
        if (configSave())
        {
            g_configDirty = false;
            g_configDirtySince = 0ULL;
        }
    }
}

static void doFactoryReset()
{
    prefs.clear();
    prefs.end();
    WiFi.softAPdisconnect(true);
    ESP.restart();
}

// =====================================================
//  HISTORY RING BUFFERS (static size, zero allocation)
// =====================================================
static int16_t g_histTemp[HISTORY_MAX];
static int16_t g_histHum[HISTORY_MAX];
static int16_t g_histSoil[HISTORY_MAX];
static uint16_t g_histHead = 0;
static uint16_t g_histCount = 0;
static uint64_t g_sampleCount = 0ULL;

static void historyPush()
{
    int16_t t = HIST_INVALID;
    int16_t h = HIST_INVALID;
    if (g_shtOnline)
    {
        t = (int16_t)lrintf(g_temperature * 10.0f);
        h = (int16_t)lrintf(g_humidity * 10.0f);
    }
    int16_t s = g_soilOnline ? (int16_t)g_soilPercent : HIST_INVALID;

    g_histTemp[g_histHead] = t;
    g_histHum[g_histHead] = h;
    g_histSoil[g_histHead] = s;
    g_histHead = (uint16_t)((g_histHead + 1U) % HISTORY_MAX);
    if (g_histCount < HISTORY_MAX)
    {
        g_histCount++;
    }
    g_sampleCount++;
}

void taskHistory()
{
    historyPush();
}

// =====================================================
//  OUTPUT DRIVERS
// =====================================================
static void applyRelay(uint8_t pin, bool activeHigh, bool on)
{
    digitalWrite(pin, (on == activeHigh) ? HIGH : LOW);
}

static void applyBuzzer(bool on)
{
    bool level = g_cfg.buzzerActiveHigh ? on : !on;
    digitalWrite(PIN_BUZZER, level ? HIGH : LOW);
}

static void ledWriteDuty(uint8_t duty)
{
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(PIN_LED, (uint32_t)duty);
#else
    ledcWrite(LEDC_CHANNEL, (uint32_t)duty);
#endif
}

void taskBuzzer()
{
    uint64_t now = nowMs();
    bool out = false;

    if (g_buzzerTestUntil != 0ULL)
    {
        if (now < g_buzzerTestUntil)
        {
            out = true;
        }
        else
        {
            g_buzzerTestUntil = 0ULL;
        }
    }
    else if (g_cfg.buzzerEnabled)
    {
        if (g_cfg.buzzerMode == 1)
        {
            out = true;
        }
        else if (g_cfg.buzzerMode == 2)
        {
            if (g_buzzerNext == 0ULL)
            {
                g_buzzerNext = now;
                g_buzzerPhase = true;
            }
            if (now >= g_buzzerNext)
            {
                g_buzzerPhase = !g_buzzerPhase;
                uint32_t span = g_buzzerPhase ? (uint32_t)g_cfg.buzzerBeepOnMs
                                              : (uint32_t)g_cfg.buzzerBeepOffMs;
                if (span < 20UL)
                {
                    span = 20UL;
                }
                g_buzzerNext = now + (uint64_t)span;
            }
            out = g_buzzerPhase;
        }
        else
        {
            g_buzzerPhase = false;
            g_buzzerNext = 0ULL;
        }
    }
    else
    {
        g_buzzerPhase = false;
        g_buzzerNext = 0ULL;
    }

    if (out != g_buzzerState)
    {
        g_buzzerState = out;
        applyBuzzer(out);
    }
}

void taskLed()
{
    uint8_t duty = 0;

    if (g_btnHoldMs != 0ULL)
    {
        duty = 255;
    }
    else if (g_cfg.ledEnabled)
    {
        if (g_cfg.ledMode == 1)
        {
            duty = 255;
        }
        else if (g_cfg.ledMode == 2)
        {
            duty = g_cfg.ledBrightness;
        }
        else if (g_cfg.ledMode == 3)
        {
            uint64_t phase = (nowMs() / 500ULL) & 1ULL;
            duty = (phase != 0ULL) ? g_cfg.ledBrightness : 0;
        }
    }

    if (g_cfg.ledInvert)
    {
        duty = (uint8_t)(255 - duty);
    }

    if (duty != g_ledDutyApplied)
    {
        g_ledDutyApplied = duty;
        ledWriteDuty(duty);
    }
}

// =====================================================
//  SENSOR DRIVERS
// =====================================================
static void readSht31()
{
    if (!g_shtBegun)
    {
        g_shtBegun = sht31.begin(SHT31_I2C_ADDR);
    }
    if (!g_shtBegun)
    {
        g_shtOnline = false;
        g_shtErrorCount++;
        return;
    }

    float t = sht31.readTemperature();
    float h = sht31.readHumidity();
    if (isnan(t) || isnan(h))
    {
        g_shtOnline = false;
        g_shtErrorCount++;
        return;
    }
    g_temperature = t;
    g_humidity = h;
    g_shtOnline = true;
}

static void readSoil()
{
    uint32_t acc = 0UL;
    for (uint8_t i = 0; i < 8; i++)
    {
        acc += (uint32_t)analogRead(PIN_SOIL_ADC);
    }
    g_soilRaw = (uint16_t)(acc >> 3);

    int32_t span = (int32_t)g_cfg.soilRawDry - (int32_t)g_cfg.soilRawWet;
    int32_t pct = 0;
    if (span > 1)
    {
        pct = 100 * ((int32_t)g_cfg.soilRawDry - (int32_t)g_soilRaw) / span;
        if (pct < 0)
        {
            pct = 0;
        }
        if (pct > 100)
        {
            pct = 100;
        }
    }
    g_soilPercent = (uint8_t)pct;
    g_soilOnline = true;
}

// =====================================================
//  RELAY POLICY (hysteresis state machine)
// =====================================================
void evaluateRelays()
{
    bool wantSht = g_relayShtState;
    if (g_cfg.shtRelayMode == 0)
    {
        wantSht = (g_cfg.shtRelayManual != 0);
    }
    else if (g_shtOnline)
    {
        bool tempOn = g_temperature >= g_cfg.shtTempOn;
        bool humOn = g_humidity >= g_cfg.shtHumOn;
        bool tempOff = g_temperature <= g_cfg.shtTempOff;
        bool humOff = g_humidity <= g_cfg.shtHumOff;
        if (tempOn || humOn)
        {
            wantSht = true;
        }
        else if (tempOff && humOff)
        {
            wantSht = false;
        }
    }
    if (wantSht != g_relayShtState)
    {
        g_relayShtState = wantSht;
        applyRelay(PIN_RELAY_SHT, g_cfg.shtRelayActiveHigh != 0, wantSht);
    }

    bool wantSoil = g_relaySoilState;
    if (g_cfg.soilRelayMode == 0)
    {
        wantSoil = (g_cfg.soilRelayManual != 0);
    }
    else if (g_soilOnline)
    {
        if ((int32_t)g_soilPercent <= (int32_t)g_cfg.soilDryPct)
        {
            wantSoil = true;
        }
        else if ((int32_t)g_soilPercent >= (int32_t)g_cfg.soilWetPct)
        {
            wantSoil = false;
        }
    }
    if (wantSoil != g_relaySoilState)
    {
        g_relaySoilState = wantSoil;
        applyRelay(PIN_RELAY_SOIL, g_cfg.soilRelayActiveHigh != 0, wantSoil);
    }
}

void taskSensors()
{
    uint64_t now = nowMs();

    if (g_cfg.shtEnabled)
    {
        if ((uint64_t)g_cfg.shtIntervalMs == 0ULL ||
            (now - g_shtLastRead) >= (uint64_t)g_cfg.shtIntervalMs)
        {
            g_shtLastRead = now;
            readSht31();
        }
    }
    if (g_cfg.soilEnabled)
    {
        if ((uint64_t)g_cfg.soilIntervalMs == 0ULL ||
            (now - g_soilLastRead) >= (uint64_t)g_cfg.soilIntervalMs)
        {
            g_soilLastRead = now;
            readSoil();
        }
    }

    evaluateRelays();
}

// =====================================================
//  OLED RENDERING (non-blocking, page rotated by timer)
// =====================================================
static void oledRenderSensors()
{
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("SHT31 ");
    display.print(g_shtOnline ? "OK" : "ERR");
    display.setCursor(92, 0);
    display.print(g_cfg.shtRelayMode ? "A" : "M");
    display.print(g_relayShtState ? "*" : "-");

    display.setTextSize(2);
    display.setCursor(0, 12);
    if (g_shtOnline)
    {
        display.print(g_temperature, 1);
        display.print("C");
    }
    else
    {
        display.print("--.-C");
    }

    display.setCursor(0, 32);
    if (g_shtOnline)
    {
        display.print(g_humidity, 1);
        display.print("%");
    }
    else
    {
        display.print("--.-%");
    }

    display.setTextSize(1);
    display.setCursor(0, 52);
    display.print("SOIL ");
    display.print(g_soilPercent);
    display.print("%");
    display.drawRect(64, 52, 60, 8, SSD1306_WHITE);
    uint8_t fill = (uint8_t)(((uint16_t)g_soilPercent * 58U) / 100U);
    if (fill > 0U)
    {
        display.fillRect(65, 53, (int16_t)fill, 6, SSD1306_WHITE);
    }
    display.display();
}

static void oledRenderStatus()
{
    char ip[16];
    snprintf(ip, sizeof(ip), "%s", WiFi.softAPIP().toString().c_str());

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.print("AP ");
    display.print(g_cfg.apSsid);

    display.setCursor(0, 10);
    display.print("IP ");
    display.print(ip);

    display.setCursor(0, 20);
    display.print("UP ");
    display.print((unsigned long)(nowMs() / 1000ULL));
    display.print("s");

    display.setCursor(0, 30);
    display.print("HEAP ");
    display.print((unsigned long)g_freeHeap);

    display.setCursor(0, 40);
    display.print("CLIENTS ");
    display.print((unsigned long)g_wifiClients);

    display.setCursor(0, 50);
    display.print("R1 ");
    display.print(g_relayShtState ? "ON" : "OFF");
    display.print(" R2 ");
    display.print(g_relaySoilState ? "ON" : "OFF");

    display.display();
}

void taskOled()
{
    if (!g_cfg.oledEnabled)
    {
        if (g_oledPowered)
        {
            display.ssd1306_command(SSD1306_DISPLAYOFF);
            g_oledPowered = false;
        }
        return;
    }

    if (!g_oledBegun)
    {
        g_oledBegun = display.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDR);
        if (!g_oledBegun)
        {
            return;
        }
        display.ssd1306_command(SSD1306_SETCONTRAST);
        display.ssd1306_command((uint8_t)g_cfg.oledContrast);
        display.clearDisplay();
        display.display();
        g_oledPowered = true;
    }

    uint64_t now = nowMs();
    if (g_oledNext == 0ULL)
    {
        g_oledNext = now;
    }
    uint32_t interval = (uint32_t)g_cfg.oledIntervalMs;
    if (interval < 200UL)
    {
        interval = 200UL;
    }
    if ((now - g_oledNext) < (uint64_t)interval)
    {
        return;
    }
    g_oledNext = now;

    uint8_t page = g_oledPage;
    if (page == 0 && !g_cfg.oledShowSensors)
    {
        page = 1;
    }
    if (page == 1 && !g_cfg.oledShowStatus)
    {
        page = 0;
    }
    if (page == 0 && !g_cfg.oledShowSensors && !g_cfg.oledShowStatus)
    {
        if (g_oledPowered)
        {
            display.ssd1306_command(SSD1306_DISPLAYOFF);
            g_oledPowered = false;
        }
        return;
    }

    if (!g_oledPowered)
    {
        display.ssd1306_command(SSD1306_DISPLAYON);
        g_oledPowered = true;
    }

    if (page == 0)
    {
        oledRenderSensors();
    }
    else
    {
        oledRenderStatus();
    }
    g_oledPage = (uint8_t)(1U - page);
}

// =====================================================
//  BOOT BUTTON (5 s hold = factory reset)
// =====================================================
void taskBootButton()
{
    bool pressed = (digitalRead(PIN_BOOT_BUTTON) == LOW);
    if (pressed)
    {
        if (g_btnHoldMs == 0ULL)
        {
            g_btnHoldMs = nowMs();
        }
        else if ((nowMs() - g_btnHoldMs) >= 5000ULL)
        {
            g_btnHoldMs = 0ULL;
            g_pendingReset = true;
        }
    }
    else
    {
        g_btnHoldMs = 0ULL;
    }
}

// =====================================================
//  SYSTEM TASK (pending reset, heap watch)
// =====================================================
void taskSystem()
{
    if (g_pendingReset)
    {
        doFactoryReset();
        return;
    }
    if (g_pendingReboot)
    {
        ESP.restart();
        return;
    }

    uint64_t now = nowMs();
    if ((g_nextHeapCheck == 0ULL) || (now >= g_nextHeapCheck))
    {
        g_nextHeapCheck = now + 30000ULL;
        uint32_t freeHeap = (uint32_t)esp_get_free_heap_size();
        uint32_t largest = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
        if (largest > freeHeap)
        {
            largest = freeHeap;
        }
        g_freeHeap = freeHeap;
        if (freeHeap < g_minFreeHeap)
        {
            g_minFreeHeap = freeHeap;
        }
        g_largestBlock = largest;
        if (freeHeap != 0UL)
        {
            g_fragPercent = (uint8_t)(((uint64_t)(freeHeap - largest) * 100ULL) / (uint64_t)freeHeap);
        }
        else
        {
            g_fragPercent = 0;
        }
        g_wifiClients = (uint8_t)WiFi.softAPgetStationNum();
    }
}

// =====================================================
//  SCHEDULER (cooperative, fully non-blocking)
// =====================================================
static Task g_tasks[10];
static uint8_t g_taskCount = 0;

static void taskAdd(TaskFn fn, uint32_t intervalMs)
{
    if (g_taskCount >= 10U)
    {
        return;
    }
    g_tasks[g_taskCount].fn = fn;
    g_tasks[g_taskCount].nextMs = nowMs() + (uint64_t)intervalMs;
    g_tasks[g_taskCount].intervalMs = intervalMs;
    g_taskCount++;
}

static void schedulerRun()
{
    uint64_t now = nowMs();
    for (uint8_t i = 0; i < g_taskCount; i++)
    {
        if (now >= g_tasks[i].nextMs)
        {
            g_tasks[i].nextMs = now + (uint64_t)g_tasks[i].intervalMs;
            g_tasks[i].fn();
        }
    }
}

// =====================================================
//  WI-FI ACCESS POINT + CAPTIVE PORTAL DNS
// =====================================================
static const IPAddress AP_LOCAL_IP(192, 168, 4, 1);
static const IPAddress AP_GATEWAY(192, 168, 4, 1);
static const IPAddress AP_SUBNET(255, 255, 255, 0);

static void startWifiAp()
{
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_LOCAL_IP, AP_GATEWAY, AP_SUBNET);
    WiFi.softAPdisconnect(true);

    const char *ssid = (g_cfg.apSsid[0] != '\0') ? g_cfg.apSsid : DEFAULT_AP_SSID;
    const char *pass = (strlen(g_cfg.apPass) >= 8U) ? g_cfg.apPass : DEFAULT_AP_PASS;
    uint8_t channel = (g_cfg.apChannel >= 1U && g_cfg.apChannel <= 13U) ? g_cfg.apChannel : 1U;
    uint8_t maxClients = (g_cfg.apMaxClients >= 1U && g_cfg.apMaxClients <= 8U) ? g_cfg.apMaxClients : 4U;

    bool ok = WiFi.softAP(ssid, pass, (int)channel, g_cfg.apHidden != 0, (int)maxClients);
    if (!ok)
    {
        WiFi.softAP(DEFAULT_AP_SSID, DEFAULT_AP_PASS, 1, false, 4);
    }
}

static void startDns()
{
    dnsServer.start(53, "*", AP_LOCAL_IP);
}

// =====================================================
//  WEB UI  (single page, offline, responsive)
// =====================================================
static const char index_html[] = R"ESP32UI(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>ESP32 SOIL.SHT Console</title>
<style>
:root{
  --bg:#0a0f1e; --panel:#121a2e; --panel-2:#0d1424; --line:#22304f;
  --text:#e8edf8; --muted:#8fa0c0; --accent:#39d98a; --accent-2:#2f81f7;
  --warn:#ffb454; --danger:#ff5d6c; --radius:14px;
}
*{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%}
body{background:var(--bg);color:var(--text);font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;font-size:15px;line-height:1.5;-webkit-font-smoothing:antialiased}
.app{display:grid;grid-template-columns:250px minmax(0,1fr);min-height:100vh}
.sidebar{background:var(--panel);border-right:1px solid var(--line);position:sticky;top:0;height:100vh;display:flex;flex-direction:column}
.brand{display:flex;align-items:center;gap:12px;padding:18px 18px 14px;border-bottom:1px solid var(--line)}
.brand-mark{width:38px;height:38px;border-radius:11px;background:linear-gradient(135deg,var(--accent-2),var(--accent));display:flex;align-items:center;justify-content:center;color:#04121f;font-weight:800;font-size:14px}
.brand h1{font-size:16px;letter-spacing:.4px}
.brand p{color:var(--muted);font-size:12px}
.nav{padding:12px 10px;display:flex;flex-direction:column;gap:3px;overflow-y:auto;flex:1}
.nav-item{display:flex;align-items:center;gap:11px;padding:10px 12px;border-radius:10px;color:var(--muted);cursor:pointer;font-weight:600;transition:background .15s,color .15s}
.nav-item svg{width:18px;height:18px;flex:0 0 auto}
.nav-item:hover{background:var(--panel-2);color:var(--text)}
.nav-item.active{background:rgba(47,129,247,.18);color:#fff}
.side-foot{padding:14px 18px;border-top:1px solid var(--line);color:var(--muted);font-size:12px;display:flex;flex-direction:column;gap:4px}
.side-foot b{color:var(--text);font-weight:600}
.main{display:flex;flex-direction:column;min-width:0}
.topbar{display:flex;align-items:center;gap:12px;padding:12px 20px;background:var(--panel);border-bottom:1px solid var(--line);position:sticky;top:0;z-index:20}
.menu-btn{display:none;background:var(--panel-2);border:1px solid var(--line);color:var(--text);border-radius:10px;padding:7px 11px;font-size:16px;cursor:pointer}
.page-title{font-size:16px;font-weight:700}
.status{margin-left:auto;display:flex;align-items:center;gap:8px;color:var(--muted);font-size:13px}
.dot{width:9px;height:9px;border-radius:50%;background:var(--accent);box-shadow:0 0 10px rgba(57,217,138,.7)}
.dot.off{background:var(--danger);box-shadow:0 0 10px rgba(255,93,108,.7)}
.content{padding:22px;width:100%;max-width:1180px;margin:0 auto}
.page{display:none}
.page.active{display:block;animation:fade .22s ease}
@keyframes fade{from{opacity:0;transform:translateY(5px)}to{opacity:1;transform:none}}
h2.page-h{font-size:20px;margin-bottom:4px}
p.page-sub{color:var(--muted);font-size:13px;margin-bottom:18px}
.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:12px;margin-bottom:16px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:var(--radius);padding:14px 16px}
.card-label{color:var(--muted);font-size:11px;text-transform:uppercase;letter-spacing:.6px;font-weight:700}
.card-value{font-size:24px;font-weight:700;margin-top:6px;font-variant-numeric:tabular-nums}
.card-sub{color:var(--muted);font-size:12px;margin-top:2px}
.chart-box{background:var(--panel);border:1px solid var(--line);border-radius:var(--radius);padding:14px 16px;margin-bottom:16px}
.chart-head{display:flex;align-items:center;justify-content:space-between;gap:10px;margin-bottom:8px}
.chart-head h3{font-size:14px;font-weight:700}
.legend{display:flex;gap:12px;color:var(--muted);font-size:12px}
.legend .lg{display:flex;align-items:center;gap:6px}
.legend i{width:10px;height:10px;border-radius:3px;display:inline-block}
canvas.chart{width:100%;height:220px;display:block}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:var(--radius);padding:18px;margin-bottom:16px}
.panel h3{font-size:15px;margin-bottom:12px}
.form-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(230px,1fr));gap:14px 18px}
.field{display:flex;flex-direction:column;gap:6px}
.field>span{color:var(--muted);font-size:11px;font-weight:600;text-transform:uppercase;letter-spacing:.5px}
.field>span b{color:var(--text)}
input[type=text],input[type=number],select{width:100%;background:var(--panel-2);border:1px solid var(--line);color:var(--text);border-radius:10px;padding:10px 12px;font-size:14px;font-family:inherit;outline:none}
input:focus,select:focus{border-color:var(--accent-2)}
input[type=range]{width:100%;accent-color:var(--accent-2)}
.switch-row{display:flex;align-items:center;justify-content:space-between;gap:12px;background:var(--panel-2);border:1px solid var(--line);border-radius:10px;padding:10px 12px;font-size:13px}
.switch{position:relative;width:44px;height:24px;appearance:none;-webkit-appearance:none;background:#2a3a5c;border-radius:999px;cursor:pointer;transition:background .15s;flex:0 0 auto;outline:none}
.switch:checked{background:var(--accent)}
.switch::after{content:"";position:absolute;top:3px;left:3px;width:18px;height:18px;border-radius:50%;background:#fff;transition:transform .15s}
.switch:checked::after{transform:translateX(20px)}
.btn{background:var(--panel-2);border:1px solid var(--line);color:var(--text);border-radius:10px;padding:10px 16px;font-size:14px;font-weight:600;cursor:pointer;font-family:inherit;transition:border-color .15s,background .15s}
.btn:hover{border-color:var(--accent-2)}
.btn-primary{background:var(--accent-2);border-color:var(--accent-2);color:#fff}
.btn-danger{background:rgba(255,93,108,.14);border-color:rgba(255,93,108,.5);color:#ffb3bb}
.btn-row{display:flex;flex-wrap:wrap;gap:10px;margin-top:6px}
.form-actions{display:flex;gap:10px;margin-top:16px;flex-wrap:wrap}
.readout{display:flex;gap:18px;flex-wrap:wrap;background:var(--panel-2);border:1px solid var(--line);border-radius:10px;padding:12px 14px;margin-bottom:16px;color:var(--muted);font-size:13px}
.readout b{color:var(--text);font-variant-numeric:tabular-nums}
.kv{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:10px 18px}
.kv div{display:flex;justify-content:space-between;gap:10px;border-bottom:1px dashed var(--line);padding:6px 0;font-size:13px;color:var(--muted)}
.kv b{color:var(--text);font-weight:600;font-variant-numeric:tabular-nums}
.note{color:var(--muted);font-size:12px;margin-top:10px}
.toast{position:fixed;left:50%;bottom:26px;transform:translateX(-50%) translateY(20px);background:var(--panel);border:1px solid var(--line);color:var(--text);padding:12px 18px;border-radius:12px;opacity:0;pointer-events:none;transition:opacity .2s,transform .2s;z-index:50;box-shadow:0 10px 30px rgba(0,0,0,.4);font-size:14px}
.toast.show{opacity:1;transform:translateX(-50%) translateY(0)}
.backdrop{display:none}
@media (max-width:900px){
  .app{grid-template-columns:1fr}
  .sidebar{position:fixed;inset:0 auto 0 0;width:260px;z-index:40;transform:translateX(-100%);transition:transform .22s ease}
  .sidebar.open{transform:none}
  .backdrop{display:block;position:fixed;inset:0;background:rgba(3,6,14,.6);z-index:30;opacity:0;pointer-events:none;transition:opacity .2s}
  .backdrop.show{opacity:1;pointer-events:auto}
  .menu-btn{display:block}
  .content{padding:16px}
  .card-value{font-size:21px}
}
</style>
</head>
<body>
<div class="app">
  <aside class="sidebar" id="sidebar">
    <div class="brand">
      <div class="brand-mark">S·S</div>
      <div><h1>SOIL.SHT</h1><p>ESP32 console</p></div>
    </div>
    <nav class="nav">
      <a class="nav-item active" data-page="dashboard"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="3" y="3" width="7" height="9" rx="1"/><rect x="14" y="3" width="7" height="5" rx="1"/><rect x="14" y="12" width="7" height="9" rx="1"/><rect x="3" y="16" width="7" height="5" rx="1"/></svg><span>Dashboard</span></a>
      <a class="nav-item" data-page="sht"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M14 14.76V3.5a2.5 2.5 0 0 0-5 0v11.26a4.5 4.5 0 1 0 5 0z"/></svg><span>SHT31 Sensor</span></a>
      <a class="nav-item" data-page="soil"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 2.7l5.66 5.66a8 8 0 1 1-11.31 0z"/></svg><span>Soil Moisture</span></a>
      <a class="nav-item" data-page="buzzer"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M18 8a6 6 0 0 0-12 0c0 7-3 9-3 9h18s-3-2-3-9"/><path d="M13.7 21a2 2 0 0 1-3.4 0"/></svg><span>Buzzer</span></a>
      <a class="nav-item" data-page="led"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M9 18h6M10 22h4M12 2a7 7 0 0 0-4 12.7V17h8v-2.3A7 7 0 0 0 12 2z"/></svg><span>LED Anode</span></a>
      <a class="nav-item" data-page="oled"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="2" y="3" width="20" height="14" rx="2"/><path d="M8 21h8M12 17v4"/></svg><span>OLED Display</span></a>
      <a class="nav-item" data-page="wifi"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M5 12.55a11 11 0 0 1 14.08 0M1.42 9a16 16 0 0 1 21.16 0M8.53 16.11a6 6 0 0 1 6.95 0M12 20h.01"/></svg><span>Wi-Fi AP</span></a>
      <a class="nav-item" data-page="system"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="4" y="4" width="16" height="16" rx="2"/><path d="M9 9h6v6H9zM9 1v3M15 1v3M9 20v3M15 20v3M1 9h3M1 15h3M20 9h3M20 15h3"/></svg><span>System</span></a>
    </nav>
    <div class="side-foot">
      <div>Firmware <b id="fwVer">--</b></div>
      <div>Uptime <b id="sideUptime">--</b></div>
    </div>
  </aside>
  <div class="backdrop" id="backdrop"></div>
  <div class="main">
    <header class="topbar">
      <button class="menu-btn" id="menuBtn" aria-label="Menu">&#9776;</button>
      <div class="page-title" id="pageTitle">Dashboard</div>
      <div class="status"><span class="dot" id="connDot"></span><span id="statusText">connecting…</span></div>
    </header>
    <main class="content">

      <section class="page active" id="page-dashboard">
        <h2 class="page-h">Dashboard</h2>
        <p class="page-sub">Live sensor status and history</p>
        <div class="cards">
          <div class="card"><div class="card-label">Temperature</div><div class="card-value" id="vTemp">--</div><div class="card-sub">SHT31</div></div>
          <div class="card"><div class="card-label">Humidity</div><div class="card-value" id="vHum">--</div><div class="card-sub">%RH</div></div>
          <div class="card"><div class="card-label">Soil moisture</div><div class="card-value" id="vSoil">--</div><div class="card-sub" id="vSoilSub">raw --</div></div>
          <div class="card"><div class="card-label">Relay SHT</div><div class="card-value" id="vRelaySht">--</div><div class="card-sub" id="vRelayShtSub">--</div></div>
          <div class="card"><div class="card-label">Relay Soil</div><div class="card-value" id="vRelaySoil">--</div><div class="card-sub" id="vRelaySoilSub">--</div></div>
          <div class="card"><div class="card-label">Uptime</div><div class="card-value" id="vUptime">--</div><div class="card-sub" id="vBoots">--</div></div>
          <div class="card"><div class="card-label">Free heap</div><div class="card-value" id="vHeap">--</div><div class="card-sub" id="vHeapSub">--</div></div>
          <div class="card"><div class="card-label">Wi-Fi clients</div><div class="card-value" id="vClients">--</div><div class="card-sub" id="vSsid">--</div></div>
        </div>
        <div class="chart-box">
          <div class="chart-head"><h3>Temperature &amp; Humidity</h3>
            <div class="legend"><span class="lg"><i style="background:#ffb454"></i>°C</span><span class="lg"><i style="background:#2f81f7"></i>%RH</span></div>
          </div>
          <canvas class="chart" id="chartTH"></canvas>
        </div>
        <div class="chart-box">
          <div class="chart-head"><h3>Soil moisture</h3>
            <div class="legend"><span class="lg"><i style="background:#39d98a"></i>%</span></div>
          </div>
          <canvas class="chart" id="chartSoil"></canvas>
        </div>
        <div class="panel">
          <h3>Quick relay control</h3>
          <div class="btn-row">
            <button class="btn" type="button" data-act="relayShtOn">SHT relay ON</button>
            <button class="btn" type="button" data-act="relayShtAuto">SHT auto</button>
            <button class="btn" type="button" data-act="relayShtOff">SHT relay OFF</button>
            <button class="btn" type="button" data-act="relaySoilOn">Soil relay ON</button>
            <button class="btn" type="button" data-act="relaySoilAuto">Soil auto</button>
            <button class="btn" type="button" data-act="relaySoilOff">Soil relay OFF</button>
          </div>
        </div>
      </section>

      <section class="page" id="page-sht">
        <h2 class="page-h">SHT31 Temperature &amp; Humidity</h2>
        <p class="page-sub">I²C bus on GPIO 21 (SDA) / GPIO 22 (SCL) · relay on GPIO 25</p>
        <div class="readout">
          <span>Temperature <b id="shtNowT">--</b> °C</span>
          <span>Humidity <b id="shtNowH">--</b> %RH</span>
          <span>Status <b id="shtNowS">--</b></span>
        </div>
        <form id="form-sht" class="panel">
          <h3>Sensor</h3>
          <div class="form-grid">
            <div class="switch-row"><span>Enable sensor</span><input class="switch" type="checkbox" name="shtEnabled" value="1"></div>
            <label class="field"><span>Read interval (ms)</span><input type="number" name="shtIntervalMs" min="250" max="60000" step="50"></label>
          </div>
          <h3 style="margin-top:18px">Relay control · GPIO 25</h3>
          <div class="form-grid">
            <label class="field"><span>Mode</span>
              <select name="shtRelayMode"><option value="1">Automatic (thresholds)</option><option value="0">Manual</option></select>
            </label>
            <div class="switch-row"><span>Manual ON</span><input class="switch" type="checkbox" name="shtRelayManual" value="1"></div>
            <div class="switch-row"><span>Active HIGH</span><input class="switch" type="checkbox" name="shtRelayActiveHigh" value="1"></div>
            <label class="field"><span>ON when temperature ≥ (°C)</span><input type="number" name="shtTempOn" step="0.1" min="-40" max="125"></label>
            <label class="field"><span>OFF when temperature ≤ (°C)</span><input type="number" name="shtTempOff" step="0.1" min="-40" max="125"></label>
            <label class="field"><span>ON when humidity ≥ (%RH)</span><input type="number" name="shtHumOn" step="0.5" min="0" max="100"></label>
            <label class="field"><span>OFF when humidity ≤ (%RH)</span><input type="number" name="shtHumOff" step="0.5" min="0" max="100"></label>
          </div>
          <div class="form-actions">
            <button class="btn btn-primary" type="submit">Save</button>
            <button class="btn" type="button" data-act="readNow">Read now</button>
          </div>
        </form>
      </section>

      <section class="page" id="page-soil">
        <h2 class="page-h">Soil Moisture</h2>
        <p class="page-sub">Analog input GPIO 34 · relay GPIO 26</p>
        <div class="readout">
          <span>Raw <b id="soilNowRaw">--</b></span>
          <span>Moisture <b id="soilNowPct">--</b></span>
        </div>
        <form id="form-soil" class="panel">
          <h3>Sensor</h3>
          <div class="form-grid">
            <div class="switch-row"><span>Enable sensor</span><input class="switch" type="checkbox" name="soilEnabled" value="1"></div>
            <label class="field"><span>Read interval (ms)</span><input type="number" name="soilIntervalMs" min="250" max="60000" step="50"></label>
            <label class="field"><span>Calibration raw DRY (air)</span><input type="number" name="soilRawDry" min="0" max="4095"></label>
            <label class="field"><span>Calibration raw WET (water)</span><input type="number" name="soilRawWet" min="0" max="4095"></label>
          </div>
          <h3 style="margin-top:18px">Relay control · GPIO 26</h3>
          <div class="form-grid">
            <label class="field"><span>Mode</span>
              <select name="soilRelayMode"><option value="1">Automatic (thresholds)</option><option value="0">Manual</option></select>
            </label>
            <div class="switch-row"><span>Manual ON</span><input class="switch" type="checkbox" name="soilRelayManual" value="1"></div>
            <div class="switch-row"><span>Active HIGH</span><input class="switch" type="checkbox" name="soilRelayActiveHigh" value="1"></div>
            <label class="field"><span>ON when moisture ≤ (%)</span><input type="number" name="soilDryPct" min="0" max="100"></label>
            <label class="field"><span>OFF when moisture ≥ (%)</span><input type="number" name="soilWetPct" min="0" max="100"></label>
          </div>
          <div class="form-actions"><button class="btn btn-primary" type="submit">Save</button></div>
          <p class="note">Place the probe in air to read DRY, then in water to read WET, and enter both raw values.</p>
        </form>
      </section>

      <section class="page" id="page-buzzer">
        <h2 class="page-h">Buzzer</h2>
        <p class="page-sub">Digital output · GPIO 27</p>
        <form id="form-buzzer" class="panel">
          <div class="form-grid">
            <div class="switch-row"><span>Enable buzzer</span><input class="switch" type="checkbox" name="buzzerEnabled" value="1"></div>
            <div class="switch-row"><span>Active HIGH</span><input class="switch" type="checkbox" name="buzzerActiveHigh" value="1"></div>
            <label class="field"><span>Mode</span>
              <select name="buzzerMode"><option value="0">Off</option><option value="1">Steady ON</option><option value="2">Beep pattern</option></select>
            </label>
            <label class="field"><span>Beep ON (ms)</span><input type="number" name="buzzerBeepOnMs" min="20" max="5000"></label>
            <label class="field"><span>Beep OFF (ms)</span><input type="number" name="buzzerBeepOffMs" min="20" max="5000"></label>
          </div>
          <div class="form-actions">
            <button class="btn btn-primary" type="submit">Save</button>
            <button class="btn" type="button" data-act="buzzerTest">Test 0.6 s</button>
          </div>
        </form>
      </section>

      <section class="page" id="page-led">
        <h2 class="page-h">LED Anode</h2>
        <p class="page-sub">Digital output with PWM dimming · GPIO 33</p>
        <form id="form-led" class="panel">
          <div class="form-grid">
            <div class="switch-row"><span>Enable LED</span><input class="switch" type="checkbox" name="ledEnabled" value="1"></div>
            <label class="field"><span>Mode</span>
              <select name="ledMode"><option value="0">Off</option><option value="1">On</option><option value="2">PWM brightness</option><option value="3">Blink</option></select>
            </label>
            <label class="field"><span>Brightness <b id="ledBrightVal">--</b></span><input type="range" name="ledBrightness" min="0" max="255" step="1"></label>
            <div class="switch-row"><span>Invert (active LOW)</span><input class="switch" type="checkbox" name="ledInvert" value="1"></div>
          </div>
          <div class="form-actions"><button class="btn btn-primary" type="submit">Save</button></div>
        </form>
      </section>

      <section class="page" id="page-oled">
        <h2 class="page-h">OLED Display · SSD1306 128×64</h2>
        <p class="page-sub">I²C bus on GPIO 21 (SDA) / GPIO 22 (SCL) · address 0x3C</p>
        <form id="form-oled" class="panel">
          <div class="form-grid">
            <div class="switch-row"><span>Enable display</span><input class="switch" type="checkbox" name="oledEnabled" value="1"></div>
            <label class="field"><span>Contrast <b id="oledContrastVal">--</b></span><input type="range" name="oledContrast" min="0" max="255" step="1"></label>
            <label class="field"><span>Page interval (ms)</span><input type="number" name="oledIntervalMs" min="200" max="10000" step="100"></label>
            <div class="switch-row"><span>Show sensor page</span><input class="switch" type="checkbox" name="oledShowSensors" value="1"></div>
            <div class="switch-row"><span>Show status page</span><input class="switch" type="checkbox" name="oledShowStatus" value="1"></div>
          </div>
          <div class="form-actions">
            <button class="btn btn-primary" type="submit">Save</button>
            <button class="btn" type="button" data-act="oledNext">Next page now</button>
          </div>
        </form>
      </section>

      <section class="page" id="page-wifi">
        <h2 class="page-h">Wi-Fi Access Point</h2>
        <p class="page-sub">Captive portal · DNS wildcard → 192.168.4.1</p>
        <div class="readout">
          <span>IP <b id="wifiIp">--</b></span>
          <span>SSID <b id="wifiSsid">--</b></span>
          <span>Clients <b id="wifiClients">--</b></span>
        </div>
        <form id="form-wifi" class="panel">
          <div class="form-grid">
            <label class="field"><span>SSID</span><input type="text" name="apSsid" maxlength="32"></label>
            <label class="field"><span>Password (min 8 chars)</span><input type="text" name="apPass" maxlength="16"></label>
            <label class="field"><span>Channel</span><input type="number" name="apChannel" min="1" max="13"></label>
            <label class="field"><span>Max clients</span><input type="number" name="apMaxClients" min="1" max="8"></label>
            <div class="switch-row"><span>Hidden SSID</span><input class="switch" type="checkbox" name="apHidden" value="1"></div>
          </div>
          <div class="form-actions"><button class="btn btn-primary" type="submit">Apply &amp; restart AP</button></div>
          <p class="note">Changing SSID or password restarts the access point. The captive portal keeps serving this console on 192.168.4.1.</p>
        </form>
      </section>

      <section class="page" id="page-system">
        <h2 class="page-h">System</h2>
        <p class="page-sub">Device information and maintenance</p>
        <div class="panel">
          <h3>Device</h3>
          <div class="kv">
            <div>Firmware <b id="sysFw">--</b></div>
            <div>Chip <b id="sysChip">--</b></div>
            <div>Cores <b id="sysCores">--</b></div>
            <div>Flash <b id="sysFlash">--</b></div>
            <div>SDK <b id="sysSdk">--</b></div>
            <div>Boot count <b id="sysBoots">--</b></div>
            <div>Uptime <b id="sysUptime">--</b></div>
            <div>Samples <b id="sysSamples">--</b></div>
            <div>Free heap <b id="sysHeap">--</b></div>
            <div>Min free heap <b id="sysMinHeap">--</b></div>
            <div>Largest block <b id="sysLargest">--</b></div>
            <div>Fragmentation <b id="sysFrag">--</b></div>
            <div>NVS writes <b id="sysNvs">--</b></div>
            <div>NVS slot <b id="sysSlot">--</b></div>
          </div>
        </div>
        <div class="panel">
          <h3>Maintenance</h3>
          <div class="btn-row">
            <button class="btn" type="button" data-act="reboot">Reboot</button>
            <button class="btn btn-danger" type="button" data-act="factoryReset">Factory reset</button>
          </div>
          <p class="note">Factory reset erases all stored settings. You can also hold the BOOT button (GPIO 0) for 5 seconds at any time.</p>
        </div>
      </section>

    </main>
  </div>
</div>
<div class="toast" id="toast"></div>
<script>
(function () {
  'use strict';
  var titles = {
    dashboard: 'Dashboard', sht: 'SHT31 Sensor', soil: 'Soil Moisture',
    buzzer: 'Buzzer', led: 'LED Anode', oled: 'OLED Display',
    wifi: 'Wi-Fi Access Point', system: 'System'
  };
  var last = null;
  var current = 'dashboard';

  function el(id) { return document.getElementById(id); }
  function setText(id, v) {
    var n = el(id);
    if (n && v !== undefined && v !== null) n.textContent = v;
  }
  function fmtUptime(sec) {
    sec = sec || 0;
    var d = Math.floor(sec / 86400);
    var h = Math.floor((sec % 86400) / 3600);
    var m = Math.floor((sec % 3600) / 60);
    var s = sec % 60;
    var out = '';
    if (d) out += d + 'd ';
    if (h || d) out += h + 'h ';
    out += m + 'm ' + s + 's';
    return out;
  }
  function num(v, digits, unit) {
    if (v === null || v === undefined || !isFinite(v)) return '--';
    return v.toFixed(digits) + unit;
  }

  function show(name) {
    if (!titles[name]) return;
    current = name;
    var pages = document.querySelectorAll('.page');
    for (var i = 0; i < pages.length; i++) {
      pages[i].classList.toggle('active', pages[i].id === 'page-' + name);
    }
    var navs = document.querySelectorAll('.nav-item');
    for (var j = 0; j < navs.length; j++) {
      navs[j].classList.toggle('active', navs[j].getAttribute('data-page') === name);
    }
    el('pageTitle').textContent = titles[name];
    var sb = el('sidebar'); if (sb) sb.classList.remove('open');
    var bd = el('backdrop'); if (bd) bd.classList.remove('show');
    if (name === 'dashboard') drawCharts();
  }

  function update(d) {
    var dot = el('connDot');
    if (dot) dot.classList.remove('off');
    setText('statusText', 'live');
    setText('vTemp', num(d.temp, 1, ' °C'));
    setText('vHum', num(d.hum, 1, ' %'));
    setText('vSoil', num(d.soil, 0, ' %'));
    setText('vSoilSub', 'raw ' + d.soilRaw);
    setText('vRelaySht', d.relaySht ? 'ON' : 'OFF');
    setText('vRelayShtSub', d.relayShtMode ? 'auto' : 'manual');
    setText('vRelaySoil', d.relaySoil ? 'ON' : 'OFF');
    setText('vRelaySoilSub', d.relaySoilMode ? 'auto' : 'manual');
    setText('vUptime', fmtUptime(d.uptime));
    setText('vBoots', d.boots + ' boots');
    setText('vHeap', d.heap + ' B');
    setText('vHeapSub', 'largest ' + d.largest + ' B');
    setText('vClients', d.clients);
    setText('vSsid', d.ssid);
    setText('sideUptime', fmtUptime(d.uptime));
    setText('fwVer', d.fw);
    setText('shtNowT', num(d.temp, 1, ''));
    setText('shtNowH', num(d.hum, 1, ''));
    setText('shtNowS', d.sht ? 'online' : 'offline');
    setText('soilNowRaw', d.soilRaw);
    setText('soilNowPct', num(d.soil, 0, ' %'));
    setText('wifiIp', d.ip);
    setText('wifiSsid', d.ssid);
    setText('wifiClients', d.clients);
    setText('sysFw', d.fw);
    setText('sysChip', d.chip);
    setText('sysCores', d.cores);
    setText('sysFlash', d.flash + ' MB');
    setText('sysSdk', d.sdk);
    setText('sysBoots', d.boots);
    setText('sysUptime', fmtUptime(d.uptime));
    setText('sysSamples', d.samples);
    setText('sysHeap', d.heap + ' B');
    setText('sysMinHeap', d.minHeap + ' B');
    setText('sysLargest', d.largest + ' B');
    setText('sysFrag', d.frag + ' %');
    setText('sysNvs', d.nvsWrites);
    setText('sysSlot', d.slot);
    setText('ledBrightVal', d.cfg ? d.cfg.ledBrightness : '--');
    setText('oledContrastVal', d.cfg ? d.cfg.oledContrast : '--');
    syncForms(d.cfg);
    if (current === 'dashboard') drawCharts();
  }

  function syncForms(cfg) {
    if (!cfg) return;
    var forms = document.querySelectorAll('form');
    for (var i = 0; i < forms.length; i++) {
      var els = forms[i].elements;
      for (var k = 0; k < els.length; k++) {
        var e = els[k];
        if (!e.name || !(e.name in cfg)) continue;
        if (document.activeElement === e) continue;
        var v = cfg[e.name];
        if (e.type === 'checkbox') {
          e.checked = (v === 1 || v === '1' || v === true);
        } else {
          e.value = v;
        }
      }
    }
  }

  function drawChart(canvasId, series) {
    var canvas = el(canvasId);
    if (!canvas) return;
    var dpr = window.devicePixelRatio || 1;
    var w = canvas.clientWidth;
    var h = canvas.clientHeight;
    if (w < 20 || h < 20) return;
    canvas.width = Math.floor(w * dpr);
    canvas.height = Math.floor(h * dpr);
    var ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    var padL = 46, padR = 14, padT = 12, padB = 24;
    var plotW = w - padL - padR;
    var plotH = h - padT - padB;
    var min = Infinity, max = -Infinity, n = 0;
    var si, k, v;
    for (si = 0; si < series.length; si++) {
      for (k = 0; k < series[si].data.length; k++) {
        v = series[si].data[k];
        if (v === null || v === undefined || !isFinite(v)) continue;
        if (v < min) min = v;
        if (v > max) max = v;
        n++;
      }
    }
    if (!isFinite(min)) { min = 0; max = 1; }
    if (min === max) { min -= 1; max += 1; }
    var pad = (max - min) * 0.12;
    min -= pad; max += pad;
    ctx.lineWidth = 1;
    ctx.strokeStyle = '#22304f';
    ctx.fillStyle = '#8fa0c0';
    ctx.font = '11px -apple-system, Segoe UI, Roboto, sans-serif';
    for (var i = 0; i <= 4; i++) {
      var y = padT + (plotH * i) / 4;
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w - padR, y); ctx.stroke();
      var val = max - ((max - min) * i) / 4;
      ctx.fillText(val.toFixed(1), 6, y + 4);
    }
    var count = 0;
    for (si = 0; si < series.length; si++) {
      if (series[si].data.length > count) count = series[si].data.length;
    }
    for (si = 0; si < series.length; si++) {
      var s = series[si];
      ctx.strokeStyle = s.color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      var started = false, lastX = 0, lastY = 0;
      for (k = 0; k < s.data.length; k++) {
        v = s.data[k];
        if (v === null || v === undefined || !isFinite(v)) continue;
        var x = padL + (s.data.length <= 1 ? plotW / 2 : (plotW * k) / (s.data.length - 1));
        var yy = padT + plotH * (1 - (v - min) / (max - min));
        if (!started) { ctx.moveTo(x, yy); started = true; }
        else { ctx.lineTo(x, yy); }
        lastX = x; lastY = yy;
      }
      ctx.stroke();
      if (started) {
        ctx.fillStyle = s.color;
        ctx.beginPath();
        ctx.arc(lastX, lastY, 3, 0, Math.PI * 2);
        ctx.fill();
      }
    }
    ctx.fillStyle = '#8fa0c0';
    ctx.font = '10px -apple-system, Segoe UI, Roboto, sans-serif';
    if (count > 1) {
      ctx.fillText('-' + (count - 1) + 's', padL, h - 6);
      var t = 'now';
      var tw = ctx.measureText(t).width;
      ctx.fillText(t, w - padR - tw, h - 6);
    }
  }

  function drawCharts() {
    if (!last || !last.hist) return;
    var t = last.hist.temp.map(function (v) { return (v === null || v === undefined) ? null : v / 10; });
    var h = last.hist.hum.map(function (v) { return (v === null || v === undefined) ? null : v / 10; });
    var s = last.hist.soil.map(function (v) { return (v === null || v === undefined) ? null : v; });
    drawChart('chartTH', [{ data: t, color: '#ffb454' }, { data: h, color: '#2f81f7' }]);
    drawChart('chartSoil', [{ data: s, color: '#39d98a' }]);
  }

  function toast(msg) {
    var t = el('toast');
    if (!t) return;
    t.textContent = msg;
    t.classList.add('show');
    setTimeout(function () { t.classList.remove('show'); }, 2200);
  }

  function postConfig(form) {
    var fd = new FormData(form);
    fetch('/api/config', { method: 'POST', body: new URLSearchParams(fd) })
      .then(function (r) { return r.json(); })
      .then(function (d) { toast(d.ok ? 'Settings saved' : 'Save rejected'); })
      .catch(function () { toast('Save failed'); });
  }

  function postAction(act) {
    if (act === 'factoryReset' && !window.confirm('Erase all settings and reboot?')) return;
    fetch('/api/action', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: 'act=' + encodeURIComponent(act)
    })
      .then(function (r) { return r.json(); })
      .then(function (d) { toast(d.ok ? 'Done' : 'Failed'); })
      .catch(function () { toast('Failed'); });
  }

  function tick() {
    fetch('/api/data', { cache: 'no-store' })
      .then(function (r) { return r.json(); })
      .then(function (d) { last = d; update(d); })
      .catch(function () {
        var dot = el('connDot');
        if (dot) dot.classList.add('off');
        setText('statusText', 'offline');
      });
  }

  function init() {
    var navs = document.querySelectorAll('.nav-item');
    for (var i = 0; i < navs.length; i++) {
      (function (n) {
        n.addEventListener('click', function () { show(n.getAttribute('data-page')); });
      })(navs[i]);
    }
    var forms = document.querySelectorAll('form');
    for (var j = 0; j < forms.length; j++) {
      (function (f) {
        f.addEventListener('submit', function (e) { e.preventDefault(); postConfig(f); });
      })(forms[j]);
    }
    var acts = document.querySelectorAll('[data-act]');
    for (var k = 0; k < acts.length; k++) {
      (function (b) {
        b.addEventListener('click', function () { postAction(b.getAttribute('data-act')); });
      })(acts[k]);
    }
    var mb = el('menuBtn');
    if (mb) mb.addEventListener('click', function () {
      var sb = el('sidebar'); if (sb) sb.classList.toggle('open');
      var bd = el('backdrop'); if (bd) bd.classList.toggle('show');
    });
    var bd = el('backdrop');
    if (bd) bd.addEventListener('click', function () {
      var sb = el('sidebar'); if (sb) sb.classList.remove('open');
      bd.classList.remove('show');
    });
    window.addEventListener('resize', function () { if (current === 'dashboard') drawCharts(); });
    setInterval(tick, 1000);
    tick();
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
  else init();
})();
</script>
</body>
</html>
)ESP32UI";

// =====================================================
//  JSON EMISSION
// =====================================================
static JsonBuf g_json;

static void jsonAppendString(JsonBuf &j, const char *s)
{
    j.append("\"");
    for (const char *p = s; *p != '\0'; p++)
    {
        unsigned char c = (unsigned char)*p;
        if (c == '"' || c == '\\')
        {
            char esc[3];
            esc[0] = '\\';
            esc[1] = (char)c;
            esc[2] = '\0';
            j.append("%s", esc);
        }
        else if (c < 0x20)
        {
            j.append("\\u%04x", (unsigned)c);
        }
        else
        {
            j.append("%c", (char)c);
        }
    }
    j.append("\"");
}

static void appendConfigJson(JsonBuf &j)
{
    j.append("\"cfg\":{");
    j.append("\"apSsid\":");
    jsonAppendString(j, g_cfg.apSsid);
    j.append(",");
    j.append("\"apPass\":");
    jsonAppendString(j, g_cfg.apPass);
    j.append(",");
    j.append("\"apChannel\":%u,", (unsigned)g_cfg.apChannel);
    j.append("\"apMaxClients\":%u,", (unsigned)g_cfg.apMaxClients);
    j.append("\"apHidden\":%u,", (unsigned)g_cfg.apHidden);
    j.append("\"shtEnabled\":%u,", (unsigned)g_cfg.shtEnabled);
    j.append("\"shtIntervalMs\":%lu,", (unsigned long)g_cfg.shtIntervalMs);
    j.append("\"shtRelayMode\":%u,", (unsigned)g_cfg.shtRelayMode);
    j.append("\"shtRelayManual\":%u,", (unsigned)g_cfg.shtRelayManual);
    j.append("\"shtRelayActiveHigh\":%u,", (unsigned)g_cfg.shtRelayActiveHigh);
    j.append("\"shtTempOn\":%.1f,", g_cfg.shtTempOn);
    j.append("\"shtTempOff\":%.1f,", g_cfg.shtTempOff);
    j.append("\"shtHumOn\":%.1f,", g_cfg.shtHumOn);
    j.append("\"shtHumOff\":%.1f,", g_cfg.shtHumOff);
    j.append("\"soilEnabled\":%u,", (unsigned)g_cfg.soilEnabled);
    j.append("\"soilIntervalMs\":%lu,", (unsigned long)g_cfg.soilIntervalMs);
    j.append("\"soilRelayMode\":%u,", (unsigned)g_cfg.soilRelayMode);
    j.append("\"soilRelayManual\":%u,", (unsigned)g_cfg.soilRelayManual);
    j.append("\"soilRelayActiveHigh\":%u,", (unsigned)g_cfg.soilRelayActiveHigh);
    j.append("\"soilDryPct\":%d,", (int)g_cfg.soilDryPct);
    j.append("\"soilWetPct\":%d,", (int)g_cfg.soilWetPct);
    j.append("\"soilRawDry\":%u,", (unsigned)g_cfg.soilRawDry);
    j.append("\"soilRawWet\":%u,", (unsigned)g_cfg.soilRawWet);
    j.append("\"buzzerEnabled\":%u,", (unsigned)g_cfg.buzzerEnabled);
    j.append("\"buzzerActiveHigh\":%u,", (unsigned)g_cfg.buzzerActiveHigh);
    j.append("\"buzzerMode\":%u,", (unsigned)g_cfg.buzzerMode);
    j.append("\"buzzerBeepOnMs\":%u,", (unsigned)g_cfg.buzzerBeepOnMs);
    j.append("\"buzzerBeepOffMs\":%u,", (unsigned)g_cfg.buzzerBeepOffMs);
    j.append("\"ledEnabled\":%u,", (unsigned)g_cfg.ledEnabled);
    j.append("\"ledMode\":%u,", (unsigned)g_cfg.ledMode);
    j.append("\"ledBrightness\":%u,", (unsigned)g_cfg.ledBrightness);
    j.append("\"ledInvert\":%u,", (unsigned)g_cfg.ledInvert);
    j.append("\"oledEnabled\":%u,", (unsigned)g_cfg.oledEnabled);
    j.append("\"oledContrast\":%u,", (unsigned)g_cfg.oledContrast);
    j.append("\"oledIntervalMs\":%lu,", (unsigned long)g_cfg.oledIntervalMs);
    j.append("\"oledShowSensors\":%u,", (unsigned)g_cfg.oledShowSensors);
    j.append("\"oledShowStatus\":%u", (unsigned)g_cfg.oledShowStatus);
    j.append("}");
}

// =====================================================
//  WEB HANDLERS
// =====================================================
static void sendJson(const char *payload)
{
    server.send_P(200, PSTR("application/json; charset=utf-8"), (PGM_P)payload, strlen(payload));
}

static void handleRoot()
{
    server.send_P(200, PSTR("text/html; charset=utf-8"), (PGM_P)index_html, sizeof(index_html) - 1U);
}

static void handleApiData()
{
    g_json.init();
    g_json.append("{\"ok\":1,");
    g_json.append("\"fw\":\"%s\",", FIRMWARE_VERSION);
    g_json.append("\"chip\":\"%s\",", ESP.getChipModel());
    g_json.append("\"cores\":%u,", (unsigned)ESP.getChipCores());
    g_json.append("\"flash\":%lu,", (unsigned long)(ESP.getFlashChipSize() / 1024UL / 1024UL));
    g_json.append("\"sdk\":\"%s\",", ESP.getSdkVersion());
    g_json.append("\"boots\":%lu,", (unsigned long)g_bootCount);
    g_json.append("\"uptime\":%llu,", (unsigned long long)(nowMs() / 1000ULL));
    g_json.append("\"samples\":%llu,", (unsigned long long)g_sampleCount);
    g_json.append("\"heap\":%lu,", (unsigned long)g_freeHeap);
    g_json.append("\"minHeap\":%lu,", (unsigned long)g_minFreeHeap);
    g_json.append("\"largest\":%lu,", (unsigned long)g_largestBlock);
    g_json.append("\"frag\":%u,", (unsigned)g_fragPercent);
    g_json.append("\"clients\":%u,", (unsigned)g_wifiClients);
    g_json.append("\"nvsWrites\":%lu,", (unsigned long)g_configWriteCount);
    g_json.append("\"slot\":\"%s\",", g_activeSlot ? "B" : "A");

    char ip[16];
    snprintf(ip, sizeof(ip), "%s", WiFi.softAPIP().toString().c_str());
    g_json.append("\"ip\":\"%s\",", ip);
    g_json.append("\"ssid\":");
    jsonAppendString(g_json, g_cfg.apSsid);
    g_json.append(",");

    g_json.append("\"temp\":");
    if (g_shtOnline)
    {
        g_json.append("%.1f,", g_temperature);
    }
    else
    {
        g_json.append("null,");
    }
    g_json.append("\"hum\":");
    if (g_shtOnline)
    {
        g_json.append("%.1f,", g_humidity);
    }
    else
    {
        g_json.append("null,");
    }
    g_json.append("\"soil\":%u,", (unsigned)g_soilPercent);
    g_json.append("\"soilRaw\":%u,", (unsigned)g_soilRaw);
    g_json.append("\"sht\":%u,", g_shtOnline ? 1U : 0U);
    g_json.append("\"shtErr\":%lu,", (unsigned long)g_shtErrorCount);
    g_json.append("\"relaySht\":%u,", g_relayShtState ? 1U : 0U);
    g_json.append("\"relaySoil\":%u,", g_relaySoilState ? 1U : 0U);
    g_json.append("\"relayShtMode\":%u,", (unsigned)g_cfg.shtRelayMode);
    g_json.append("\"relaySoilMode\":%u,", (unsigned)g_cfg.soilRelayMode);
    g_json.append("\"buzzer\":%u,", g_buzzerState ? 1U : 0U);
    g_json.append("\"led\":%u", (unsigned)g_ledDutyApplied);

    g_json.append(",");
    appendConfigJson(g_json);

    g_json.append(",\"hist\":{\"temp\":[");
    for (uint16_t i = 0; i < g_histCount; i++)
    {
        uint16_t idx = (uint16_t)((g_histHead + HISTORY_MAX - g_histCount + i) % HISTORY_MAX);
        if (i != 0U)
        {
            g_json.append(",");
        }
        if (g_histTemp[idx] == HIST_INVALID)
        {
            g_json.append("null");
        }
        else
        {
            g_json.append("%d", (int)g_histTemp[idx]);
        }
    }
    g_json.append("],\"hum\":[");
    for (uint16_t i = 0; i < g_histCount; i++)
    {
        uint16_t idx = (uint16_t)((g_histHead + HISTORY_MAX - g_histCount + i) % HISTORY_MAX);
        if (i != 0U)
        {
            g_json.append(",");
        }
        if (g_histHum[idx] == HIST_INVALID)
        {
            g_json.append("null");
        }
        else
        {
            g_json.append("%d", (int)g_histHum[idx]);
        }
    }
    g_json.append("],\"soil\":[");
    for (uint16_t i = 0; i < g_histCount; i++)
    {
        uint16_t idx = (uint16_t)((g_histHead + HISTORY_MAX - g_histCount + i) % HISTORY_MAX);
        if (i != 0U)
        {
            g_json.append(",");
        }
        if (g_histSoil[idx] == HIST_INVALID)
        {
            g_json.append("null");
        }
        else
        {
            g_json.append("%d", (int)g_histSoil[idx]);
        }
    }
    g_json.append("]}");
    g_json.append("}");

    if (g_json.overflow)
    {
        sendJson("{\"ok\":0,\"error\":\"payload\"}");
    }
    else
    {
        sendJson(g_json.buf);
    }
}

static long argToInt(const char *name, long cur, long lo, long hi)
{
    if (!server.hasArg(name))
    {
        return cur;
    }
    long v = strtol(server.arg(name).c_str(), NULL, 10);
    if (v < lo)
    {
        v = lo;
    }
    if (v > hi)
    {
        v = hi;
    }
    return v;
}

static float argToFloat(const char *name, float cur, float lo, float hi)
{
    if (!server.hasArg(name))
    {
        return cur;
    }
    float v = strtof(server.arg(name).c_str(), NULL);
    if (isnan(v))
    {
        return cur;
    }
    if (v < lo)
    {
        v = lo;
    }
    if (v > hi)
    {
        v = hi;
    }
    return v;
}

static bool argToBool(const char *name)
{
    if (!server.hasArg(name))
    {
        return false;
    }
    String v = server.arg(name);
    return (v == "1") || (v == "on") || (v == "true");
}

void handleApiConfig()
{
    bool wifiChanged = false;
    bool any = (server.args() > 0);

    if (server.hasArg("apSsid"))
    {
        sanitizeCopy(g_cfg.apSsid, sizeof(g_cfg.apSsid), server.arg("apSsid").c_str());
        if (g_cfg.apSsid[0] == '\0')
        {
            copyString(g_cfg.apSsid, sizeof(g_cfg.apSsid), DEFAULT_AP_SSID);
        }
        wifiChanged = true;
    }
    if (server.hasArg("apPass"))
    {
        char tmp[17];
        sanitizeCopy(tmp, sizeof(tmp), server.arg("apPass").c_str());
        if (strlen(tmp) >= 8U)
        {
            copyString(g_cfg.apPass, sizeof(g_cfg.apPass), tmp);
        }
        wifiChanged = true;
    }
    if (server.hasArg("apChannel"))
    {
        g_cfg.apChannel = (uint8_t)argToInt("apChannel", (long)g_cfg.apChannel, 1, 13);
        wifiChanged = true;
    }
    if (server.hasArg("apMaxClients"))
    {
        g_cfg.apMaxClients = (uint8_t)argToInt("apMaxClients", (long)g_cfg.apMaxClients, 1, 8);
        wifiChanged = true;
    }
    if (server.hasArg("apHidden"))
    {
        g_cfg.apHidden = argToBool("apHidden") ? 1U : 0U;
        wifiChanged = true;
    }

    if (server.hasArg("shtEnabled"))
    {
        g_cfg.shtEnabled = argToBool("shtEnabled") ? 1U : 0U;
    }
    g_cfg.shtIntervalMs = (uint32_t)argToInt("shtIntervalMs", (long)g_cfg.shtIntervalMs, 250, 60000);
    if (server.hasArg("shtRelayMode"))
    {
        g_cfg.shtRelayMode = (uint8_t)argToInt("shtRelayMode", (long)g_cfg.shtRelayMode, 0, 1);
    }
    if (server.hasArg("shtRelayManual"))
    {
        g_cfg.shtRelayManual = argToBool("shtRelayManual") ? 1U : 0U;
    }
    if (server.hasArg("shtRelayActiveHigh"))
    {
        g_cfg.shtRelayActiveHigh = argToBool("shtRelayActiveHigh") ? 1U : 0U;
    }
    g_cfg.shtTempOn = argToFloat("shtTempOn", g_cfg.shtTempOn, -40.0f, 125.0f);
    g_cfg.shtTempOff = argToFloat("shtTempOff", g_cfg.shtTempOff, -40.0f, 125.0f);
    g_cfg.shtHumOn = argToFloat("shtHumOn", g_cfg.shtHumOn, 0.0f, 100.0f);
    g_cfg.shtHumOff = argToFloat("shtHumOff", g_cfg.shtHumOff, 0.0f, 100.0f);

    if (server.hasArg("soilEnabled"))
    {
        g_cfg.soilEnabled = argToBool("soilEnabled") ? 1U : 0U;
    }
    g_cfg.soilIntervalMs = (uint32_t)argToInt("soilIntervalMs", (long)g_cfg.soilIntervalMs, 250, 60000);
    if (server.hasArg("soilRelayMode"))
    {
        g_cfg.soilRelayMode = (uint8_t)argToInt("soilRelayMode", (long)g_cfg.soilRelayMode, 0, 1);
    }
    if (server.hasArg("soilRelayManual"))
    {
        g_cfg.soilRelayManual = argToBool("soilRelayManual") ? 1U : 0U;
    }
    if (server.hasArg("soilRelayActiveHigh"))
    {
        g_cfg.soilRelayActiveHigh = argToBool("soilRelayActiveHigh") ? 1U : 0U;
    }
    g_cfg.soilDryPct = (int16_t)argToInt("soilDryPct", (long)g_cfg.soilDryPct, 0, 100);
    g_cfg.soilWetPct = (int16_t)argToInt("soilWetPct", (long)g_cfg.soilWetPct, 0, 100);
    g_cfg.soilRawDry = (uint16_t)argToInt("soilRawDry", (long)g_cfg.soilRawDry, 0, 4095);
    g_cfg.soilRawWet = (uint16_t)argToInt("soilRawWet", (long)g_cfg.soilRawWet, 0, 4095);

    if (server.hasArg("buzzerEnabled"))
    {
        g_cfg.buzzerEnabled = argToBool("buzzerEnabled") ? 1U : 0U;
    }
    if (server.hasArg("buzzerActiveHigh"))
    {
        g_cfg.buzzerActiveHigh = argToBool("buzzerActiveHigh") ? 1U : 0U;
    }
    if (server.hasArg("buzzerMode"))
    {
        g_cfg.buzzerMode = (uint8_t)argToInt("buzzerMode", (long)g_cfg.buzzerMode, 0, 2);
    }
    g_cfg.buzzerBeepOnMs = (uint16_t)argToInt("buzzerBeepOnMs", (long)g_cfg.buzzerBeepOnMs, 20, 5000);
    g_cfg.buzzerBeepOffMs = (uint16_t)argToInt("buzzerBeepOffMs", (long)g_cfg.buzzerBeepOffMs, 20, 5000);

    if (server.hasArg("ledEnabled"))
    {
        g_cfg.ledEnabled = argToBool("ledEnabled") ? 1U : 0U;
    }
    if (server.hasArg("ledMode"))
    {
        g_cfg.ledMode = (uint8_t)argToInt("ledMode", (long)g_cfg.ledMode, 0, 3);
    }
    g_cfg.ledBrightness = (uint8_t)argToInt("ledBrightness", (long)g_cfg.ledBrightness, 0, 255);
    if (server.hasArg("ledInvert"))
    {
        g_cfg.ledInvert = argToBool("ledInvert") ? 1U : 0U;
    }

    if (server.hasArg("oledEnabled"))
    {
        g_cfg.oledEnabled = argToBool("oledEnabled") ? 1U : 0U;
    }
    g_cfg.oledContrast = (uint8_t)argToInt("oledContrast", (long)g_cfg.oledContrast, 0, 255);
    g_cfg.oledIntervalMs = (uint32_t)argToInt("oledIntervalMs", (long)g_cfg.oledIntervalMs, 200, 10000);
    if (server.hasArg("oledShowSensors"))
    {
        g_cfg.oledShowSensors = argToBool("oledShowSensors") ? 1U : 0U;
    }
    if (server.hasArg("oledShowStatus"))
    {
        g_cfg.oledShowStatus = argToBool("oledShowStatus") ? 1U : 0U;
    }

    if (any)
    {
        markConfigDirty();
    }
    if (wifiChanged)
    {
        startWifiAp();
    }
    sendJson("{\"ok\":1}");
}

void handleApiAction()
{
    if (!server.hasArg("act"))
    {
        sendJson("{\"ok\":0,\"error\":\"missing act\"}");
        return;
    }

    String act = server.arg("act");
    bool cfgChange = false;

    if (act == "relayShtOn")
    {
        g_cfg.shtRelayMode = 0;
        g_cfg.shtRelayManual = 1U;
        cfgChange = true;
    }
    else if (act == "relayShtOff")
    {
        g_cfg.shtRelayMode = 0;
        g_cfg.shtRelayManual = 0U;
        cfgChange = true;
    }
    else if (act == "relayShtAuto")
    {
        g_cfg.shtRelayMode = 1U;
        cfgChange = true;
    }
    else if (act == "relaySoilOn")
    {
        g_cfg.soilRelayMode = 0;
        g_cfg.soilRelayManual = 1U;
        cfgChange = true;
    }
    else if (act == "relaySoilOff")
    {
        g_cfg.soilRelayMode = 0;
        g_cfg.soilRelayManual = 0U;
        cfgChange = true;
    }
    else if (act == "relaySoilAuto")
    {
        g_cfg.soilRelayMode = 1U;
        cfgChange = true;
    }
    else if (act == "readNow")
    {
        g_shtLastRead = 0ULL;
        g_soilLastRead = 0ULL;
    }
    else if (act == "buzzerTest")
    {
        g_buzzerTestUntil = nowMs() + 600ULL;
    }
    else if (act == "oledNext")
    {
        g_oledPage = (uint8_t)(1U - g_oledPage);
        g_oledNext = 0ULL;
    }
    else if (act == "reboot")
    {
        sendJson("{\"ok\":1}");
        g_pendingReboot = true;
        return;
    }
    else if (act == "factoryReset")
    {
        sendJson("{\"ok\":1}");
        g_pendingReset = true;
        return;
    }
    else
    {
        sendJson("{\"ok\":0,\"error\":\"unknown act\"}");
        return;
    }

    if (cfgChange)
    {
        markConfigDirty();
    }
    sendJson("{\"ok\":1}");
}

void handleNotFound()
{
    server.sendHeader("Location", "http://192.168.4.1/");
    server.sendHeader("Cache-Control", "no-cache, no-store");
    server.send(302, "text/html",
                "<!doctype html><html><head><meta http-equiv=\"refresh\" content=\"0;url=http://192.168.4.1/\"></head>"
                "<body><p>Redirecting to <a href=\"http://192.168.4.1/\">ESP32 SOIL.SHT</a></p></body></html>");
}

static void serverSetup()
{
    server.on("/", HTTP_GET, handleRoot);
    server.on("/index.html", HTTP_GET, handleRoot);
    server.on("/api/data", HTTP_GET, handleApiData);
    server.on("/api/config", HTTP_POST, handleApiConfig);
    server.on("/api/action", HTTP_POST, handleApiAction);
    server.onNotFound(handleNotFound);
    server.begin();
}

// =====================================================
//  SETUP
// =====================================================
void setup()
{
    pinMode(PIN_RELAY_SHT, OUTPUT);
    pinMode(PIN_RELAY_SOIL, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);
    pinMode(PIN_SOIL_ADC, INPUT);

    digitalWrite(PIN_RELAY_SHT, LOW);
    digitalWrite(PIN_RELAY_SOIL, LOW);
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(PIN_LED, LOW);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000UL);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(PIN_LED, LEDC_FREQ, LEDC_RES);
#else
    ledcSetup(LEDC_CHANNEL, (double)LEDC_FREQ, LEDC_RES);
    ledcAttachPin(PIN_LED, LEDC_CHANNEL);
#endif
    ledWriteDuty(0);

    g_bootCount++;

    prefs.begin(CFG_NAMESPACE, false);
    configLoad();
    if (g_configFresh)
    {
        configSave();
    }

    applyRelay(PIN_RELAY_SHT, g_cfg.shtRelayActiveHigh != 0, false);
    applyRelay(PIN_RELAY_SOIL, g_cfg.soilRelayActiveHigh != 0, false);
    evaluateRelays();

    startWifiAp();
    startDns();
    serverSetup();

    taskAdd(taskSensors, 200UL);
    taskAdd(taskHistory, 1000UL);
    taskAdd(taskBuzzer, 25UL);
    taskAdd(taskLed, 20UL);
    taskAdd(taskOled, 100UL);
    taskAdd(taskBootButton, 50UL);
    taskAdd(taskConfigFlush, 500UL);
    taskAdd(taskSystem, 250UL);
}

// =====================================================
//  MAIN LOOP
// =====================================================
void loop()
{
    updateClock();
    schedulerRun();
    dnsServer.processNextRequest();
    server.handleClient();
}
