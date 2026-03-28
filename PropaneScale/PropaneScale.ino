/*
 * PropaneScale.ino
 *
 * ESP8266 Propane Tank Weight Monitor
 *
 * Hardware
 *   - SparkFun ESP8266 Thing (ESP8266 SoC, 80 MHz, 512 KB SRAM, 4 MB flash)
 *   - Avia Semiconductor HX711 24-bit ADC for Weigh Scales
 *   - Single-point load cell (rated for your tank + full-propane weight)
 *
 * Features
 *   - Reads weight via HX711 and computes propane level (%)
 *   - Serves a responsive web UI with live gauge and controls
 *   - JSON REST API: GET /api/data, POST /api/tare, POST /api/calibrate
 *   - mDNS: http://propanescale.local/  (station mode)
 *   - Falls back to soft-AP "PropaneScale" when WIFI_SSID is unreachable
 *   - Persists calibration factor + tare offset in EEPROM
 *   - Serial command interface at 115200 baud (type HELP)
 *
 * Libraries required (install via Arduino Library Manager)
 *   - ESP8266 Arduino core  (Board: "SparkFun ESP8266 Thing")
 *   No third-party libraries needed beyond the ESP8266 core.
 *
 * First-time setup
 *   1. Edit WIFI_SSID / WIFI_PASSWORD in config.h
 *   2. Flash firmware; open Serial Monitor at 115200 baud
 *   3. Follow the calibration procedure (see README or type HELP)
 */

#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#include "config.h"
#include "HX711.h"
#include "webpage.h"

// ── Forward declarations ──────────────────────────────────────────────────
static void loadCalibration();
static void saveCalibration();
static void setupWiFi();
static void setupWebServer();
static void handleSerialCommands();

// ── Global objects ────────────────────────────────────────────────────────
static HX711            scale;
static ESP8266WebServer server(WEB_SERVER_PORT);

// ── Runtime state ─────────────────────────────────────────────────────────
static float         g_calFactor    = DEFAULT_CAL_FACTOR;
static long          g_tareOffset   = 0L;
static float         g_weightLbs    = 0.0f;
static bool          g_isApMode     = false;
static unsigned long g_lastReadMs   = 0UL;

// ── EEPROM layout ─────────────────────────────────────────────────────────
struct __attribute__((packed)) CalibData {
    uint32_t magic;
    float    factor;
    long     offset;
};

// ============================================================
// EEPROM helpers
// ============================================================
static void loadCalibration() {
    EEPROM.begin(sizeof(CalibData));
    CalibData cal;
    EEPROM.get(0, cal);
    EEPROM.end();

    if (cal.magic == EEPROM_VALID_MAGIC) {
        g_calFactor  = cal.factor;
        g_tareOffset = cal.offset;
        Serial.printf("[EEPROM] Loaded  factor=%.4f  offset=%ld\n",
                      g_calFactor, g_tareOffset);
    } else {
        Serial.println(F("[EEPROM] No stored calibration — using defaults."));
        Serial.println(F("[EEPROM] Run calibration before trusting readings."));
    }
}

static void saveCalibration() {
    CalibData cal = { EEPROM_VALID_MAGIC, g_calFactor, g_tareOffset };
    EEPROM.begin(sizeof(CalibData));
    EEPROM.put(0, cal);
    bool ok = EEPROM.commit();
    EEPROM.end();
    if (ok) {
        Serial.printf("[EEPROM] Saved  factor=%.4f  offset=%ld\n",
                      g_calFactor, g_tareOffset);
    } else {
        Serial.println(F("[EEPROM] ERROR: commit failed."));
    }
}

// ============================================================
// WiFi setup — station mode with AP fallback
// ============================================================
static void setupWiFi() {
    Serial.printf("[WiFi]  Connecting to \"%s\"", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    const unsigned long deadline = millis() + WIFI_TIMEOUT_MS;
    while (WiFi.status() != WL_CONNECTED && millis() < deadline) {
        delay(500);
        Serial.print('.');
        yield();
    }

    if (WiFi.status() == WL_CONNECTED) {
        g_isApMode = false;
        Serial.printf("\n[WiFi]  Connected — IP: %s\n",
                      WiFi.localIP().toString().c_str());
    } else {
        Serial.println(F("\n[WiFi]  Timeout — starting Access Point."));
        WiFi.mode(WIFI_AP);
        WiFi.softAP(AP_SSID, AP_PASSWORD);
        g_isApMode = true;
        Serial.printf("[WiFi]  AP \"%s\"  IP: %s\n",
                      AP_SSID, WiFi.softAPIP().toString().c_str());
    }
}

// ============================================================
// Web server handlers
// ============================================================

// GET / — serve full HTML page from flash
static void handleRoot() {
    server.send_P(200, "text/html", WEBPAGE);
}

// GET /api/data — JSON with weight, propane level, and device info
static void handleApiData() {
    float propaneLbs = g_weightLbs - EMPTY_TANK_WEIGHT_LBS;
    if (propaneLbs < 0.0f) propaneLbs = 0.0f;
    float propanePct = (propaneLbs / PROPANE_CAPACITY_LBS) * 100.0f;
    if (propanePct > 100.0f) propanePct = 100.0f;

    String ip = g_isApMode ? WiFi.softAPIP().toString()
                           : WiFi.localIP().toString();

    char json[320];
    snprintf(json, sizeof(json),
        "{"
        "\"weight_lbs\":%.3f,"
        "\"propane_lbs\":%.3f,"
        "\"propane_pct\":%.1f,"
        "\"empty_lbs\":%.1f,"
        "\"capacity_lbs\":%.1f,"
        "\"cal_factor\":%.4f,"
        "\"tare_offset\":%ld,"
        "\"uptime_s\":%lu,"
        "\"ip\":\"%s\","
        "\"ap_mode\":%s"
        "}",
        g_weightLbs,
        propaneLbs,
        propanePct,
        (float)EMPTY_TANK_WEIGHT_LBS,
        (float)PROPANE_CAPACITY_LBS,
        g_calFactor,
        g_tareOffset,
        millis() / 1000UL,
        ip.c_str(),
        g_isApMode ? "true" : "false");

    server.send(200, "application/json", json);
}

// POST /api/tare — zero the scale with current load removed
static void handleApiTare() {
    Serial.println(F("[Scale] Taring (8 samples)…"));

    scale.tare(8);
    g_tareOffset = scale.getOffset();
    g_weightLbs  = 0.0f;
    saveCalibration();

    Serial.printf("[Scale] Tare offset: %ld\n", g_tareOffset);
    server.send(200, "application/json",
                "{\"success\":true,\"message\":\"Tare complete\"}");
}

// POST /api/calibrate?weight=<lbs>
// Place a known weight on the scale, then call this endpoint.
static void handleApiCalibrate() {
    if (!server.hasArg("weight")) {
        server.send(400, "application/json",
            "{\"success\":false,\"message\":\"Missing ?weight= parameter\"}");
        return;
    }

    float knownLbs = server.arg("weight").toFloat();
    if (knownLbs <= 0.0f) {
        server.send(400, "application/json",
            "{\"success\":false,\"message\":\"Weight must be a positive number\"}");
        return;
    }

    long rawAvg = scale.readAverage(8);
    long delta  = rawAvg - g_tareOffset;

    if (delta == 0L) {
        server.send(400, "application/json",
            "{\"success\":false,\"message\":\"Zero ADC delta — place known weight on scale first\"}");
        return;
    }

    g_calFactor = static_cast<float>(delta) / knownLbs;
    scale.setScale(g_calFactor);
    saveCalibration();

    Serial.printf("[Scale] New cal factor: %.4f  (%.3f lbs)\n",
                  g_calFactor, knownLbs);

    char json[128];
    snprintf(json, sizeof(json),
             "{\"success\":true,\"message\":\"Calibration saved\",\"factor\":%.4f}",
             g_calFactor);
    server.send(200, "application/json", json);
}

// GET /api/health — simple liveness check
static void handleApiHealth() {
    char json[64];
    snprintf(json, sizeof(json),
             "{\"status\":\"ok\",\"uptime_s\":%lu}", millis() / 1000UL);
    server.send(200, "application/json", json);
}

static void handleNotFound() {
    server.send(404, "text/plain", "Not Found");
}

static void setupWebServer() {
    server.on("/",              HTTP_GET,  handleRoot);
    server.on("/api/data",      HTTP_GET,  handleApiData);
    server.on("/api/health",    HTTP_GET,  handleApiHealth);
    server.on("/api/tare",      HTTP_POST, handleApiTare);
    server.on("/api/calibrate", HTTP_POST, handleApiCalibrate);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.printf("[Web]   Server on port %d\n", WEB_SERVER_PORT);
}

// ============================================================
// Serial command interface
// ============================================================
static void handleSerialCommands() {
    if (!Serial.available()) return;

    String raw = Serial.readStringUntil('\n');
    raw.trim();
    String cmd = raw;
    cmd.toUpperCase();

    if (cmd == F("TARE")) {
        // ── TARE — zero the scale ─────────────────────────────────────
        Serial.println(F("[Scale] Taring (8 samples)…"));
        scale.tare(8);
        g_tareOffset = scale.getOffset();
        g_weightLbs  = 0.0f;
        saveCalibration();
        Serial.printf("[Scale] Tare offset: %ld\n", g_tareOffset);

    } else if (cmd.startsWith("CALIBRATE ")) {
        // ── CALIBRATE <lbs> — apply calibration with known weight ─────
        float knownLbs = raw.substring(10).toFloat();
        if (knownLbs <= 0.0f) {
            Serial.println(F("[Error] Usage: CALIBRATE <weight_in_lbs>"));
            return;
        }
        Serial.printf("[Scale] Reading raw average (8 samples) for %.2f lbs…\n", knownLbs);
        long rawAvg = scale.readAverage(8);
        long delta  = rawAvg - g_tareOffset;
        if (delta == 0L) {
            Serial.println(F("[Error] Zero ADC delta. Place known weight on scale first."));
            return;
        }
        g_calFactor = static_cast<float>(delta) / knownLbs;
        scale.setScale(g_calFactor);
        saveCalibration();
        Serial.printf("[Scale] Cal factor: %.4f\n", g_calFactor);
        Serial.printf("[Scale] Verification read: %.3f lbs\n",
                      static_cast<float>(scale.getUnits(4)));

    } else if (cmd == F("RESET")) {
        // ── RESET — restore factory calibration defaults ──────────────
        g_calFactor  = DEFAULT_CAL_FACTOR;
        g_tareOffset = 0L;
        scale.setScale(g_calFactor);
        scale.setOffset(g_tareOffset);
        saveCalibration();
        Serial.println(F("[Scale] Calibration reset to factory defaults."));

    } else if (cmd == F("RAW")) {
        // ── RAW — print a single raw ADC reading ─────────────────────
        long raw24 = scale.readRaw();
        Serial.printf("[Scale] Raw ADC: %ld  (0x%06lX)\n", raw24, raw24 & 0xFFFFFFL);

    } else if (cmd == F("WEIGHT")) {
        // ── WEIGHT — print current weight ─────────────────────────────
        float w = static_cast<float>(scale.getUnits(4));
        Serial.printf("[Scale] Weight: %.3f lbs\n", w);

    } else if (cmd == F("STATUS")) {
        // ── STATUS — print all runtime info ──────────────────────────
        float propaneLbs = g_weightLbs - EMPTY_TANK_WEIGHT_LBS;
        if (propaneLbs < 0.0f) propaneLbs = 0.0f;
        float propanePct = (propaneLbs / PROPANE_CAPACITY_LBS) * 100.0f;

        Serial.println(F("────────────────────────────────────────"));
        Serial.printf("[Scale]  weight=%.3f lbs  propane=%.3f lbs  level=%.1f%%\n",
                      g_weightLbs, propaneLbs, propanePct);
        Serial.printf("[Scale]  cal factor=%.4f  tare offset=%ld\n",
                      g_calFactor, g_tareOffset);
        Serial.printf("[WiFi]   mode=%s  IP=%s  RSSI=%d dBm\n",
                      g_isApMode ? "AP" : "STA",
                      g_isApMode ? WiFi.softAPIP().toString().c_str()
                                 : WiFi.localIP().toString().c_str(),
                      g_isApMode ? 0 : WiFi.RSSI());
        Serial.printf("[Boot]   uptime=%lu s\n", millis() / 1000UL);
        Serial.println(F("────────────────────────────────────────"));

    } else if (cmd == F("HELP")) {
        // ── HELP — list commands ───────────────────────────────────────
        Serial.println(F("Serial commands (115200 baud):"));
        Serial.println(F("  TARE                 — zero the scale (remove all weight first)"));
        Serial.println(F("  CALIBRATE <lbs>       — calibrate with known weight placed on scale"));
        Serial.println(F("  RESET                — restore default calibration factor"));
        Serial.println(F("  RAW                  — print raw 24-bit ADC reading"));
        Serial.println(F("  WEIGHT               — print current weight (4-sample average)"));
        Serial.println(F("  STATUS               — print weight, calibration, and WiFi info"));
        Serial.println(F("  HELP                 — this message"));

    } else if (cmd.length() > 0) {
        Serial.printf("[Error] Unknown command \"%s\". Type HELP.\n", raw.c_str());
    }
}

// ============================================================
// setup()
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(200);

    Serial.println(F("\n========================================"));
    Serial.println(F("  ESP8266 Propane Scale  v1.0.0"));
    Serial.println(F("  SparkFun Thing + HX711 24-bit ADC"));
    Serial.println(F("========================================"));

    // Restore calibration from EEPROM (or use defaults)
    loadCalibration();

    // Initialise HX711
    scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
    scale.setScale(g_calFactor);
    scale.setOffset(g_tareOffset);
    Serial.println(F("[Scale] HX711 initialised."));

    // Connect to WiFi (or start soft-AP)
    setupWiFi();

    // mDNS — registers "propanescale.local" in station mode
    if (!g_isApMode && MDNS.begin(MDNS_HOSTNAME)) {
        MDNS.addService("http", "tcp", WEB_SERVER_PORT);
        Serial.printf("[mDNS]  http://%s.local/\n", MDNS_HOSTNAME);
    }

    // Start web server
    setupWebServer();

    String ip = g_isApMode ? WiFi.softAPIP().toString()
                           : WiFi.localIP().toString();
    Serial.printf("[Boot]  Web UI  →  http://%s/\n", ip.c_str());
    Serial.println(F("[Boot]  Serial  →  type HELP for commands"));
    Serial.println(F("========================================\n"));
}

// ============================================================
// loop()
// ============================================================
void loop() {
    server.handleClient();
    MDNS.update();
    handleSerialCommands();

    // Periodic weight reading
    if (millis() - g_lastReadMs >= READ_INTERVAL_MS) {
        g_weightLbs  = static_cast<float>(scale.getUnits(NUM_SAMPLES));
        g_lastReadMs = millis();
    }
}
