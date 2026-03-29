/*
 * PropaneScale.ino
 *
 * ESP8266 Propane Tank Weight Monitor
 *
 * Hardware
 *   - SparkFun ESP8266 Thing (ESP8266 SoC, 80 MHz, 512 KB SRAM, 4 MB flash)
 *   - Avia Semiconductor HX711 24-bit ADC for Weigh Scales
 *   - 4x SparkFun SEN-13329 (10 kg) straight bar load sensors
 *   - SparkFun Load Sensor Combinator (or equivalent wiring) into HX711
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
#include <limits.h>
#include <math.h>

#include "config.h"
#include "HX711.h"
#include "webpage.h"

// ── Forward declarations ──────────────────────────────────────────────────
static void loadCalibration();
static void saveCalibration();
static void setupWiFi();
static void setupWebServer();
static void handleSerialCommands();
static void runScaleStartupSelfCheck();
static void runGuidedCalibration(float knownLbs);
static bool waitForUserEnter(const __FlashStringHelper* prompt, unsigned long timeoutMs);
static void handleApiGuidedCalStart();
static void handleApiGuidedCalNext();
static void handleApiGuidedCalStatus();
static void handleApiGuidedCalCancel();

// ── Global objects ────────────────────────────────────────────────────────
static HX711            scale;
static ESP8266WebServer server(WEB_SERVER_PORT);

// ── Runtime state ─────────────────────────────────────────────────────────
static float         g_calFactor    = DEFAULT_CAL_FACTOR;
static long          g_tareOffset   = 0L;
static float         g_weightLbs    = 0.0f;
static bool          g_isApMode     = false;
static unsigned long g_lastReadMs   = 0UL;

enum GuidedCalStep : uint8_t {
    GUIDED_IDLE = 0,
    GUIDED_WAIT_EMPTY_CONFIRM = 1,
    GUIDED_WAIT_WEIGHT_CONFIRM = 2
};

struct GuidedCalState {
    GuidedCalStep step;
    float knownLbs;
    unsigned long startedMs;
};

static GuidedCalState g_guidedCal = { GUIDED_IDLE, 0.0f, 0UL };

static const char* guidedCalStepPrompt(GuidedCalStep step) {
    switch (step) {
        case GUIDED_WAIT_EMPTY_CONFIRM:
            return "Remove all weight, then click Continue";
        case GUIDED_WAIT_WEIGHT_CONFIRM:
            return "Place known weight, then click Continue";
        default:
            return "Idle";
    }
}

static void resetGuidedCal() {
    g_guidedCal.step = GUIDED_IDLE;
    g_guidedCal.knownLbs = 0.0f;
    g_guidedCal.startedMs = 0UL;
}

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
// Scale diagnostics and guided calibration
// ============================================================
static bool waitForUserEnter(const __FlashStringHelper* prompt, unsigned long timeoutMs) {
    Serial.println(prompt);
    Serial.println(F("[Scale] Press ENTER to continue (or wait to cancel)."));

    const unsigned long deadline = millis() + timeoutMs;
    while (millis() < deadline) {
        if (Serial.available()) {
            String line = Serial.readStringUntil('\n');
            line.trim();
            return true;
        }
        delay(10);
        yield();
    }

    Serial.println(F("[Scale] Timed out waiting for user input."));
    return false;
}

static void runScaleStartupSelfCheck() {
    Serial.println(F("[Scale] Startup self-check (HX711)..."));

    const uint8_t samples = 8;
    long minRaw = LONG_MAX;
    long maxRaw = LONG_MIN;
    bool firstWasZero = false;

    for (uint8_t i = 0; i < samples; ++i) {
        long raw = scale.readRaw();
        if (i == 0 && raw == 0L) {
            firstWasZero = true;
        }
        if (raw < minRaw) minRaw = raw;
        if (raw > maxRaw) maxRaw = raw;
    }

    const long span = maxRaw - minRaw;
    Serial.printf("[Scale] Self-check raw window: min=%ld max=%ld span=%ld\n",
                  minRaw, maxRaw, span);

    if (firstWasZero) {
        Serial.println(F("[Warn] First HX711 read was 0. Check wiring/power if this repeats."));
    }
    if (maxRaw > 8380000L || minRaw < -8380000L) {
        Serial.println(F("[Warn] HX711 raw reading near rail. Possible bridge wiring or overload issue."));
    }
    if (span <= 2L) {
        Serial.println(F("[Warn] Raw span is extremely small. Signal may be stuck/noisy wiring issue."));
    } else if (span > 500000L) {
        Serial.println(F("[Warn] Raw span is very large. Mechanical vibration or electrical noise likely."));
    } else {
        Serial.println(F("[Scale] Self-check looks nominal."));
    }
}

static void runGuidedCalibration(float knownLbs) {
    Serial.println(F("[Scale] Guided calibration start"));
    Serial.printf("[Scale] Known reference: %.3f lbs\n", knownLbs);

    if (!waitForUserEnter(
            F("[Step 1/3] Remove all weight from platform."),
            45000UL)) {
        Serial.println(F("[Scale] Guided calibration cancelled."));
        return;
    }

    Serial.println(F("[Step 2/3] Capturing tare (12 samples)..."));
    scale.tare(12);
    g_tareOffset = scale.getOffset();
    Serial.printf("[Scale] New tare offset: %ld\n", g_tareOffset);

    if (!waitForUserEnter(
            F("[Step 3/3] Place known weight on platform, then press ENTER."),
            90000UL)) {
        Serial.println(F("[Scale] Guided calibration cancelled."));
        return;
    }

    const uint8_t n = 12;
    long samples[n];
    long sum = 0L;
    for (uint8_t i = 0; i < n; ++i) {
        samples[i] = scale.readRaw();
        sum += samples[i];
    }
    const long rawAvg = sum / static_cast<long>(n);
    const long delta = rawAvg - g_tareOffset;

    float variance = 0.0f;
    for (uint8_t i = 0; i < n; ++i) {
        const float d = static_cast<float>(samples[i] - rawAvg);
        variance += d * d;
    }
    variance /= static_cast<float>(n);
    const float noiseStdCounts = sqrtf(variance);

    if (delta == 0L) {
        Serial.println(F("[Error] Zero ADC delta. Ensure the reference mass is on the platform."));
        return;
    }

    g_calFactor = static_cast<float>(delta) / knownLbs;
    scale.setOffset(g_tareOffset);
    scale.setScale(g_calFactor);
    saveCalibration();

    const float verifyLbs = static_cast<float>(scale.getUnits(8));
    const float errPct = ((verifyLbs - knownLbs) / knownLbs) * 100.0f;

    Serial.printf("[Scale] Raw avg: %ld  delta: %ld  noise std: %.1f counts\n",
                  rawAvg, delta, noiseStdCounts);
    Serial.printf("[Scale] New cal factor: %.4f counts/lb\n", g_calFactor);
    Serial.printf("[Scale] Verification: %.3f lbs (error %.2f%%)\n",
                  verifyLbs, errPct);

    if (fabsf(errPct) > 2.0f) {
        Serial.println(F("[Warn] Verification error > 2%. Re-run with a heavier known weight if possible."));
    } else {
        Serial.println(F("[Scale] Guided calibration completed successfully."));
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

    char json[420];
    snprintf(json, sizeof(json),
        "{"
        "\"weight_lbs\":%.3f,"
        "\"propane_lbs\":%.3f,"
        "\"propane_pct\":%.1f,"
        "\"empty_lbs\":%.1f,"
        "\"capacity_lbs\":%.1f,"
        "\"cal_factor\":%.4f,"
        "\"tare_offset\":%ld,"
        "\"load_cell_count\":%u,"
        "\"load_cell_array_capacity_lbs\":%.1f,"
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
        (unsigned)LOAD_CELL_COUNT,
        (float)LOAD_CELL_ARRAY_CAPACITY_LBS,
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

// POST /api/guidedcal/start?weight=<lbs>
// Step 1: initialise guided flow and wait for empty-platform confirmation.
static void handleApiGuidedCalStart() {
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

    g_guidedCal.step = GUIDED_WAIT_EMPTY_CONFIRM;
    g_guidedCal.knownLbs = knownLbs;
    g_guidedCal.startedMs = millis();

    Serial.printf("[Scale] Web guided calibration started (known=%.3f lbs)\n", knownLbs);

    char json[280];
    snprintf(json, sizeof(json),
             "{\"success\":true,\"active\":true,\"step\":%u,\"known_lbs\":%.3f,"
             "\"prompt\":\"%s\",\"message\":\"Guided calibration started\"}",
             static_cast<unsigned>(g_guidedCal.step),
             g_guidedCal.knownLbs,
             guidedCalStepPrompt(g_guidedCal.step));
    server.send(200, "application/json", json);
}

// POST /api/guidedcal/next
// Advances one guided step: capture tare, then capture known-weight sample.
static void handleApiGuidedCalNext() {
    if (g_guidedCal.step == GUIDED_IDLE) {
        server.send(400, "application/json",
            "{\"success\":false,\"message\":\"No guided calibration in progress\"}");
        return;
    }

    if (g_guidedCal.step == GUIDED_WAIT_EMPTY_CONFIRM) {
        scale.tare(12);
        g_tareOffset = scale.getOffset();
        g_weightLbs = 0.0f;
        g_guidedCal.step = GUIDED_WAIT_WEIGHT_CONFIRM;

        Serial.printf("[Scale] Web guided tare complete. offset=%ld\n", g_tareOffset);

        char json[320];
        snprintf(json, sizeof(json),
                 "{\"success\":true,\"active\":true,\"step\":%u,\"known_lbs\":%.3f,"
                 "\"tare_offset\":%ld,\"prompt\":\"%s\","
                 "\"message\":\"Tare captured. Place known weight, then continue\"}",
                 static_cast<unsigned>(g_guidedCal.step),
                 g_guidedCal.knownLbs,
                 g_tareOffset,
                 guidedCalStepPrompt(g_guidedCal.step));
        server.send(200, "application/json", json);
        return;
    }

    if (g_guidedCal.step == GUIDED_WAIT_WEIGHT_CONFIRM) {
        const uint8_t n = 12;
        long samples[n];
        long sum = 0L;
        for (uint8_t i = 0; i < n; ++i) {
            samples[i] = scale.readRaw();
            sum += samples[i];
        }

        const long rawAvg = sum / static_cast<long>(n);
        const long delta = rawAvg - g_tareOffset;

        float variance = 0.0f;
        for (uint8_t i = 0; i < n; ++i) {
            const float d = static_cast<float>(samples[i] - rawAvg);
            variance += d * d;
        }
        variance /= static_cast<float>(n);
        const float noiseStdCounts = sqrtf(variance);

        if (delta == 0L) {
            server.send(400, "application/json",
                "{\"success\":false,\"message\":\"Zero ADC delta. Place known weight first\"}");
            return;
        }

        g_calFactor = static_cast<float>(delta) / g_guidedCal.knownLbs;
        scale.setOffset(g_tareOffset);
        scale.setScale(g_calFactor);
        saveCalibration();

        const float verifyLbs = static_cast<float>(scale.getUnits(8));
        const float errPct = ((verifyLbs - g_guidedCal.knownLbs) / g_guidedCal.knownLbs) * 100.0f;

        Serial.printf("[Scale] Web guided calibration done. factor=%.4f verify=%.3f err=%.2f%%\n",
                      g_calFactor, verifyLbs, errPct);

        char json[420];
        snprintf(json, sizeof(json),
                 "{\"success\":true,\"active\":false,\"step\":0,\"known_lbs\":%.3f,"
                 "\"cal_factor\":%.4f,\"tare_offset\":%ld,\"raw_avg\":%ld,\"delta\":%ld,"
                 "\"noise_std_counts\":%.1f,\"verify_lbs\":%.3f,\"verify_error_pct\":%.2f,"
                 "\"message\":\"Guided calibration completed\"}",
                 g_guidedCal.knownLbs,
                 g_calFactor,
                 g_tareOffset,
                 rawAvg,
                 delta,
                 noiseStdCounts,
                 verifyLbs,
                 errPct);

        resetGuidedCal();
        server.send(200, "application/json", json);
        return;
    }

    resetGuidedCal();
    server.send(500, "application/json",
        "{\"success\":false,\"message\":\"Guided calibration state error\"}");
}

// GET /api/guidedcal/status
static void handleApiGuidedCalStatus() {
    const bool active = (g_guidedCal.step != GUIDED_IDLE);
    char json[260];
    snprintf(json, sizeof(json),
             "{\"active\":%s,\"step\":%u,\"known_lbs\":%.3f,\"started_ms\":%lu,"
             "\"prompt\":\"%s\"}",
             active ? "true" : "false",
             static_cast<unsigned>(g_guidedCal.step),
             g_guidedCal.knownLbs,
             g_guidedCal.startedMs,
             guidedCalStepPrompt(g_guidedCal.step));
    server.send(200, "application/json", json);
}

// POST /api/guidedcal/cancel
static void handleApiGuidedCalCancel() {
    resetGuidedCal();
    server.send(200, "application/json",
        "{\"success\":true,\"active\":false,\"message\":\"Guided calibration cancelled\"}");
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
    server.on("/api/guidedcal/start",  HTTP_POST, handleApiGuidedCalStart);
    server.on("/api/guidedcal/next",   HTTP_POST, handleApiGuidedCalNext);
    server.on("/api/guidedcal/status", HTTP_GET,  handleApiGuidedCalStatus);
    server.on("/api/guidedcal/cancel", HTTP_POST, handleApiGuidedCalCancel);
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

    } else if (cmd.startsWith("GUIDEDCAL ")) {
        // ── GUIDEDCAL <lbs> — interactive calibration wizard ──────────
        float knownLbs = raw.substring(10).toFloat();
        if (knownLbs <= 0.0f) {
            Serial.println(F("[Error] Usage: GUIDEDCAL <weight_in_lbs>"));
            return;
        }
        runGuidedCalibration(knownLbs);

    } else if (cmd == F("GUIDEDCAL")) {
        Serial.println(F("[Error] Usage: GUIDEDCAL <weight_in_lbs>"));

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
        Serial.printf("[Scale]  sensors=%u (SEN-13329)  array capacity=%.1f lbs\n",
                  (unsigned)LOAD_CELL_COUNT,
                  (float)LOAD_CELL_ARRAY_CAPACITY_LBS);
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
        Serial.println(F("  GUIDEDCAL <lbs>       — guided tare + calibration wizard"));
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
    Serial.println(F("  SparkFun Thing + HX711 + 4x SEN-13329"));
    Serial.println(F("========================================"));

    // Restore calibration from EEPROM (or use defaults)
    loadCalibration();

    // Initialise HX711
    scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
    scale.setScale(g_calFactor);
    scale.setOffset(g_tareOffset);
    Serial.println(F("[Scale] HX711 initialised."));
    runScaleStartupSelfCheck();

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
