/*
 * PropaneScale.ino
 * ESP32 Propane Tank Weight Monitor
 * Hardware
 *   - SparkFun ESP32 Thing (ESP32 SoC)
 *   - Avia Semiconductor HX711 24-bit ADC for Weigh Scales
 *   - 4x SparkFun SEN-13329 (10 kg) straight bar load sensors
 *   - SparkFun Load Sensor Combinator (or equivalent wiring) into HX711
 * Features
 *   - Reads weight via HX711 and computes propane level (%)
 *   - Serves a responsive web UI with live gauge and controls
 *   - JSON REST API: GET /api/data, POST /api/tare, POST /api/calibrate
 *   - mDNS: http://propanescale.local/  (station mode)
 *   - Falls back to soft-AP "PropaneScale" when WIFI_SSID is unreachable
 *   - Persists calibration factor + tare offset in EEPROM
 *   - Serial command interface at 115200 baud (type HELP)
 * Libraries required (install via Arduino Library Manager)
 *   - ESP32 Arduino core  (Board: "SparkFun ESP32 Thing")
 *   - HX711 Arduino Library by Bogde
 * First-time setup
 *   1. Edit WIFI_SSID / WIFI_PASSWORD in config.h
 *   2. Flash firmware; open Serial Monitor at 115200 baud
 *   3. Follow the calibration procedure (see README or type HELP)
 */

#include <EEPROM.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <limits.h>
#include <math.h>

#include "config.h"
#include <HX711.h>
#include "webpage.h"

// ── Forward declarations ──────────────────────────────────────────────────
static void loadCalibration();
static void saveCalibration();
static void setupWiFi();
static void setupWebServer();
static void handleSerialCommands();
static void runScaleStartupSelfCheck();
static void runGuidedCalibration(float knownLbs);
static void startManualCalFactor(float targetLbs);
static void handleManualCalFactor();
static void printManualCalFactorStatus();
static bool waitForUserEnter(const __FlashStringHelper* prompt, unsigned long timeoutMs);
static void handleApiGuidedCalStart();
static void handleApiGuidedCalNext();
static void handleApiGuidedCalStatus();
static void handleApiGuidedCalCancel();

/**
 * @brief Aggregated statistics for a window of raw HX711 samples.
 */
struct RawSampleStats {
    long rawAvg;
    float noiseStdCounts;
    uint8_t invalidCount;
    uint8_t zeroCount;
    long minRaw;
    long maxRaw;
};

static RawSampleStats captureRawSampleStats(uint8_t n);
static long computeAdaptiveMinDeltaCounts(float noiseStdCounts);
static uint8_t invalidFailThreshold(uint8_t n);
static uint8_t zeroFailThreshold(uint8_t n);

// ── Global objects ────────────────────────────────────────────────────────
static HX711            scale;
static WebServer server(WEB_SERVER_PORT);

// ── Runtime state ─────────────────────────────────────────────────────────
static float         g_calFactor    = DEFAULT_CAL_FACTOR;
static long          g_tareOffset   = 0L;
static float         g_weightLbs    = 0.0f;
static bool          g_isApMode     = false;
static unsigned long g_lastReadMs   = 0UL;
static bool          g_manualCalActive = false;
static float         g_manualCalTargetLbs = 0.0f;
static float         g_manualCalStartFactor = DEFAULT_CAL_FACTOR;
static unsigned long g_manualCalLastPrintMs = 0UL;

/**
 * @brief Guided calibration state machine steps.
 */
enum GuidedCalStep : uint8_t {
    GUIDED_IDLE = 0,
    GUIDED_WAIT_EMPTY_CONFIRM = 1,
    GUIDED_WAIT_WEIGHT_CONFIRM = 2
};

/**
 * @brief Runtime state for guided calibration flow.
 */
struct GuidedCalState {
    GuidedCalStep step;
    float knownLbs;
    unsigned long startedMs;
};

static GuidedCalState g_guidedCal = { GUIDED_IDLE, 0.0f, 0UL };

/**
 * @brief Returns the user prompt for a guided calibration step.
 * @param[in] {GuidedCalStep} step Current guided calibration step.
 * @return {const char*} Prompt text for the given step.
 */
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

/**
 * @brief Resets guided calibration runtime state.
 */
static void resetGuidedCal() {
    g_guidedCal.step = GUIDED_IDLE;
    g_guidedCal.knownLbs = 0.0f;
    g_guidedCal.startedMs = 0UL;
}

/**
 * @brief Captures a set of raw HX711 sample statistics.
 * @details Collects up to n samples, ignoring timeout sentinels, and computes
 * average, noise standard deviation, invalid count, zero count, and min/max.
 * @param[in] {uint8_t} n Number of samples to gather; 0 is treated as 1.
 * @return {RawSampleStats} Aggregated statistics for the sample window.
 */
static RawSampleStats captureRawSampleStats(uint8_t n) {
    if (n == 0U) n = 1U;

    double mean = 0.0;
    double m2 = 0.0;
    uint8_t invalidCount = 0U;
    uint8_t zeroCount = 0U;
    uint8_t validCount = 0U;
    long minRaw = LONG_MAX;
    long maxRaw = LONG_MIN;

    for (uint8_t i = 0; i < n; ++i) {
        const long raw = scale.read();
        if (raw == LONG_MIN) {
            ++invalidCount;
            continue;
        }
        if (raw == 0L) {
            ++zeroCount;
        }
        ++validCount;
        if (raw < minRaw) minRaw = raw;
        if (raw > maxRaw) maxRaw = raw;

        const double x = static_cast<double>(raw);
        const double delta = x - mean;
        mean += delta / static_cast<double>(i + 1U);
        const double delta2 = x - mean;
        m2 += delta * delta2;
    }

    RawSampleStats stats;
    stats.rawAvg = (validCount > 0U) ? static_cast<long>(mean) : 0L;
    stats.noiseStdCounts = (validCount > 0U)
        ? static_cast<float>(sqrt(m2 / static_cast<double>(validCount)))
        : 0.0f;
    stats.invalidCount = invalidCount;
    stats.zeroCount = zeroCount;
    stats.minRaw = (validCount > 0U) ? minRaw : 0L;
    stats.maxRaw = (validCount > 0U) ? maxRaw : 0L;
    return stats;
}

/**
 * @brief Computes minimum required delta counts for calibration acceptance.
 * @details Uses measured noise multiplied by CAL_NOISE_THRESHOLD_MULTIPLIER
 * and clamps to CAL_MIN_DELTA_FLOOR_COUNTS.
 * @param[in] {float} noiseStdCounts Estimated standard deviation in ADC counts.
 * @return {long} Minimum absolute delta counts needed for calibration.
 */
static long computeAdaptiveMinDeltaCounts(float noiseStdCounts) {
    float adaptive = noiseStdCounts * CAL_NOISE_THRESHOLD_MULTIPLIER;
    if (adaptive < static_cast<float>(CAL_MIN_DELTA_FLOOR_COUNTS)) {
        adaptive = static_cast<float>(CAL_MIN_DELTA_FLOOR_COUNTS);
    }
    return static_cast<long>(ceilf(adaptive));
}

/**
 * @brief Returns invalid sample count threshold that indicates failure.
 * @details Uses ceil(n/2), matching prior behavior for 8 and 12 samples.
 */
static uint8_t invalidFailThreshold(uint8_t n) {
    if (n == 0U) return 1U;
    return static_cast<uint8_t>((n + 1U) / 2U);
}

/**
 * @brief Returns zero sample count threshold that indicates failure.
 * @details Uses n-1, matching prior behavior for 8 and 12 samples.
 */
static uint8_t zeroFailThreshold(uint8_t n) {
    if (n <= 1U) return 1U;
    return static_cast<uint8_t>(n - 1U);
}

/**
 * @brief Starts interactive manual calibration-factor mode over serial.
 * @param[in] {float} targetLbs Optional known target weight in pounds.
 */
static void startManualCalFactor(float targetLbs) {
    Serial.println(F("[Scale] Manual calibration-factor mode start"));
    Serial.println(F("[Step 1/2] Remove all weight from the scale before starting."));
    Serial.printf("[Scale] Capturing tare (%u samples)...\n", static_cast<unsigned>(TARE_SAMPLE_COUNT));

    scale.tare(TARE_SAMPLE_COUNT);
    g_tareOffset = scale.get_offset();
    scale.set_offset(g_tareOffset);
    g_weightLbs = 0.0f;

    g_manualCalActive = true;
    g_manualCalTargetLbs = targetLbs;
    g_manualCalStartFactor = g_calFactor;
    g_manualCalLastPrintMs = 0UL;

    Serial.printf("[Scale] Tare offset: %ld\n", g_tareOffset);
    Serial.println(F("[Step 2/2] Place the known weight on the scale."));
    Serial.println(F("[Input] Press '+' to increase factor by 10 counts/lb."));
    Serial.println(F("[Input] Press '-' to decrease factor by 10 counts/lb."));
    Serial.println(F("[Input] Press 'a' to increase factor by 1 count/lb."));
    Serial.println(F("[Input] Press 'z' to decrease factor by 1 count/lb."));
    Serial.println(F("[Input] Press 's' to save and exit, or 'q' to quit without saving."));
    if (targetLbs > 0.0f) {
        Serial.printf("[Scale] Target weight: %.3f lbs\n", targetLbs);
    }
    printManualCalFactorStatus();
}

/**
 * @brief Prints current manual calibration reading and diagnostics.
 */
static void printManualCalFactorStatus() {
    const RawSampleStats stats = captureRawSampleStats(8);
    float reading = 0.0f;
    if (stats.invalidCount < 8U) {
        reading = static_cast<float>(stats.rawAvg - g_tareOffset) / g_calFactor;
    }

    Serial.printf("[Scale] ManualCal reading=%.3f lbs factor=%.4f", reading, g_calFactor);
    if (g_manualCalTargetLbs > 0.0f) {
        const float error = reading - g_manualCalTargetLbs;
        Serial.printf(" target=%.3f err=%+.3f lbs", g_manualCalTargetLbs, error);
    }
    Serial.println();

    Serial.printf("[Diag] raw_avg=%ld tare=%ld span=%ld invalid=%u/8 zero=%u/8\n",
                  stats.rawAvg,
                  g_tareOffset,
                  stats.maxRaw - stats.minRaw,
                  static_cast<unsigned>(stats.invalidCount),
                  static_cast<unsigned>(stats.zeroCount));

    if (stats.invalidCount >= 4U) {
        Serial.println(F("[Warn] Many HX711 timeout samples. Check DOUT/SCK wiring and power."));
    } else if (stats.zeroCount >= 7U) {
        Serial.println(F("[Warn] HX711 samples are all zero. Check A+/A-/E+/E- bridge wiring and DOUT state."));
    }
}

/**
 * @brief Handles serial key controls during manual calibration mode.
 * @details Supports factor increments/decrements, save, cancel, and periodic
 * status output while manual calibration mode is active.
 */
static void handleManualCalFactor() {
    while (Serial.available()) {
        const int rawCh = Serial.read();
        if (rawCh < 0) break;

        const char ch = static_cast<char>(rawCh);
        if (ch == '\r' || ch == '\n' || ch == ' ') {
            continue;
        }

        bool changed = false;
        if (ch == '+') {
            g_calFactor += 10.0f;
            changed = true;
        } else if (ch == '-') {
            g_calFactor -= 10.0f;
            changed = true;
        } else if (ch == 'a' || ch == 'A') {
            g_calFactor += 1.0f;
            changed = true;
        } else if (ch == 'z' || ch == 'Z') {
            g_calFactor -= 1.0f;
            changed = true;
        } else if (ch == 's' || ch == 'S') {
            g_manualCalActive = false;
            scale.set_scale(g_calFactor);
            saveCalibration();
            Serial.printf("[Scale] Manual calibration saved. factor=%.4f\n", g_calFactor);
            return;
        } else if (ch == 'q' || ch == 'Q') {
            g_manualCalActive = false;
            g_calFactor = g_manualCalStartFactor;
            scale.set_scale(g_calFactor);
            Serial.printf("[Scale] Manual calibration cancelled. Restored factor=%.4f\n", g_calFactor);
            return;
        } else {
            Serial.printf("[Input] Unknown key '%c'. Use + / - / a / z / s / q.\n", ch);
        }

        if (g_calFactor < 1.0f) {
            g_calFactor = 1.0f;
        }

        if (changed) {
            scale.set_scale(g_calFactor);
            printManualCalFactorStatus();
            g_manualCalLastPrintMs = millis();
        }
    }

    if (millis() - g_manualCalLastPrintMs >= 1000UL) {
        printManualCalFactorStatus();
        g_manualCalLastPrintMs = millis();
    }
}

// ── EEPROM layout ─────────────────────────────────────────────────────────
/**
 * @brief Persisted calibration data layout in EEPROM.
 * @details Packed structure stored at EEPROM address 0. The magic field
 * validates stored data before applying factor/offset at boot.
 */
struct __attribute__((packed)) CalibData {
    uint32_t magic;
    float    factor;
    long     offset;
};

// ============================================================
// EEPROM helpers
// ============================================================
/**
 * @brief Loads calibration data from EEPROM if present.
 * @details Reads CalibData at address 0 and applies values when magic is valid.
 * Otherwise keeps default runtime values.
 */
static void loadCalibration() {
    EEPROM.begin(sizeof(CalibData));
    CalibData cal;
    EEPROM.get(0, cal);
    EEPROM.end();

    if (cal.magic == EEPROM_VALID_MAGIC) {
        g_calFactor  = cal.factor;
        g_tareOffset = cal.offset;
        Serial.printf("[EEPROM] Loaded factor=%.4f offset=%ld\n", g_calFactor, g_tareOffset);
    } else {
        Serial.println(F("[EEPROM] No stored calibration — using defaults."));
        Serial.println(F("[EEPROM] Run calibration before trusting readings."));
    }
}

/**
 * @brief Saves calibration data to EEPROM.
 * @details Writes magic, factor, and offset, then commits EEPROM changes.
 */
static void saveCalibration() {
    CalibData cal = { EEPROM_VALID_MAGIC, g_calFactor, g_tareOffset };
    EEPROM.begin(sizeof(CalibData));
    EEPROM.put(0, cal);
    bool ok = EEPROM.commit();
    EEPROM.end();
    if (ok) {
        Serial.printf("[EEPROM] Saved factor=%.4f offset=%ld\n", g_calFactor, g_tareOffset);
    } else {
        Serial.println(F("[EEPROM] ERROR: commit failed."));
    }
}

// ============================================================
// Scale diagnostics and guided calibration
// ============================================================
/**
 * @brief Waits for user acknowledgement on serial input.
 * @details Accepts NEXT, GO, OK, CONTINUE, or empty input as continue; accepts
 * CANCEL as abort; times out after timeoutMs.
 * @param[in] {const __FlashStringHelper*} prompt Prompt printed to serial.
 * @param[in] {unsigned long} timeoutMs Timeout in milliseconds.
 * @return {bool} True when user confirms; false on cancel or timeout.
 */
static bool waitForUserEnter(const __FlashStringHelper* prompt, unsigned long timeoutMs) {
    Serial.println(prompt);
    Serial.println(F("[Input] To continue: type NEXT (or GO), then press Send/ENTER."));
    Serial.println(F("[Input] To cancel: type CANCEL, then press Send/ENTER."));

    const unsigned long deadline = millis() + timeoutMs;
    while (millis() < deadline) {
        if (Serial.available()) {
            // readString handles monitors that do not append '\n' line endings.
            String line = Serial.readString();
            line.trim();
            line.toUpperCase();
            if (line == F("CANCEL")) {
                Serial.println(F("[Scale] Guided calibration cancelled by user."));
                return false;
            }
            if (line.length() == 0 ||
                line == F("NEXT") ||
                line == F("GO") ||
                line == F("OK") ||
                line == F("CONTINUE")) {
                Serial.println(F("[Input] Step acknowledged. Continuing..."));
                return true;
            }

            Serial.printf("[Input] Received \"%s\". Type NEXT (or GO) to continue, or CANCEL to abort.\n", line.c_str());
        }
        delay(10);
        yield();
    }

    Serial.println(F("[Scale] Timed out waiting for user input."));
    return false;
}

/**
 * @brief Performs HX711 startup health checks.
 * @details Reads an initial sample window and prints diagnostics/warnings for
 * timeout conditions, rail conditions, and suspicious signal span.
 */
static void runScaleStartupSelfCheck() {
    Serial.println(F("[Scale] Startup self-check (HX711)..."));

    const uint8_t samples = 8;
    long minRaw = LONG_MAX;
    long maxRaw = LONG_MIN;
    bool firstWasInvalid = false;
    uint8_t invalidCount = 0U;

    for (uint8_t i = 0; i < samples; ++i) {
        long raw = scale.read();
        if (raw == LONG_MIN) {
            ++invalidCount;
            if (i == 0) {
                firstWasInvalid = true;
            }
            continue;
        }
        if (raw < minRaw) minRaw = raw;
        if (raw > maxRaw) maxRaw = raw;
    }

    const long span = maxRaw - minRaw;
    Serial.printf("[Scale] Self-check raw window: min=%ld max=%ld span=%ld invalid=%u/%u\n",
                  minRaw, maxRaw, span,
                  static_cast<unsigned>(invalidCount),
                  static_cast<unsigned>(samples));

    if (firstWasInvalid) {
        Serial.println(F("[Warn] First HX711 read timed out. Check wiring/power if this repeats."));
    }
    if (invalidCount > 0U) {
        Serial.println(F("[Warn] HX711 timeout samples detected during startup check."));
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

/**
 * @brief Runs interactive guided calibration via serial prompts.
 * @details Captures tare with empty platform, captures known-weight samples,
 * validates data quality, updates calibration, saves EEPROM, and prints verify.
 * @param[in] {float} knownLbs Known reference weight in pounds.
 */
static void runGuidedCalibration(float knownLbs) {
    Serial.println(F("[Scale] Guided calibration start"));
    Serial.printf("[Scale] Known reference: %.3f lbs\n", knownLbs);

    if (!waitForUserEnter(
            F("[Step 1/3] Remove all weight from platform."),
            45000UL)) {
        Serial.println(F("[Scale] Guided calibration cancelled."));
        return;
    }
    Serial.println(F("[Step 1/3] Complete: empty platform confirmed."));

    Serial.printf("[Step 2/3] Capturing tare (%u samples)...\n", static_cast<unsigned>(TARE_SAMPLE_COUNT));
    scale.tare(TARE_SAMPLE_COUNT);
    g_tareOffset = scale.get_offset();
    Serial.printf("[Scale] New tare offset: %ld\n", g_tareOffset);

    const RawSampleStats tareStats = captureRawSampleStats(TARE_SAMPLE_COUNT);
    if (tareStats.invalidCount >= invalidFailThreshold(TARE_SAMPLE_COUNT)) {
        Serial.println(F("[Error] HX711 returned too many timeout samples during tare."));
        Serial.println(F("[Hint] Check HX711 wiring/power (DOUT/SCK/VCC/GND) and retry GUIDEDCAL."));
        return;
    }
    if (tareStats.zeroCount >= zeroFailThreshold(TARE_SAMPLE_COUNT)) {
        Serial.println(F("[Error] HX711 tare samples are all zero (or nearly all zero)."));
        Serial.println(F("[Hint] DOUT may be stuck LOW / shorted, or A+/A- input is not connected correctly."));
        return;
    }
    Serial.printf("[Scale] Tare check: avg=%ld span=%ld invalid=%u/%u zero=%u/%u\n",
                  tareStats.rawAvg,
                  tareStats.maxRaw - tareStats.minRaw,
                  static_cast<unsigned>(tareStats.invalidCount),
                  static_cast<unsigned>(TARE_SAMPLE_COUNT),
                  static_cast<unsigned>(tareStats.zeroCount),
                  static_cast<unsigned>(TARE_SAMPLE_COUNT));

    Serial.println(F("[Step 2/3] Complete: tare captured."));
    Serial.println(F("[Step 3/3] Next: place known weight, then type NEXT and press ENTER."));

    if (!waitForUserEnter(
            F("[Step 3/3] Place known weight on platform."),
            90000UL)) {
        Serial.println(F("[Scale] Guided calibration cancelled."));
        return;
    }
    Serial.println(F("[Step 3/3] Complete: known-weight sample acknowledged."));

    const RawSampleStats stats = captureRawSampleStats(CAL_SAMPLE_COUNT);
    const long rawAvg = stats.rawAvg;
    const long delta = rawAvg - g_tareOffset;
    const float noiseStdCounts = stats.noiseStdCounts;
    const long minDeltaCounts = computeAdaptiveMinDeltaCounts(noiseStdCounts);

    if (stats.invalidCount >= invalidFailThreshold(CAL_SAMPLE_COUNT)) {
        Serial.println(F("[Error] HX711 returned too many timeout samples with known weight."));
        Serial.println(F("[Hint] Sensor is not providing valid data. Check wiring/power and try again."));
        return;
    }
    if (stats.zeroCount >= zeroFailThreshold(CAL_SAMPLE_COUNT)) {
        Serial.println(F("[Error] HX711 known-weight samples are all zero (or nearly all zero)."));
        Serial.println(F("[Hint] Check load-cell bridge wiring to HX711 A+/A- and ensure DOUT is not stuck low."));
        return;
    }

    const long deltaAbs = (delta >= 0L) ? delta : -delta;
    if (deltaAbs < minDeltaCounts) {
        Serial.printf("[Error] ADC delta too small for calibration (delta=%ld, min=%ld).\n", delta, minDeltaCounts);
        Serial.printf("[Diag] Raw avg=%ld tare=%ld noise=%.1f invalid=%u/%u zero=%u/%u\n",
                      rawAvg, g_tareOffset, noiseStdCounts,
                      static_cast<unsigned>(stats.invalidCount),
                  static_cast<unsigned>(CAL_SAMPLE_COUNT),
                  static_cast<unsigned>(stats.zeroCount),
                  static_cast<unsigned>(CAL_SAMPLE_COUNT));
        Serial.println(F("[Hint] Re-run guided calibration: tare with empty platform, then place known weight."));
        return;
    }

    g_calFactor = static_cast<float>(delta) / knownLbs;
    scale.set_offset(g_tareOffset);
    scale.set_scale(g_calFactor);
    saveCalibration();

    const float verifyLbs = static_cast<float>(scale.get_units(CAL_SAMPLE_COUNT));
    const float errPct = ((verifyLbs - knownLbs) / knownLbs) * 100.0f;

    Serial.printf("[Scale] Raw avg: %ld  delta: %ld  noise std: %.1f counts\n", rawAvg, delta, noiseStdCounts);
    Serial.printf("[Scale] New cal factor: %.4f counts/lb\n", g_calFactor);
    Serial.printf("[Scale] Verification: %.3f lbs (error %.2f%%)\n", verifyLbs, errPct);

    if (fabsf(errPct) > 2.0f) {
        Serial.println(F("[Warn] Verification error > 2%. Re-run with a heavier known weight if possible."));
    } else {
        Serial.println(F("[Scale] Guided calibration completed successfully."));
    }
}

// ============================================================
// WiFi setup — station mode with AP fallback
// ============================================================
/**
 * @brief Initializes WiFi in station mode with AP fallback.
 * @details Attempts to connect to configured STA credentials until timeout.
 * On failure, starts soft AP and updates mode state.
 */
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
        Serial.printf("\n[WiFi]  Connected — IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println(F("\n[WiFi]  Timeout — starting Access Point."));
        WiFi.mode(WIFI_AP);
        WiFi.softAP(AP_SSID, AP_PASSWORD);
        g_isApMode = true;
        Serial.printf("[WiFi]  AP \"%s\"  IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
    }
}

// ============================================================
// Web server handlers
// ============================================================

/**
 * @brief Handles HTTP GET / and serves the web UI.
 */
static void handleRoot() {
    server.send_P(200, "text/html", WEBPAGE);
}

/**
 * @brief Handles HTTP GET /api/data for live telemetry.
 * @details Returns JSON containing weight, propane metrics, calibration values,
 * network mode/details, and uptime.
 */
static void handleApiData() {
    float propaneLbs = g_weightLbs - EMPTY_TANK_WEIGHT_LBS;
    if (propaneLbs < 0.0f) propaneLbs = 0.0f;
    float propanePct = (propaneLbs / PROPANE_CAPACITY_LBS) * 100.0f;
    if (propanePct > 100.0f) propanePct = 100.0f;

    String ip = g_isApMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();

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

/**
 * @brief Handles HTTP POST /api/tare and performs tare operation.
 */
static void handleApiTare() {
    Serial.printf("[Scale] Taring (%u samples)...\n", static_cast<unsigned>(TARE_SAMPLE_COUNT));

    scale.tare(TARE_SAMPLE_COUNT);
    g_tareOffset = scale.get_offset();
    g_weightLbs  = 0.0f;
    saveCalibration();

    Serial.printf("[Scale] Tare offset: %ld\n", g_tareOffset);
    server.send(200, "application/json",
                "{\"success\":true,\"message\":\"Tare complete\"}");
}

/**
 * @brief Handles HTTP POST /api/calibrate for known-weight calibration.
 * @details Validates request parameters and sample quality before computing and
 * persisting a new calibration factor.
 */
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

    const RawSampleStats stats = captureRawSampleStats(CAL_SAMPLE_COUNT);
    long rawAvg = stats.rawAvg;
    long delta  = rawAvg - g_tareOffset;
    const long minDeltaCounts = computeAdaptiveMinDeltaCounts(stats.noiseStdCounts);

    if (stats.invalidCount >= invalidFailThreshold(CAL_SAMPLE_COUNT)) {
        server.send(503, "application/json",
            "{\"success\":false,\"message\":\"HX711 timeout during calibration sampling. Check wiring/power and retry\"}");
        return;
    }
    if (stats.zeroCount >= zeroFailThreshold(CAL_SAMPLE_COUNT)) {
        server.send(503, "application/json",
            "{\"success\":false,\"message\":\"HX711 samples are all zero. Check A+/A- bridge wiring and DOUT line\"}");
        return;
    }

    const long deltaAbs = (delta >= 0L) ? delta : -delta;
    if (deltaAbs < minDeltaCounts) {
        server.send(400, "application/json",
            "{\"success\":false,\"message\":\"ADC delta too small for calibration. Tare empty scale, then place known weight and retry\"}");
        return;
    }

    g_calFactor = static_cast<float>(delta) / knownLbs;
    scale.set_scale(g_calFactor);
    saveCalibration();

    Serial.printf("[Scale] New cal factor: %.4f  (%.3f lbs)\n", g_calFactor, knownLbs);

    char json[128];
    snprintf(json, sizeof(json),
             "{\"success\":true,\"message\":\"Calibration saved\",\"factor\":%.4f}",
             g_calFactor);
    server.send(200, "application/json", json);
}

/**
 * @brief Handles HTTP POST /api/guidedcal/start.
 * @details Validates known weight and initializes guided calibration state.
 */
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

/**
 * @brief Handles HTTP POST /api/guidedcal/next.
 * @details Advances guided calibration state machine, capturing tare and then
 * known-weight sample on successive calls.
 */
static void handleApiGuidedCalNext() {
    if (g_guidedCal.step == GUIDED_IDLE) {
        server.send(400, "application/json",
            "{\"success\":false,\"message\":\"No guided calibration in progress\"}");
        return;
    }

    if (g_guidedCal.step == GUIDED_WAIT_EMPTY_CONFIRM) {
        scale.tare(TARE_SAMPLE_COUNT);
        g_tareOffset = scale.get_offset();
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
        const RawSampleStats stats = captureRawSampleStats(CAL_SAMPLE_COUNT);
        const long rawAvg = stats.rawAvg;
        const long delta = rawAvg - g_tareOffset;
        const float noiseStdCounts = stats.noiseStdCounts;
        const long minDeltaCounts = computeAdaptiveMinDeltaCounts(noiseStdCounts);

        if (stats.invalidCount >= invalidFailThreshold(CAL_SAMPLE_COUNT)) {
            server.send(503, "application/json",
                "{\"success\":false,\"message\":\"HX711 timeout while reading known weight. Check wiring/power and retry\"}");
            return;
        }
        if (stats.zeroCount >= zeroFailThreshold(CAL_SAMPLE_COUNT)) {
            server.send(503, "application/json",
                "{\"success\":false,\"message\":\"HX711 known-weight samples are all zero. Check A+/A- bridge wiring and DOUT line\"}");
            return;
        }

        const long deltaAbs = (delta >= 0L) ? delta : -delta;
        if (deltaAbs < minDeltaCounts) {
            server.send(400, "application/json",
                "{\"success\":false,\"message\":\"ADC delta too small. Re-tare empty platform, then place known weight\"}");
            return;
        }

        g_calFactor = static_cast<float>(delta) / g_guidedCal.knownLbs;
        scale.set_offset(g_tareOffset);
        scale.set_scale(g_calFactor);
        saveCalibration();

        const float verifyLbs = static_cast<float>(scale.get_units(CAL_SAMPLE_COUNT));
        const float errPct = ((verifyLbs - g_guidedCal.knownLbs) / g_guidedCal.knownLbs) * 100.0f;

        Serial.printf("[Scale] Web guided calibration done. factor=%.4f verify=%.3f err=%.2f%%\n", g_calFactor, verifyLbs, errPct);

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

/**
 * @brief Handles HTTP GET /api/guidedcal/status.
 * @details Returns current guided calibration activity/state as JSON.
 */
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

/**
 * @brief Handles HTTP POST /api/guidedcal/cancel.
 * @details Cancels and resets guided calibration state.
 */
static void handleApiGuidedCalCancel() {
    resetGuidedCal();
    server.send(200, "application/json",
        "{\"success\":true,\"active\":false,\"message\":\"Guided calibration cancelled\"}");
}

/**
 * @brief Handles HTTP GET /api/health liveness check.
 */
static void handleApiHealth() {
    char json[64];
    snprintf(json, sizeof(json),
             "{\"status\":\"ok\",\"uptime_s\":%lu}", millis() / 1000UL);
    server.send(200, "application/json", json);
}

/**
 * @brief Handles unmatched HTTP routes.
 */
static void handleNotFound() {
    server.send(404, "text/plain", "Not Found");
}

/**
 * @brief Configures all web routes and starts the HTTP server.
 */
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
/**
 * @brief Processes serial commands for diagnostics and calibration control.
 * @details Handles command parsing and dispatch for tare, calibration, reset,
 * raw/status reporting, and help output.
 */
static void handleSerialCommands() {
    if (g_manualCalActive) {
        handleManualCalFactor();
        return;
    }

    if (!Serial.available()) return;

    String raw = Serial.readStringUntil('\n');
    raw.trim();
    String cmd = raw;
    cmd.toUpperCase();

    if (cmd == F("TARE")) {
        // ── TARE — zero the scale ─────────────────────────────────────
        Serial.printf("[Scale] Taring (%u samples)...\n", static_cast<unsigned>(TARE_SAMPLE_COUNT));
        scale.tare(TARE_SAMPLE_COUNT);
        g_tareOffset = scale.get_offset();
        g_weightLbs  = 0.0f;
        saveCalibration();
        Serial.printf("[Scale] Tare offset: %ld\n", g_tareOffset);
        if (g_tareOffset == 0L) {
            Serial.println(F("[Warn] Tare offset is zero — HX711 may be unresponsive or DOUT stuck LOW."));
            Serial.println(F("[Hint] Check DOUT line state and load-cell bridge wiring (A+/A-/E+/E-)."));
        }

    } else if (cmd.startsWith("CALIBRATE ")) {
        // ── CALIBRATE <lbs> — apply calibration with known weight ─────
        float knownLbs = raw.substring(10).toFloat();
        if (knownLbs <= 0.0f) {
            Serial.println(F("[Error] Usage: CALIBRATE <weight_in_lbs>"));
            return;
        }
        Serial.printf("[Scale] Reading raw average (%u samples) for %.2f lbs...\n", static_cast<unsigned>(CAL_SAMPLE_COUNT), knownLbs);
        const RawSampleStats stats = captureRawSampleStats(CAL_SAMPLE_COUNT);
        long rawAvg = stats.rawAvg;
        long delta  = rawAvg - g_tareOffset;
        const long minDeltaCounts = computeAdaptiveMinDeltaCounts(stats.noiseStdCounts);
        if (stats.invalidCount >= invalidFailThreshold(CAL_SAMPLE_COUNT)) {
            Serial.println(F("[Error] HX711 timeout while sampling for CALIBRATE."));
            Serial.println(F("[Hint] Check wiring/power and retry. Use GUIDEDCAL for step-by-step diagnostics."));
            return;
        }
        if (stats.zeroCount >= zeroFailThreshold(CAL_SAMPLE_COUNT)) {
            Serial.println(F("[Error] HX711 CALIBRATE samples are all zero (or nearly all zero)."));
            Serial.println(F("[Hint] Check load-cell bridge wiring (A+/A-/E+/E-) and DOUT line state."));
            return;
        }
        long deltaAbs = (delta >= 0L) ? delta : -delta;
        if (deltaAbs < minDeltaCounts) {
            Serial.printf("[Error] ADC delta too small for calibration (delta=%ld, min=%ld).\n", delta, minDeltaCounts);
            Serial.println(F("[Hint] Run TARE with empty platform first, then CALIBRATE with known weight."));
            return;
        }
        g_calFactor = static_cast<float>(delta) / knownLbs;
        scale.set_scale(g_calFactor);
        saveCalibration();
        Serial.printf("[Scale] Cal factor: %.4f\n", g_calFactor);
        Serial.printf("[Scale] Verification read: %.3f lbs\n", static_cast<float>(scale.get_units(CAL_SAMPLE_COUNT)));

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

    } else if (cmd.startsWith("CALFACTOR ")) {
        float targetLbs = raw.substring(10).toFloat();
        if (targetLbs <= 0.0f) {
            Serial.println(F("[Error] Usage: CALFACTOR <known_weight_lbs>"));
            return;
        }
        startManualCalFactor(targetLbs);

    } else if (cmd == F("CALFACTOR")) {
        startManualCalFactor(0.0f);

    } else if (cmd == F("RESET")) {
        // ── RESET — restore factory calibration defaults ──────────────
        g_calFactor  = DEFAULT_CAL_FACTOR;
        g_tareOffset = 0L;
        scale.set_scale(g_calFactor);
        scale.set_offset(g_tareOffset);
        saveCalibration();
        Serial.println(F("[Scale] Calibration reset to factory defaults."));

    } else if (cmd == F("RAW")) {
        // ── RAW — print a single raw ADC reading ─────────────────────
        long raw24 = scale.read();
        if (raw24 == LONG_MIN) {
            Serial.println(F("[Scale] Raw ADC: TIMEOUT (HX711 not ready within 1 s)"));
            return;
        }
        Serial.printf("[Scale] Raw ADC: %ld  (0x%06lX)\n", raw24, raw24 & 0xFFFFFFL);

    } else if (cmd == F("WEIGHT")) {
        // ── WEIGHT — print current weight ─────────────────────────────
        float w = static_cast<float>(scale.get_units(4));
        Serial.printf("[Scale] Weight: %.3f lbs\n", w);

    } else if (cmd == F("STATUS")) {
        // ── STATUS — print all runtime info ──────────────────────────
        float propaneLbs = g_weightLbs - EMPTY_TANK_WEIGHT_LBS;
        if (propaneLbs < 0.0f) propaneLbs = 0.0f;
        float propanePct = (propaneLbs / PROPANE_CAPACITY_LBS) * 100.0f;

        Serial.println(F("────────────────────────────────────────"));
        Serial.printf("[Scale]  weight=%.3f lbs  propane=%.3f lbs  level=%.1f%%\n", g_weightLbs, propaneLbs, propanePct);
        Serial.printf("[Scale]  cal factor=%.4f  tare offset=%ld\n", g_calFactor, g_tareOffset);
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
        Serial.println(F("  CALFACTOR [lbs]       — interactive calibration-factor tuning mode"));
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
/**
 * @brief Arduino setup entry point.
 * @details Initializes serial, calibration state, HX711, diagnostics, WiFi,
 * mDNS, and web server.
 */
void setup() {
    Serial.begin(115200);
    delay(200);

    Serial.println(F("\n========================================"));
    Serial.println(F("  ESP32 Propane Scale  v1.0.0"));
    Serial.println(F("  SparkFun Thing + HX711 + 4x SEN-13329"));
    Serial.println(F("========================================"));

    // Restore calibration from EEPROM (or use defaults)
    loadCalibration();

    // Initialise HX711
    scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
    scale.set_scale(g_calFactor);
    scale.set_offset(g_tareOffset);
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

    String ip = g_isApMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    Serial.printf("[Boot]  Web UI  →  http://%s/\n", ip.c_str());
    Serial.println(F("[Boot]  Serial  →  type HELP for commands"));
    Serial.println(F("========================================\n"));
}

// ============================================================
// loop()
// ============================================================
/**
 * @brief Arduino main loop.
 * @details Services HTTP and serial handlers, and periodically refreshes
 * weight readings when not in manual calibration mode.
 */
void loop() {
    server.handleClient();
    handleSerialCommands();

    if (g_manualCalActive) {
        g_weightLbs = static_cast<float>(scale.get_units(NUM_SAMPLES));
        return;
    }

    // Periodic weight reading
    if (millis() - g_lastReadMs >= READ_INTERVAL_MS) {
        g_weightLbs  = static_cast<float>(scale.get_units(NUM_SAMPLES));
        g_lastReadMs = millis();
    }
}

