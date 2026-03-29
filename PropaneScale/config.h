#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// WiFi — Station mode credentials
// ============================================================
#define WIFI_SSID         "YOUR_WIFI_SSID"
#define WIFI_PASSWORD     "YOUR_WIFI_PASSWORD"
#define WIFI_TIMEOUT_MS   30000UL   // 30 s before falling back to AP mode

// Access-Point fallback (when WIFI_SSID is unreachable)
#define AP_SSID           "PropaneScale"
#define AP_PASSWORD       "propane123"  // minimum 8 characters for WPA2

// ============================================================
// Scale hardware configuration
//
// This project is configured for a 4-sensor platform:
//   - 4x SparkFun SEN-13329 (10 kg) straight bar load sensors
//   - 1x SparkFun Load Sensor Combinator (or equivalent wiring)
//   - 1x HX711 load-cell amplifier/ADC
//
// The combinator output is treated as one full Wheatstone bridge by HX711.
// ============================================================
#define LOAD_CELL_COUNT               4U
#define LOAD_CELL_ARRAY_CAPACITY_LBS  88.0f   // 4 x 10 kg ≈ 88.2 lb total

// ============================================================
// HX711 Pin Definitions  — SparkFun ESP8266 Thing
// ============================================================
//
//   HX711 module     SparkFun ESP8266 Thing
//   ─────────────    ──────────────────────
//   DOUT        ──►  GPIO 12  (J2 header, safe general-purpose IO)
//   SCK         ──►  GPIO 13  (J2 header, safe general-purpose IO)
//   VCC         ──►  3.3 V
//   GND         ──►  GND
//
//   Combinator/HX711 colour convention (typical):
//   Red   → E+    Black → E-    White → A+    Green → A-
//
// ============================================================
#define HX711_DOUT_PIN    12   // GPIO12 — data output from HX711
#define HX711_SCK_PIN     13   // GPIO13 — clock input  to  HX711

// ============================================================
// Propane tank configuration  (standard 20-lb BBQ cylinder)
// These values are used to calculate propane level percentage.
// Adjust to match your actual cylinder's stamped tare weight.
// ============================================================
#define EMPTY_TANK_WEIGHT_LBS   17.0f   // Tare weight of empty cylinder (lbs)
#define PROPANE_CAPACITY_LBS    20.0f   // Usable propane when full (lbs)
#define FULL_TANK_WEIGHT_LBS    (EMPTY_TANK_WEIGHT_LBS + PROPANE_CAPACITY_LBS)

// ============================================================
// Sampling
// ============================================================
#define NUM_SAMPLES         4       // Samples averaged per weight reading
                                    // At 10 SPS: 4 samples ≈ 400 ms per read
#define READ_INTERVAL_MS    2000UL  // Milliseconds between weight updates

// ============================================================
// Web server
// ============================================================
#define WEB_SERVER_PORT     80
#define MDNS_HOSTNAME       "propanescale"   // http://propanescale.local/

// ============================================================
// EEPROM — calibration persistence
// Layout (12 bytes total):
//   offset 0 : uint32_t magic   (0xCAFEF00D)
//   offset 4 : float    factor  (raw ADC counts per lb)
//   offset 8 : long     offset  (tare offset in raw ADC counts)
// ============================================================
#define EEPROM_VALID_MAGIC  0xCAFEF00DUL

// ============================================================
// Calibration default (placeholder — MUST run calibration
// procedure before use; see README for instructions)
// ============================================================
#define DEFAULT_CAL_FACTOR  2000.0f   // raw counts per lb (load-cell dependent)

#endif // CONFIG_H
