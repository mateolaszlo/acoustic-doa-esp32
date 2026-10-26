#include "ui/serial_ui.h"
#include "config/app_config.h"
#include <Wire.h>

void SerialUI::printI2cScan() {
    bool found_device = false;
    Serial.println("I2C scan:");
    for (uint8_t address = 1; address < 127; ++address) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  Found device at 0x%02X\n", address);
            found_device = true;
        }
    }
    if (!found_device) {
        Serial.println("  No I2C devices found");
    }
}

void SerialUI::printSelfTestReport(const BootSelfTestResult &res, uint32_t sample_rate) {
    Serial.println("\n==================================================");
    Serial.println("           ACOUSTIC LOCATOR BOOT SELF-TEST        ");
    Serial.println("==================================================");
    Serial.printf("[%s] Octal PSRAM : %s (%u KB)\n",
                  res.psram_ok ? "PASS" : "FAIL",
                  res.psram_ok ? "Found" : "Missing",
                  static_cast<unsigned>(res.psram_bytes / 1024));

    if (res.bme_ok) {
        Serial.printf("[PASS] BME280 I2C  : OK (T: %.1fC, P: %.1f hPa)\n",
                      res.bme_temp_c, res.bme_pres_hpa);
    } else {
        Serial.printf("[WARN] BME280 I2C  : NOT FOUND (Using default c=%.1f m/s)\n",
                      DEFAULT_SOUND_SPEED_MPS);
    }

    Serial.printf("[%s] I2S Driver  : %s (%u Hz Dual-Bus Quad)\n",
                  res.i2s_driver_ok ? "PASS" : "FAIL",
                  res.i2s_driver_ok ? "Initialized (I2S0 Master + I2S1 Slave)" : "Failed",
                  static_cast<unsigned>(sample_rate));

    static const char* const mic_labels[AUDIO_NUM_CHANNELS] = {
        "M0 (RB/I2S0L)", "M1 (RF/I2S0R)", "M2 (LF/I2S1L)", "M3 (LB/I2S1R)"
    };
    for (size_t c = 0; c < AUDIO_NUM_CHANNELS; ++c) {
        const bool ok = res.mic_health.ch_ok[c];
        const bool is_flatline = (res.mic_health.ch_min[c] == res.mic_health.ch_max[c]) ||
                                 (res.mic_health.ch_rms_db[c] <= POST_MIC_FLATLINE_RMS_DB);
        const char* state = ok
            ? "ACTIVE"
            : (is_flatline ? "FLATLINE" : "RAIL SHORT");
        Serial.printf("[%s] %-13s: %s (RMS: %.1f dBFS, min=%ld, max=%ld)\n",
                      ok ? "PASS" : "FAIL", mic_labels[c], state,
                      res.mic_health.ch_rms_db[c],
                      static_cast<long>(res.mic_health.ch_min[c]),
                      static_cast<long>(res.mic_health.ch_max[c]));
    }

    const bool sync_ok = res.i2s_driver_ok &&
                         res.mic_health.bus0_distinct &&
                         res.mic_health.bus1_distinct &&
                         res.mic_health.cross_bus_distinct;
    Serial.printf("[%s] Bus Sync    : Bus0=%s, Bus1=%s, CrossBus=%s (max identical=%u/%u)\n",
                  sync_ok ? "PASS" : "FAIL",
                  res.mic_health.bus0_distinct ? "OK" : "COLLISION",
                  res.mic_health.bus1_distinct ? "OK" : "COLLISION",
                  res.mic_health.cross_bus_distinct ? "OK" : "SHORTED",
                  static_cast<unsigned>(res.mic_health.identical_samples),
                  static_cast<unsigned>(POST_SAMPLE_COUNT));

    Serial.println("==================================================");
    Serial.printf("STATUS: %s\n", res.all_nominal ? "ALL SYSTEMS NOMINAL" : "HARDWARE DEGRADED - CHECK WIRING");
    Serial.println("==================================================\n");
}

void SerialUI::printHardwareDegradedAlert() {
    Serial.println("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("  CRITICAL: BOOT SELF-TEST FAILED - HALTING!");
    Serial.println("  Real-time DOA processing and network streaming");
    Serial.println("  are suspended to prevent corrupt measurements.");
    Serial.println("  Fix hardware wiring and press RST to reboot,");
    Serial.println("  or hold BOOT (GPIO 0) to enter setup mode.");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
}

void SerialUI::printDoaEvent(float angle_deg, float confidence, float calibrated_db,
                             uint8_t source_idx, size_t total_sources) {
    if (total_sources > 1) {
        Serial.printf("[Src %u/%u] Detected angle: %.1f deg, R: %.2f, dB: %.1f\n",
                      static_cast<unsigned>(source_idx + 1),
                      static_cast<unsigned>(total_sources),
                      angle_deg, confidence, calibrated_db);
    } else {
        Serial.printf("Detected angle: %.1f deg, R: %.2f, dB: %.1f\n",
                      angle_deg, confidence, calibrated_db);
    }
}

void SerialUI::printDspDiagnostics(uint32_t blocks, uint32_t misses, uint32_t max_us,
                                  uint32_t budget_us, uint32_t i2s_fails,
                                  uint32_t disp_drops, uint32_t net_drops,
                                  unsigned int stack_free) {
    Serial.printf(
        "DSP stats: blocks=%lu, deadline_misses=%lu, max_processing=%lu us, "
        "budget=%lu us, i2s_failures=%lu, display_drops=%lu, network_drops=%lu, "
        "stack_free=%u\n",
        static_cast<unsigned long>(blocks),
        static_cast<unsigned long>(misses),
        static_cast<unsigned long>(max_us),
        static_cast<unsigned long>(budget_us),
        static_cast<unsigned long>(i2s_fails),
        static_cast<unsigned long>(disp_drops),
        static_cast<unsigned long>(net_drops),
        stack_free);
}

namespace {
constexpr uint32_t WIFI_THROTTLE_MS = 15000;
constexpr uint32_t API_ERROR_THROTTLE_MS = 15000;
constexpr uint32_t API_BATCH_SUMMARY_COUNT = 50;
constexpr uint32_t API_BATCH_SUMMARY_INTERVAL_MS = 10000;

static uint8_t s_wifi_last_reason = 255;
static uint32_t s_wifi_disconnect_count = 0;
static uint32_t s_wifi_last_disconnect_log_ms = 0;
static uint32_t s_wifi_reconnect_last_log_ms = 0;

static uint32_t s_api_batch_success_count = 0;
static uint32_t s_api_batch_last_success_log_ms = 0;
static bool s_api_batch_is_streaming = false;
static int s_api_batch_last_error_code = 0;
static uint32_t s_api_batch_last_error_log_ms = 0;
static uint32_t s_api_batch_error_count = 0;
} // namespace

WiFiDisconnectDiagnosis SerialUI::diagnoseWiFiDisconnect(uint8_t reason_code) {
    WiFiDisconnectDiagnosis diag;
    diag.reason_code = reason_code;
    diag.is_wrong_password = false;
    diag.is_wrong_ssid = false;

    switch (reason_code) {
        case 1:
            diag.reason_name = "UNSPECIFIED";
            diag.description = "Unspecified error";
            break;
        case 2:
            diag.reason_name = "AUTH_EXPIRE";
            diag.description = "Authentication expired";
            break;
        case 3:
            diag.reason_name = "AUTH_LEAVE";
            diag.description = "Deauthenticated by AP";
            break;
        case 4:
            diag.reason_name = "ASSOC_EXPIRE";
            diag.description = "Association expired";
            break;
        case 5:
            diag.reason_name = "ASSOC_TOOMANY";
            diag.description = "AP full (too many clients)";
            break;
        case 6:
            diag.reason_name = "NOT_AUTHED";
            diag.description = "Not authenticated";
            break;
        case 7:
            diag.reason_name = "NOT_ASSOCED";
            diag.description = "Not associated";
            break;
        case 8:
            diag.reason_name = "ASSOC_LEAVE";
            diag.description = "Disassociated by AP";
            break;
        case 15:
            diag.reason_name = "4WAY_HANDSHAKE_TIMEOUT";
            diag.description = "4-way handshake timeout (incorrect Wi-Fi password)";
            diag.is_wrong_password = true;
            break;
        case 200:
            diag.reason_name = "BEACON_TIMEOUT";
            diag.description = "Beacon timeout (connection lost / signal too weak)";
            break;
        case 201:
            diag.reason_name = "NO_AP_FOUND";
            diag.description = "SSID not found (incorrect SSID name or out of range)";
            diag.is_wrong_ssid = true;
            break;
        case 202:
            diag.reason_name = "AUTH_FAIL";
            diag.description = "Authentication failed (incorrect Wi-Fi password)";
            diag.is_wrong_password = true;
            break;
        case 203:
            diag.reason_name = "ASSOC_FAIL";
            diag.description = "Association failed";
            break;
        case 204:
            diag.reason_name = "HANDSHAKE_TIMEOUT";
            diag.description = "Handshake failed (incorrect Wi-Fi password)";
            diag.is_wrong_password = true;
            break;
        case 205:
            diag.reason_name = "CONNECTION_FAIL";
            diag.description = "Connection failed";
            break;
        default:
            diag.reason_name = "UNKNOWN";
            diag.description = "Unknown disconnect reason";
            break;
    }
    return diag;
}

void SerialUI::printWiFiConnecting(const char* ssid) {
    Serial.printf("[WiFi] Connecting to SSID: '%s'...\n", ssid ? ssid : "");
}

void SerialUI::printWiFiConnected(const char* ssid, const char* ip, int rssi, int channel) {
    s_wifi_disconnect_count = 0;
    s_wifi_last_reason = 255;
    s_wifi_last_disconnect_log_ms = 0;
    s_wifi_reconnect_last_log_ms = 0;

    if (channel > 0) {
        Serial.printf("[WiFi] Connected to '%s'! IP: %s, RSSI: %d dBm, Channel: %d\n",
                      ssid ? ssid : "", ip ? ip : "", rssi, channel);
    } else {
        Serial.printf("[WiFi] Connected to '%s'! IP: %s, RSSI: %d dBm\n",
                      ssid ? ssid : "", ip ? ip : "", rssi);
    }
}

void SerialUI::printWiFiDisconnected(uint8_t reason_code, uint32_t now_ms) {
    if (now_ms == 0) {
        now_ms = millis();
    }
    const WiFiDisconnectDiagnosis diag = diagnoseWiFiDisconnect(reason_code);
    s_wifi_disconnect_count++;

    const bool is_new_reason = (reason_code != s_wifi_last_reason);
    const bool is_first = (s_wifi_disconnect_count == 1);
    const bool throttle_expired = (now_ms - s_wifi_last_disconnect_log_ms >= WIFI_THROTTLE_MS);

    if (is_new_reason || is_first) {
        s_wifi_last_reason = reason_code;
        s_wifi_last_disconnect_log_ms = now_ms;
        if (diag.is_wrong_password) {
            Serial.printf("[WiFi] Disconnected: %s (reason %u: %s). Check Wi-Fi password in setup!\n",
                          diag.description, diag.reason_code, diag.reason_name);
        } else if (diag.is_wrong_ssid) {
            Serial.printf("[WiFi] Disconnected: %s (reason %u: %s). Check SSID name or router distance!\n",
                          diag.description, diag.reason_code, diag.reason_name);
        } else {
            Serial.printf("[WiFi] Disconnected: %s (reason %u: %s)\n",
                          diag.description, diag.reason_code, diag.reason_name);
        }
    } else if (throttle_expired) {
        s_wifi_last_disconnect_log_ms = now_ms;
        Serial.printf("[WiFi] Still disconnected: %s (reason %u: %s, %lu attempts)\n",
                      diag.description, diag.reason_code, diag.reason_name,
                      static_cast<unsigned long>(s_wifi_disconnect_count));
    }
}

void SerialUI::printWiFiReconnecting(const char* ssid, uint32_t attempt, uint32_t now_ms) {
    if (now_ms == 0) {
        now_ms = millis();
    }
    if (attempt == 1 || now_ms - s_wifi_reconnect_last_log_ms >= WIFI_THROTTLE_MS) {
        s_wifi_reconnect_last_log_ms = now_ms;
        Serial.printf("[WiFi] Reconnecting to SSID: '%s' (attempt %lu)...\n",
                      ssid ? ssid : "", static_cast<unsigned long>(attempt));
    }
}

void SerialUI::printApiCalibrationSuccess(uint32_t device_id, uint32_t sample_rate,
                                         float distance_m, float corr_threshold) {
    Serial.printf("[API] Calibration synchronized (HTTP 200): device_id=%lu, sample_rate=%lu Hz, mic_distance=%.3f m, corr_threshold=%.2f\n",
                  static_cast<unsigned long>(device_id),
                  static_cast<unsigned long>(sample_rate),
                  distance_m, corr_threshold);
}

void SerialUI::printApiCalibrationError(int http_code, const char* response, uint32_t device_id) {
    if (http_code == 401 || http_code == 403) {
        Serial.printf("[API] CRITICAL: Calibration rejected (HTTP %d)! Invalid or unauthorized API key for device ID %lu.\n",
                      http_code, static_cast<unsigned long>(device_id));
    } else if (http_code == 404) {
        Serial.printf("[API] ERROR: Device ID %lu not found on server (HTTP 404).\n",
                      static_cast<unsigned long>(device_id));
    } else if (http_code < 0) {
        Serial.printf("[API] ERROR: Calibration connection failed (code %d): Server unreachable\n",
                      http_code);
    } else {
        Serial.printf("[API] ERROR: Calibration GET failed (HTTP %d): %s\n",
                      http_code, response ? response : "");
    }
}

void SerialUI::printApiBatchSuccess(int http_code, size_t batch_bytes, uint32_t now_ms) {
    if (now_ms == 0) {
        now_ms = millis();
    }
    s_api_batch_success_count++;
    s_api_batch_error_count = 0;

    if (!s_api_batch_is_streaming) {
        s_api_batch_is_streaming = true;
        s_api_batch_last_success_log_ms = now_ms;
        Serial.printf("[API] Measurement stream active (HTTP %d, batch size: %u bytes)\n",
                      http_code, static_cast<unsigned>(batch_bytes));
    } else if (s_api_batch_success_count % API_BATCH_SUMMARY_COUNT == 0 ||
               now_ms - s_api_batch_last_success_log_ms >= API_BATCH_SUMMARY_INTERVAL_MS) {
        s_api_batch_last_success_log_ms = now_ms;
        Serial.printf("[API] Measurement streaming nominal: %lu batches delivered (HTTP %d)\n",
                      static_cast<unsigned long>(s_api_batch_success_count), http_code);
    }
}

void SerialUI::printApiBatchError(int http_code, const char* response, uint32_t now_ms) {
    if (now_ms == 0) {
        now_ms = millis();
    }
    s_api_batch_is_streaming = false;
    s_api_batch_error_count++;

    const bool is_new_error = (http_code != s_api_batch_last_error_code);
    const bool is_first = (s_api_batch_error_count == 1);
    const bool throttle_expired = (now_ms - s_api_batch_last_error_log_ms >= API_ERROR_THROTTLE_MS);

    if (is_new_error || is_first || throttle_expired) {
        s_api_batch_last_error_code = http_code;
        s_api_batch_last_error_log_ms = now_ms;

        if (http_code == 401 || http_code == 403) {
            Serial.printf("[API] CRITICAL: Measurement batch rejected (HTTP %d)! Invalid or unauthorized API key.\n",
                          http_code);
        } else if (http_code == 404) {
            Serial.println("[API] ERROR: POST /measurements not found (HTTP 404). Check api_url in setup.");
        } else if (http_code == 400) {
            Serial.printf("[API] ERROR: Measurement batch rejected (HTTP 400): %s\n",
                          response ? response : "");
        } else if (http_code < 0) {
            Serial.printf("[API] ERROR: Measurement upload failed (code %d): Server unreachable\n",
                          http_code);
        } else {
            Serial.printf("[API] ERROR: Measurement POST failed (HTTP %d): %s\n",
                          http_code, response ? response : "");
        }
    }
}

void SerialUI::resetThrottles() {
    s_wifi_last_reason = 255;
    s_wifi_disconnect_count = 0;
    s_wifi_last_disconnect_log_ms = 0;
    s_wifi_reconnect_last_log_ms = 0;

    s_api_batch_success_count = 0;
    s_api_batch_last_success_log_ms = 0;
    s_api_batch_is_streaming = false;
    s_api_batch_last_error_code = 0;
    s_api_batch_last_error_log_ms = 0;
    s_api_batch_error_count = 0;
}
