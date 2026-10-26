#pragma once
#include <Arduino.h>
#include <cstdint>
#include "ui/radar_ui.h"

struct WiFiDisconnectDiagnosis {
    uint8_t reason_code;
    const char* reason_name;
    const char* description;
    bool is_wrong_password;
    bool is_wrong_ssid;
};

class SerialUI {
public:
    static void printSelfTestReport(const BootSelfTestResult &res, uint32_t sample_rate);
    static void printHardwareDegradedAlert();
    static void printDoaEvent(float angle_deg, float confidence, float calibrated_db,
                              uint8_t source_idx = 0, size_t total_sources = 1);
    static void printI2cScan();
    static void printDspDiagnostics(uint32_t blocks, uint32_t misses, uint32_t max_us,
                                    uint32_t budget_us, uint32_t i2s_fails,
                                    uint32_t disp_drops, uint32_t net_drops,
                                    unsigned int stack_free);

    // Wi-Fi Telemetry & Diagnostics
    static WiFiDisconnectDiagnosis diagnoseWiFiDisconnect(uint8_t reason_code);
    static void printWiFiConnecting(const char* ssid, int pass_len = -1);
    static void printWiFiConnected(const char* ssid, const char* ip, int rssi, int channel = 0);
    static void printWiFiDisconnected(uint8_t reason_code, uint32_t now_ms = 0);
    static void printWiFiReconnecting(const char* ssid, uint32_t attempt, uint32_t now_ms = 0);

    // REST API Telemetry & Diagnostics
    static void printApiCalibrationSuccess(uint32_t device_id, uint32_t sample_rate,
                                           float distance_m, float corr_threshold);
    static void printApiCalibrationError(int http_code, const char* response, uint32_t device_id);
    static void printApiBatchSuccess(int http_code, size_t batch_bytes, uint32_t now_ms = 0);
    static void printApiBatchError(int http_code, const char* response, uint32_t now_ms = 0);

    // Reset throttling state (useful on successful reconnect or in tests)
    static void resetThrottles();
};
