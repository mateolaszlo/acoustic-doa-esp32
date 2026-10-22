#pragma once
#include <Arduino.h>
#include "config/app_config.h"

static constexpr char DEFAULT_WIFI_SSID[]     = "";
static constexpr char DEFAULT_WIFI_PASSWORD[] = "";
static constexpr char DEFAULT_API_URL[]       = "";
static constexpr char DEFAULT_API_KEY[]       = "";

struct DeviceConfig {
    String wifi_ssid                    = DEFAULT_WIFI_SSID;
    String wifi_password                = DEFAULT_WIFI_PASSWORD;
    String api_url                      = DEFAULT_API_URL;
    String api_key                      = DEFAULT_API_KEY;
    uint32_t device_id                  = 0;
    float microphone_distance_m         = DEFAULT_MIC_DISTANCE_M;
    float correlation_threshold         = DEFAULT_CORRELATION_THRESHOLD;
    float sound_level_threshold_db_spl  = DEFAULT_SOUND_LEVEL_THRESHOLD_DB_SPL;
    float db_spl_offset                 = DEFAULT_DB_SPL_OFFSET;
    float angle_offset_deg              = DEFAULT_ANGLE_OFFSET_DEG;
    uint32_t sample_rate                = DEFAULT_SAMPLE_RATE;
    uint32_t window_size                = DEFAULT_WINDOW_SIZE;
    bool adaptive_threshold_enabled     = DEFAULT_ADAPTIVE_THRESHOLD_ENABLED;
    float snr_margin_db                 = DEFAULT_SNR_MARGIN_DB;
};
