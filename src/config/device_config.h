#pragma once
#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <string>
using String = std::string;
#endif
#include <cmath>
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

inline bool validate_device_config(const DeviceConfig &config) {
    if (!std::isfinite(config.microphone_distance_m) ||
        config.microphone_distance_m < MIN_MIC_DISTANCE_M ||
        config.microphone_distance_m > MAX_MIC_DISTANCE_M) {
        return false;
    }
    if (!std::isfinite(config.correlation_threshold) ||
        config.correlation_threshold < CORRELATION_THRESHOLD_MIN ||
        config.correlation_threshold > CORRELATION_THRESHOLD_MAX) {
        return false;
    }
    if (!std::isfinite(config.sound_level_threshold_db_spl) ||
        config.sound_level_threshold_db_spl < SPL_THRESHOLD_MIN_DB ||
        config.sound_level_threshold_db_spl > SPL_THRESHOLD_MAX_DB) {
        return false;
    }
    if (!std::isfinite(config.db_spl_offset) ||
        config.db_spl_offset < DB_SPL_OFFSET_MIN_DB ||
        config.db_spl_offset > DB_SPL_OFFSET_MAX_DB) {
        return false;
    }
    if (!std::isfinite(config.angle_offset_deg) ||
        config.angle_offset_deg < ANGLE_OFFSET_MIN_DEG ||
        config.angle_offset_deg > ANGLE_OFFSET_MAX_DEG) {
        return false;
    }
    if (!std::isfinite(config.snr_margin_db) ||
        config.snr_margin_db < SNR_MARGIN_MIN_DB ||
        config.snr_margin_db > SNR_MARGIN_MAX_DB) {
        return false;
    }
    if (!is_supported_audio_config(config.sample_rate, config.window_size)) {
        return false;
    }
    return true;
}

inline void sanitize_device_config(DeviceConfig &config) {
    if (!std::isfinite(config.microphone_distance_m) ||
        config.microphone_distance_m < MIN_MIC_DISTANCE_M ||
        config.microphone_distance_m > MAX_MIC_DISTANCE_M) {
        config.microphone_distance_m = DEFAULT_MIC_DISTANCE_M;
    }
    if (!std::isfinite(config.correlation_threshold) ||
        config.correlation_threshold < CORRELATION_THRESHOLD_MIN ||
        config.correlation_threshold > CORRELATION_THRESHOLD_MAX) {
        config.correlation_threshold = DEFAULT_CORRELATION_THRESHOLD;
    }
    if (!std::isfinite(config.sound_level_threshold_db_spl) ||
        config.sound_level_threshold_db_spl < SPL_THRESHOLD_MIN_DB ||
        config.sound_level_threshold_db_spl > SPL_THRESHOLD_MAX_DB) {
        config.sound_level_threshold_db_spl = DEFAULT_SOUND_LEVEL_THRESHOLD_DB_SPL;
    }
    if (!std::isfinite(config.db_spl_offset) ||
        config.db_spl_offset < DB_SPL_OFFSET_MIN_DB ||
        config.db_spl_offset > DB_SPL_OFFSET_MAX_DB) {
        config.db_spl_offset = DEFAULT_DB_SPL_OFFSET;
    }
    if (!std::isfinite(config.angle_offset_deg) ||
        config.angle_offset_deg < ANGLE_OFFSET_MIN_DEG ||
        config.angle_offset_deg > ANGLE_OFFSET_MAX_DEG) {
        config.angle_offset_deg = DEFAULT_ANGLE_OFFSET_DEG;
    }
    if (!std::isfinite(config.snr_margin_db) ||
        config.snr_margin_db < SNR_MARGIN_MIN_DB ||
        config.snr_margin_db > SNR_MARGIN_MAX_DB) {
        config.snr_margin_db = DEFAULT_SNR_MARGIN_DB;
    }
    if (config.sample_rate != 44100 && config.sample_rate != 48000) {
        config.sample_rate = DEFAULT_SAMPLE_RATE;
    }
    config.window_size = DEFAULT_WINDOW_SIZE;
}

inline bool is_device_configured(const DeviceConfig &config) {
    return config.wifi_ssid.length() > 0 &&
           config.api_url.length() > 0 &&
           config.api_key.length() > 0 &&
           config.device_id != 0;
}

