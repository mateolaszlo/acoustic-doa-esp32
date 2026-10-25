#include "config/config_store.h"
#include <Preferences.h>
#include <cmath>

static Preferences preferences;

static String get_preference_string(const char *key, const char *fallback) {
    return preferences.isKey(key) ? preferences.getString(key) : String(fallback);
}

static float get_preference_float(const char *key, float fallback) {
    return preferences.isKey(key) ? preferences.getFloat(key) : fallback;
}

bool load_device_config(DeviceConfig &config) {
    preferences.begin("acoustic", false);

    config.wifi_ssid = get_preference_string("wifi_ssid", config.wifi_ssid.c_str());
    config.wifi_password = get_preference_string("wifi_pass", config.wifi_password.c_str());
    config.api_url = get_preference_string("api_url", config.api_url.c_str());
    config.api_key = get_preference_string("api_key", config.api_key.c_str());
    config.device_id = preferences.getUInt("device_id", config.device_id);
    config.microphone_distance_m = get_preference_float("mic_distance", config.microphone_distance_m);
    config.correlation_threshold = get_preference_float("corr_threshold", config.correlation_threshold);
    if (!std::isfinite(config.correlation_threshold) || config.correlation_threshold < DEFAULT_CORRELATION_THRESHOLD) {
        config.correlation_threshold = DEFAULT_CORRELATION_THRESHOLD;
    }
    config.sound_level_threshold_db_spl = get_preference_float("spl_threshold", config.sound_level_threshold_db_spl);
    config.db_spl_offset = get_preference_float("spl_offset", config.db_spl_offset);
    config.angle_offset_deg = get_preference_float("ang_offset", config.angle_offset_deg);

    if (!std::isfinite(config.angle_offset_deg) ||
        config.angle_offset_deg < ANGLE_OFFSET_MIN_DEG ||
        config.angle_offset_deg > ANGLE_OFFSET_MAX_DEG) {
        config.angle_offset_deg = DEFAULT_ANGLE_OFFSET_DEG;
    }

    config.sample_rate = preferences.getUInt("sample_rate", config.sample_rate);
    config.window_size = DEFAULT_WINDOW_SIZE;
    config.adaptive_threshold_enabled =
        preferences.isKey("adapt_thresh") ? preferences.getBool("adapt_thresh") : config.adaptive_threshold_enabled;
    config.snr_margin_db = get_preference_float("snr_margin", config.snr_margin_db);

    if (!std::isfinite(config.snr_margin_db) ||
        config.snr_margin_db < SNR_MARGIN_MIN_DB ||
        config.snr_margin_db > SNR_MARGIN_MAX_DB) {
        config.snr_margin_db = DEFAULT_SNR_MARGIN_DB;
    }

    return is_device_configured(config);
}

void save_device_config(const DeviceConfig &config) {
    preferences.putString("wifi_ssid", config.wifi_ssid);
    preferences.putString("wifi_pass", config.wifi_password);
    preferences.putString("api_url", config.api_url);
    preferences.putString("api_key", config.api_key);
    preferences.putUInt("device_id", config.device_id);
    preferences.putFloat("mic_distance", config.microphone_distance_m);
    preferences.putFloat("corr_threshold", config.correlation_threshold);
    preferences.putFloat("spl_threshold", config.sound_level_threshold_db_spl);
    preferences.putFloat("spl_offset", config.db_spl_offset);
    preferences.putFloat("ang_offset", config.angle_offset_deg);
    preferences.putUInt("sample_rate", config.sample_rate);
    preferences.putUInt("window_size", config.window_size);
    preferences.putBool("adapt_thresh", config.adaptive_threshold_enabled);
    preferences.putFloat("snr_margin", config.snr_margin_db);
}

bool is_device_configured(const DeviceConfig &config) {
    return config.wifi_ssid.length() > 0 &&
           config.api_url.length() > 0 &&
           config.api_key.length() > 0 &&
           config.device_id != 0;
}
