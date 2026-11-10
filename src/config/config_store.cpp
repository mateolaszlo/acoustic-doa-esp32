#include "config/config_store.h"
#include <Preferences.h>
#include <cmath>

static Preferences preferences;

namespace {
class PreferencesScope {
public:
    PreferencesScope(Preferences &prefs, const char *name, bool read_only)
        : prefs_(prefs), opened_(prefs.begin(name, read_only)) {}

    ~PreferencesScope() {
        if (opened_) {
            prefs_.end();
        }
    }

    bool isOpen() const { return opened_; }

private:
    Preferences &prefs_;
    bool opened_;
};
}

static String get_preference_string(const char *key, const char *fallback) {
    return preferences.isKey(key) ? preferences.getString(key) : String(fallback);
}

static float get_preference_float(const char *key, float fallback) {
    return preferences.isKey(key) ? preferences.getFloat(key) : fallback;
}

static inline void put_preference_string_if_changed(const char *key, const String &val) {
    if (!preferences.isKey(key) || preferences.getString(key) != val) {
        preferences.putString(key, val);
    }
}

static inline void put_preference_float_if_changed(const char *key, float val, float epsilon = 1e-4f) {
    if (!preferences.isKey(key) || !float_equal(preferences.getFloat(key), val, epsilon)) {
        preferences.putFloat(key, val);
    }
}

static inline void put_preference_uint_if_changed(const char *key, uint32_t val) {
    if (!preferences.isKey(key) || preferences.getUInt(key) != val) {
        preferences.putUInt(key, val);
    }
}

static inline void put_preference_bool_if_changed(const char *key, bool val) {
    if (!preferences.isKey(key) || preferences.getBool(key) != val) {
        preferences.putBool(key, val);
    }
}

bool load_device_config(DeviceConfig &config) {
    PreferencesScope scope(preferences, "acoustic", false);
    if (!scope.isOpen()) {
        return false;
    }

    config.wifi_ssid = get_preference_string("wifi_ssid", config.wifi_ssid.c_str());
    config.wifi_password = get_preference_string("wifi_pass", config.wifi_password.c_str());
    config.api_url = get_preference_string("api_url", config.api_url.c_str());
    config.api_key = get_preference_string("api_key", config.api_key.c_str());
    config.device_id = preferences.getUInt("device_id", config.device_id);
    config.microphone_distance_m = get_preference_float("mic_distance", config.microphone_distance_m);
    config.correlation_threshold = get_preference_float("corr_threshold", config.correlation_threshold);
    config.sound_level_threshold_db_spl = get_preference_float("spl_threshold", config.sound_level_threshold_db_spl);
    config.db_spl_offset = get_preference_float("spl_offset", config.db_spl_offset);
    config.angle_offset_deg = get_preference_float("ang_offset", config.angle_offset_deg);
    config.sample_rate = preferences.getUInt("sample_rate", config.sample_rate);
    config.window_size = DEFAULT_WINDOW_SIZE;
    config.adaptive_threshold_enabled =
        preferences.isKey("adapt_thresh") ? preferences.getBool("adapt_thresh") : config.adaptive_threshold_enabled;
    config.snr_margin_db = get_preference_float("snr_margin", config.snr_margin_db);
    config.sector_width_deg = get_preference_float("sec_width", config.sector_width_deg);
    config.sector_hysteresis_deg = get_preference_float("sec_hyst", config.sector_hysteresis_deg);
    config.tracker_hold_timeout_ms =
        preferences.isKey("trk_hold") ? preferences.getUInt("trk_hold", config.tracker_hold_timeout_ms)
                                      : config.tracker_hold_timeout_ms;

    sanitize_device_config(config);
    return is_device_configured(config);
}

void save_device_config(const DeviceConfig &config) {
    DeviceConfig safe_config = config;
    sanitize_device_config(safe_config);

    PreferencesScope scope(preferences, "acoustic", false);
    if (!scope.isOpen()) {
        return;
    }

    put_preference_string_if_changed("wifi_ssid", safe_config.wifi_ssid);
    put_preference_string_if_changed("wifi_pass", safe_config.wifi_password);
    put_preference_string_if_changed("api_url", safe_config.api_url);
    put_preference_string_if_changed("api_key", safe_config.api_key);
    put_preference_uint_if_changed("device_id", safe_config.device_id);
    put_preference_float_if_changed("mic_distance", safe_config.microphone_distance_m);
    put_preference_float_if_changed("corr_threshold", safe_config.correlation_threshold);
    put_preference_float_if_changed("spl_threshold", safe_config.sound_level_threshold_db_spl);
    put_preference_float_if_changed("spl_offset", safe_config.db_spl_offset);
    put_preference_float_if_changed("ang_offset", safe_config.angle_offset_deg);
    put_preference_uint_if_changed("sample_rate", safe_config.sample_rate);
    put_preference_uint_if_changed("window_size", safe_config.window_size);
    put_preference_bool_if_changed("adapt_thresh", safe_config.adaptive_threshold_enabled);
    put_preference_float_if_changed("snr_margin", safe_config.snr_margin_db);
    put_preference_float_if_changed("sec_width", safe_config.sector_width_deg);
    put_preference_float_if_changed("sec_hyst", safe_config.sector_hysteresis_deg);
    put_preference_uint_if_changed("trk_hold", safe_config.tracker_hold_timeout_ms);
}

