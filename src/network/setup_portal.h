#pragma once
#include <functional>
#include <cstdio>
#if defined(ARDUINO)
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include "ui/radar_ui.h"
#endif
#include "config/device_config.h"

class SetupPortal {
public:
    static String escapeHtmlAttr(const String &input) {
        String out;
        out.reserve(input.length() + 16);
        for (size_t i = 0; i < input.length(); ++i) {
            const char c = input[i];
            switch (c) {
                case '&':  out += "&amp;";  break;
                case '\'': out += "&#39;";  break;
                case '"':  out += "&quot;"; break;
                case '<':  out += "&lt;";   break;
                case '>':  out += "&gt;";   break;
                default:   out += c;        break;
            }
        }
        return out;
    }

    static String generateHtmlPage(const DeviceConfig &config) {
        char num_buf[64];
        String page;
        page.reserve(2800);
        page += "<!doctype html><html><head><meta name='viewport' content='width=device-width'>";
        page += "<title>Acoustic Locator Setup</title></head><body>";
        page += "<h1>Acoustic Locator Setup</h1><form method='POST' action='/save'>";
        page += "Wi-Fi SSID<br><input name='wifi_ssid' value='" + escapeHtmlAttr(config.wifi_ssid) + "'><br>";
        const String pass_hint = config.wifi_password.length() > 0 ? " (leave blank to retain stored)" : "";
        page += "Wi-Fi password" + pass_hint + "<br><input type='password' name='wifi_password'><br>";
        page += "API URL<br><input name='api_url' value='" + escapeHtmlAttr(config.api_url) + "'><br>";
        const String api_key_hint = config.api_key.length() > 0 ? " (leave blank to retain stored)" : "";
        page += "API key" + api_key_hint + "<br><input type='password' name='api_key'><br>";

        std::snprintf(num_buf, sizeof(num_buf), "%lu", static_cast<unsigned long>(config.device_id));
        page += "Device ID<br><input type='number' name='device_id' value='";
        page += num_buf;
        page += "'><br>";

        std::snprintf(num_buf, sizeof(num_buf), "%.4f", config.microphone_distance_m);
        page += "Microphone distance (m)<br><input name='mic_distance' value='";
        page += num_buf;
        page += "'><br>";

        std::snprintf(num_buf, sizeof(num_buf), "%.2f", config.db_spl_offset);
        page += "dB SPL offset<br><input name='spl_offset' value='";
        page += num_buf;
        page += "'><br>";

        std::snprintf(num_buf, sizeof(num_buf), "%.1f", config.sound_level_threshold_db_spl);
        page += "Fixed threshold (dB SPL)<br><input name='spl_threshold' value='";
        page += num_buf;
        page += "'><br>";

        std::snprintf(num_buf, sizeof(num_buf), "%.2f", config.correlation_threshold);
        page += "Correlation threshold (min R, e.g. 0.45)<br><input name='corr_threshold' step='0.01' value='";
        page += num_buf;
        page += "'><br>";

        std::snprintf(num_buf, sizeof(num_buf), "%.1f", config.angle_offset_deg);
        page += "Zero angle offset (deg)<br><input name='ang_offset' value='";
        page += num_buf;
        page += "'><br>";

        page += "<label><input type='checkbox' name='adapt_thresh' value='1'";
        if (config.adaptive_threshold_enabled) {
            page += " checked";
        }
        page += "> Enable adaptive noise floor threshold</label><br>";

        std::snprintf(num_buf, sizeof(num_buf), "%.1f", config.snr_margin_db);
        page += "SNR margin (dB)<br><input name='snr_margin' value='";
        page += num_buf;
        page += "'><br>";

        std::snprintf(num_buf, sizeof(num_buf), "%.1f", config.sector_width_deg);
        page += "Sector width (deg, 5-90)<br><input name='sector_width' step='0.5' value='";
        page += num_buf;
        page += "'><br>";

        std::snprintf(num_buf, sizeof(num_buf), "%.1f", config.sector_hysteresis_deg);
        page += "Sector hysteresis (deg, 0-15)<br><input name='sector_hysteresis' step='0.5' value='";
        page += num_buf;
        page += "'><br>";

        std::snprintf(num_buf, sizeof(num_buf), "%lu", static_cast<unsigned long>(config.tracker_hold_timeout_ms));
        page += "Speech pause hold timeout (ms, 200-30000)<br><input type='number' name='tracker_hold_ms' value='";
        page += num_buf;
        page += "'><br><br>";

        page += "<button type='submit'>Save and restart</button></form></body></html>";
        return page;
    }

#if defined(ARDUINO)
    static void enter(DeviceConfig &config, RadarDisplay &radar,
                      std::function<void(const DeviceConfig&)> on_save);
#endif
};

