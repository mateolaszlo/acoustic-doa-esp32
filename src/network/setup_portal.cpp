#include "network/setup_portal.h"

String SetupPortal::generateHtmlPage(const DeviceConfig &config) {
    String page;
    page.reserve(2400);
    page += "<!doctype html><html><head><meta name='viewport' content='width=device-width'>";
    page += "<title>Acoustic Locator Setup</title></head><body>";
    page += "<h1>Acoustic Locator Setup</h1><form method='POST' action='/save'>";
    page += "Wi-Fi SSID<br><input name='wifi_ssid' value='" + escapeHtmlAttr(config.wifi_ssid) + "'><br>";
    const String pass_hint = config.wifi_password.length() > 0 ? " (leave blank to retain stored)" : "";
    page += "Wi-Fi password" + pass_hint + "<br><input type='password' name='wifi_password'><br>";
    page += "API URL<br><input name='api_url' value='" + escapeHtmlAttr(config.api_url) + "'><br>";
    const String api_key_hint = config.api_key.length() > 0 ? " (leave blank to retain stored)" : "";
    page += "API key" + api_key_hint + "<br><input type='password' name='api_key'><br>";
    page += "Device ID<br><input type='number' name='device_id' value='" +
            String(config.device_id) + "'><br>";
    page += "Microphone distance (m)<br><input name='mic_distance' value='" +
            String(config.microphone_distance_m, 4) + "'><br>";
    page += "dB SPL offset<br><input name='spl_offset' value='" +
            String(config.db_spl_offset, 2) + "'><br>";
    page += "Fixed threshold (dB SPL)<br><input name='spl_threshold' value='" +
            String(config.sound_level_threshold_db_spl, 1) + "'><br>";
    page += "Correlation threshold (min R, e.g. 0.45)<br><input name='corr_threshold' step='0.01' value='" +
            String(config.correlation_threshold, 2) + "'><br>";
    page += "Zero angle offset (deg)<br><input name='ang_offset' value='" +
            String(config.angle_offset_deg, 1) + "'><br>";
    page += "<label><input type='checkbox' name='adapt_thresh' value='1'" +
            String(config.adaptive_threshold_enabled ? " checked" : "") +
            "> Enable adaptive noise floor threshold</label><br>";
    page += "SNR margin (dB)<br><input name='snr_margin' value='" +
            String(config.snr_margin_db, 1) + "'><br><br>";
    page += "<button type='submit'>Save and restart</button></form></body></html>";
    return page;
}

void SetupPortal::enter(DeviceConfig &config, RadarDisplay &radar,
                        std::function<void(const DeviceConfig&)> on_save) {
    radar.drawSetupPortal();
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(SETUP_AP_NAME, SETUP_AP_PASSWORD)) {
        Serial.println("Setup access point could not be started");
        while (true) {
            delay(1000);
        }
    }

    DNSServer dns_server;
    WebServer web_server(80);
    dns_server.start(53, "*", WiFi.softAPIP());

    auto redirect_handler = [&]() {
        web_server.sendHeader("Location", "http://" + WiFi.softAPIP().toString(), true);
        web_server.send(302, "text/plain", "Redirecting to setup");
    };

    web_server.on("/", HTTP_GET, [&]() {
        web_server.send(200, "text/html", generateHtmlPage(config));
    });

    web_server.on("/save", HTTP_POST, [&]() {
        if (!web_server.hasArg("wifi_ssid") ||
            !web_server.hasArg("api_url") ||
            !web_server.hasArg("device_id") ||
            !web_server.hasArg("mic_distance") ||
            !web_server.hasArg("spl_offset")) {
            web_server.send(400, "text/plain", "Missing required fields");
            return;
        }

        DeviceConfig candidate = config;
        candidate.wifi_ssid = web_server.arg("wifi_ssid");
        String wifi_password = web_server.arg("wifi_password");
        if (wifi_password.length() > 0) {
            candidate.wifi_password = wifi_password;
        }
        candidate.api_url = web_server.arg("api_url");
        String api_key = web_server.arg("api_key");
        if (api_key.length() > 0) {
            candidate.api_key = api_key;
        }
        const long raw_device_id = web_server.arg("device_id").toInt();
        if (raw_device_id <= 0) {
            web_server.send(400, "text/plain", "Invalid device ID");
            return;
        }
        candidate.device_id = static_cast<uint32_t>(raw_device_id);
        candidate.microphone_distance_m = web_server.arg("mic_distance").toFloat();
        candidate.db_spl_offset = web_server.arg("spl_offset").toFloat();

        candidate.adaptive_threshold_enabled = web_server.hasArg("adapt_thresh");
        if (web_server.hasArg("snr_margin")) {
            candidate.snr_margin_db = web_server.arg("snr_margin").toFloat();
        }
        if (web_server.hasArg("spl_threshold")) {
            candidate.sound_level_threshold_db_spl = web_server.arg("spl_threshold").toFloat();
        }
        if (web_server.hasArg("corr_threshold")) {
            candidate.correlation_threshold = web_server.arg("corr_threshold").toFloat();
        }
        if (web_server.hasArg("ang_offset")) {
            candidate.angle_offset_deg = web_server.arg("ang_offset").toFloat();
        }

        if (!validate_device_config(candidate) || !is_device_configured(candidate)) {
            web_server.send(400, "text/plain", "Invalid configuration");
            return;
        }

        config = candidate;
        if (on_save) {
            on_save(config);
        }

        web_server.send(200, "text/plain", "Saved. Restarting...");
        delay(500);
        ESP.restart();
    });

    web_server.on("/generate_204", HTTP_GET, redirect_handler);
    web_server.on("/hotspot-detect.html", HTTP_GET, redirect_handler);
    web_server.on("/connecttest.txt", HTTP_GET, redirect_handler);
    web_server.on("/ncsi.txt", HTTP_GET, redirect_handler);
    web_server.onNotFound(redirect_handler);
    web_server.begin();

    Serial.print("Setup AP active at ");
    Serial.println(WiFi.softAPIP());

    while (true) {
        dns_server.processNextRequest();
        web_server.handleClient();
        delay(2);
    }
}
