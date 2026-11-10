#include "network/setup_portal.h"

void SetupPortal::enter(DeviceConfig &config, RadarDisplay &radar,
                        std::function<void(const DeviceConfig&)> on_save) {
    radar.drawSetupPortal();
    WiFi.disconnect(true);
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
        String wifi_ssid = web_server.arg("wifi_ssid");
        wifi_ssid.trim();
        candidate.wifi_ssid = wifi_ssid;

        String wifi_password = web_server.arg("wifi_password");
        while (wifi_password.endsWith("\r") || wifi_password.endsWith("\n")) {
            wifi_password.remove(wifi_password.length() - 1);
        }
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
        if (web_server.hasArg("sector_width")) {
            candidate.sector_width_deg = web_server.arg("sector_width").toFloat();
        }
        if (web_server.hasArg("sector_hysteresis")) {
            candidate.sector_hysteresis_deg = web_server.arg("sector_hysteresis").toFloat();
        }
        if (web_server.hasArg("tracker_hold_ms")) {
            const long raw_hold = web_server.arg("tracker_hold_ms").toInt();
            if (raw_hold <= 0) {
                web_server.send(400, "text/plain", "Invalid tracker hold timeout");
                return;
            }
            candidate.tracker_hold_timeout_ms = static_cast<uint32_t>(raw_hold);
        }

        if (!validate_device_config(candidate) || !is_device_configured(candidate)) {
            web_server.send(400, "text/plain", "Invalid configuration");
            return;
        }

        config = candidate;
        if (on_save) {
            on_save(config);
        }

        Serial.printf("[Setup] Configuration saved: SSID='%s' (pass_len=%u), device_id=%lu\n",
                      candidate.wifi_ssid.c_str(),
                      static_cast<unsigned>(candidate.wifi_password.length()),
                      static_cast<unsigned long>(candidate.device_id));

        web_server.send(200, "text/plain", "Saved. Restarting...");
        delay(500);
        WiFi.softAPdisconnect(true);
        WiFi.mode(WIFI_OFF);
        delay(100);
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
