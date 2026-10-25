#pragma once
#include <functional>
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

#if defined(ARDUINO)
    static void enter(DeviceConfig &config, RadarDisplay &radar,
                      std::function<void(const DeviceConfig&)> on_save);
    static String generateHtmlPage(const DeviceConfig &config);
#endif
};

