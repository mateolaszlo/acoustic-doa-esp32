#include "ui/radar_ui.h"
#include <cmath>

RadarDisplay::RadarDisplay()
    : tft_spi(FSPI),
      display(&tft_spi, TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN),
      canvas(TFT_WIDTH, TFT_HEIGHT),
      initialized(false),
      hit_blip_head(0) {
    for (size_t b = 0; b < MAX_DECAY_BLIPS; ++b) {
        hit_blips[b] = {0.0f, 0, false};
    }
}

bool RadarDisplay::init() {
    tft_spi.begin(TFT_SCLK_PIN, -1, TFT_MOSI_PIN, -1);
    display.initR(INITR_BLACKTAB);
    display.setSPISpeed(27000000); // 27 MHz SPI
    display.setRotation(0);        // Portrait 128x160
    if (TFT_INVERT_COLORS) {
        display.invertDisplay(true);
    }
    initialized = true;
    return true;
}

void RadarDisplay::drawRadar(float angle_deg, float temp_c, float hum, float pres, float c_sound,
                            float db_spl, bool api_available, bool adaptive_active) {
    const float angles[1] = { angle_deg };
    const float confs[1] = { 1.0f };
    drawRadarMulti(angles, confs, 1, temp_c, hum, pres, c_sound, db_spl, api_available, adaptive_active);
}

void RadarDisplay::drawRadarMulti(const float* angles_deg, const float* confidences, size_t count,
                                 float temp_c, float hum, float pres, float c_sound,
                                 float db_spl, bool api_available, bool adaptive_active) {
    if (!initialized) return;
    canvas.fillScreen(ST77XX_BLACK);

    // 1. Izris 360-stopinjskega krožnega radarja (cx = 64, cy = 44, r = 34)
    const int cx = RADAR_CENTER_X;
    const int cy = RADAR_CENTER_Y;
    const int r = RADAR_RADIUS;

    // Značka načina delovanja (AUTO ali FIX) v zgornjem desnem kotu
    if (adaptive_active) {
        canvas.drawRoundRect(91, 2, 34, 11, 2, ST77XX_CYAN);
        canvas.setTextSize(1);
        canvas.setTextColor(ST77XX_CYAN);
        canvas.setCursor(96, 4);
        canvas.print("AUTO");
    } else {
        canvas.drawRoundRect(96, 2, 29, 11, 2, COLOR_BADGE_BORDER);
        canvas.setTextSize(1);
        canvas.setTextColor(COLOR_MUTED_GRAY);
        canvas.setCursor(102, 4);
        canvas.print("FIX");
    }

    // Koncentrična obroča in križne osi N/E/S/W (temno zelena mreža)
    canvas.drawCircle(cx, cy, r, COLOR_RADAR_GRID);
    canvas.drawCircle(cx, cy, r / 2, COLOR_RADAR_GRID);
    canvas.drawLine(cx - r, cy, cx + r, cy, COLOR_RADAR_GRID);
    canvas.drawLine(cx, cy - r, cx, cy + r, COLOR_RADAR_GRID);

    // 30-stopinjske kompasne oznake po obodu radarja
    for (int deg = 30; deg < 360; deg += 30) {
        if (deg % 90 == 0) continue; // Kardinalne osi že imajo polne križne črte
        const float tick_rad = static_cast<float>(deg - 90) * (static_cast<float>(M_PI) / 180.0f);
        const float cos_t = cosf(tick_rad);
        const float sin_t = sinf(tick_rad);
        const int x1 = cx + static_cast<int>(roundf((r - 2) * cos_t));
        const int y1 = cy + static_cast<int>(roundf((r - 2) * sin_t));
        const int x2 = cx + static_cast<int>(roundf(r * cos_t));
        const int y2 = cy + static_cast<int>(roundf(r * sin_t));
        canvas.drawLine(x1, y1, x2, y2, COLOR_RADAR_GRID);
    }

    // Kardinalne oznake azimuta (0, +90, 180, -90)
    canvas.setTextSize(1);
    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(cx - 2, 1);
    canvas.print("0");
    canvas.setCursor(cx + r + 4, cy - 3);
    canvas.print("+90");
    canvas.setCursor(cx - r - 21, cy - 3);
    canvas.print("-90");
    canvas.setCursor(cx + 3, cy + r - 9);
    canvas.print("180");

    // 2. Izris razpadajočih fosfornih sledi (phosphor decay trail za hipne zvoke)
    const size_t num_needles = (count > DOA_MAX_SOURCES) ? DOA_MAX_SOURCES : count;
    const uint32_t now_ms = millis();
    for (size_t s = 0; s < num_needles; ++s) {
        if (angles_deg != nullptr) {
            hit_blips[hit_blip_head] = { angles_deg[s], now_ms, true };
            hit_blip_head = (hit_blip_head + 1) % MAX_DECAY_BLIPS;
        }
    }

    for (size_t b = 0; b < MAX_DECAY_BLIPS; ++b) {
        if (!hit_blips[b].active) continue;
        const uint32_t age_ms = now_ms - hit_blips[b].timestamp_ms;
        if (age_ms > 1200) {
            hit_blips[b].active = false;
            continue;
        }
        if (age_ms < 60) continue; // Trenutni okvir že riše živo iglo

        uint16_t fade_color = (age_ms < 400) ? COLOR_MUTED_GRAY :
                              ((age_ms < 800) ? 0x39E7 : 0x18C3);
        const float b_rad = (hit_blips[b].angle_deg - 90.0f) * (static_cast<float>(M_PI) / 180.0f);
        const int bx = cx + static_cast<int>(roundf(r * cosf(b_rad)));
        const int by = cy + static_cast<int>(roundf(r * sinf(b_rad)));
        canvas.fillCircle(bx, by, 1, fade_color);
    }

    // 3. Izris radarskih igel (do DOA_MAX_SOURCES = 3 zvočnih virov)
    canvas.fillCircle(cx, cy, 2, COLOR_MUTED_GRAY);

    for (size_t s = 0; s < num_needles; ++s) {
        const float angle = (angles_deg != nullptr) ? angles_deg[s] : 0.0f;
        const float rad = (angle - 90.0f) * (static_cast<float>(M_PI) / 180.0f);
        const int target_x = cx + static_cast<int>(roundf(r * cosf(rad)));
        const int target_y = cy + static_cast<int>(roundf(r * sinf(rad)));

        uint16_t needle_color;
        int tip_radius = 2;
        if (s == 0) {
            // Primarni vir: barva glede na glasnost (rdeča/rumena/zelena), večja konica
            needle_color = (db_spl > NEEDLE_HIGH_SPL_THRESHOLD) ? ST77XX_RED :
                           ((db_spl > NEEDLE_MED_SPL_THRESHOLD) ? ST77XX_YELLOW : ST77XX_GREEN);
            tip_radius = 3;
        } else if (s == 1) {
            // Sekundarni vir: cian
            needle_color = ST77XX_CYAN;
        } else {
            // Terciarni vir: magenta
            needle_color = ST77XX_MAGENTA;
        }

        canvas.drawLine(cx, cy, target_x, target_y, needle_color);
        canvas.fillCircle(target_x, target_y, tip_radius, needle_color);
    }

    // 3. Spodnja nadzorna plošča s telemetrijo (y = 84..159)
    canvas.drawFastHLine(0, 82, TFT_WIDTH, COLOR_MUTED_GRAY);

    const float primary_angle = (angles_deg != nullptr && num_needles > 0) ? angles_deg[0] : 0.0f;
    if (num_needles <= 1) {
        canvas.setTextSize(2);
        canvas.setTextColor(ST77XX_CYAN);
        canvas.setCursor(4, 84);
        canvas.printf("%+6.1f deg", primary_angle);
    } else {
        canvas.setTextSize(1);
        const int chip_w = (num_needles == 2) ? 58 : 39;
        const int gap = (num_needles == 2) ? 8 : 4;
        int x_pos = (TFT_WIDTH - (static_cast<int>(num_needles) * chip_w + static_cast<int>(num_needles - 1) * gap)) / 2;

        for (size_t s = 0; s < num_needles; ++s) {
            const float ang = (angles_deg != nullptr) ? angles_deg[s] : 0.0f;
            uint16_t chip_color;
            if (s == 0) {
                chip_color = (db_spl > NEEDLE_HIGH_SPL_THRESHOLD) ? ST77XX_RED :
                             ((db_spl > NEEDLE_MED_SPL_THRESHOLD) ? ST77XX_YELLOW : ST77XX_GREEN);
            } else if (s == 1) {
                chip_color = ST77XX_CYAN;
            } else {
                chip_color = ST77XX_MAGENTA;
            }

            canvas.drawRoundRect(x_pos, 85, chip_w, 14, 2, chip_color);
            canvas.setTextColor(chip_color);
            canvas.setCursor(x_pos + 3, 88);
            if (num_needles == 2) {
                canvas.printf("%u:%+4.0f", static_cast<unsigned>(s + 1), ang);
            } else {
                canvas.printf("%u:%+3.0f", static_cast<unsigned>(s + 1), ang);
            }
            x_pos += chip_w + gap;
        }
    }

    // 3.1 Nivo zvoka in dinamični VU/SNR stolpec (y = 104..112)
    canvas.setTextSize(1);
    canvas.setTextColor(ST77XX_YELLOW);
    canvas.setCursor(4, 104);
    canvas.printf("%4.1fdB", db_spl);

    // Mini horizontalni merilnik dinamike zvoka (x = 52, y = 104, w = 72, h = 7)
    constexpr int bar_x = 52;
    constexpr int bar_y = 104;
    constexpr int bar_w = 72;
    constexpr int bar_h = 7;
    canvas.drawRect(bar_x, bar_y, bar_w, bar_h, COLOR_MUTED_GRAY);

    // Pretvorba dB SPL v širino stolpca (območje 30..95 dB SPL)
    constexpr float min_bar_db = 30.0f;
    constexpr float max_bar_db = 95.0f;
    float norm_level = (db_spl - min_bar_db) / (max_bar_db - min_bar_db);
    if (norm_level < 0.0f) norm_level = 0.0f;
    if (norm_level > 1.0f) norm_level = 1.0f;
    const int fill_w = static_cast<int>(roundf(norm_level * (bar_w - 2)));

    if (fill_w > 0) {
        uint16_t bar_color = (db_spl > NEEDLE_HIGH_SPL_THRESHOLD) ? ST77XX_RED :
                             ((db_spl > NEEDLE_MED_SPL_THRESHOLD) ? ST77XX_YELLOW : ST77XX_GREEN);
        canvas.fillRect(bar_x + 1, bar_y + 1, fill_w, bar_h - 2, bar_color);
    }

    // Oznaka praga srednje glasnosti na merilniku
    const int med_tick_x = bar_x + static_cast<int>(roundf(((NEEDLE_MED_SPL_THRESHOLD - min_bar_db) / (max_bar_db - min_bar_db)) * bar_w));
    canvas.drawFastVLine(med_tick_x, bar_y - 1, bar_h + 2, COLOR_MUTED_GRAY);

    // 3.2 Združena vrstica okoljske kompenzacije: hitrost zvoka in temperatura/vlaga (y = 117)
    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(4, 117);
    canvas.printf("c:%4.1fm/s  %2.0fC %2.0f%%", c_sound, temp_c, hum);

    // 3.3 Barometrični tlak in konfiguracija vzorčenja (y = 130)
    canvas.setTextColor(COLOR_MUTED_GRAY);
    canvas.setCursor(4, 130);
    canvas.printf("P:%4.0fhPa  44k/1024", pres / 100.0f);

    // 3.4 API status (y = 145..155)
    int status_y = 148;

    if (api_available) {
        canvas.fillCircle(8, status_y, 3, ST77XX_GREEN);
        canvas.setTextColor(ST77XX_GREEN);
        canvas.setCursor(18, 145);
        canvas.print("API ONLINE");
    } else {
        canvas.drawCircle(8, status_y, 3, ST77XX_RED);
        canvas.setTextColor(ST77XX_RED);
        canvas.setCursor(18, 145);
        canvas.print("API OFFLINE");
    }

    display.drawRGBBitmap(0, 0, canvas.getBuffer(), TFT_WIDTH, TFT_HEIGHT);
}

void RadarDisplay::drawBootPrompt(uint32_t remaining_ms, uint32_t total_ms, const BootSelfTestResult &post) {
    if (!initialized) return;
    canvas.fillScreen(ST77XX_BLACK);

    // Naslov
    canvas.setTextSize(1);
    canvas.setTextColor(ST77XX_CYAN);
    canvas.setCursor(14, 6);
    canvas.println("ACOUSTIC LOCATOR");
    canvas.drawFastHLine(8, 16, 112, COLOR_MUTED_GRAY);

    const uint32_t elapsed_ms = (total_ms >= remaining_ms) ? (total_ms - remaining_ms) : total_ms;

    // POST Checklist (staggered display for user experience)
    // PSRAM (revealed at >= 200 ms)
    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(8, 21);
    canvas.print("PSRAM :");
    canvas.setCursor(54, 21);
    if (elapsed_ms >= 200) {
        if (post.psram_ok) {
            canvas.setTextColor(ST77XX_GREEN);
            canvas.printf("[OK] %uMB", static_cast<unsigned>(post.psram_bytes / (1024 * 1024)));
        } else {
            canvas.setTextColor(ST77XX_RED);
            canvas.print("[FAIL]");
        }
    } else {
        canvas.setTextColor(COLOR_MUTED_GRAY);
        canvas.print("[..]");
    }

    // BME280 (revealed at >= 400 ms)
    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(8, 32);
    canvas.print("BME280:");
    canvas.setCursor(54, 32);
    if (elapsed_ms >= 400) {
        if (post.bme_ok) {
            canvas.setTextColor(ST77XX_GREEN);
            canvas.printf("[OK] %.0fC", post.bme_temp_c);
        } else {
            canvas.setTextColor(COLOR_MUTED_GRAY);
            canvas.print("[--] Default");
        }
    } else {
        canvas.setTextColor(COLOR_MUTED_GRAY);
        canvas.print("[..]");
    }

    // M0 & M1 (Bus 0: GPIO 6, revealed at >= 600 ms and >= 800 ms)
    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(8, 43);
    canvas.print("M0:");
    if (elapsed_ms >= 600) {
        canvas.setTextColor(post.mic_health.ch_ok[0] ? ST77XX_GREEN : ST77XX_RED);
        canvas.print(post.mic_health.ch_ok[0] ? "[OK]" : "[ER]");
    } else {
        canvas.setTextColor(COLOR_MUTED_GRAY);
        canvas.print("[..]");
    }

    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(68, 43);
    canvas.print("M1:");
    if (elapsed_ms >= 800) {
        canvas.setTextColor(post.mic_health.ch_ok[1] ? ST77XX_GREEN : ST77XX_RED);
        canvas.print(post.mic_health.ch_ok[1] ? "[OK]" : "[ER]");
    } else {
        canvas.setTextColor(COLOR_MUTED_GRAY);
        canvas.print("[..]");
    }

    // M2 & M3 (Bus 1: GPIO 7, revealed at >= 1000 ms and >= 1200 ms)
    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(8, 54);
    canvas.print("M2:");
    if (elapsed_ms >= 1000) {
        canvas.setTextColor(post.mic_health.ch_ok[2] ? ST77XX_GREEN : ST77XX_RED);
        canvas.print(post.mic_health.ch_ok[2] ? "[OK]" : "[ER]");
    } else {
        canvas.setTextColor(COLOR_MUTED_GRAY);
        canvas.print("[..]");
    }

    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(68, 54);
    canvas.print("M3:");
    if (elapsed_ms >= 1200) {
        canvas.setTextColor(post.mic_health.ch_ok[3] ? ST77XX_GREEN : ST77XX_RED);
        canvas.print(post.mic_health.ch_ok[3] ? "[OK]" : "[ER]");
    } else {
        canvas.setTextColor(COLOR_MUTED_GRAY);
        canvas.print("[..]");
    }

    // Dual-I2S Bus Synchronization & Channel Independence (revealed at >= 1400 ms)
    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(8, 65);
    canvas.print("SYNC  :");
    canvas.setCursor(54, 65);
    if (elapsed_ms >= 1400) {
        if (post.i2s_driver_ok &&
            post.mic_health.bus0_distinct &&
            post.mic_health.bus1_distinct &&
            post.mic_health.cross_bus_distinct) {
            canvas.setTextColor(ST77XX_GREEN);
            canvas.print("[OK] Quad");
        } else if (!post.i2s_driver_ok) {
            canvas.setTextColor(ST77XX_RED);
            canvas.print("[FAIL] I2S");
        } else if (!post.mic_health.bus0_distinct || !post.mic_health.bus1_distinct) {
            canvas.setTextColor(ST77XX_RED);
            canvas.print("[FAIL] BusLR");
        } else {
            canvas.setTextColor(ST77XX_RED);
            canvas.print("[FAIL] XBus");
        }
    } else {
        canvas.setTextColor(COLOR_MUTED_GRAY);
        canvas.print("[..]");
    }

    canvas.drawFastHLine(8, 78, 112, COLOR_MUTED_GRAY);

    // Poziv za setup
    canvas.setTextColor(ST77XX_YELLOW);
    canvas.setCursor(8, 86);
    canvas.println("Hold BOOT for Setup");

    // Vrstica napredka
    const int bar_x = 14;
    const int bar_y = 100;
    const int bar_w = 100;
    const int bar_h = 7;
    canvas.drawRect(bar_x, bar_y, bar_w, bar_h, ST77XX_WHITE);
    int fill_w = (remaining_ms * (bar_w - 2)) / total_ms;
    if (fill_w > 0) {
        canvas.fillRect(bar_x + 1, bar_y + 1, fill_w, bar_h - 2, ST77XX_CYAN);
    }

    // Besedilo odštevanja
    canvas.setTextColor(COLOR_MUTED_GRAY);
    canvas.setCursor(14, 114);
    int sec = (remaining_ms + 999) / 1000;
    canvas.printf("Starting in %d s...", sec);

    // Status celote (revealed at >= 1600 ms, otherwise shows TESTING...)
    canvas.setCursor(8, 136);
    if (elapsed_ms < 1600) {
        canvas.setTextColor(ST77XX_CYAN);
        canvas.print("STATUS: TESTING...");
    } else if (post.all_nominal) {
        canvas.setTextColor(ST77XX_GREEN);
        canvas.print("STATUS: ALL NOMINAL");
    } else {
        canvas.setTextColor(ST77XX_RED);
        canvas.print("STATUS: DEGRADED");
    }

    display.drawRGBBitmap(0, 0, canvas.getBuffer(), TFT_WIDTH, TFT_HEIGHT);
}

void RadarDisplay::drawHardwareDegraded(const BootSelfTestResult &post) {
    if (!initialized) return;
    canvas.fillScreen(ST77XX_BLACK);

    // Red Header Banner
    canvas.fillRect(0, 0, TFT_WIDTH, 18, ST77XX_RED);
    canvas.setTextSize(1);
    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(13, 5);
    canvas.println("HARDWARE DEGRADED");

    // Failure checklist
    canvas.setCursor(8, 24);
    if (!post.psram_ok) {
        canvas.setTextColor(ST77XX_RED);
        canvas.println("PSRAM : [FAIL]");
    } else {
        canvas.setTextColor(COLOR_MUTED_GRAY);
        canvas.printf("PSRAM : [OK] %uMB\n", static_cast<unsigned>(post.psram_bytes / (1024 * 1024)));
    }

    // M0..M3
    static const char* const mic_names[AUDIO_NUM_CHANNELS] = {"M0", "M1", "M2", "M3"};
    for (size_t c = 0; c < AUDIO_NUM_CHANNELS; ++c) {
        const int y = 36 + static_cast<int>(c) * 11;
        canvas.setCursor(8, y);
        if (!post.mic_health.ch_ok[c]) {
            const char* reason = (post.mic_health.ch_min[c] == post.mic_health.ch_max[c]) ? "FLATLINE" : "RAIL SHORT";
            canvas.setTextColor(ST77XX_RED);
            canvas.printf("%s    : [FAIL] %s", mic_names[c], reason);
        } else {
            canvas.setTextColor(ST77XX_GREEN);
            canvas.printf("%s    : [OK]", mic_names[c]);
        }
    }

    // Bus Sync
    canvas.setCursor(8, 80);
    if (!post.i2s_driver_ok) {
        canvas.setTextColor(ST77XX_RED);
        canvas.println("SYNC  : [FAIL] Driver");
    } else if (!post.mic_health.bus0_distinct) {
        canvas.setTextColor(ST77XX_RED);
        canvas.println("SYNC  : [FAIL] Bus0 LR");
    } else if (!post.mic_health.bus1_distinct) {
        canvas.setTextColor(ST77XX_RED);
        canvas.println("SYNC  : [FAIL] Bus1 LR");
    } else if (!post.mic_health.cross_bus_distinct) {
        canvas.setTextColor(ST77XX_RED);
        canvas.println("SYNC  : [FAIL] X-Bus");
    } else {
        canvas.setTextColor(ST77XX_GREEN);
        canvas.println("SYNC  : [OK] Quad Synced");
    }

    // Divider
    canvas.drawFastHLine(8, 96, 112, COLOR_MUTED_GRAY);

    // User Guidance
    canvas.setTextColor(ST77XX_YELLOW);
    canvas.setCursor(8, 104);
    canvas.println("Fix wiring & reboot.");
    canvas.setCursor(8, 116);
    canvas.println("Operation halted.");

    canvas.setTextColor(ST77XX_CYAN);
    canvas.setCursor(8, 134);
    canvas.println("Hold BOOT (GPIO 0)");
    canvas.setCursor(8, 146);
    canvas.println("for Setup Portal.");

    display.drawRGBBitmap(0, 0, canvas.getBuffer(), TFT_WIDTH, TFT_HEIGHT);
}

void RadarDisplay::drawSetupPortal() {
    if (!initialized) return;
    canvas.fillScreen(ST77XX_BLACK);

    canvas.setTextSize(1);
    canvas.setTextColor(ST77XX_YELLOW);
    canvas.setCursor(16, 12);
    canvas.println("=== SETUP MODE ===");
    canvas.drawFastHLine(10, 24, 108, COLOR_MUTED_GRAY);

    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(8, 34);
    canvas.println("Connect to Wi-Fi AP:");
    canvas.setTextColor(ST77XX_CYAN);
    canvas.setCursor(8, 46);
    canvas.println(SETUP_AP_NAME);

    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(8, 64);
    canvas.println("Password:");
    canvas.setTextColor(ST77XX_CYAN);
    canvas.setCursor(8, 76);
    canvas.println(SETUP_AP_PASSWORD);

    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(8, 94);
    canvas.println("Open in browser:");
    canvas.setTextColor(ST77XX_GREEN);
    canvas.setCursor(8, 106);
    canvas.println("http://" SETUP_AP_DEFAULT_IP_STR);

    canvas.setTextColor(COLOR_MUTED_GRAY);
    canvas.setCursor(8, 130);
    canvas.println("Save & reboot");
    canvas.setCursor(8, 142);
    canvas.println("when finished.");

    display.drawRGBBitmap(0, 0, canvas.getBuffer(), TFT_WIDTH, TFT_HEIGHT);
}
