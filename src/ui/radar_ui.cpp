#include "ui/radar_ui.h"
#include <cmath>

// 3x5 Micro-font for compact numeric annotations (digits 0..9)
// 5 rows of 3 bits per glyph (packed into uint16_t, MSB first)
static void drawMicroDigit(GFXcanvas16& canvas, int x, int y, uint8_t digit, uint16_t color) {
    if (digit > 9) return;
    static const uint16_t FONT3X5[10] = {
        0b1111011011011110, // 0: ### / # # / # # / # # / ###
        0b0101100100101110, // 1:  #  / ##  /  #  /  #  / ###
        0b1110011111001110, // 2: ### /   # / ### / #   / ###
        0b1110011110011110, // 3: ### /   # / ### /   # / ###
        0b1011011110010010, // 4: # # / # # / ### /   # /   #
        0b1111001110011110, // 5: ### / #   / ### /   # / ###
        0b1111001111011110, // 6: ### / #   / ### / # # / ###
        0b1110010010010010, // 7: ### /   # /   # /   # /   #
        0b1111011111011110, // 8: ### / # # / ### / # # / ###
        0b1111011110011110  // 9: ### / # # / ### /   # / ###
    };
    const uint16_t glyph = FONT3X5[digit];
    for (int r = 0; r < 5; ++r) {
        for (int c = 0; c < 3; ++c) {
            if (glyph & (1 << (15 - (r * 3 + c)))) {
                canvas.drawPixel(x + c, y + r, color);
            }
        }
    }
}

static void drawMicroNumber(GFXcanvas16& canvas, int x, int y, int value, uint16_t color) {
    if (value < 0) value = 0;
    if (value >= 100) value = 99;
    if (value >= 10) {
        drawMicroDigit(canvas, x, y, static_cast<uint8_t>(value / 10), color);
        drawMicroDigit(canvas, x + 4, y, static_cast<uint8_t>(value % 10), color);
    } else {
        drawMicroDigit(canvas, x, y, static_cast<uint8_t>(value), color);
    }
}

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

void RadarDisplay::addDecayBlips(const float* angles_deg, size_t count, uint32_t now_ms) {
    if (angles_deg == nullptr || count == 0) return;
    const size_t num_needles = (count > DOA_MAX_SOURCES) ? DOA_MAX_SOURCES : count;
    for (size_t s = 0; s < num_needles; ++s) {
        const float angle = angles_deg[s];
        if (!std::isfinite(angle)) continue;
        bool duplicate = false;
        for (size_t b = 0; b < MAX_DECAY_BLIPS; ++b) {
            if (hit_blips[b].active &&
                now_ms >= hit_blips[b].timestamp_ms &&
                (now_ms - hit_blips[b].timestamp_ms < 150)) {
                float diff = fabsf(hit_blips[b].angle_deg - angle);
                if (diff > 180.0f) diff = 360.0f - diff;
                if (diff < 8.0f) {
                    duplicate = true;
                    hit_blips[b].timestamp_ms = now_ms; // Refresh blip timestamp while sound persists at the same bearing
                    break;
                }
            }
        }
        if (!duplicate) {
            hit_blips[hit_blip_head] = { angle, now_ms, true };
            hit_blip_head = (hit_blip_head + 1) % MAX_DECAY_BLIPS;
        }
    }
}

void RadarDisplay::drawRadar(float angle_deg, float temp_c, float hum, float pres, float c_sound,
                            float db_spl, bool api_available, bool adaptive_active,
                            float snr_db, float snr_margin_db,
                            uint32_t sample_rate,
                            bool wifi_connected,
                            float fixed_threshold_db) {
    const float angles[1] = { angle_deg };
    const float confs[1] = { 1.0f };
    const uint32_t now = millis();
    addDecayBlips(angles, 1, now);
    drawRadarMulti(angles, confs, 1, temp_c, hum, pres, c_sound, db_spl, api_available, adaptive_active,
                   snr_db, snr_margin_db, nullptr, sample_rate, now, wifi_connected, fixed_threshold_db);
}

void RadarDisplay::drawRadarMulti(const float* angles_deg, const float* confidences, size_t count,
                                 float temp_c, float hum, float pres, float c_sound,
                                 float db_spl, bool api_available, bool adaptive_active,
                                 float snr_db, float snr_margin_db,
                                 const uint8_t* slot_ids,
                                 uint32_t sample_rate,
                                 uint32_t now_ms,
                                 bool wifi_connected,
                                 float fixed_threshold_db) {
    (void)confidences;
    if (!initialized) return;
    canvas.fillScreen(ST77XX_BLACK);

    const size_t num_needles = (count > DOA_MAX_SOURCES) ? DOA_MAX_SOURCES : count;
    const uint32_t current_time_ms = (now_ms > 0) ? now_ms : millis();

    // 1. Draw 360-degree circular radar scope (cx = 64, cy = 44, r = 34)
    const int cx = RADAR_CENTER_X;
    const int cy = RADAR_CENTER_Y;
    const int r = RADAR_RADIUS;

    // Threshold mode pill badge (AUTO or FIX) in top-right corner
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

    // Concentric range rings and N/E/S/W crosshair axes (dark green grid)
    canvas.drawCircle(cx, cy, r, COLOR_RADAR_GRID);
    canvas.drawCircle(cx, cy, r / 2, COLOR_RADAR_GRID);
    canvas.drawLine(cx - r, cy, cx + r, cy, COLOR_RADAR_GRID);
    canvas.drawLine(cx, cy - r, cx, cy + r, COLOR_RADAR_GRID);

    // 30-degree compass tick marks around the perimeter
    for (int deg = 30; deg < 360; deg += 30) {
        if (deg % 90 == 0) continue; // Cardinal axes already have full crosshair lines
        const float tick_rad = static_cast<float>(deg - 90) * (static_cast<float>(M_PI) / 180.0f);
        const float cos_t = cosf(tick_rad);
        const float sin_t = sinf(tick_rad);
        const int x1 = cx + static_cast<int>(roundf((r - 2) * cos_t));
        const int y1 = cy + static_cast<int>(roundf((r - 2) * sin_t));
        const int x2 = cx + static_cast<int>(roundf(r * cos_t));
        const int y2 = cy + static_cast<int>(roundf(r * sin_t));
        canvas.drawLine(x1, y1, x2, y2, COLOR_RADAR_GRID);
    }

    // Cardinal azimuth labels (0, +90, 180, -90)
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

    // 2. Render decaying P31 green phosphor trail blips (500 ms decay for transient impulses)
    for (size_t b = 0; b < MAX_DECAY_BLIPS; ++b) {
        if (!hit_blips[b].active) continue;
        const uint32_t age_ms = (current_time_ms >= hit_blips[b].timestamp_ms) ? (current_time_ms - hit_blips[b].timestamp_ms) : 0;
        if (age_ms > PHOSPHOR_DECAY_DURATION_MS) {
            hit_blips[b].active = false;
            continue;
        }

        uint16_t fade_color;
        if (age_ms < 120) {
            fade_color = 0x57EA; // Bright neon phosphor green
        } else if (age_ms < 250) {
            fade_color = 0x2DE4; // Medium-bright phosphor green
        } else if (age_ms < 380) {
            fade_color = 0x0BE0; // Medium emerald phosphor green
        } else {
            fade_color = 0x0980; // Dimmed phosphor green
        }

        const float b_rad = (hit_blips[b].angle_deg - 90.0f) * (static_cast<float>(M_PI) / 180.0f);
        // Position blip at r - 4, just inside the outer radar ring (r = 34)
        const int blip_r = r - 4;
        const int bx = cx + static_cast<int>(roundf(blip_r * cosf(b_rad)));
        const int by = cy + static_cast<int>(roundf(blip_r * sinf(b_rad)));
        canvas.fillCircle(bx, by, 2, fade_color);
    }

    // 3. Draw polar radar needles (up to DOA_MAX_SOURCES = 3 concurrent sources)
    canvas.fillCircle(cx, cy, 2, COLOR_MUTED_GRAY);

    for (size_t s = 0; s < num_needles; ++s) {
        const float angle = (angles_deg != nullptr) ? angles_deg[s] : 0.0f;
        const float rad = (angle - 90.0f) * (static_cast<float>(M_PI) / 180.0f);
        const int target_x = cx + static_cast<int>(roundf(r * cosf(rad)));
        const int target_y = cy + static_cast<int>(roundf(r * sinf(rad)));

        const uint8_t slot = (slot_ids != nullptr) ? slot_ids[s] : static_cast<uint8_t>(s);
        uint16_t needle_color;
        int tip_radius = 2;
        if (slot == 0) {
            // Primary source (Slot 0): colored by SPL (red/yellow/green) with larger tip
            needle_color = (db_spl > NEEDLE_HIGH_SPL_THRESHOLD) ? ST77XX_RED :
                           ((db_spl > NEEDLE_MED_SPL_THRESHOLD) ? ST77XX_YELLOW : ST77XX_GREEN);
            tip_radius = 3;
        } else if (slot == 1) {
            // Secondary source (Slot 1): cyan
            needle_color = ST77XX_CYAN;
        } else {
            // Tertiary source (Slot 2): magenta
            needle_color = ST77XX_MAGENTA;
        }

        canvas.drawLine(cx, cy, target_x, target_y, needle_color);
        canvas.fillCircle(target_x, target_y, tip_radius, needle_color);
    }

    // 4. Lower telemetry dashboard (y = 84..159)
    canvas.drawFastHLine(0, 82, TFT_WIDTH, COLOR_MUTED_GRAY);

    const float primary_angle = (angles_deg != nullptr && num_needles > 0) ? angles_deg[0] : 0.0f;
    if (num_needles == 0) {
        canvas.setTextSize(2);
        canvas.setTextColor(COLOR_MUTED_GRAY);
        canvas.setCursor(4, 84);
        canvas.printf(" ---.- deg");
    } else if (num_needles == 1) {
        const uint8_t slot = (slot_ids != nullptr) ? slot_ids[0] : 0;
        canvas.setTextSize(2);
        canvas.setTextColor((slot == 2) ? ST77XX_MAGENTA : ST77XX_CYAN);
        canvas.setCursor(4, 84);
        canvas.printf("%+6.1f deg", primary_angle);
    } else {
        canvas.setTextSize(1);
        const int chip_w = (num_needles == 2) ? 58 : 39;
        const int gap = (num_needles == 2) ? 8 : 4;
        int x_pos = (TFT_WIDTH - (static_cast<int>(num_needles) * chip_w + static_cast<int>(num_needles - 1) * gap)) / 2;

        for (size_t s = 0; s < num_needles; ++s) {
            const float ang = (angles_deg != nullptr) ? angles_deg[s] : 0.0f;
            const uint8_t slot = (slot_ids != nullptr) ? slot_ids[s] : static_cast<uint8_t>(s);
            uint16_t chip_color;
            if (slot == 0) {
                chip_color = (db_spl > NEEDLE_HIGH_SPL_THRESHOLD) ? ST77XX_RED :
                             ((db_spl > NEEDLE_MED_SPL_THRESHOLD) ? ST77XX_YELLOW : ST77XX_GREEN);
            } else if (slot == 1) {
                chip_color = ST77XX_CYAN;
            } else {
                chip_color = ST77XX_MAGENTA;
            }

            canvas.drawRoundRect(x_pos, 85, chip_w, 14, 2, chip_color);
            canvas.setTextColor(chip_color);
            canvas.setCursor(x_pos + 3, 88);
            if (num_needles == 2) {
                canvas.printf("%u:%+4.0f", static_cast<unsigned>(slot + 1), ang);
            } else {
                canvas.printf("%u:%+3.0f", static_cast<unsigned>(slot + 1), ang);
            }
            x_pos += chip_w + gap;
        }
    }

    // 4.1 Sound pressure level and dynamic SNR / SPL bar with trigger gate notch (y = 103..110)
    canvas.setTextSize(1);
    canvas.setCursor(2, 103);
    canvas.setTextColor(ST77XX_YELLOW);
    canvas.printf("%2.0fdB", (db_spl > 0.0f) ? db_spl : 0.0f);

    const bool has_detection = (count > 0);

    // Mini horizontal meter with trigger threshold notch (x = 50, y = 103, w = 74, h = 7)
    constexpr int bar_x = 50;
    constexpr int bar_y = 103;
    constexpr int bar_w = 74;
    constexpr int bar_h = 7;
    canvas.drawRect(bar_x, bar_y, bar_w, bar_h, COLOR_MUTED_GRAY);

    int gate_offset = 0;
    int fill_w = 0;
    int gate_val = 0;
    uint16_t bar_color = COLOR_MUTED_GRAY;

    if (adaptive_active) {
        // --- AUTO MODE (Relative SNR) ---
        // Numeric SNR readout (+XX dB)
        canvas.setCursor(28, 103);
        if (has_detection && snr_db >= 20.0f) {
            canvas.setTextColor(ST77XX_GREEN);
        } else if (has_detection && snr_db >= snr_margin_db) {
            canvas.setTextColor(ST77XX_YELLOW);
        } else {
            canvas.setTextColor(COLOR_MUTED_GRAY);
        }
        canvas.printf("+%1.0f", (snr_db > 0.0f) ? snr_db : 0.0f);

        // Trigger threshold gate notch position along the SNR meter (0..30 dB)
        constexpr float max_snr_db = 30.0f;
        gate_offset = static_cast<int>(roundf((snr_margin_db / max_snr_db) * (bar_w - 2)));

        float norm_snr = snr_db / max_snr_db;
        if (norm_snr < 0.0f) norm_snr = 0.0f;
        if (norm_snr > 1.0f) norm_snr = 1.0f;
        fill_w = static_cast<int>(roundf(norm_snr * (bar_w - 2)));

        if (!has_detection && fill_w >= gate_offset) {
            fill_w = (gate_offset > 1) ? (gate_offset - 1) : 0;
        }

        if (fill_w > 0) {
            if (has_detection && snr_db >= 20.0f) {
                bar_color = ST77XX_GREEN; // High SNR / optimal localization confidence
            } else if (has_detection && snr_db >= snr_margin_db) {
                bar_color = ST77XX_YELLOW; // Active trigger above threshold / moderate SNR
            } else {
                bar_color = COLOR_MUTED_GRAY; // Sub-threshold ambient noise
            }
        }
        gate_val = static_cast<int>(roundf(snr_margin_db));
    } else {
        // --- FIX MODE (Absolute dB SPL: 30..90 dB SPL) ---
        constexpr float min_spl_meter = 30.0f;
        constexpr float max_spl_meter = 90.0f;
        constexpr float spl_range = max_spl_meter - min_spl_meter;

        const float delta_db = db_spl - fixed_threshold_db;

        // Numeric margin above fixed gate (+XX dB)
        canvas.setCursor(28, 103);
        if (has_detection && delta_db >= 15.0f) {
            canvas.setTextColor(ST77XX_GREEN);
        } else if (has_detection && delta_db >= 0.0f) {
            canvas.setTextColor(ST77XX_YELLOW);
        } else {
            canvas.setTextColor(COLOR_MUTED_GRAY);
        }
        canvas.printf("+%1.0f", (delta_db > 0.0f) ? delta_db : 0.0f);

        // Gate notch position along the SPL meter (30..90 dB)
        float norm_gate = (fixed_threshold_db - min_spl_meter) / spl_range;
        if (norm_gate < 0.0f) norm_gate = 0.0f;
        if (norm_gate > 1.0f) norm_gate = 1.0f;
        gate_offset = static_cast<int>(roundf(norm_gate * (bar_w - 2)));

        // Map sound level to bar fill width (30..90 dB)
        float norm_spl = (db_spl - min_spl_meter) / spl_range;
        if (norm_spl < 0.0f) norm_spl = 0.0f;
        if (norm_spl > 1.0f) norm_spl = 1.0f;
        fill_w = static_cast<int>(roundf(norm_spl * (bar_w - 2)));

        if (!has_detection && fill_w >= gate_offset) {
            fill_w = (gate_offset > 1) ? (gate_offset - 1) : 0;
        }

        if (fill_w > 0) {
            if (has_detection && delta_db >= 15.0f) {
                bar_color = ST77XX_GREEN; // Loud / well above threshold
            } else if (has_detection && delta_db >= 0.0f) {
                bar_color = ST77XX_YELLOW; // Active trigger above threshold
            } else {
                bar_color = COLOR_MUTED_GRAY; // Sub-threshold ambient noise
            }
        }
        gate_val = static_cast<int>(roundf(fixed_threshold_db));
    }

    if (gate_offset < 0) gate_offset = 0;
    if (gate_offset > bar_w - 2) gate_offset = bar_w - 2;
    const int gate_x = bar_x + gate_offset;

    if (fill_w > 0) {
        if (fill_w > bar_w - 2) fill_w = bar_w - 2;
        canvas.fillRect(bar_x + 1, bar_y + 1, fill_w, bar_h - 2, bar_color);
    }

    canvas.drawFastVLine(gate_x, bar_y - 1, bar_h + 2, ST77XX_WHITE);

    // Render numeric threshold in 3x5 micro-font directly beneath the gate notch
    const int digit_offset = (gate_val >= 10) ? 3 : 1;
    const int num_w = (gate_val >= 10) ? 7 : 3;
    int num_x = gate_x - digit_offset;
    if (num_x < bar_x) num_x = bar_x;
    if (num_x + num_w > bar_x + bar_w) num_x = bar_x + bar_w - num_w;
    drawMicroNumber(canvas, num_x, bar_y + bar_h + 1, gate_val, ST77XX_WHITE);

    // 4.2 Consolidated environmental compensation row: sound speed, temperature, humidity (y = 117)
    canvas.setTextColor(ST77XX_WHITE);
    canvas.setCursor(4, 117);
    canvas.printf("c:%4.1fm/s  %2.0fC %2.0f%%", c_sound, temp_c, hum);

    // 4.3 Barometric pressure and audio sampling configuration (y = 130)
    canvas.setTextColor(COLOR_MUTED_GRAY);
    canvas.setCursor(4, 130);
    const unsigned sr_k = static_cast<unsigned>(roundf(sample_rate / 1000.0f));
    canvas.printf("P:%4.0fhPa  %uk/1024", pres / 100.0f, sr_k);

    // 4.4 Backend API connectivity status (y = 145..155)
    int status_y = 148;

    if (api_available) {
        canvas.fillCircle(8, status_y, 3, ST77XX_GREEN);
        canvas.setTextColor(ST77XX_GREEN);
        canvas.setCursor(18, 145);
        canvas.print("API ONLINE");
    } else {
        // Blinks (toggles fill) at 1 Hz if Wi-Fi is connected but cannot access API
        const bool blink_filled = wifi_connected && ((current_time_ms / 500) % 2 == 0);
        if (blink_filled) {
            canvas.fillCircle(8, status_y, 3, ST77XX_RED);
        } else {
            canvas.drawCircle(8, status_y, 3, ST77XX_RED);
        }
        canvas.setTextColor(ST77XX_RED);
        canvas.setCursor(18, 145);
        canvas.print("API OFFLINE");
    }

    display.drawRGBBitmap(0, 0, canvas.getBuffer(), TFT_WIDTH, TFT_HEIGHT);
}

bool RadarDisplay::hasActiveDecayBlips(uint32_t now_ms) const {
    for (size_t b = 0; b < MAX_DECAY_BLIPS; ++b) {
        if (hit_blips[b].active &&
            now_ms >= hit_blips[b].timestamp_ms &&
            (now_ms - hit_blips[b].timestamp_ms <= PHOSPHOR_DECAY_DURATION_MS)) {
            return true;
        }
    }
    return false;
}

void RadarDisplay::drawBootPrompt(uint32_t remaining_ms, uint32_t total_ms, const BootSelfTestResult &post) {
    if (!initialized) return;
    canvas.fillScreen(ST77XX_BLACK);

    // Header title
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

    // Setup mode prompt
    canvas.setTextColor(ST77XX_YELLOW);
    canvas.setCursor(8, 86);
    canvas.println("Hold BOOT for Setup");

    // Countdown progress bar
    const int bar_x = 14;
    const int bar_y = 100;
    const int bar_w = 100;
    const int bar_h = 7;
    canvas.drawRect(bar_x, bar_y, bar_w, bar_h, ST77XX_WHITE);
    int fill_w = (remaining_ms * (bar_w - 2)) / total_ms;
    if (fill_w > 0) {
        canvas.fillRect(bar_x + 1, bar_y + 1, fill_w, bar_h - 2, ST77XX_CYAN);
    }

    // Countdown timer text
    canvas.setTextColor(COLOR_MUTED_GRAY);
    canvas.setCursor(14, 114);
    int sec = (remaining_ms + 999) / 1000;
    canvas.printf("Starting in %d s...", sec);

    // Overall system status (revealed at >= 1600 ms, otherwise shows TESTING...)
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
            const bool is_flatline = (post.mic_health.ch_min[c] == post.mic_health.ch_max[c]) ||
                                     (post.mic_health.ch_rms_db[c] <= POST_MIC_FLATLINE_RMS_DB);
            const char* reason = is_flatline ? "FLATLINE" : "RAIL SHORT";
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
