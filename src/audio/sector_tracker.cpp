#include "audio/sector_tracker.h"
#include <cmath>

MultiSourceSectorTracker::MultiSourceSectorTracker(const SectorTrackerConfig& cfg) {
    reset();
    configure(cfg);
}

void MultiSourceSectorTracker::configure(const SectorTrackerConfig& cfg) {
    const float prev_width = config.sector_width_deg;
    config = cfg;

    if (!std::isfinite(config.sector_width_deg) ||
        config.sector_width_deg < SECTOR_WIDTH_MIN_DEG ||
        config.sector_width_deg > SECTOR_WIDTH_MAX_DEG) {
        config.sector_width_deg = DEFAULT_SECTOR_WIDTH_DEG;
    }
    if (!std::isfinite(config.hysteresis_deg) ||
        config.hysteresis_deg < SECTOR_HYSTERESIS_MIN_DEG ||
        config.hysteresis_deg > SECTOR_HYSTERESIS_MAX_DEG ||
        config.hysteresis_deg > config.sector_width_deg) {
        config.hysteresis_deg = (DEFAULT_SECTOR_HYSTERESIS_DEG <= config.sector_width_deg)
                                    ? DEFAULT_SECTOR_HYSTERESIS_DEG
                                    : (0.5f * config.sector_width_deg);
    }
    if (!std::isfinite(config.association_gate_deg) || config.association_gate_deg <= 0.0f) {
        config.association_gate_deg = DEFAULT_TRACKER_ASSOC_GATE_DEG;
    }
    if (!std::isfinite(config.ema_alpha) || config.ema_alpha <= 0.0f || config.ema_alpha > 1.0f) {
        config.ema_alpha = DEFAULT_TRACKER_EMA_ALPHA;
    }
    if (config.birth_confirm_frames == 0) {
        config.birth_confirm_frames = 1;
    }
    if (config.transition_confirm_frames == 0) {
        config.transition_confirm_frames = 1;
    }
    if (config.hold_timeout_ms < TRACKER_HOLD_TIMEOUT_MIN_MS ||
        config.hold_timeout_ms > TRACKER_HOLD_TIMEOUT_MAX_MS) {
        config.hold_timeout_ms = DEFAULT_TRACKER_HOLD_TIMEOUT_MS;
    }

    const bool width_changed = (std::fabs(prev_width - config.sector_width_deg) > 1e-4f);

    for (size_t i = 0; i < SECTOR_TRACKER_MAX_SLOTS; ++i) {
        filters[i].setEmaAlpha(config.ema_alpha);
        filters[i].setResetSilenceMs(config.hold_timeout_ms);

        // If sector grid width changed while a track is active/coasting, re-quantize its
        // confirmed_sector_id to the new sector grid at its current smoothed azimuth
        if (width_changed &&
            (slots[i].state == TRACK_STATE_ACTIVE || slots[i].state == TRACK_STATE_COASTING)) {
            slots[i].confirmed_sector_id = quantizeAngleToSector(slots[i].smoothed_angle_deg);
            slots[i].pending_sector_id = INVALID_SECTOR_ID;
            slots[i].pending_sector_hits = 0;
        }
    }
}

void MultiSourceSectorTracker::updateCalibration(float sector_width_deg,
                                                 float hysteresis_deg,
                                                 uint32_t hold_timeout_ms) {
    if (std::isfinite(sector_width_deg) &&
        std::isfinite(hysteresis_deg) &&
        std::fabs(config.sector_width_deg - sector_width_deg) <= 1e-4f &&
        std::fabs(config.hysteresis_deg - hysteresis_deg) <= 1e-4f &&
        config.hold_timeout_ms == hold_timeout_ms) {
        return;
    }
    SectorTrackerConfig updated = config;
    updated.sector_width_deg = sector_width_deg;
    updated.hysteresis_deg = hysteresis_deg;
    updated.hold_timeout_ms = hold_timeout_ms;
    configure(updated);
}

void MultiSourceSectorTracker::reset() {
    for (size_t i = 0; i < SECTOR_TRACKER_MAX_SLOTS; ++i) {
        filters[i].reset();
        slots[i].state = TRACK_STATE_INACTIVE;
        slots[i].slot_id = static_cast<uint8_t>(i);
        slots[i].confirmed_sector_id = INVALID_SECTOR_ID;
        slots[i].pending_sector_id = INVALID_SECTOR_ID;
        slots[i].pending_sector_hits = 0;
        slots[i].birth_hits = 0;
        slots[i].smoothed_angle_deg = 0.0f;
        slots[i].last_valid_angle_deg = 0.0f;
        slots[i].smoothed_rms_db = -100.0f;
        slots[i].smoothed_confidence = 0.0f;
        slots[i].last_detection_ms = 0;
        slots[i].last_emitted_ms = 0;
        slots[i].expired_at_ms = 0;
        slots[i].has_expired_affinity = false;
    }
}

uint8_t MultiSourceSectorTracker::getNumSectors() const {
    const int n = static_cast<int>(std::round(360.0f / config.sector_width_deg));
    if (n < 1) return 1;
    if (n > 180) return 180;
    return static_cast<uint8_t>(n);
}

uint8_t MultiSourceSectorTracker::quantizeAngleToSector(float angle_deg) const {
    if (!std::isfinite(angle_deg)) {
        return 0;
    }
    const float wrapped = wrap_angle_180(angle_deg);
    const int n = static_cast<int>(getNumSectors());
    int idx = static_cast<int>(std::floor((wrapped + 180.0f) / config.sector_width_deg + 0.5f));
    idx = ((idx % n) + n) % n;
    return static_cast<uint8_t>(idx);
}

float MultiSourceSectorTracker::getSectorCenterDeg(uint8_t sector_id) const {
    const uint8_t n = getNumSectors();
    if (sector_id >= n) {
        return 0.0f;
    }
    const float center = -180.0f + static_cast<float>(sector_id) * config.sector_width_deg;
    return wrap_angle_180(center);
}

bool MultiSourceSectorTracker::isWithinSectorHysteresis(float angle_deg, uint8_t sector_id) const {
    if (sector_id >= getNumSectors() || !std::isfinite(angle_deg)) {
        return false;
    }
    const float center = getSectorCenterDeg(sector_id);
    const float diff = std::fabs(shortest_angular_diff(angle_deg, center));
    const float half_width_plus_hyst = 0.5f * config.sector_width_deg + config.hysteresis_deg;
    return diff <= (half_width_plus_hyst + 1e-4f);
}

uint8_t MultiSourceSectorTracker::getActiveSpeakerCount() const {
    uint8_t count = 0;
    for (size_t i = 0; i < SECTOR_TRACKER_MAX_SLOTS; ++i) {
        if (slots[i].state == TRACK_STATE_ACTIVE || slots[i].state == TRACK_STATE_COASTING) {
            count++;
        }
    }
    return count;
}

const TrackSectorState& MultiSourceSectorTracker::getSlotState(size_t slot_id) const {
    if (slot_id >= SECTOR_TRACKER_MAX_SLOTS) {
        return slots[0];
    }
    return slots[slot_id];
}

size_t MultiSourceSectorTracker::update(const MultiDoaResult& multi_res,
                                        uint32_t now_ms,
                                        SectorTransitionEvent* out_events,
                                        size_t max_events) {
    // 1. Extract valid detections from the multi-source DOA frame
    float det_angles[DOA_MAX_SOURCES];
    float det_confs[DOA_MAX_SOURCES];
    float det_rms[DOA_MAX_SOURCES];
    size_t num_dets = 0;

    if (multi_res.valid && multi_res.count > 0) {
        const size_t limit = (multi_res.count > DOA_MAX_SOURCES) ? DOA_MAX_SOURCES : multi_res.count;
        for (size_t s = 0; s < limit; ++s) {
            const DoaResult& src = multi_res.sources[s];
            if (src.valid && std::isfinite(src.angle_deg) && std::isfinite(src.confidence)) {
                det_angles[num_dets] = wrap_angle_180(src.angle_deg);
                det_confs[num_dets] = src.confidence;
                det_rms[num_dets] = (std::isfinite(src.rms_db) && src.rms_db > -120.0f)
                                        ? src.rms_db
                                        : multi_res.rms_db;
                num_dets++;
            }
        }
    }

    // 2. Clamp non-monotonic timestamps and prune stale CANDIDATE tracks / expired affinities
    for (size_t t = 0; t < SECTOR_TRACKER_MAX_SLOTS; ++t) {
        if (now_ms < slots[t].last_detection_ms) {
            slots[t].last_detection_ms = now_ms;
        }
        if (now_ms < slots[t].last_emitted_ms) {
            slots[t].last_emitted_ms = now_ms;
        }
        if (now_ms < slots[t].expired_at_ms) {
            slots[t].expired_at_ms = now_ms;
        }

        if (slots[t].state == TRACK_STATE_CANDIDATE) {
            if (now_ms - slots[t].last_detection_ms > config.birth_window_ms) {
                slots[t].state = TRACK_STATE_INACTIVE;
                slots[t].birth_hits = 0;
                slots[t].pending_sector_id = INVALID_SECTOR_ID;
                slots[t].pending_sector_hits = 0;
                filters[t].reset();
            }
        } else if (slots[t].state == TRACK_STATE_INACTIVE && slots[t].has_expired_affinity) {
            if (now_ms - slots[t].expired_at_ms > config.slot_retention_ms) {
                slots[t].has_expired_affinity = false;
            }
        }
    }

    // 3. Multi-pass greedy nearest-neighbor S^1 association
    bool det_assigned[DOA_MAX_SOURCES] = {false};
    bool slot_assigned[SECTOR_TRACKER_MAX_SLOTS] = {false};
    int slot_det_idx[SECTOR_TRACKER_MAX_SLOTS];
    for (size_t t = 0; t < SECTOR_TRACKER_MAX_SLOTS; ++t) {
        slot_det_idx[t] = -1;
    }

    // Pass 1: Associate against confirmed ACTIVE and non-expired COASTING slots
    for (size_t pass = 0; pass < num_dets; ++pass) {
        float best_dist = 1e9f;
        int best_det = -1;
        int best_slot = -1;

        for (size_t d = 0; d < num_dets; ++d) {
            if (det_assigned[d]) continue;
            for (size_t t = 0; t < SECTOR_TRACKER_MAX_SLOTS; ++t) {
                if (slot_assigned[t]) continue;
                if (slots[t].state != TRACK_STATE_ACTIVE && slots[t].state != TRACK_STATE_COASTING) {
                    continue;
                }
                if (now_ms >= slots[t].last_detection_ms &&
                    (now_ms - slots[t].last_detection_ms > config.hold_timeout_ms)) {
                    continue;
                }
                const float dist = std::fabs(
                    shortest_angular_diff(det_angles[d], slots[t].smoothed_angle_deg));
                if (dist < best_dist) {
                    best_dist = dist;
                    best_det = static_cast<int>(d);
                    best_slot = static_cast<int>(t);
                }
            }
        }

        if (best_det >= 0 && best_slot >= 0 && best_dist <= config.association_gate_deg) {
            det_assigned[best_det] = true;
            slot_assigned[best_slot] = true;
            slot_det_idx[best_slot] = best_det;
        } else {
            break;
        }
    }

    // Pass 2: Associate remaining detections against CANDIDATE slots
    for (size_t pass = 0; pass < num_dets; ++pass) {
        float best_dist = 1e9f;
        int best_det = -1;
        int best_slot = -1;

        for (size_t d = 0; d < num_dets; ++d) {
            if (det_assigned[d]) continue;
            for (size_t t = 0; t < SECTOR_TRACKER_MAX_SLOTS; ++t) {
                if (slot_assigned[t] || slots[t].state != TRACK_STATE_CANDIDATE) continue;
                const float dist = std::fabs(
                    shortest_angular_diff(det_angles[d], slots[t].smoothed_angle_deg));
                if (dist < best_dist) {
                    best_dist = dist;
                    best_det = static_cast<int>(d);
                    best_slot = static_cast<int>(t);
                }
            }
        }

        if (best_det >= 0 && best_slot >= 0 && best_dist <= config.association_gate_deg) {
            det_assigned[best_det] = true;
            slot_assigned[best_slot] = true;
            slot_det_idx[best_slot] = best_det;
        } else {
            break;
        }
    }

    // Pass 3: Associate remaining detections against INACTIVE slots holding expired angle affinity
    for (size_t pass = 0; pass < num_dets; ++pass) {
        float best_dist = 1e9f;
        int best_det = -1;
        int best_slot = -1;

        for (size_t d = 0; d < num_dets; ++d) {
            if (det_assigned[d]) continue;
            for (size_t t = 0; t < SECTOR_TRACKER_MAX_SLOTS; ++t) {
                if (slot_assigned[t] ||
                    slots[t].state != TRACK_STATE_INACTIVE ||
                    !slots[t].has_expired_affinity) {
                    continue;
                }
                const float dist = std::fabs(
                    shortest_angular_diff(det_angles[d], slots[t].last_valid_angle_deg));
                if (dist < best_dist) {
                    best_dist = dist;
                    best_det = static_cast<int>(d);
                    best_slot = static_cast<int>(t);
                }
            }
        }

        if (best_det >= 0 && best_slot >= 0 && best_dist <= config.association_gate_deg) {
            det_assigned[best_det] = true;
            slot_assigned[best_slot] = true;
            slot_det_idx[best_slot] = best_det;
        } else {
            break;
        }
    }

    // Pass 4: Assign remaining detections to any free INACTIVE slot
    for (size_t d = 0; d < num_dets; ++d) {
        if (det_assigned[d]) continue;
        for (size_t t = 0; t < SECTOR_TRACKER_MAX_SLOTS; ++t) {
            if (!slot_assigned[t] && slots[t].state == TRACK_STATE_INACTIVE) {
                det_assigned[d] = true;
                slot_assigned[t] = true;
                slot_det_idx[t] = static_cast<int>(d);
                break;
            }
        }
    }

    // 4. Advance per-slot state machines and collect pending transitions
    SectorTransitionEvent pending_events[SECTOR_TRACKER_MAX_SLOTS];
    size_t pending_count = 0;

    for (size_t t = 0; t < SECTOR_TRACKER_MAX_SLOTS; ++t) {
        const int d = slot_det_idx[t];

        if (d >= 0) {
            const float raw_angle = det_angles[d];
            const float raw_conf = det_confs[d];
            const float raw_rms = det_rms[d];

            if (slots[t].state == TRACK_STATE_INACTIVE) {
                filters[t].reset();
                slots[t].state = TRACK_STATE_CANDIDATE;
                slots[t].smoothed_angle_deg = filters[t].update(raw_angle, now_ms);
                slots[t].last_valid_angle_deg = slots[t].smoothed_angle_deg;
                slots[t].smoothed_rms_db = raw_rms;
                slots[t].smoothed_confidence = raw_conf;
                slots[t].birth_hits = 1;
                slots[t].last_detection_ms = now_ms;
                slots[t].pending_sector_id = INVALID_SECTOR_ID;
                slots[t].pending_sector_hits = 0;

                if (slots[t].birth_hits >= config.birth_confirm_frames) {
                    const uint8_t sec_id = quantizeAngleToSector(slots[t].smoothed_angle_deg);
                    slots[t].state = TRACK_STATE_ACTIVE;
                    slots[t].confirmed_sector_id = sec_id;
                    slots[t].last_emitted_ms = now_ms;
                    slots[t].has_expired_affinity = false;

                    pending_events[pending_count++] = {
                        getSectorCenterDeg(sec_id),
                        slots[t].smoothed_rms_db,
                        slots[t].smoothed_confidence,
                        slots[t].slot_id,
                        sec_id,
                        SECTOR_EVENT_ENTER,
                        0
                    };
                }
            } else if (slots[t].state == TRACK_STATE_CANDIDATE) {
                slots[t].smoothed_angle_deg = filters[t].update(raw_angle, now_ms);
                slots[t].last_valid_angle_deg = slots[t].smoothed_angle_deg;
                slots[t].smoothed_rms_db = 0.70f * slots[t].smoothed_rms_db + 0.30f * raw_rms;
                slots[t].smoothed_confidence = 0.70f * slots[t].smoothed_confidence + 0.30f * raw_conf;
                slots[t].last_detection_ms = now_ms;
                slots[t].birth_hits++;

                if (slots[t].birth_hits >= config.birth_confirm_frames) {
                    const uint8_t sec_id = quantizeAngleToSector(slots[t].smoothed_angle_deg);
                    slots[t].state = TRACK_STATE_ACTIVE;
                    slots[t].confirmed_sector_id = sec_id;
                    slots[t].pending_sector_id = INVALID_SECTOR_ID;
                    slots[t].pending_sector_hits = 0;
                    slots[t].last_emitted_ms = now_ms;
                    slots[t].has_expired_affinity = false;

                    pending_events[pending_count++] = {
                        getSectorCenterDeg(sec_id),
                        slots[t].smoothed_rms_db,
                        slots[t].smoothed_confidence,
                        slots[t].slot_id,
                        sec_id,
                        SECTOR_EVENT_ENTER,
                        0
                    };
                }
            } else {
                // TRACK_STATE_ACTIVE or TRACK_STATE_COASTING receiving a valid hit
                slots[t].state = TRACK_STATE_ACTIVE;
                slots[t].smoothed_angle_deg = filters[t].update(raw_angle, now_ms);
                slots[t].last_valid_angle_deg = slots[t].smoothed_angle_deg;
                slots[t].smoothed_rms_db = 0.75f * slots[t].smoothed_rms_db + 0.25f * raw_rms;
                slots[t].smoothed_confidence = 0.75f * slots[t].smoothed_confidence + 0.25f * raw_conf;
                slots[t].last_detection_ms = now_ms;

                if (isWithinSectorHysteresis(raw_angle, slots[t].confirmed_sector_id)) {
                    slots[t].pending_sector_id = INVALID_SECTOR_ID;
                    slots[t].pending_sector_hits = 0;

                    if (config.heartbeat_interval_ms > 0 &&
                        now_ms >= slots[t].last_emitted_ms &&
                        (now_ms - slots[t].last_emitted_ms >= config.heartbeat_interval_ms)) {
                        slots[t].last_emitted_ms = now_ms;
                        pending_events[pending_count++] = {
                            getSectorCenterDeg(slots[t].confirmed_sector_id),
                            slots[t].smoothed_rms_db,
                            slots[t].smoothed_confidence,
                            slots[t].slot_id,
                            slots[t].confirmed_sector_id,
                            SECTOR_EVENT_HEARTBEAT,
                            0
                        };
                    }
                } else {
                    const uint8_t cand_sector = quantizeAngleToSector(raw_angle);
                    if (cand_sector == slots[t].confirmed_sector_id) {
                        slots[t].pending_sector_id = INVALID_SECTOR_ID;
                        slots[t].pending_sector_hits = 0;
                    } else {
                        if (slots[t].pending_sector_id == cand_sector) {
                            slots[t].pending_sector_hits++;
                        } else {
                            slots[t].pending_sector_id = cand_sector;
                            slots[t].pending_sector_hits = 1;
                        }

                        if (slots[t].pending_sector_hits >= config.transition_confirm_frames) {
                            slots[t].confirmed_sector_id = cand_sector;
                            slots[t].pending_sector_id = INVALID_SECTOR_ID;
                            slots[t].pending_sector_hits = 0;
                            slots[t].smoothed_angle_deg = raw_angle;
                            slots[t].last_valid_angle_deg = raw_angle;
                            slots[t].last_emitted_ms = now_ms;

                            pending_events[pending_count++] = {
                                getSectorCenterDeg(cand_sector),
                                slots[t].smoothed_rms_db,
                                slots[t].smoothed_confidence,
                                slots[t].slot_id,
                                cand_sector,
                                SECTOR_EVENT_MOVE,
                                0
                            };
                        }
                    }
                }
            }
        } else {
            // No detection associated with slot t on this frame
            if (slots[t].state == TRACK_STATE_ACTIVE || slots[t].state == TRACK_STATE_COASTING) {
                if (now_ms >= slots[t].last_detection_ms &&
                    (now_ms - slots[t].last_detection_ms > config.hold_timeout_ms)) {
                    const float leave_center_deg = getSectorCenterDeg(slots[t].confirmed_sector_id);
                    const float leave_rms = slots[t].smoothed_rms_db;
                    const float leave_conf = slots[t].smoothed_confidence;

                    slots[t].last_valid_angle_deg = leave_center_deg;
                    slots[t].has_expired_affinity = true;
                    slots[t].expired_at_ms = now_ms;
                    slots[t].state = TRACK_STATE_INACTIVE;
                    slots[t].confirmed_sector_id = INVALID_SECTOR_ID;
                    slots[t].pending_sector_id = INVALID_SECTOR_ID;
                    slots[t].pending_sector_hits = 0;
                    slots[t].birth_hits = 0;
                    slots[t].last_emitted_ms = now_ms;
                    filters[t].reset();

                    pending_events[pending_count++] = {
                        leave_center_deg,
                        leave_rms,
                        leave_conf,
                        slots[t].slot_id,
                        INVALID_SECTOR_ID,
                        SECTOR_EVENT_LEAVE,
                        0
                    };
                } else {
                    slots[t].state = TRACK_STATE_COASTING;
                }
            }
        }
    }

    // 5. Stamp post-transition active speaker count and copy to caller buffer
    const uint8_t active_cnt = getActiveSpeakerCount();
    const size_t out_cnt = (pending_count > max_events) ? max_events : pending_count;
    if (out_events != nullptr) {
        for (size_t i = 0; i < out_cnt; ++i) {
            pending_events[i].active_sources = active_cnt;
            out_events[i] = pending_events[i];
        }
    }
    return out_cnt;
}
