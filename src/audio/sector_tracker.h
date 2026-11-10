#pragma once
#include <cstdint>
#include <cstddef>
#include "config/app_config.h"
#include "audio/gcc_phat.h"
#include "ui/display_filter.h"

/**
 * @brief State transition event codes encoded at byte offset 10 of the 20-byte binary record.
 */
enum SectorEventType : uint8_t {
    SECTOR_EVENT_NONE      = 0,
    SECTOR_EVENT_ENTER     = 1, // New speaker track confirmed in a sector
    SECTOR_EVENT_MOVE      = 2, // Active speaker transitioned to a new confirmed sector
    SECTOR_EVENT_HEARTBEAT = 3, // Periodic keep-alive while speaker remains active in sector
    SECTOR_EVENT_LEAVE     = 4  // Speaker track expired after conversational silence hold timeout
};

/**
 * @brief Internal lifecycle state of a speaker slot.
 */
enum TrackLifecycleState : uint8_t {
    TRACK_STATE_INACTIVE  = 0, // Slot is free (may hold expired angle affinity up to slot_retention_ms)
    TRACK_STATE_CANDIDATE = 1, // Accumulating consistent birth detections (no network events yet)
    TRACK_STATE_ACTIVE    = 2, // Confirmed speaker actively voiced
    TRACK_STATE_COASTING  = 3  // Confirmed speaker in conversational pause (<= hold_timeout_ms)
};

/**
 * @brief Configuration parameters for MultiSourceSectorTracker.
 */
struct SectorTrackerConfig {
    float    sector_width_deg;          // Discrete sector width in degrees (default: 15.0f -> 24 sectors)
    float    hysteresis_deg;            // Schmitt-trigger margin beyond sector border (default: 3.0f deg)
    float    association_gate_deg;      // Nearest-neighbor S^1 association gate (default: 45.0f deg)
    float    ema_alpha;                 // Circular S^1 smoothing factor (default: 0.25f)
    uint8_t  birth_confirm_frames;      // Consecutive/windowed hits to confirm ENTER (default: 3)
    uint32_t birth_window_ms;           // Max gap between candidate birth hits (default: 400 ms)
    uint8_t  transition_confirm_frames; // Hits outside hysteresis zone to confirm MOVE (default: 3)
    uint32_t hold_timeout_ms;           // Conversational speech-pause hold before LEAVE (default: 2500 ms)
    uint32_t slot_retention_ms;         // Post-expiration slot_id angle affinity memory (default: 5000 ms)
    uint32_t heartbeat_interval_ms;     // Periodic HEARTBEAT interval while active (default: 5000 ms, 0 = disabled)

    SectorTrackerConfig()
        : sector_width_deg(DEFAULT_SECTOR_WIDTH_DEG),
          hysteresis_deg(DEFAULT_SECTOR_HYSTERESIS_DEG),
          association_gate_deg(DEFAULT_TRACKER_ASSOC_GATE_DEG),
          ema_alpha(DEFAULT_TRACKER_EMA_ALPHA),
          birth_confirm_frames(DEFAULT_TRACKER_BIRTH_FRAMES),
          birth_window_ms(DEFAULT_TRACKER_BIRTH_WINDOW_MS),
          transition_confirm_frames(DEFAULT_TRACKER_TRANSITION_FRAMES),
          hold_timeout_ms(DEFAULT_TRACKER_HOLD_TIMEOUT_MS),
          slot_retention_ms(DEFAULT_TRACKER_SLOT_RETENTION_MS),
          heartbeat_interval_ms(DEFAULT_TRACKER_HEARTBEAT_MS) {}
};

/**
 * @brief Per-slot state for a tracked speaker.
 */
struct TrackSectorState {
    TrackLifecycleState state;
    uint8_t  slot_id;                // Fixed slot index (0 or 1)
    uint8_t  confirmed_sector_id;    // Locked sector index (0 .. num_sectors-1) or INVALID_SECTOR_ID
    uint8_t  pending_sector_id;      // Candidate new sector index outside hysteresis band
    uint8_t  pending_sector_hits;    // Consecutive hits supporting pending_sector_id
    uint8_t  birth_hits;             // Accumulated birth hits in TRACK_STATE_CANDIDATE
    float    smoothed_angle_deg;     // Circular S^1 smoothed azimuth [-180, +180]
    float    last_valid_angle_deg;   // Retained azimuth for post-LEAVE slot affinity matching
    float    smoothed_rms_db;        // Smoothed digital RMS in dBFS
    float    smoothed_confidence;    // Smoothed correlation confidence [0, 1]
    uint32_t last_detection_ms;      // Monotonic timestamp of most recent associated acoustic hit
    uint32_t last_emitted_ms;        // Monotonic timestamp of most recent network event (for HEARTBEAT)
    uint32_t expired_at_ms;          // Monotonic timestamp when track transitioned to INACTIVE (LEAVE)
    bool     has_expired_affinity;   // True if slot recently expired and holds angle affinity
};

/**
 * @brief Discrete state-transition event produced by MultiSourceSectorTracker.
 */
struct SectorTransitionEvent {
    float           angle_deg;       // Quantized sector center angle in [-180.0, +180.0] deg
    float           rms_db;          // Digital RMS level in dBFS
    float           confidence;      // Tracker confidence in [0.0, 1.0]
    uint8_t         slot_id;         // Persistent speaker slot (0 or 1)
    uint8_t         sector_id;       // Confirmed sector index (0..N-1, or 0xFF on LEAVE)
    SectorEventType event_type;      // 1=ENTER, 2=MOVE, 3=HEARTBEAT, 4=LEAVE
    uint8_t         active_sources;  // Count of currently active/coasting speakers (0..2) after event
};

/**
 * @brief Stateful 2-speaker circular sector tracker with Schmitt-trigger boundary hysteresis.
 */
class MultiSourceSectorTracker {
public:
    explicit MultiSourceSectorTracker(const SectorTrackerConfig& cfg = SectorTrackerConfig());

    void configure(const SectorTrackerConfig& cfg);
    void updateCalibration(float sector_width_deg, float hysteresis_deg, uint32_t hold_timeout_ms);
    const SectorTrackerConfig& getConfig() const { return config; }
    void reset();

    /**
     * @brief Processes a multi-source DOA frame (or silence when multi_res.valid == false)
     *        and emits any resulting state transition events (up to SECTOR_TRACKER_MAX_SLOTS).
     *
     * @param multi_res Multi-source output from GccPhat::processMulti().
     * @param now_ms Current monotonic time in milliseconds.
     * @param out_events Caller-supplied buffer of size >= SECTOR_TRACKER_MAX_SLOTS.
     * @param max_events Capacity of out_events buffer.
     * @return Number of state-transition events emitted (0..SECTOR_TRACKER_MAX_SLOTS).
     */
    size_t update(const MultiDoaResult& multi_res,
                  uint32_t now_ms,
                  SectorTransitionEvent* out_events,
                  size_t max_events);

    uint8_t getNumSectors() const;
    uint8_t quantizeAngleToSector(float angle_deg) const;
    float   getSectorCenterDeg(uint8_t sector_id) const;
    bool    isWithinSectorHysteresis(float angle_deg, uint8_t sector_id) const;

    uint8_t getActiveSpeakerCount() const;
    const TrackSectorState& getSlotState(size_t slot_id) const;

    /**
     * @brief Populates out_targets with currently confirmed ACTIVE or COASTING speaker tracks
     *        at their quantized sector center angles and persistent slot_ids (0 or 1) for
     *        local TFT radar display rendering.
     *
     * @param out_targets Caller-supplied array of TrackedTarget.
     * @param max_targets Capacity of out_targets array.
     * @return Number of active/coasting targets written (0..SECTOR_TRACKER_MAX_SLOTS).
     */
    size_t getActiveTargets(TrackedTarget* out_targets, size_t max_targets) const;

private:
    SectorTrackerConfig    config;
    DisplaySmoothingFilter filters[SECTOR_TRACKER_MAX_SLOTS];
    TrackSectorState       slots[SECTOR_TRACKER_MAX_SLOTS];
};
