### 1. Executive Summary & Architectural Overview

The `acoustic-doa-esp32` firmware is a sophisticated embedded signal-processing system targeting the dual-core ESP32-S3 (N16R8). Its high-level architecture—separating real-time 4-channel I2S DMA capture and 6-pair wideband GCC-PHAT / SRP-PHAT / CLEAN deconvolution on Core 1 (`dsp_task`) from SPI TFT rendering and I2C telemetry on Core 0 (`sensor_display_task`) and binary batch HTTP transmission (`network_task`)—demonstrates strong mechanical sympathy for dual-core FreeRTOS constraints. Static internal SRAM allocation for FFT working buffers and an off-target ASAN/UBSAN test harness are commendable engineering practices.

However, a deep-dive audit across the DSP pipeline, FreeRTOS synchronization primitives, and networking stack reveals several **critical correctness, memory-safety, and availability flaws**. Most notably:
1. **Stack Buffer Overflow in Multi-Source DSP (`GccPhat::processMulti`)**: The local peak estimator (`estimateLocalPeakAmplitude`) and SRP-seeded WLS refiner (`solveFromSrpSeed`) use a fixed stack lag buffer (`PAIR_CORR_RADIUS = 22`, 45 elements) without bounding the initial `best_idx` or clamping both neighbors before indexing. Any runtime-configured microphone spacing $d \ge 0.13\text{ m}$ (which passes all configuration validators) or post-CLEAN correlation window with values $\le -1.0$ causes an immediate out-of-bounds stack read (verified via AddressSanitizer).
2. **Broken Calibration & Threshold Gates**: In `GccPhat::processMulti()`, when a primary candidate fails the configured `correlation_threshold` (default `0.45`), the Pass-0 "chimera fallback" re-evaluates the frame using `sec_threshold` (`0.20`) and validates the primary source against `sec_threshold` instead of `correlation_threshold`—effectively bypassing user/API correlation thresholds down to `0.20`. Conversely, `load_device_config()` and `apply_dsp_calibration()` silently clamp any configured threshold below `0.45` back up to `0.45`. Simultaneously, `dsp_task` enforces the fixed SPL threshold (`sound_level_threshold_db_spl`) even when adaptive noise floor tracking (`AUTO`) is enabled.
3. **Systemic Hang & Watchdog Vulnerabilities**: Dual-bus I2S DMA reads use `portMAX_DELAY`, rendering error recovery (`resync_i2s_ports()`) unreachable on clock/bus stalls and hanging the boot self-test before the Task Watchdog is initialized. Meanwhile, `RestClient::sendBatch()` reuses an `HTTPClient` instance without draining 2xx response bodies or resetting request state, causing keep-alive stream desynchronization on every second batch, while `network_task` can exceed the 30-second watchdog window during backend outages.

---

### 2. Critical & High-Priority Findings (Triage Matrix)

| ID | Category | File / Component | Summary of Issue | Severity |
|---|---|---|---|---|
| **CRIT-01** | Memory Safety / DSP | `src/audio/gcc_phat.cpp` (`estimateLocalPeakAmplitude`, `solveFromSrpSeed`) | Stack buffer overflow/underflow on `pair_corr[6][45]` when theoretical lag exceeds $\pm 22$ samples ($d \ge 0.13\text{ m}$) or window values are $\le -1.0$. | **Critical** |
| **CRIT-02** | DSP Logic / Calibration | `src/audio/gcc_phat.cpp` (`processMulti`), `src/main.cpp`, `src/config/config_store.cpp` | Pass-0 SRP fallback validates primary source against `sec_threshold` (`0.20f`), bypassing `correlation_threshold`; `config_store.cpp` and `main.cpp` clamp values $< 0.45$ to `0.45`. | **Critical** |
| **CRIT-03** | Concurrency / Hardware | `src/audio/i2s_sampler.cpp` (`init_i2s_microphones`, `read_i2s_quad`) | `i2s_read` with `portMAX_DELAY` blocks indefinitely on Slave RX clock fault, hanging boot POST before WDT init and making `resync_i2s_ports()` dead code. | **Critical** |
| **HIGH-01** | Network / HTTP Client | `src/network/rest_client.cpp` (`RestClient::sendBatch`) | Unread HTTP 2xx response bodies on reused `HTTPClient` corrupt the TCP stream for subsequent `POST` requests (`HTTPC_ERROR_INVALID_SERVER`). | **High** |
| **HIGH-02** | Time Sync / Data Integrity | `src/network/rest_client.cpp` (`fetchCalibration`), `src/main.cpp` (`network_task`) | Non-monotonic `server_utc_offset_ms` updates on 60s calibration polls cause negative intra-batch timestamp deltas, serialization failure, false `API OFFLINE`, and batch loss. | **High** |
| **HIGH-03** | FreeRTOS / Watchdog | `src/main.cpp` (`network_task`, `sensor_display_task`) | `network_task` blocks up to 33–43s across calibration + 3 HTTP retries without resetting the 30s WDT; `sensor_display_task` calls `esp_task_wdt_reset()` without `esp_task_wdt_add()`. | **High** |
| **HIGH-04** | DSP / Threshold Gating | `src/main.cpp` (`dsp_task`), `src/audio/gcc_phat.cpp` (`getEffectiveThresholdDb`) | `dsp_task` enforces fixed `sound_level_threshold_db_spl` even in adaptive (`AUTO`) mode, dropping valid adaptive events after running full FFT/CLEAN DSP. | **High** |
| **MED-01** | UI / Target Tracker | `src/ui/display_filter.cpp` (`MultiTargetDisplayTracker`), `src/ui/radar_ui.cpp` | Conflating 1500 ms EMA reset timeout with active target presence freezes dB/SNR telemetry, forces 20 Hz SPI redraws in silence, and delays 500 ms phosphor decay by 1.5s. | **Medium** |
| **MED-02** | Security / Provisioning | `src/network/setup_portal.cpp` (`generateHtmlPage`, `enter`), `src/config/config_store.cpp` | Unescaped HTML attributes break SSIDs containing `'` and allow stored XSS; `NaN`/`Inf` floats bypass `<= 0.0f` checks; `save_device_config` omits `preferences.begin()`. | **Medium** |
| **MED-03** | Algorithmic Contracts | `src/audio/gcc_phat.cpp` (`process`, `processMulti`), `src/ui/radar_ui.cpp` | 2-channel `process(left, right)` wrapper locks to $\pm 45^\circ / \pm 135^\circ$; `MultiDoaResult::sources` is unsorted; Pass 1..2 clobbers `last_pair_*` getters; POST mislabels `-95 dBFS` flatline as `RAIL SHORT`. | **Medium** |

---

### 3. Deep-Dive Issues & Conceptual Flaws

---

#### [CRIT-01] Stack Buffer Overflow/Underflow in `estimateLocalPeakAmplitude` and `solveFromSrpSeed`
- **Location:** `src/audio/gcc_phat.cpp` (`estimateLocalPeakAmplitude` at lines 37–58; `GccPhat::solveFromSrpSeed` at lines 486–505; `GccPhat::evaluateSrpPhat` at lines 439–445).
- **The Problem:**
  `GccPhat::processMulti()` caches cross-correlation lags on the stack in `float pair_corr[AUDIO_NUM_PAIRS][PAIR_CORR_SIZE]`, where `PAIR_CORR_RADIUS = 22` (`PAIR_CORR_SIZE = 45`, covering lags $[-22, +22]$).
  In both `estimateLocalPeakAmplitude()` and `solveFromSrpSeed()`:
  ```cpp
  const int center_lag = static_cast<int>(roundf(center_float_lag));
  int best_idx = center_lag;
  float best_val = -1.0f;

  for (int l = center_lag - search_radius; l <= center_lag + search_radius; ++l) {
      if (l < -PAIR_CORR_RADIUS || l > PAIR_CORR_RADIUS) continue;
      const size_t k = static_cast<size_t>(l + PAIR_CORR_RADIUS);
      const float val = pair_corr[k];
      if (val > best_val) {
          best_val = val;
          best_idx = l;
      }
  }

  const int l_m1 = (best_idx - 1 >= -PAIR_CORR_RADIUS) ? (best_idx - 1) : best_idx;
  const int l_p1 = (best_idx + 1 <= PAIR_CORR_RADIUS) ? (best_idx + 1) : best_idx;
  const float alpha = pair_corr[static_cast<size_t>(l_m1 + PAIR_CORR_RADIUS)];
  const float beta  = best_val;
  const float gamma = pair_corr[static_cast<size_t>(l_p1 + PAIR_CORR_RADIUS)];
  ```
  There are two fatal flaws here:
  1. `microphone_distance_m` (`d`) has no upper-bound validation in `rest_client.cpp`, `setup_portal.cpp`, or `apply_dsp_calibration()`. For any array with $d \ge 0.13\text{ m}$ at $f_s = 48000\text{ Hz}$ (e.g., $d = 0.14\text{ m}$ at $-45^\circ$), the diagonal baseline lag is $\frac{d\sqrt{2}}{c} f_s \approx 27.6\text{ samples}$ (`center_lag = 28`). Every iteration of the `for` loop hits `l > PAIR_CORR_RADIUS` and `continue`s, leaving `best_idx = 28`.
  2. Even at the default $d = 0.082\text{ m}$, if CLEAN subtraction in Pass 0/1 pushes the local correlation values in the search window to $\le -1.0$, `val > best_val` is never satisfied, leaving `best_idx = center_lag`.
  3. Subsequently, `l_m1` only checks `>= -PAIR_CORR_RADIUS` (so for `best_idx = 28`, `l_m1 = 27`, indexing `pair_corr[49]` on a 45-element array!) and `l_p1` only checks `<= PAIR_CORR_RADIUS` (so for `best_idx = -28`, `l_m1 = -28`, `l_m1 + 22 = -6`, which casts to `size_t` `SIZE_MAX - 5` and causes an immediate wild pointer read / `LoadProhibited` panic on ESP32-S3).
- **Root Cause:** Implementation bug in index clamping combined with missing upper-bound validation on `microphone_distance_m` relative to `PAIR_CORR_RADIUS`.
- **Recommended Remediation:**
  1. Enforce a physical upper bound `MAX_MIC_DISTANCE_M` in `app_config.h` such that $\lceil \frac{d_{\max}\sqrt{2}}{c_{\min}} f_{s,\max} \rceil + 3 \le \text{PAIR\_CORR\_RADIUS}$, or increase `PAIR_CORR_RADIUS` to `32`.
  2. Clamp `best_idx`, `l_m1`, and `l_p1` on both bounds before indexing `pair_corr`, and initialize `best_val = -INFINITY`:
  ```cpp
  // Before (src/audio/gcc_phat.cpp:39-57)
  const int center_lag = static_cast<int>(roundf(center_float_lag));
  int best_idx = center_lag;
  float best_val = -1.0f;
  for (int l = center_lag - search_radius; l <= center_lag + search_radius; ++l) {
      if (l < -PAIR_CORR_RADIUS || l > PAIR_CORR_RADIUS) continue;
      const size_t k = static_cast<size_t>(l + PAIR_CORR_RADIUS);
      const float val = pair_corr[k];
      if (val > best_val) {
          best_val = val;
          best_idx = l;
      }
  }
  const int l_m1 = (best_idx - 1 >= -PAIR_CORR_RADIUS) ? (best_idx - 1) : best_idx;
  const int l_p1 = (best_idx + 1 <= PAIR_CORR_RADIUS) ? (best_idx + 1) : best_idx;
  const float alpha = pair_corr[static_cast<size_t>(l_m1 + PAIR_CORR_RADIUS)];
  const float beta  = best_val;
  const float gamma = pair_corr[static_cast<size_t>(l_p1 + PAIR_CORR_RADIUS)];

  // After
  auto clamp_lag = [](int lag) -> int {
      if (lag < -PAIR_CORR_RADIUS) return -PAIR_CORR_RADIUS;
      if (lag > PAIR_CORR_RADIUS) return PAIR_CORR_RADIUS;
      return lag;
  };
  const int center_lag = clamp_lag(static_cast<int>(roundf(center_float_lag)));
  int best_idx = center_lag;
  float best_val = -INFINITY;

  for (int l = center_lag - search_radius; l <= center_lag + search_radius; ++l) {
      if (l < -PAIR_CORR_RADIUS || l > PAIR_CORR_RADIUS) continue;
      const size_t k = static_cast<size_t>(l + PAIR_CORR_RADIUS);
      const float val = pair_corr[k];
      if (val > best_val) {
          best_val = val;
          best_idx = l;
      }
  }
  if (!std::isfinite(best_val)) {
      return 0.0f;
  }

  const int l_m1 = clamp_lag(best_idx - 1);
  const int l_p1 = clamp_lag(best_idx + 1);
  const float alpha = pair_corr[static_cast<size_t>(l_m1 + PAIR_CORR_RADIUS)];
  const float beta  = best_val;
  const float gamma = pair_corr[static_cast<size_t>(l_p1 + PAIR_CORR_RADIUS)];
  ```
  Apply the identical `clamp_lag` and `-INFINITY` initialization in `GccPhat::solveFromSrpSeed()`.

---

#### [CRIT-02] Primary Source `correlation_threshold` Bypass in Pass-0 Fallback & Contradictory Floor Clamping
- **Location:** `src/audio/gcc_phat.cpp` (`GccPhat::processMulti`, lines 598–618); `src/config/config_store.cpp` (`load_device_config`, line 25); `src/main.cpp` (`apply_dsp_calibration`, line 299).
- **The Problem:**
  In `GccPhat::processMulti()`, Pass 0 attempts to solve the primary source azimuth using `solveWlsAzimuth()`, which validates weighted confidence against `correlation_threshold` (default `0.45f`). If that fails, Pass 0 enters the SRP-PHAT fallback:
  ```cpp
  const float sec_threshold = (correlation_threshold < DEFAULT_CLEAN_SECONDARY_THRESHOLD)
                                  ? correlation_threshold
                                  : DEFAULT_CLEAN_SECONDARY_THRESHOLD;

  DoaResult primary = solveWlsAzimuth(pair_lags_samples, pair_confidences, multi_res.rms_db);
  if (!primary.valid || primary.confidence < correlation_threshold) {
      float srp_seed_angle = 0.0f;
      float srp_seed_power = 0.0f;
      evaluateSrpPhat(pair_corr, srp_seed_angle, srp_seed_power);
      if (srp_seed_power >= sec_threshold && std::isfinite(srp_seed_power)) {
          primary = solveFromSrpSeed(pair_corr, srp_seed_angle, scale, multi_res.rms_db, sec_threshold);
      }
  }

  multi_res.sources[0] = primary;
  if (!primary.valid || primary.confidence < sec_threshold) {
      return multi_res;
  }
  ```
  Because `solveFromSrpSeed` is passed `sec_threshold` (`0.20f`) and line 614 checks `primary.confidence < sec_threshold` (`0.20f`), **any single noisy/reverberant source with confidence in `[0.20, correlation_threshold)`** fails the initial `solveWlsAzimuth()`, triggers the fallback, passes `solveFromSrpSeed(..., 0.20f)`, and is returned as a valid primary source! Setting `correlation_threshold = 0.80f` has zero effect on rejecting single sources with confidence `0.25f`.
  Conversely, `config_store.cpp:25` and `main.cpp:299` force `correlation_threshold >= DEFAULT_CORRELATION_THRESHOLD (0.45f)`, silently overriding valid thresholds in `[0.10, 0.45)` configured via the Setup Portal (`setup_portal.cpp:99`) or the REST API (`docs/rest-api.md` specifies `"prag_korelacije": 0.40`).
- **Root Cause:** Architectural confusion between multi-talker chimera recovery (where pairwise argmax picks lags from two different strong sources, causing triangle loop closure to fail while local SRP peaks remain strong) and a genuinely low-confidence single source.
- **Recommended Remediation:**
  1. Only trigger the Pass-0 SRP fallback when pairwise argmax failed due to **loop-closure inconsistency** (chimera split-brain), or require the SRP-seeded primary source to meet a meaningful primary threshold (or `correlation_threshold`).
  2. Define `CORRELATION_THRESHOLD_MIN = 0.10f` and `CORRELATION_THRESHOLD_MAX = 1.0f` in `app_config.h` and use them consistently across `config_store.cpp`, `main.cpp`, `rest_client.cpp`, and `setup_portal.cpp`.
  ```cpp
  // Before (src/main.cpp:299)
  const float correlation_threshold = (device_config.correlation_threshold < DEFAULT_CORRELATION_THRESHOLD)
                                          ? DEFAULT_CORRELATION_THRESHOLD
                                          : device_config.correlation_threshold;

  // After (src/main.cpp:299 & src/config/config_store.cpp:25)
  const float correlation_threshold =
      (!std::isfinite(device_config.correlation_threshold) ||
       device_config.correlation_threshold < CORRELATION_THRESHOLD_MIN ||
       device_config.correlation_threshold > CORRELATION_THRESHOLD_MAX)
          ? DEFAULT_CORRELATION_THRESHOLD
          : device_config.correlation_threshold;
  ```

---

#### [CRIT-03] Unbounded `portMAX_DELAY` in Dual-Bus I2S DMA Reads Defeats Fault Recovery & Hangs Boot POST
- **Location:** `src/audio/i2s_sampler.cpp` (`init_i2s_microphones` at lines 99–100; `read_i2s_quad` at lines 135, 146; `read_i2s_stereo` at lines 169, 181).
- **The Problem:**
  Every `i2s_read()` call passes `portMAX_DELAY` as `ticks_to_wait`:
  ```cpp
  esp_err_t res0 = i2s_read(I2S_MASTER_PORT, interleaved_buf, bytes_requested, &bytes_read_0, portMAX_DELAY);
  if (res0 != ESP_OK || bytes_read_0 != bytes_requested) {
      resync_i2s_ports();
      return false;
  }
  ```
  In ESP-IDF's I2S driver, `i2s_read()` loops over `xQueueReceive(p_i2s[i2s_num]->rx->queue, ..., ticks_to_wait)` until `bytes_requested` bytes are read or `ticks_to_wait` expires. With `portMAX_DELAY`, `i2s_read()` **never times out**:
  - During `run_boot_self_test()` in `setup()` (`src/main.cpp:74`), `init_i2s_microphones()` is called **before** `initialize_task_watchdog()` (`src/main.cpp:585`). If `I2S0` or `I2S1` fails to clock DMA buffers, the MCU hangs permanently in `setup()` before rendering the `HARDWARE DEGRADED` screen.
  - During `dsp_task()`, a stalled I2S bus blocks `dsp_task` forever while holding `i2s_mutex`, never incrementing `dsp_i2s_read_failures`, never calling `resync_i2s_ports()`, and causing a 30-second Task Watchdog reboot.
- **Root Cause:** Misunderstanding of ESP-IDF `i2s_read()` blocking semantics when `ticks_to_wait == portMAX_DELAY`.
- **Recommended Remediation:**
  Use a bounded timeout (e.g., `pdMS_TO_TICKS(100)`—roughly $4\times$ the 23.2 ms audio block duration) for all `i2s_read()` calls so incomplete/stalled DMA transfers time out deterministically, trigger `resync_i2s_ports()`, and release `i2s_mutex`:
  ```cpp
  // Before (src/audio/i2s_sampler.cpp:135-150)
  esp_err_t res0 = i2s_read(I2S_MASTER_PORT, interleaved_buf, bytes_requested, &bytes_read_0, portMAX_DELAY);
  if (res0 != ESP_OK || bytes_read_0 != bytes_requested) {
      resync_i2s_ports();
      return false;
  }
  ...
  esp_err_t res1 = i2s_read(I2S_SLAVE_PORT, interleaved_buf, bytes_requested, &bytes_read_1, portMAX_DELAY);

  // After
  constexpr TickType_t I2S_READ_TIMEOUT_TICKS = pdMS_TO_TICKS(100);
  esp_err_t res0 = i2s_read(I2S_MASTER_PORT, interleaved_buf, bytes_requested, &bytes_read_0, I2S_READ_TIMEOUT_TICKS);
  if (res0 != ESP_OK || bytes_read_0 != bytes_requested) {
      resync_i2s_ports();
      return false;
  }
  for (size_t i = 0; i < count; i++) {
      ch0[i] = interleaved_buf[2 * i];
      ch1[i] = interleaved_buf[2 * i + 1];
  }

  esp_err_t res1 = i2s_read(I2S_SLAVE_PORT, interleaved_buf, bytes_requested, &bytes_read_1, I2S_READ_TIMEOUT_TICKS);
  if (res1 != ESP_OK || bytes_read_1 != bytes_requested) {
      resync_i2s_ports();
      return false;
  }
  ```

---

#### [HIGH-01] HTTP Keep-Alive Socket Corruption from Unread 2xx Response Bodies in `RestClient::sendBatch`
- **Location:** `src/network/rest_client.cpp` (`RestClient::sendBatch`, lines 18–64).
- **The Problem:**
  `RestClient::sendBatch()` initializes `HTTPClient http` once (`http_initialized = true`) and calls `http.POST(payload, payload_size)` repeatedly without calling `http.end()` or reading the response payload on `2xx` status codes:
  ```cpp
  const int httpResponseCode = http.POST(payload, payload_size);

  if (httpResponseCode < 200 || httpResponseCode >= 300) {
      const String response = http.getString();
      Serial.printf("HTTP POST /meritve failed: %d, response: %s\n",
                    httpResponseCode, response.c_str());
      http.end();
      http_initialized = false;
      return false;
  }
  ```
  Per `docs/rest-api.md`, the backend returns a JSON body on 2xx (e.g., `{"status":"success","dogodek_id":4821,...}`). In Arduino-ESP32's `HTTPClient`, `http.POST()` only parses the response headers; it does **not** consume the response body until `http.getString()` or `http.end()` is called. When `http.setReuse(true)` is enabled, calling `http.end()` drains the remaining response bytes while keeping the underlying TCP socket open for reuse. By skipping `http.end()` on 2xx, the unread JSON body remains in the TCP receive buffer. On the next `sendBatch()` call, `HTTPClient::handleHeaderResponse()` reads the leftover JSON body instead of `HTTP/1.1 200 OK`, fails with `HTTPC_ERROR_INVALID_SERVER (-11)`, closes the socket, and forces a 1-second backoff retry.
  Additionally, if `api_url` or `api_key` is updated while `http_initialized == true`, `RestClient` continues using the stale URL and API key.
- **Root Cause:** Misuse of the Arduino-ESP32 `HTTPClient` keep-alive lifecycle API.
- **Recommended Remediation:**
  Track the configured URL/key or call `http.end()` after every request (which preserves the TCP socket when `setReuse(true)` is active while draining the response stream):
  ```cpp
  // After (src/network/rest_client.cpp)
  if (!http_initialized || api_url != current_url || api_key != current_key) {
      close();
      http.setReuse(true);
      http.useHTTP10(false);
      http.setTimeout(5000);
      current_url = api_url;
      current_key = api_key;
      http_initialized = true;
  }

  String url = current_url + "/meritve";
  if (!http.begin(url)) {
      Serial.println("HTTP client initialization failed");
      http_initialized = false;
      return false;
  }
  http.addHeader("Content-Type", "application/octet-stream");
  http.addHeader("Connection", "keep-alive");
  http.addHeader("x-api-key", current_key);

  const int httpResponseCode = http.POST(payload, payload_size);
  if (httpResponseCode < 200 || httpResponseCode >= 300) {
      const String response = http.getString();
      Serial.printf("HTTP POST /meritve failed: %d, response: %s\n",
                    httpResponseCode, response.c_str());
      close();
      return false;
  }

  http.end(); // With setReuse(true), drains unread 2xx body while keeping TCP socket alive
  return true;
  ```

---

#### [HIGH-02] Non-Monotonic `server_utc_offset_ms` Updates Cause Batch Serialization Failures & Data Loss
- **Location:** `src/network/rest_client.cpp` (`fetchCalibration`, lines 94–104); `src/network/rest_protocol.h` (`serialize_measurement_batch`, lines 83–88); `src/main.cpp` (`network_task`, lines 493–504).
- **The Problem:**
  Every 60 seconds, `fetchCalibration()` unconditionally overwrites `server_utc_offset_ms`:
  ```cpp
  const int64_t midpoint_ms = request_start_ms + (request_end_ms - request_start_ms) / 2;
  if (server_time_ms >= static_cast<uint64_t>(MIN_VALID_EPOCH) * 1000ULL && midpoint_ms >= 0) {
      server_utc_offset_ms.store(static_cast<int64_t>(server_time_ms) - midpoint_ms);
      server_time_valid.store(true);
  }
  ```
  Because HTTP RTT over Wi-Fi fluctuates by 5–100 ms between polls, `server_utc_offset_ms` can step backwards by tens of milliseconds while `server_time_valid` is already `true`. If `dsp_task` enqueues measurements across a calibration poll, `batch[i].timestamp_ms` can be smaller than `batch[0].timestamp_ms` (`base_timestamp_ms`) while sharing the same `clock_flags` (`0x0003`).
  When `serialize_measurement_batch()` encounters `measurements[index].timestamp_ms < base_timestamp_ms`, it returns `0`. `network_task` then retries `rest_client.sendBatch()` 3 times with 1s + 2s sleeps on a batch that can never serialize, sets `server_available.store(false)` (`API OFFLINE`), and drops all 24 measurements.
- **Root Cause:** Lack of monotonic clamping or EWMA filtering on `server_utc_offset_ms` updates, and conflating local serialization errors with transient HTTP transport errors in `network_task`.
- **Recommended Remediation:**
  1. Only set `server_utc_offset_ms` on initial sync (matching SNTP behavior) or clamp/monotonically smooth offset updates so estimated UTC never steps backwards.
  2. In `serialize_measurement_batch()` (or `network_task`), clamp `delta_ms` to be non-decreasing within a batch, and do not retry or mark the server offline on a local serialization error.
  ```cpp
  // Before (src/network/rest_client.cpp:98-100)
  if (server_time_ms >= static_cast<uint64_t>(MIN_VALID_EPOCH) * 1000ULL && midpoint_ms >= 0) {
      server_utc_offset_ms.store(static_cast<int64_t>(server_time_ms) - midpoint_ms);
      server_time_valid.store(true);
  }

  // After
  if (server_time_ms >= static_cast<uint64_t>(MIN_VALID_EPOCH) * 1000ULL && midpoint_ms >= 0) {
      const int64_t new_offset_ms = static_cast<int64_t>(server_time_ms) - midpoint_ms;
      if (!server_time_valid.load()) {
          server_utc_offset_ms.store(new_offset_ms);
          server_time_valid.store(true);
      }
  }
  ```

---

#### [HIGH-03] Task Watchdog (TWDT) Starvation in `network_task` and Unsubscribed Reset in `sensor_display_task`
- **Location:** `src/main.cpp` (`network_task` at lines 395–507; `sensor_display_task` at lines 548–652).
- **The Problem:**
  1. In `sensor_display_task()`, `esp_task_wdt_add(nullptr)` is never called at task startup, yet `esp_task_wdt_reset()` is called at the bottom of the loop (`line 650`) every 50 ms. This fails with `ESP_ERR_NOT_FOUND` on every iteration and leaves `sensor_display_task` unmonitored by the watchdog if the I2C or SPI bus hangs.
  2. In `network_task()`, `esp_task_wdt_reset()` is only called at the very bottom of the loop (`line 506`). When the backend server IP is unreachable on an active Wi-Fi network, a single loop iteration can execute `fetchCalibration()` (up to 5s TCP connect timeout + 5s HTTP timeout = 10s) followed immediately by 3 `sendBatch()` attempts (3 $\times$ 10s + 1s + 2s backoff = 33s), totaling up to **43 seconds** without feeding the 30-second Task Watchdog (`TASK_WATCHDOG_TIMEOUT_SECONDS = 30`), triggering a watchdog panic.
- **Root Cause:** Missing `esp_task_wdt_add(nullptr)` in `sensor_display_task` and coarse-grained WDT resets in `network_task`.
- **Recommended Remediation:**
  Subscribe `sensor_display_task` to the TWDT and feed the watchdog after `fetchCalibration()` and between `sendBatch()` retry attempts:
  ```cpp
  // In sensor_display_task (src/main.cpp:560)
  esp_task_wdt_add(nullptr);

  // In network_task (src/main.cpp:460 & 493-500)
  last_calibration_refresh = millis();
  esp_task_wdt_reset();
  ...
  for (int attempt = 1; attempt <= HTTP_RETRY_COUNT && !sent; ++attempt) {
      esp_task_wdt_reset();
      sent = rest_client.sendBatch(batch, batch_count, current_api_url, current_api_key, db_spl_offset.load());
      esp_task_wdt_reset();
      if (!sent && attempt < HTTP_RETRY_COUNT) {
          vTaskDelay(pdMS_TO_TICKS(1000 * attempt));
      }
  }
  ```

---

#### [HIGH-04] Fixed SPL Threshold Gate in `dsp_task` Overrides Adaptive Noise Floor (`AUTO`) Mode
- **Location:** `src/main.cpp` (`dsp_task`, lines 234–235, 269–270); `src/audio/gcc_phat.cpp` (`getEffectiveThresholdDb`, lines 208–216).
- **The Problem:**
  In `GccPhat::updateNoiseFloorAndCheckGate()`, when `adaptive_threshold_enabled` is `true` (`AUTO` mode), acoustic frames with `rms_db >= noise_floor_db + snr_margin_db` pass the energy gate and execute the full 4-channel FFT, 6-pair IFFT, SRP-PHAT, CLEAN, and WLS pipeline.
  However, in `dsp_task()` (`src/main.cpp:269–270`):
  ```cpp
  const float sound_spl = multi_res.rms_db + db_spl_offset.load();
  if (multi_res.valid && multi_res.count > 0 && sound_spl >= min_spl_threshold) {
      enqueue_latest(display_queue, multi_res, display_queue_drops);
  ```
  `dsp_task` unconditionally requires `sound_spl >= min_spl_threshold` (`device_config.sound_level_threshold_db_spl`, default `55.0 dB SPL`) even when `adaptive_threshold_enabled` is `true`. In a quiet room (e.g., `35 dB SPL` noise floor + `8 dB` SNR margin = `43 dB SPL` trigger), any sound between `43 dB SPL` and `55 dB SPL` incurs the full DSP computation cost in `GccPhat::processMulti()` only to be silently discarded at line 270.
  Additionally, in `GccPhat::getEffectiveThresholdDb()`, when `adaptive_threshold_enabled` is `false` (`FIX` mode), `fixed_dbfs` is clamped to `NOISE_FLOOR_MAX_DB = -20.0f` dBFS and uses `DEFAULT_DB_SPL_OFFSET` instead of the configured offset if `db_threshold >= 0.0f`.
- **Root Cause:** Duplicate, inconsistent threshold gating split between `GccPhat::updateNoiseFloorAndCheckGate()` and `dsp_task()`.
- **Recommended Remediation:**
  Gate solely inside `GccPhat::updateNoiseFloorAndCheckGate()` (or only enforce `sound_spl >= min_spl_threshold` in `dsp_task` when `!adaptive_threshold_active.load()`):
  ```cpp
  // Before (src/main.cpp:269-271)
  const float sound_spl = multi_res.rms_db + db_spl_offset.load();
  if (multi_res.valid && multi_res.count > 0 && sound_spl >= min_spl_threshold) {

  // After
  const float sound_spl = multi_res.rms_db + db_spl_offset.load();
  const bool spl_gate_ok = adaptive_threshold_active.load() || (sound_spl >= min_spl_threshold);
  if (multi_res.valid && multi_res.count > 0 && spl_gate_ok) {
  ```

---

#### [MED-01] `MultiTargetDisplayTracker` Conflates 1500 ms EMA Reset Timeout with Active Target Presence
- **Location:** `src/ui/display_filter.cpp` (`MultiTargetDisplayTracker::prune`, lines 108–115); `src/main.cpp` (`sensor_display_task`, lines 578–647); `src/ui/radar_ui.cpp` (`RadarDisplay::drawRadarMulti`, lines 122–145).
- **The Problem:**
  `MultiTargetDisplayTracker::prune()` keeps `track_active[i] = true` for `DisplaySmoothingFilter::DEFAULT_RESET_SILENCE_MS = 1500 ms` after the last valid acoustic frame. In `sensor_display_task()`, when no new events arrive from `display_queue`, it queries `tracker.getActiveTracks(..., now_ms)` which returns `num_tracks > 0` for 1.5 seconds of complete silence. This causes three cascading UI bugs:
  1. `has_detection = (num_tracks > 0)` remains `true` for 1.5s of silence, freezing the displayed dB SPL and SNR bar at `last_event.rms_db` instead of returning to the ambient noise floor.
  2. `needs_redraw` is set to `true` on every 50 ms queue timeout (`20 Hz`), transferring 40 KB over SPI 30 times during silence.
  3. `RadarDisplay::drawRadarMulti()` sees `count > 0` on every 50 ms redraw during those 1.5 seconds and executes `hit_blips[b].timestamp_ms = now_ms`, preventing the 500 ms phosphor decay trail from even starting until 1,500 ms after the sound ended.
- **Root Cause:** Conflating the filter state retention window (1500 ms for EMA continuity) with active needle/telemetry visibility (~150–250 ms hold time).
- **Recommended Remediation:**
  Separate track active visibility (`TRACK_HOLD_TIMEOUT_MS = 250`) from EMA filter reset (`DEFAULT_RESET_SILENCE_MS = 1500`), and only refresh phosphor blips in `sensor_display_task` when a new event is actually dequeued from `display_queue`.

---

#### [MED-02] Captive Portal HTML Injection, Unvalidated `NaN`/`Inf` Inputs, and Missing `preferences.begin()`
- **Location:** `src/network/setup_portal.cpp` (`generateHtmlPage` at lines 3–35; `enter` at lines 59–120); `src/config/config_store.cpp` (`save_device_config` at lines 53–68).
- **The Problem:**
  1. `SetupPortal::generateHtmlPage()` concatenates `config.wifi_ssid` and `config.api_url` directly into single-quoted HTML attributes (`value='" + config.wifi_ssid + "'`). Any SSID containing an apostrophe (e.g., `Mateo's iPhone`) breaks out of the `value` attribute, truncating the SSID on subsequent saves and allowing stored XSS.
  2. In `/save`, `config.microphone_distance_m` and `config.db_spl_offset` are parsed via `.toFloat()` without `std::isfinite()` or upper-bound checks. In IEEE-754 arithmetic, `NaN <= 0.0f` evaluates to `false`, allowing `NaN` or `Inf` (or `mic_distance = 1.0f`, triggering **CRIT-01**) to be persisted to NVS.
  3. `save_device_config()` in `src/config/config_store.cpp` never calls `preferences.begin("acoustic", false)`, relying on `load_device_config()` having left the global handle open forever.
- **Root Cause:** Missing input sanitization and reliance on implicit global NVS handle state.
- **Recommended Remediation:**
  Add an HTML attribute escaper in `setup_portal.cpp`, validate all float fields with `std::isfinite()` and explicit `[min, max]` bounds, and scope `preferences.begin()` / `preferences.end()` inside both `load_device_config()` and `save_device_config()`:
  ```cpp
  // Helper in src/network/setup_portal.cpp
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
  ```

---

#### [MED-03] Broken 2-Channel Compatibility Wrapper, Unsorted Multi-Source Output, Clobbered Diagnostics, and POST Flatline Label Bug
- **Location:** `src/audio/gcc_phat.cpp` (`process` at lines 526–529; `processMulti` at lines 604–675); `src/ui/radar_ui.cpp` (`drawHardwareDegraded` at line 447).
- **The Problem:**
  1. **2-Channel Wrapper (`GccPhat::process(left, right)`)**: Calls `process(left, right, left, right)`, placing `left` on diagonal corners (`M0`, `M2`) and `right` on (`M1`, `M3`). This yields $g_x = \pm \tau d$ and $g_y = \pm \tau d$, forcing $\text{atan2}(u_x, u_y)$ to lock exclusively to $\pm 45^\circ$ or $\pm 135^\circ$ for any non-zero inter-channel delay $\tau$.
  2. **Unsorted `MultiDoaResult::sources` & Clobbered `last_pair_*` State**: `processMulti()` never sorts `multi_res.sources[0..count-1]` by `confidence` descending as documented in `gcc_phat.h` and `docs/firmware.md`. Furthermore, Pass 1..2 calls `solveFromSrpSeed()` $\rightarrow$ `solveWlsAzimuth()`, which overwrites `last_pair_lags_samples`, `last_pair_confidences`, `last_pair_psi`, and `last_pair_weights` of the primary source even when the secondary candidate is rejected.
  3. **POST Flatline Misdiagnosis on TFT**: `evaluate_single_channel()` marks a channel failed as flatline if `out_min == out_max` **or** `ac_rms_db <= POST_MIC_FLATLINE_RMS_DB (-95.0f)`. However, `RadarDisplay::drawHardwareDegraded()` (`src/ui/radar_ui.cpp:447`) checks only `(post.mic_health.ch_min[c] == post.mic_health.ch_max[c]) ? "FLATLINE" : "RAIL SHORT"`, misreporting a `-98 dBFS` floating/flatlined microphone with 1-LSB toggle noise as `"RAIL SHORT"`.
- **Root Cause:** Incomplete migration from 2-channel to 4-channel multi-source architecture and coupled diagnostic state inside `solveWlsAzimuth()`.
- **Recommended Remediation:**
  - Sort `multi_res.sources[0..multi_res.count - 1]` by `confidence` descending at the end of `processMulti()`, and save/restore `last_pair_*` after Pass 0.
  - In `RadarDisplay::drawHardwareDegraded()` (`src/ui/radar_ui.cpp:447`) and `SerialUI::printSelfTestReport()` (`src/ui/serial_ui.cpp:50`), check:
    ```cpp
    const bool is_flatline = (post.mic_health.ch_min[c] == post.mic_health.ch_max[c]) ||
                             (post.mic_health.ch_rms_db[c] <= POST_MIC_FLATLINE_RMS_DB);
    const char* reason = is_flatline ? "FLATLINE" : "RAIL SHORT";
    ```

---

### 4. Candidate Modules for Reimplementation / Refactoring

1. **`src/main.cpp` (God File / Shared State Orchestrator)**
   - **Current State:** `src/main.cpp` holds 19 file-scope `std::atomic` variables, 3 FreeRTOS mutexes, 2 queues, boot self-test orchestration, environmental sensor polling, audio reconfiguration rollback logic, and all 3 FreeRTOS task loops.
   - **Target Architecture:** Split into:
     - `SystemState` / `TelemetryStore`: Encapsulating thread-safe environmental, clock-sync, and diagnostic counters behind a clean read/write interface.
     - `CalibrationManager`: Centralizing validation of `DeviceConfig` (currently duplicated across `main.cpp`, `rest_client.cpp`, `setup_portal.cpp`, and `config_store.cpp` with subtle discrepancies).
     - Dedicated task modules (`dsp_runner.cpp`, `network_worker.cpp`, `ui_controller.cpp`).

2. **`src/audio/gcc_phat.cpp` (`GccPhat` Class)**
   - **Current State:** Combines FFT buffer lifecycle, noise floor tracking, 6-pair cross-correlation, loop-closure WLS solving, SRP-PHAT grid search, CLEAN kernel deconvolution, and mutable diagnostic state (`last_pair_*`) in a single 900-line class.
   - **Target Architecture:** Separate pure, stateless solvers (`WlsAzimuthSolver`, `SrpCleanDeconvolver`) from stateful per-block spectral processing (`QuadFftProcessor`) and `AdaptiveNoiseFloorTracker`. Pass diagnostic outputs (`PairDiagnostics`) by pointer/struct rather than mutating member arrays inside `solveWlsAzimuth()`.

3. **`src/ui/display_filter.cpp` & `src/ui/radar_ui.cpp` (Target Tracking vs. Rendering)**
   - **Current State:** `RadarDisplay::drawRadarMulti()` mutates internal state (`hit_blips` ring buffer) using `millis()` during rendering, while `MultiTargetDisplayTracker` conflates EMA state expiration (1500 ms) with active target display hold time.
   - **Target Architecture:** Move phosphor trail state into `MultiTargetDisplayTracker` (driven by explicit `now_ms` timestamps on new event arrivals) so `RadarDisplay` becomes a pure, stateless renderer of a `RadarViewModel` struct.

---

### 5. Step-by-Step Modernization Roadmap

#### Phase 1: Immediate Bugfixes & Memory Safety (Blocking Release)
- [x] **Fix Stack Buffer Overflow (`CRIT-01`):** Clamp `center_lag`, `l_m1`, and `l_p1` to `[-PAIR_CORR_RADIUS, +PAIR_CORR_RADIUS]` in `estimateLocalPeakAmplitude()` and `solveFromSrpSeed()`, initialize `best_val = -INFINITY`, and enforce `MAX_MIC_DISTANCE_M = 0.11f` (or increase `PAIR_CORR_RADIUS` to `32`).
- [x] **Fix Correlation & SPL Threshold Gates (`CRIT-02`, `HIGH-04`):** Stop validating Pass-0 primary fallback candidates against `sec_threshold` (`0.20f`); remove the `< 0.45f` upward clamping in `config_store.cpp` and `main.cpp`; bypass fixed `min_spl_threshold` gating in `dsp_task` when `adaptive_threshold_active` is enabled.
- [x] **Fix I2S Blocking & Watchdog Coverage (`CRIT-03`, `HIGH-03`):** Replace `portMAX_DELAY` in `i2s_sampler.cpp` with a bounded `pdMS_TO_TICKS(100)` timeout; add `esp_task_wdt_add(nullptr)` to `sensor_display_task`; add `esp_task_wdt_reset()` between HTTP retries and calibration polls in `network_task`.
- [x] **Fix Keep-Alive Socket & Monotonic Server Time (`HIGH-01`, `HIGH-02`):** Call `http.end()` after 2xx responses in `RestClient::sendBatch()` to drain response bodies; prevent `server_utc_offset_ms` from stepping backwards on subsequent calibration polls; do not retry HTTP on local batch serialization failure.

#### Phase 2: Security, Provisioning & Contract Hardening
- [x] **Harden Setup Portal & NVS (`MED-02`):** HTML-escape all user strings in `SetupPortal::generateHtmlPage()`; unify `DeviceConfig` validation into a single `validate_device_config()` function checking `std::isfinite()` and bounds on all floats; add `preferences.begin()` / `preferences.end()` RAII scoping in `config_store.cpp`.
- [x] **Fix Multi-Source Sorting & Diagnostics (`MED-03`):** Sort `MultiDoaResult::sources` descending by `confidence`; preserve Pass-0 `last_pair_*` diagnostic arrays across secondary CLEAN passes; fix the flatline vs. rail-short check in `RadarDisplay::drawHardwareDegraded()`.

#### Phase 3: UI Responsiveness & Architectural Decoupling
- [x] **Decouple Active Track Hold from EMA Reset (`MED-01`):** Introduce a 250 ms active display hold timer distinct from the 1500 ms EMA silence reset timer so telemetry, phosphor decay blips, and SPI redraw cadence return to idle promptly after transient sounds.
- [x] **Expand Host ASAN/UBSAN Test Suite:** Add unit tests covering $d > 0.12\text{ m}$ array geometries, single-source rejection when `confidence` is between `0.20` and `correlation_threshold`, and non-monotonic server timestamp clamping.