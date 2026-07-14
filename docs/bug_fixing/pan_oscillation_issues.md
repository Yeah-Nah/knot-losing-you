# Pan Oscillation — Open Issues

Nine root causes identified for the remaining pan servo overcompensation and damped oscillation. Each is independent and can be tackled in isolation.

## Quick Diagnostic Checklist (single run)

Use this checklist to separate detector jitter, vision-frame lag, and pan-feedback lag in one capture session.

1. Enable debug logs and record 20-30 seconds while standing still near image centre.
2. Log per-cycle fields in one line: `dt`, `bbox_centre_u/v`, `corrected`, `scaled`, `measured_pan`, `base_pan`, `delta`, `pan_cmd`.
3. Mark each pan sample with age/source metadata if possible (for example, immediate `query_pan_deg()` result vs cached fallback).
4. Look for these signatures:
	- `corrected` and bbox values wobble while `measured_pan` is stable: detector/centroid jitter dominates.
	- `corrected` is stable but `measured_pan` jumps/discontinues and `pan_cmd` follows immediately: pan-feedback lag/queue issue dominates.
	- both `corrected` and `measured_pan` show delayed step-like behavior after commands: combined vision + feedback lag.
5. Confirm command math consistency on any suspect step: check whether `pan_cmd ~= base_pan + delta`.

If the large command jump occurs with near-constant `corrected` and near-constant `delta`, the jump is coming from `base_pan` (measured-feedback path), not visual heading.

## Test result — 2026-04-29 (conservative parameter run)

`tracking_gain_kp` was reduced to `0.25`, `tracking_delta_max_deg` to `1.25`, `tracking_hysteresis_enter_deg` to `2.25`, and `tracking_hysteresis_exit_deg` to `4.5`.

**Result:** Pan settled with target centred and held without oscillation. Pan tracking was very slow.

**What this tells us:**

- The control architecture is fundamentally sound — the original oscillation was tuning-driven, not a structural flaw.
- The slowness is almost entirely caused by `delta_max_deg`. At a ~10 Hz loop rate, `1.25 deg/cycle` caps pan speed at just **12.5 °/s**. From 30° off-centre, that is ~2.4 s to reach centre.
- Issues 1 and 2 (loop latency and stale frames) are what *force* conservative gains. A slow loop with stale frames requires a small `delta_max` to avoid oscillation, but that makes tracking sluggish. **Fixing Issues 1 and 2 first is the priority** — once loop latency is reduced and frames are fresh, `delta_max` and `gain_kp` can be raised back toward their original values, achieving fast tracking *and* stability together.
- Issue 3 (open-loop servo estimate) remains valid and will determine the ceiling on how aggressively gains can be raised after Issues 1 and 2 are fixed.
- Issue 4 (unused calibration model) is a longer-term accuracy improvement, lower urgency.

**Revised priority order: Issue 2 → Issue 1 → Issue 5 → Issue 3 → Issue 4**

## Latest diagnostic result — 2026-07-13 (40 s, subject centred and stationary)

Run conditions: subject intentionally remained near image centre and did not move.

**Observed in logs:**

- Pan telemetry remains the dominant bottleneck: repeated `query_pan_deg()` timeouts with `lines_read=0` at `timeout=0.200s`.
- Fresh pan samples arrive in sparse bursts (roughly every ~7.2 s), then long gaps of `None`/stale cycles.
- During telemetry gaps, control uses `base=estimated` and continues issuing pan commands.
- When a fresh sample eventually arrives, estimate-vs-measure residuals are large (examples logged: `+34.23°`, `+41.17°`, `-74.22°`), causing abrupt command re-anchoring.
- Net effect: self-generated pan motion/oscillation can occur even while the target is stationary.

**Conclusion:** the current failure mode is primarily **insufficient true-angle feedback density** (Issue 9), with Issue 3 (estimate fallback behavior) acting as the amplifier.

---

## Issue 1 — Control-loop latency (fixed sleep + blocking inference) — COMPLETE

**File:** `ugv-follower/src/ugv_follower/pipeline.py`

The main loop calls YOLO inference and camera capture synchronously, then appends a fixed 100 ms sleep at the end. Effective cycle time is therefore:

```
T_cycle ≈ T_inference + T_camera_read + 100 ms
```

On a Pi running yolo11n this can easily push cycle times to 200–400 ms. Every pan command is based on a detection that is one full cycle old. With a delay that large, the controller issues a correction, but the pan motor has already moved (or the target has moved), and by the time the next frame arrives the error has overshot in the opposite direction. This is the classic recipe for lagged damped oscillation.

**Goal:** Decouple the fixed sleep from actual cycle time so the loop runs as fast as inference allows, or move to a timestamp-based approach that tracks true elapsed time and scales command magnitude accordingly.

---

## Issue 2 — Stale camera frames (no V4L2 buffer-size cap) — COMPLETE

**File:** `ugv-follower/src/ugv_follower/perception/waveshare_camera.py`

`cv2.VideoCapture` under V4L2 maintains an internal capture buffer (typically 4 frames by default). The control loop calls `cap.read()` once per cycle at ~2–5 Hz. Because the camera is capturing at 30 fps, the buffer fills faster than it drains, so `cap.read()` returns the oldest buffered frame rather than the latest one. The control decision is therefore based on imagery that may be 100–400 ms old before the cycle delay is even counted.

**Why this matters even with a stationary target:** After the pan servo moves toward centre, the next `cap.read()` may return a frame captured *before* the servo moved. The controller sees the person still off-centre, issues another correction in the same direction, and overshoots. The servo then crosses centre, the next stale frame confirms the overshoot, and the command swings back. This creates continuous oscillation around the centred position even when the target has not moved at all, and also explains why the system sometimes never settles — if every read returns a frame that is one servo-move behind, the controller is always chasing its own tail.

Setting `cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)` after opening the device limits V4L2 to holding a single frame, so each `read()` retrieves the most recent one.

**Goal:** Reduce the effective camera-to-command latency by capping the V4L2 capture buffer to 1 frame.

---

## Issue 3 — Pan position estimate based on dead reckoning (not available when telemetry gaps occur) - COMPLETE

**Files:** `ugv-follower/src/ugv_follower/control/pan_controller.py`, `ugv-follower/configs/sensor_config.yaml`

**Root Cause:**
The pan controller uses measured pan as `base_pan` (the starting position for the next correction cycle). When fresh telemetry is unavailable (query timeout, RX buffer stale, plausibility guard rejection), the controller falls back to a cached last-known value. If that cache is minutes old or the servo has been in motion for many cycles, `base_pan` becomes increasingly inaccurate.

With `base_pan` stale, the next correction delta is applied from a wrong starting point. The servo moves toward the intended target but the controller's model of "where we are" diverges from reality. Once fresh telemetry finally arrives, a large jump occurs (see Issue 8 Layer 2).

**Why this matters even with a stationary target:** Between telemetry samples, there is no mechanism to predict where the servo has moved. The controller either waits (losing responsiveness) or blindly applies deltas from a stale position (causing jumps and wrong-direction brief movements when fresh telemetry finally arrives).

**Solution: Dead Reckoning Position Estimate**

Maintain a continuously-updated estimate of pan position by tracking:
1. Last accepted measured pan value (from a fresh, plausible telemetry sample)
2. All pan commands sent since that measurement (`pan_cmd` values, timestamped)
3. Simple servo motion model: the servo ramps toward commanded angle at a maximum velocity (e.g., ~120°/sec, hardware-dependent)

**Each cycle:**
- Compute elapsed time since last measurement
- For each command in the buffer, predict motion: servo moves toward command at capped velocity (from `tracking_max_measured_velocity_deg_per_s`)
- Update estimate: `estimated_pan ≈ last_measured + motion_accumulated`
- If estimate reaches the commanded angle, hold it there (servo can't go further)
- When fresh telemetry arrives, correct the estimate: `estimated_pan += 0.3 × (fresh_measured − estimated_pan)` (smoothing blend)

**Configuration:** Uses existing `tracking_max_measured_velocity_deg_per_s` from `sensor_config.yaml` under `pan_tilt_servo` (this parameter is part of the plausibility guard from Issue 8 Layer 2, and is reused here to bound motion prediction).

**Key distinction:** This estimate is based on *commanded motion history and elapsed time*, not on the calibrated servo curve. The calibrated curve (Issue 4) corrects command outputs; the estimate predicts current state.

**Implementation sequence:** Issue 8 Layer 2 (plausibility guard) must be complete first. The guard validates that incoming measurements are trustworthy before the estimate uses them for correction.

**Goal:** Provide continuity between telemetry samples so `base_pan` is always approximately correct, reducing command jumps and allowing fresh telemetry to smoothly correct the estimate rather than create a discontinuity.

**Implementation (no-blend variant):** `_estimated_pan_deg` is propagated each cycle toward `_last_pan_cmd_deg` at a velocity capped by `tracking_max_measured_velocity_deg_per_s`. Fresh valid telemetry hard-replaces the estimate (no blend). Base-pan source priority: `measured-fresh` → `estimated` → `initialising`. Consecutive stale cycles beyond `tracking_stale_telemetry_threshold_cycles` activate degraded mode, scaling `delta_max` by `tracking_degraded_delta_scale` to reduce command aggressiveness. Degraded mode exits automatically on the next fresh measurement.

---

## Issue 4 — Calibrated servo curve and backlash not applied at runtime

**Files:** `ugv-follower/configs/sensor_config.yaml`, `ugv-follower/src/ugv_follower/control/pan_controller.py`, `ugv-follower/src/ugv_follower/settings.py`

The servo calibration workflow produces a rich model stored in `sensor_config.yaml`:

- `piecewise_linear.combined` — maps commanded angle to actual angle (and its inverse).
- `hysteresis_mean_deg: 2.3182` — measured servo backlash.
- `dead_band_pos_deg / dead_band_neg_deg` — hardware dead band.
- `phi_min_deg / phi_max_deg` — actual achievable angle range.

None of this is currently read by the runtime pan pipeline. `Settings` only exposes the four tracking tuning parameters (`gain_kp`, `delta_max_deg`, `hysteresis_enter_deg`, `hysteresis_exit_deg`) and the raw command limits (`cmd_min`, `cmd_max`). As a result:

- Commands near the centre may fall inside the servo hardware dead band and produce no physical motion, but the controller still increments `_current_pan_deg`, accumulating phantom offset.
- The commanded angle is sent directly without inversion through the calibrated curve, so a 10° command may only produce ~9° of actual travel (or less), causing systematic under-correction that the integrating loop then compensates for in the next cycle, again overshooting.

**Goal:** Expose the calibrated curve and backlash data through `Settings` and apply the inverse mapping in `PanController` (or a new servo model layer) so commanded angles are pre-compensated for known nonlinearity and the hardware dead band is respected.

---

## Issue 5 — Fixed loop sleep still throttles response (replace with adaptive pacing)

**File:** `ugv-follower/src/ugv_follower/pipeline.py`

Although `dt` is now measured per iteration and pan delta is scaled by elapsed time, the main loop still ends with a fixed sleep (`time.sleep(self._loop_period_s)` with `_loop_period_s = 0.1`). This enforces an additional 100 ms idle delay every cycle regardless of how quickly camera read and inference complete.

In practice this limits the control update rate and increases reaction lag when the target changes direction quickly. The system can therefore still feel hesitant even after stale-frame buffering and `dt` scaling improvements.

Setting the loop period to zero removes this delay but can create a tight busy loop (high CPU load, timing jitter, and noisy command updates). A more robust approach is adaptive pacing:

```python
target_period_s = 0.02  # example: 50 Hz cap
loop_start = time.monotonic()

# ... read sensors, run inference, update control ...

elapsed = time.monotonic() - loop_start
sleep_s = max(0.0, target_period_s - elapsed)
time.sleep(sleep_s)
```

This keeps a bounded maximum loop rate when processing is fast, while automatically skipping extra sleep when processing is slow.

**Goal:** Replace fixed end-of-loop sleep with adaptive pacing so control latency is minimized without introducing a CPU-saturating busy loop.

---

## Issue 6 — Hysteresis is applied after gain scaling

**File:** `ugv-follower/src/ugv_follower/control/pan_controller.py`

`PanController` currently applies proportional gain first and then checks the hysteresis thresholds against the scaled error. This couples `tracking_gain_kp` and the effective deadband size:

```python
scaled = self._gain_kp * corrected
if within_deadband(scaled, enter_deg, -enter_deg):
	...
```

As a result, reducing `tracking_gain_kp` does not just make motion gentler — it also makes the raw heading error required to exit hold much larger. This can make low-gain tuning look artificially slow or pause-heavy, while raising gain can reduce hesitation but reintroduce overshoot.

Applying hysteresis to the raw corrected heading, and only then applying gain to the motion command, would decouple “when to move” from “how aggressively to move.”

**Goal:** Evaluate whether hysteresis should be applied to raw corrected heading error rather than gain-scaled error so deadband behaviour remains consistent across gain changes.

---

## Issue 7 — Vision pipeline lag path can create self-generated pan motion

**Files:** `ugv-follower/src/ugv_follower/pipeline.py`, `ugv-follower/src/ugv_follower/perception/waveshare_camera.py`

Even with a stationary target, any lag between camera exposure time and command emission can produce apparent "phantom" motion in closed-loop tracking. The controller acts on centroid data that may describe an earlier pan state. If the servo has already moved by the time that frame is processed, the next command can continue correcting in the old direction and push past centre.

This lag path is independent of servo telemetry quality: it exists even if measured pan feedback is perfect.

**Observable signature:**

- Bounding box and `corrected` heading update in delayed, step-like fashion relative to visible pan movement.
- Commands continue in one direction for 1-2 cycles after the target appears centred in the live stream.

**Goal:** Instrument and bound camera-to-command latency (capture timestamp to `set_pan_tilt`) and keep fresh-frame semantics under load.

---

## Issue 8 — Pan telemetry lag/queue path can create self-generated pan motion

**Files:** `ugv-follower/src/ugv_follower/control/ugv_controller.py`, `ugv-follower/src/ugv_follower/control/pan_controller.py`, `ugv-follower/src/ugv_follower/pipeline.py`

The pan controller uses measured pan as the command base whenever available. If telemetry samples are stale, delayed, or discontinuous, `base_pan` can jump between cycles while visual error remains nearly unchanged. Because command is formed as `target = base_pan + delta`, the command can jump in lockstep with telemetry jumps and produce unnecessary servo motion.

This lag path is independent of vision lag: it can appear even with stable detections and near-constant heading error.

**Observable signature:**

- `corrected` and `scaled` remain nearly constant while `base_pan` jumps (for example, ~49° to ~34°).
- `pan_cmd` jump magnitude matches `base_pan` jump, since `delta` is nearly unchanged.

**Goal:** Add telemetry freshness guards (sample age and jump sanity checks), and only trust measured pan when fresh and physically plausible; otherwise fall back to controlled estimate/cached value.

**Layers of Defence***
- Flush before you ask (the camera-buffer analogue) - COMPLETE
Drain the RX buffer immediately before sending the T=130 request. That way any stale T=1001 packets queued from previous cycles are discarded first, and the only thing that can arrive in the read window is the response to this specific request. This is the direct equivalent of CAP_PROP_BUFFERSIZE=1 — you are capping the effective queue depth to one.

- Plausibility / jump guard - COMPLETE
Even after flushing, telemetry can occasionally be wrong (noise, a dropped byte, serial bus contention from concurrent T=133 traffic). Before accepting a new measured value, check whether the implied servo movement is physically possible: if |new_measured − last_accepted| implies a servo velocity that exceeds what the hardware can produce in one dt, reject the sample entirely and fall back to the last accepted value. This bounds the damage from any single bad reading.

- Blend rather than replace (longer term)
The current design uses measured pan as the full base for the next command. That gives a single stale or wrong sample full authority over the command. A more robust approach is to use the measured value to correct an accumulated estimate rather than replace it outright — similar to how a complementary filter works. The estimate provides continuity and the measurement provides drift correction, so neither can cause a large command jump on its own.

---

## Issue 9 — True pan-angle reads are too sparse and weakly correlated to control updates

**Files:** `ugv-follower/src/ugv_follower/pipeline.py`, `ugv-follower/src/ugv_follower/control/ugv_controller.py`, `ugv-follower/src/ugv_follower/control/pan_controller.py`

The control loop requests pan telemetry once per loop (`query_pan_deg()`), then immediately computes the next command. In practice, loop cadence can be around 2 Hz under load, while the servo can traverse a large angle between samples. A fast-moving servo can therefore move substantially before the next trusted angle read arrives.

Current query behavior also prioritizes freshness over continuity:

- RX buffer is flushed before query, which removes queued stale packets but can discard delayed valid responses.
- Query timeout is still short relative to observed reply timing (`timeout_s=0.2` in current runs), so late responses still become `None` for that cycle.
- The protocol path has no request ID/timestamp correlation between `T=130` request and `T=1001` response.

When this happens repeatedly, `base_pan` remains on cached measurements for many cycles. As visual error changes sign, command output swings around an outdated anchor (`target = base_pan + delta`), which can reintroduce large side-to-side motion even for a stationary target.

**Latest finding (2026-05-10 diagnostic run):**

- Dominant failure mode is **no telemetry reply in the read window** (Option 1), not guard rejection.
- `query_pan_deg()` repeatedly logged: `timeout after 0.100s (lines_read=0, no valid T=1001 pan)`.
- `lines_read=0` indicates no serial line arrived during the query window (not malformed JSON, not wrong message type).
- Only one fresh sample was accepted (`1.49°`), then the controller ran for many cycles on `measured-cached` fallback.
- No `telemetry guard rejected` logs were observed in this run.

This confirms the immediate bottleneck is telemetry availability/timing on the serial path, not plausibility-threshold tuning.

**Latest finding (2026-07-13 diagnostic run):**

- Dominant failure mode remains **no telemetry reply in the read window**, now observed with `timeout=0.200s`.
- Logs repeatedly show: `timeout after 0.200s (lines_read=0, no valid T=1001 pan)`.
- Fresh samples are intermittent and bursty (`seq` increments in short clusters), separated by multi-second stale periods.
- During stale periods, controller falls back to `estimated` base and continues applying non-trivial deltas.
- On fresh return, large estimate residuals are observed (e.g., `34.23°`, `41.17°`, `-74.22°`), which is consistent with estimate drift plus hard re-anchor.

This run strengthens the prior conclusion: immediate bottleneck is still telemetry availability/timing, now confirmed under the newer 0.2 s query timeout and threaded poller design.

**Latest finding (2026-07-13 diagnostic run, `timeout=0.300s`):**

- Increasing `query_pan_deg()` timeout from `0.2s` to `0.3s` did **not** resolve sparse telemetry updates.
- Parsed run stats: `T=130 sent=209`, `query timeouts=195`, `fresh samples=14` (about **6.7%** query success).
- Timeout signature remains dominant: `194/195` timeouts were `lines_read=0` (no serial line arrived in the read window).
- Fresh telemetry arrives in bursts with repeating long gaps. Measured fresh inter-arrival pattern includes multiple `~7.02s` gaps (examples: `7.019s`, `7.092s`, `7.018s`, `7.022s`, `7.076s`, `7.020s`).
- Between long gaps, short sub-second clusters occur (`~0.07s` to `~0.20s` between fresh samples), indicating intermittent windows where replies are briefly available.

**Latest finding (2026-07-13 telemetry-only serial test, `timeout=0.300s`, no `T=1`/`T=133` traffic):**

- Telemetry remained highly sparse even without mixed traffic: `103` total queries, `4` successes, `99` timeouts (**3.9%** success).
- Timeout signature stayed dominant and clean: timeout entries were consistently `lines_read=0` and `non_telemetry_lines=0` (no serial line received during most query windows).
- Fresh replies appeared at near-regular long intervals (success IDs `20`, `45`, `70`, `95`), implying about `25` timeout-paced queries between successes (approximately `7.2s` cadence at this run's effective query period).
- Successful reply latency stayed low once a reply actually arrived (`54.74-117.80 ms`, median `57.90 ms`), and reported pan angle was stable (`10.3736 deg`) across all successes.

This telemetry-only run weakens the mixed-traffic contention hypothesis as the primary cause. The dominant bottleneck now points more strongly to response scheduling/rate-limiting on the firmware/device side, or to host-side query/flush timing that systematically misses reply windows.

**Latest finding (2026-07-13 telemetry-only request-rate/timeout sweep):**

- Four telemetry-only runs were compared.
- `poll=2.0s, timeout=0.3s`: `1/15` success (**6.7%**)
- `poll=0.2s, timeout=0.3s`: `4/103` success (**3.9%**)
- `poll=2.0s, timeout=2.0s`: `4/15` success (**26.7%**)
- `poll=0.5s, timeout=2.0s`: `4/17` success (**23.5%**)
- Across all four runs, successful samples remained sparse with a consistent long-gap pattern (roughly **7-8 s** between accepted replies in the multi-success runs).
- Increasing request rate alone (`2.0s` to `0.2s` poll) did not improve telemetry availability at `timeout=0.3s`; success ratio slightly worsened.
- Increasing timeout to `2.0s` improved capture probability and raised success ratio, but successes still arrived with similar multi-second spacing and often near the tail of the read window (`~0.86-1.95 s` latency in one run, `~1.00 s` in another).

**Latest finding (2026-07-13 telemetry-only fixed-timeout request-rate matrix, `timeout=2.0s`) — COMPLETE:**

- Fixed-timeout matrix now includes all planned poll rates (`2.0s`, `1.0s`, `0.5s`, `0.2s`) at `timeout=2.0s` in telemetry-only mode.
- `poll=2.0s, timeout=2.0s`: `4/15` success (**26.7%**)
- `poll=1.0s, timeout=2.0s`: `4/17` success (**23.5%**)
- `poll=0.5s, timeout=2.0s`: `4/17` success (**23.5%**)
- `poll=0.2s, timeout=2.0s`: `5/18` success (**27.8%**)
- Success ratio stayed in a narrow band (`23.5-27.8%`) across the full poll-rate range, showing no meaningful improvement from higher host request rate at fixed long timeout.
- Long inter-success gaps remained essentially unchanged at about `~7s` (`7.280s` in the `poll=1.0s` run, `7.045s` in the `poll=0.2s` run), consistent with an upstream cadence bottleneck rather than host poll cadence.

**Latest finding (2026-07-14 telemetry-only serial test, defaults with pre-query RX flush OFF):**

- Run produced `509` total queries with `433` successes and `76` timeouts (**85.1%** success), a large increase in returned `T=1001` parses versus prior flush-ON telemetry-only runs.
- Despite higher success count, long sparse windows still exist: longest measured inter-success gap was `7.213s` (`send_t 158.082 -> 165.295`), consistent with the previously observed `~7s` cadence ceiling.
- Returned values were overwhelmingly repeated: `428/433` successes reported `pan_deg=10.3736` and only `5/433` reported `-179.9560`, indicating very low measurement diversity.
- Success latency profile was dominated by near-immediate reads (`318/433` with `latency_ms < 2`), with many bursty back-to-back successes, which is consistent with draining queued replies rather than proving one fresh reply per request.
- Interpretation: disabling pre-query flush increases apparent query success/capture, but weakens freshness correlation and does not remove the long-gap cadence bottleneck.

**Latest finding (2026-07-14 flush ON vs OFF comparative matrix across runs):**

- Comparative summary now includes four ON/OFF pairs:
	- `default_30s`: flush ON `3.9%` (`4/103`) vs flush OFF `85.0%` (`431/507`)
	- `default_t2p0`: flush ON `23.5%` (`4/17`) vs flush OFF `79.4%` (`50/63`)
	- `poll0p2_t0p3`: flush ON `2.9%` (`3/103`) vs flush OFF `77.3%` (`116/150`)
	- `poll2p0_t0p3`: flush ON `6.7%` (`1/15`) vs flush OFF `100.0%` (`15/15`, very small sample)
- Flush OFF consistently raised parsed-success ratio, but **all eight runs** reported `uniqPan=1` with dominant value `10.3736`, indicating near-zero measurement diversity.
- Long sparse windows persisted in both modes. Max inter-success gaps remained multi-second (`~5.6-7.9s` in most runs), so higher parsed-success under flush OFF did not eliminate cadence sparsity.
- Flush OFF latency pattern remained strongly queue-like (many `<2 ms` successes and bursty clusters), while flush ON primarily showed sparse accepted samples near longer-latency buckets.
- Timeout signature remained clean (`to_lr0%` ~`99-100%` where timeouts occurred), reinforcing that missing read-window arrivals are still the dominant timeout mode.

Interpretation: the ON/OFF matrix strengthens the prior conclusion: pre-query flush controls freshness-vs-continuity tradeoff, but neither mode restores true fresh-per-request telemetry cadence. Flush OFF improves capture of buffered packets; flush ON preserves correlation intent but misses most delayed replies.

Interpretation: current evidence points to a cadence/availability bottleneck upstream of host poll rate (firmware scheduling, device-side rate limit, or host request-window alignment). The host can improve probability of catching replies with longer waits, but this does not yet demonstrate one reply per request.

**Code-path interpretation for this run (important):**

- Poller tracking cadence is configured to `0.05s`, but the serial worker can only issue one query roughly every timeout period when no reply arrives. In this run the effective query period is about `0.30s`, matching the timeout-dominated loop.
- `query_pan_deg()` flushes RX (`reset_input_buffer()`) immediately before each `T=130`. This protects freshness, but if replies are often delayed beyond the active read window, delayed-but-valid `T=1001` packets can be dropped by the next pre-query flush.
- Combined with concurrent traffic (`T=1` drive commands and `T=133` pan commands), logs support that the immediate bottleneck is still serial reply availability/correlation, not telemetry plausibility guard rejection.

**Observable signature:**

- Frequent cycles where telemetry query returns `None` (or no accepted measurement update).
- `base` repeatedly logged as cached source while `corrected` changes materially.
- Large command reversals with stable/slowly varying target position.
- Query diagnostics showing `lines_read=0` across consecutive `T=130` requests.

**Goal:** Increase effective true-angle update quality by improving telemetry cadence and freshness correlation with control updates.

**Consolidated finding and current status:**

- Timeout extension to `0.300s` has been explicitly tested and did not resolve sparsity; dominant failure remains timeout with `lines_read=0`, so further small timeout-only tuning is unlikely to fix root cause.
- Mixed-traffic contention is no longer the leading hypothesis: telemetry-only testing still showed severe sparsity, shifting primary suspicion toward firmware/device-side response scheduling or host-side query/flush timing mismatch.
- Request-rate increase alone did not improve telemetry density, while longer timeout improved capture ratio without removing the long inter-success gap. This suggests better "catch" behavior, not restored per-request response behavior.
- Full flush ON/OFF matrix now confirms this pattern across multiple poll/timeout settings: flush OFF materially increases parsed-success ratio, but `uniqPan` remains `1` and long sparse gaps persist, so the gain is capture/queue-drain behavior rather than proven improvement in fresh correlated telemetry.
- Mitigations that reduce control impact are already in place (degraded-mode behavior during stale telemetry and decoupled telemetry polling path/thread), but they do not restore true-angle sample density.
- Existing diagnostics (`measured-fresh` vs `measured-cached`, timeout details including `lines_read`) should be retained as acceptance criteria for any telemetry-path fix.

**Focused next investigation (updated):**

- Controlled **request-rate matrix at fixed long timeout** (`poll=2.0s`, `1.0s`, `0.5s`, `0.2s` with `timeout=2.0s`, telemetry-only, 30 s window) — **COMPLETE (2026-07-13)**. Result: success ratio remained flat (~`23.5-27.8%`) with persistent `~7s` inter-success gaps.
- Controlled **flush ON vs OFF A/B comparisons** across multiple poll/timeout settings — **COMPLETE (2026-07-14)**. Result: flush OFF raises parsed-success strongly but does not restore diversity/cadence; queue-drain signature dominates.
- Next highest-value step: add definitive **request/response correlation metadata** (firmware echo token or sequence ID) so each `T=1001` can be attributed to a specific `T=130`.
- In parallel, log host-side serial edge timestamps (`send_ts`, `first_byte_ts`, `parse_ts`, `timeout_ts`) for every query and retain raw receive timestamps to separate true no-reply from late-reply/drop behavior.
- If firmware changes are available, instrument device timing (`T=130` RX time, `T=1001` TX time, device-side cadence counter) to directly test whether the ~7 s pattern is generated upstream.

**Recommendation (next steps):**

- Keep **flush ON** in control-path runtime for now to preserve freshness intent and avoid commanding on clearly queued/stale bursts.
- Prioritize a short firmware+host correlation experiment (sequence echo plus edge timing logs) as the immediate gating task before further host timeout/poll tuning.
- In parallel, run a quick firmware sanity check to confirm actual servo-bus baud/rate configuration and half-duplex turnaround behavior match expected hardware settings.
- Define pass/fail acceptance for Issue 9 after correlation is available:
	- At least `80%` of accepted pan samples should be provably matched to same-cycle requests.
	- Median inter-success gap under telemetry-only should be `<0.5s` with no recurring `~7s` ceiling.
	- `uniqPan` should reflect real motion context (not single-value dominance over long windows).
- If correlation confirms firmware-side cadence limiting, shift primary fix to firmware scheduler/rate-limit behavior; if correlation shows host-side misses, then redesign host read strategy (continuous reader with freshness tagging instead of strict flush-window polling).

**Tooling update (2026-07-14) — correlation instrumentation added to `check_pan_telemetry_only.py`:**

`ugv-follower/tools/check_pan_telemetry_only.py` now supports the edge-timing and correlation instrumentation this section calls for, all opt-in and off by default so existing recorded runs remain comparable:

- `--seq-token` (with `--token-field NAME`, default `S`) — attach a host-generated token to each `T=130` and check whether `T=1001` echoes it back. Degrades gracefully to "no match data" (not an error) if the firmware doesn't support it, since the field name/support is still unconfirmed (see `docs/engineering_theory/firmware_host_correlation_experiment.md` §1).
- `--linger-ms N` — keep reading up to `N` extra milliseconds past the nominal `--timeout`, to separate true no-reply from late-reply, without changing `success`/`timeout` accounting. Keep well under `--poll-interval`.
- `--retain-raw-lines` — keep every raw line and timestamp seen per query, for post-hoc audit.
- `--log-jsonl PATH` — write the extended per-query schema (edge timestamps, token fields, raw lines) as JSONL for offline analysis.

When any of these are used, the console summary additionally reports:

- **Matched ratio (rho)** — fraction of successful queries with a proven token match; only shown when `--seq-token` was used. Target: `>= 0.8` per the acceptance criteria above.
- **Median inter-success gap (g~)** — distinct from the pre-existing "longest gap" line; target `<0.5s` with no recurring `~7s` ceiling.
- **Unique pan values (C_uniq)** — should track real motion context, not stay pinned at `1` as seen in the flush-OFF runs above.
- **Late replies caught in linger** — count of replies that arrived only after the nominal timeout window, when `--linger-ms` is set.

This closes the tooling gap identified in "Focused next investigation" above. What remains outside this tool's scope is firmware coordination to confirm whether `T=1001` can actually carry an echoed token field, and the hardware validation run itself — both still open.
