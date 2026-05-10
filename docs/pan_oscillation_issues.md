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
- Query timeout is short (`timeout_s=0.1`), so a late response becomes `None` for that cycle.
- The protocol path has no request ID/timestamp correlation between `T=130` request and `T=1001` response.

When this happens repeatedly, `base_pan` remains on cached measurements for many cycles. As visual error changes sign, command output swings around an outdated anchor (`target = base_pan + delta`), which can reintroduce large side-to-side motion even for a stationary target.

**Latest finding (2026-05-10 diagnostic run):**

- Dominant failure mode is **no telemetry reply in the read window** (Option 1), not guard rejection.
- `query_pan_deg()` repeatedly logged: `timeout after 0.100s (lines_read=0, no valid T=1001 pan)`.
- `lines_read=0` indicates no serial line arrived during the query window (not malformed JSON, not wrong message type).
- Only one fresh sample was accepted (`1.49°`), then the controller ran for many cycles on `measured-cached` fallback.
- No `telemetry guard rejected` logs were observed in this run.

This confirms the immediate bottleneck is telemetry availability/timing on the serial path, not plausibility-threshold tuning.

**Observable signature:**

- Frequent cycles where telemetry query returns `None` (or no accepted measurement update).
- `base` repeatedly logged as cached source while `corrected` changes materially.
- Large command reversals with stable/slowly varying target position.
- Query diagnostics showing `lines_read=0` across consecutive `T=130` requests.

**Goal:** Increase effective true-angle update quality by improving telemetry cadence and freshness correlation with control updates.

Suggested directions:

- Increase `query_pan_deg()` timeout from `0.1s` to `0.25-0.35s` and re-test first; current logs show near-total timeout at `0.1s`.
- Add telemetry-age/degraded mode guard: if no fresh measurement for N consecutive cycles, clamp delta/gain more aggressively to reduce stale-base command swings.
- Decouple telemetry polling from the vision loop (dedicated poll path/thread) so pan reads are not starved by inference cadence.
- Audit firmware response behavior for `T=130` under concurrent traffic (`T=1`, `T=133`) and ensure a prompt `T=1001` is always emitted.
- If firmware supports it, add request/response correlation metadata (sequence or timestamp) to distinguish late-but-valid responses from stale context.
- Keep the new logging in place (`measured-fresh` vs `measured-cached`, query timeout/lines_read) and use it as acceptance criteria for fixes.
