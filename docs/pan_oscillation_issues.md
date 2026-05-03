# Pan Oscillation — Open Issues

Four root causes identified for the remaining pan servo overcompensation and damped oscillation. Each is independent and can be tackled in isolation.

## Test result — 2026-04-29 (conservative parameter run)

`tracking_gain_kp` was reduced to `0.25`, `tracking_delta_max_deg` to `1.25`, `tracking_hysteresis_enter_deg` to `2.25`, and `tracking_hysteresis_exit_deg` to `4.5`.

**Result:** Pan settled with target centred and held without oscillation. Pan tracking was very slow.

**What this tells us:**

- The control architecture is fundamentally sound — the original oscillation was tuning-driven, not a structural flaw.
- The slowness is almost entirely caused by `delta_max_deg`. At a ~10 Hz loop rate, `1.25 deg/cycle` caps pan speed at just **12.5 °/s**. From 30° off-centre, that is ~2.4 s to reach centre.
- Issues 1 and 2 (loop latency and stale frames) are what *force* conservative gains. A slow loop with stale frames requires a small `delta_max` to avoid oscillation, but that makes tracking sluggish. **Fixing Issues 1 and 2 first is the priority** — once loop latency is reduced and frames are fresh, `delta_max` and `gain_kp` can be raised back toward their original values, achieving fast tracking *and* stability together.
- Issue 3 (open-loop servo estimate) remains valid and will determine the ceiling on how aggressively gains can be raised after Issues 1 and 2 are fixed.
- Issue 4 (unused calibration model) is a longer-term accuracy improvement, lower urgency.

**Revised priority order: Issue 2 → Issue 1 → Issue 3 → Issue 4**

---

## Issue 1 — Control-loop latency (fixed sleep + blocking inference)

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

## Issue 3 — Open-loop servo position estimate (no feedback on true pan angle)

**File:** `ugv-follower/src/ugv_follower/control/pan_controller.py`

`PanController` tracks `_current_pan_deg` by accumulating commanded deltas. It has no knowledge of the servo's actual physical position. Sources of divergence between commanded and actual angle include:

- Servo response lag (the hardware takes time to reach the commanded angle).
- Servo backlash / hysteresis mean (~2.3°, measured during calibration and stored in `sensor_config.yaml`).
- Linear-fit error up to ~3°.

When the commanded position has not yet been reached but the next detection arrives, the controller adds another delta on top of an already-in-flight correction. This stacks increments in the same direction, overshooting the centre, then the next frame swings the command back — producing the observed oscillation.

**Why this matters even with a stationary target:** A still target removes any ambiguity — the oscillation is entirely self-generated by the control loop. When the servo has not yet reached the commanded angle, the controller believes it is already there. The next detection still shows an error (because the servo is mid-travel), so another delta is stacked on top of an in-flight correction. The servo eventually arrives and overshoots, then the whole process repeats in the opposite direction, indefinitely.

**Goal:** Either (a) add a simple first-order servo lag model so `_current_pan_deg` tracks an estimated true position rather than the commanded position, or (b) investigate whether the servo can report actual position and close the loop on that.

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
