# Pan Oscillation After the Telemetry Fix: Vision Latency and Close-Range Centroid Error

**Status:** diagnosis only, no code changed yet (2026-10-22).
**Evidence:** [logs/ugv_run_tracking_test.jsonl](logs/ugv_run_tracking_test.jsonl), a `ugv-run` session from 2026-10-21 12:13:42–12:14:37 with the target standing still.
**Builds on:** [telemetry_7s_gap_resolution.md](telemetry_7s_gap_resolution.md). The ~7 s gap is fixed, so this is the remaining oscillation.

---

## 1. Problem statement

With the firmware IMU stall fixed, pan tracking is better but still not stable:

1. **Standing still at normal range:** the pan never settles. It swings between about **10° and 36°** with a period of about 4 s, while the person stays at about **20°**.
2. **Close range** (bounding box wider than about 2/3 of the frame): the pan still hunts and never centres the person. It is less wild than before the telemetry fix.

Telemetry is no longer the main cause. Fresh `T=1001` samples arrive throughout the run, the longest controller-visible gap is under 0.5 s, and the measured pan follows the commands closely.

---

## 2. What the log shows

### 2.1 The control loop runs at about 2.2 Hz, not 100 Hz

| Metric | Value |
|---|---|
| Control cycles in 55 s | 120 |
| Cycle period (median / min / max) | **0.453 s** / 0.426 s / 0.615 s |
| Configured `loop_period_s` | 0.01 s |

Every iteration of `Pipeline._main_loop()` does these steps in sequence: LiDAR read, `camera.get_frame()`, YOLO `track()` on a 1920×1080 frame, `results[0].plot()` at full resolution, and a JPEG encode for the MJPEG stream. The 0.01 s sleep is negligible. The ~0.45 s period is set by inference plus the annotation and encode work.

Two consequences:
- `dt` is about 0.45 s, so the slew cap `60 deg/s × dt` allows about **27° per cycle**. In practice there is no slew limiting.
- With one decision every 0.45 s, any latency in the vision path is a large fraction of a cycle.

### 2.2 The vision error is about 0.6–0.9 s old when it is acted on

Each cycle's `corrected=` heading error should equal `(target bearing − pan at the moment the frame was captured)`. To check this, I fitted that model against the logged pan telemetry for a range of assumed frame ages:

| Assumed frame age | RMS fit error |
|---|---|
| 0.0 s (frame is current) | 6.9° |
| 0.4 s | 4.2° |
| **0.6–0.75 s** | **3.8–3.9° (best)** |
| 1.0 s | 5.1° |

The best fit gives a target bearing of **20.0°** and a slope of **−0.92**. A slope near −1 means the sign convention is correct. The person really was standing still at about 20°, and the error the controller saw was **about 1.5–2 control cycles old**.

The raw log shows this directly. The error stays frozen while the pan moves 15–25°:

```
13:53.560  corrected=10.53°  base= 9.52°(estimated)       pan_cmd=19.00°
13:53.995  corrected=10.51°  base=19.00°(estimated)       pan_cmd=28.46°
13:54.449  corrected=10.45°  base=26.37°(measured-fresh)  pan_cmd=35.78°   <- 3 cycles, same error, 26° of travel
13:54.952  entering hold, |scaled|=1.19°                                    <- this frame was captured with the pan at ~19°, about 0.9 s earlier
13:55.872  corrected=-13.00° base=33.58°(measured-fresh)  pan_cmd=21.89°   <- the overshoot finally becomes visible
13:56.305  corrected=-13.72° base=21.89°(estimated)       pan_cmd= 9.53°   <- same stale error applied again
```

The same pattern occurs at 13:49.3–13:52.1 and at 13:57.7–14:01.0. In one case `corrected=-15.50°` appears in two consecutive cycles with identical values, which suggests a frame that was reused or never updated.

### 2.3 Where the latency comes from (most likely)

Two parts add up to roughly the fitted 0.6–0.9 s:

1. **Processing time within the cycle, about 0.4 s.** The frame is grabbed early in the iteration. The pan command is issued only after YOLO has run on it.
2. **Frame age at grab time, about 0.3–0.5 s, from V4L2 buffering.** `WaveshareCamera.get_frame()` calls `cap.read()` once per cycle. The camera produces frames at 30 fps, but they are consumed at about 2 fps. When every queued V4L2 buffer is full, the driver drops *new* frames rather than overwriting old ones. The frame dequeued at the next `read()` was therefore captured just after the *previous* read, nearly a full cycle earlier. `CAP_PROP_BUFFERSIZE=1` doesn't prevent this: with only one buffer, the frame it holds was still captured right after the previous read. Many UVC drivers also ignore the setting and keep 2–4 buffers, which would make the frame even older.

This explanation fits the timing, but it still needs to be measured directly (see §4, step 1).

### 2.4 Why latency makes this controller oscillate

`PanController.update()` computes:

```
pan_cmd = base_pan(now) + kp × error(frame captured ~0.7 s ago)
```

`base_pan` is the *current* measured or estimated pan. The error, however, describes where the target was relative to the pan **at frame time**. Once the servo has moved since then, adding the old error to the new base **counts the same correction twice**. With an error delayed by `d` cycles, the loop is

```
p[k+1] = p[k] + K · (θ − p[k−d])        K ≈ kp × 0.92 ≈ 0.83
characteristic polynomial: z^(d+1) − z^d + K = 0
```

| Delay `d` (cycles) | Largest stable `K` | Our `K ≈ 0.83` |
|---|---|---|
| 0 | 2.0 | stable, no overshoot |
| 1 | 1.0 | stable but rings (|z| = 0.91) |
| 2 | 0.62 | **unstable** |

At a measured delay of about 1.5–2 cycles, the loop sits on or past the stability edge. Without the hysteresis it would diverge. With it, the result is the limit cycle seen in the log. The pan stops only when a stale frame happens to land inside the ±3.3° hold band, as it did at 13:44.5–13:48.9, where it held at 12.66° for 4 s. Otherwise it drives through the target and swings back.

A rough simulation of this loop gives the same picture. With the logged period, a servo speed of about 117°/s and 1–2° of detection noise, the loop wanders by ±5–7° once the delay passes about 0.5 s, and it diverges at about 0.9 s.

These factors make the problem worse but aren't the root cause:
- **The slew limit is effectively off** (27°/cycle at this `dt`), so nothing caps the overshoot.
- **The base alternates between `measured-fresh` and `estimated`.** 45% of `T=130` polls time out (158/350), the alternating pattern from the ~50 ms firmware gate (§4.3 of the telemetry doc). Each timeout also blocks the worker for 0.3 s. The estimate assumes 270°/s and always lands exactly on the last command, while the real servo (about 117°/s) can still be moving. The base therefore jumps between two references from cycle to cycle.
- **Hold is entered on stale frames.** At 13:57.277 the controller entered hold (`|scaled|=0.93°`) while the pan was at 10° and the person was at 20°. That frame had been captured while the servo was sweeping past 20°.

---

## 3. Close-range oscillation (bounding box > 2/3 of the frame)

The latency problem above still applies at close range. Several geometry and detection effects add to it:

1. **Clipped boxes move the centroid.** When the person fills the frame, YOLO clips the box at the image edges. The box centre is then the midpoint of *what is visible*, not of the person. When the pan moves, a different part of the body is clipped, so the centroid shifts along with the camera instead of staying fixed to the person. That breaks the assumption that `error = bearing − pan`, and the loop can settle off-centre or chase its own motion.
2. **Box jitter grows with box size.** Arm movement, posture changes and partial-body detections change a near-full-frame box by tens of pixels. At `fx ≈ 812 px`, 50 px is about 3.5°. That is comparable to the hold band (enter 3.3°, exit 5.0°, in unscaled error terms), so noise alone repeatedly triggers moves out of hold, and each move is then amplified by the latency above.
3. **Parallax from the camera offset.** The lens sits 6.65 cm in front of the pan axis (`camera_forward_offset_m`). For a target at range `r`, the error seen by the camera is magnified by about `r / (r − 0.0665)`: ×1.07 at 1 m, ×1.15 at 0.5 m and ×1.28 at 0.3 m. This raises the effective loop gain exactly where the loop is already at its stability limit.
4. **The chosen detection can switch.** `select_target_centroid()` takes the highest-confidence box in each frame and ignores the tracker ID, even though `persist=True` is set. At close range YOLO often outputs overlapping full-body and partial boxes, and switching between them produces jumps of several degrees.
5. **The fisheye edges are less accurate.** The edges of a box this large lie in the strongly distorted part of the lens, where calibration residuals are largest. This is a smaller effect than items 1–4.

---

## 4. Recommended next approach

The steps are ordered so that each one can be validated on its own with the same stand-still test.

### Step 1: Measure the latency (cheap; confirms the theory)
- Log `t_grab` (just after `cap.read()` returns), `t_infer_done` and `t_cmd_sent` for every cycle. Also log the inference time.
- **Measure the frame age directly.** Point the camera at a screen that shows a millisecond clock, read the clock in the saved frame and compare it with `t_grab`. Another option is to command a pan step and count how many cycles pass before the image shifts.
- Expected result: frame age at grab about 0.3–0.5 s, and grab-to-command about 0.4 s.

### Step 2: Always process the newest frame
- Move capture into a **background grabber thread** that calls `cap.grab()`/`read()` continuously at camera rate. It keeps only the latest frame together with its capture timestamp, following the pattern `LidarAccess` already uses. `get_frame()` then returns `(frame, t_capture)`.
- **Remove full-resolution work from the control path.** Run `results[0].plot()` and the JPEG encode on a different thread, at a reduced rate or a reduced resolution. Alternatively, capture at 1280×720 or 640×480: YOLO resizes to 640 anyway, and the intrinsics can be scaled.
- Consider exporting the model to NCNN or ONNX for the Pi, which typically runs 2–3× faster than the `.pt` model on CPU.
- Goal: grab-to-command latency under about 150 ms at 5 Hz or faster.

### Step 3: Time-align the base with the frame (the core controller fix)
Stop adding an old error to a new base. Instead:

1. Keep a short **pan history** in `PanTelemetryPoller`, holding the last ~2 s of `(t, pan_deg)` samples. Add `pan_at(t)`, which interpolates between samples.
2. In `update()`, compute the target's **absolute bearing in the rover frame at the moment of capture**:
   `target_bearing = pan_at(t_capture) + corrected_error`
3. Filter `target_bearing` with a simple EMA or alpha-beta filter, then command `pan_cmd = filtered_target_bearing`. Apply the hold band and slew cap to `|pan_cmd − last_cmd|` rather than to the raw image error.

For a stationary target, `target_bearing` stays constant however far the servo has moved since the frame was captured, so there is no double counting. Latency then only delays the response; it can't destabilise the loop. In the same simulation, this version settled at 20° with no limit cycle at both 0.7 s and 0.9 s of latency.

### Step 4: Fix the close-range behaviour
- **Handle clipped boxes.** When `x1 ≤ margin` or `x2 ≥ W − margin`, treat the horizontal centroid as unreliable. Either hold, or use a reference that doesn't depend on clipping, such as the head or torso. A pose model's shoulder or neck keypoints, or the box top-centre, are good options at close range.
- **Lock onto the track.** With `persist=True`, select the box by **track ID**, and switch only when the current track is lost.
- **Make the gain or band depend on box size.** When the box is wider than about 50% of the frame, lower the gain and widen the hold band, because width is a rough proxy for range. LiDAR range is better once it is fused.
- **Correct for parallax.** Once a range estimate exists (LiDAR or box height), apply `error_pivot ≈ error_cam × (r − 0.0665) / r`.

### Step 5: Tidy up the telemetry path
- Switch to the **continuous stream reader** (`{"T":131,"cmd":1}`), as already recommended in §6.2 of the telemetry doc. This removes the 45% `T=130` timeouts, the 0.3 s blocking and the alternation between `measured-fresh` and `estimated`. Timestamp every sample so it can feed the `pan_at(t)` history from Step 3.
- Lower `tracking_max_measured_velocity_deg_per_s` used by `_propagate_estimate()` to the measured value of about 120°/s, or drop the estimate once the stream provides a fresh sample about every 51 ms.

### Step 6: Retune
- After Steps 2–3, re-check `kp` (start at 0.5–0.7), and recompute the slew cap from the real `dt`. The hold band can probably shrink to about ±2°.
- **Acceptance test:** stand still at 1.5 m and at 0.5 m for 60 s. The pan command's peak-to-peak range should be under about 3°, with no hold-exit/re-enter cycles caused only by noise.

---

## 5. Quick-win option (if you want to test the theory before refactoring)

Before building any of the above, set `tracking_gain_kp: 0.4` and rerun the stand-still test. At a 2-cycle delay the stability limit is `K < 0.62`. If the oscillation stops (with slower convergence), that confirms latency × gain as the cause. This is a diagnostic, not the fix: tracking a moving person at `kp=0.4` and 2 Hz will lag badly.

Lower the hysteresis thresholds in proportion to `kp` (for example `enter: 1.3`, `exit: 2.0`). The hold band is applied to the *scaled* error, so at `kp=0.4` the current values would let the pan settle up to about 7.5° off-centre. That would look like "stable", but for the wrong reason.
