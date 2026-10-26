# Pan Oscillation After the Telemetry Fix: Vision Latency and Close-Range Centroid Error

**Status (2026-10-22):** Step 2 partly done. The background grabber thread is validated on the rover, and the `T=130` timeouts are gone (see §3). The oscillation is smaller but still there. Next: fix the camera frame rate (Step 2), then time-align the controller (Step 3).
**Evidence:**
- [logs/ugv_run_tracking_test.jsonl](logs/ugv_run_tracking_test.jsonl): `ugv-run` from 2026-10-21 12:13:42–12:14:37, target standing still. This is the original diagnosis in §2.
- [logs/post_port_release.jsonl](logs/post_port_release.jsonl): `ugv-run` from 2026-10-22 11:46–11:47, with the grabber thread and the UART conflict fixed. Covered in §3.

**Builds on:** [telemetry_7s_gap_resolution.md](telemetry_7s_gap_resolution.md) (the ~7 s gap is fixed) and [pan_telemetry_serial_worker_crash.md](pan_telemetry_serial_worker_crash.md) (Waveshare's `app.py` was sharing the UART).

---

## 1. Problem statement

With the firmware IMU stall fixed, pan tracking is better but still not stable:

1. **Standing still at normal range:** the pan never settles. It swings between about **10° and 36°** with a period of about 4 s, while the person stays at about **20°**.
2. **Close range** (bounding box wider than about 2/3 of the frame): the pan still hunts and never centres the person. It is less wild than before the telemetry fix.

Telemetry is no longer the main cause. Fresh `T=1001` samples arrive throughout the run, the longest controller-visible gap is under 0.5 s, and the measured pan follows the commands closely.

---

## 2. What the 2026-10-21 log shows

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

### 2.3 Where the latency came from

Two parts added up to roughly the fitted 0.6–0.9 s:

1. **Processing time within the cycle, about 0.4 s.** The frame is grabbed early in the iteration. The pan command is issued only after YOLO has run on it. *Still present; see §3.*
2. **Frame age at grab time, about 0.3–0.5 s, from V4L2 buffering.** `cap.read()` was called once per cycle, so the frame dequeued was captured just after the *previous* read, nearly a full cycle earlier. *Fixed by the background grabber thread: frame age at grab is now 0.12 s on average (§3).*

### 2.4 Why latency makes this controller oscillate

`PanController.update()` computes:

```
pan_cmd = base_pan(now) + kp × error(frame captured some time ago)
```

`base_pan` is the *current* measured or estimated pan ([pan_controller.py:409-419](../../ugv-follower/src/ugv_follower/control/pan_controller.py#L409-L419)). The error, however, describes where the target was relative to the pan **at frame time**. Once the servo has moved since then, adding the old error to the new base **counts the same correction twice**. With an error delayed by `d` cycles, the loop is

```
p[k+1] = p[k] + K · (θ − p[k−d])        K ≈ kp × 0.92 ≈ 0.83
characteristic polynomial: z^(d+1) − z^d + K = 0
```

| Delay `d` (cycles) | Largest stable `K` | Our `K ≈ 0.83` |
|---|---|---|
| 0 | 2.0 | stable, no overshoot |
| 1 | 1.0 | stable but rings (\|z\| = 0.91) |
| 2 | 0.62 | **unstable** |

On 2026-10-21, with a delay of about 1.5–2 cycles, the loop sat on or past the stability edge. Without the hysteresis it would diverge. With it, the result was the limit cycle seen in the log. The pan stopped only when a stale frame happened to land inside the hold band, as it did at 13:44.5–13:48.9, where it held at 12.66° for 4 s. Otherwise it drove through the target and swung back.

A rough simulation of this loop gives the same picture. With the logged period, a servo speed of about 117°/s and 1–2° of detection noise, the loop wanders by ±5–7° once the delay passes about 0.5 s, and it diverges at about 0.9 s.

These factors made it worse but weren't the root cause:
- **The slew limit is effectively off** (27°/cycle at this `dt`), so nothing caps the overshoot.
- **Hold is entered on stale frames.** At 13:57.277 the controller entered hold (`|scaled|=0.93°`) while the pan was at 10° and the person was at 20°. That frame had been captured while the servo was sweeping past 20°.
- **The base alternated between `measured-fresh` and `estimated`.** 45% of `T=130` polls timed out (158/350). *Resolved: this was almost certainly Waveshare's `app.py` reading the same UART. With it disabled, 0 of 873 polls time out and every cycle uses `measured-fresh` (§3). The ~50 ms reply gate in §4.3 of the telemetry doc was traced to the same cause.*

---

## 3. Re-run after the grabber thread and UART fix (2026-10-22)

[logs/post_port_release.jsonl](logs/post_port_release.jsonl). This was **not** a clean stand-still test: the person started at the edge of the frame, then stood up and walked a few metres to the other side. Observed: the pan followed in both directions and settled centred at the end. Oscillation is still visible but smaller than on 2026-10-21.

### 3.1 Measurements

| Metric | 2026-10-21 | 2026-10-22 |
|---|---|---|
| Control cycle period (median) | 0.453 s | 0.475 s |
| Frame age at grab (mean / max) | ~0.3–0.5 s (inferred) | **0.12 s / 0.24 s** (logged) |
| Grab → pan command | ~0.4 s | ~0.46 s |
| Capture → pan command (total) | 0.6–0.9 s (fitted) | **~0.58 s ≈ 1.2 cycles** |
| Camera frames delivered | — | **~4 fps** (`seq` +2 per cycle; 30 fps configured) |
| `T=130` timeouts | 158 / 350 (45%) | **0 / 873** |
| Query round trip (median / max) | — | 15 ms / 63 ms |
| Base used by the controller | alternates | always `measured-fresh` |
| Cycles with no detection | — | 17 of 86, mostly right after a large move |

### 3.2 What changed and what didn't

- **The grabber works.** Frame age at grab dropped from about 0.3–0.5 s to 0.12 s, and `seq` increases steadily with no repeated frames.
- **Telemetry is no longer a factor.** No timeouts, a 50 ms sample interval and a fresh measured base every cycle.
- **The loop rate didn't change.** It is still about 2.1 Hz, as expected, because inference and annotation still set the period.
- **The camera delivers about 4 fps, not 30.** `_open_capture` in [waveshare_camera.py](../../ugv-follower/src/ugv_follower/perception/waveshare_camera.py) never sets `CAP_PROP_FOURCC`, so the camera has most likely fallen back to raw YUYV, which most USB 2 webcams limit to about 5 fps at 1080p. This also caps frame age at about 0.25 s and makes motion blur more likely, which fits the detection dropouts after big moves.
- **The delay is now about one cycle, so the loop rings instead of limit-cycling.** At `d ≈ 1` and `K ≈ 0.83`, the poles are at `0.5 ± 0.76j` (|z| = 0.91), giving a predicted ringing period of about 6.3 cycles ≈ 3 s. The final approach in the log matches: commands of 21.9 → 5.2 → −8.0 → 4.6 → 14.0 → 15.0 → 8.3 → 1.2 → −5.6 → 0.9 → 8.6 → 14.9 → 7.7 → 2.2 → hold, a slowly decaying swing with a period of about 3–4 s. The 45–55 s section (alternating between about 21° and 47°) overlaps with the person walking, so it is weaker evidence.
- **Hold now lasts.** The pan held for 7 s at 58.5°, 4 s at 31.4° and 5 s at 3.3° (until the end of the run), compared with one 4 s hold in the whole 2026-10-21 run.

---

## 4. Close-range oscillation (bounding box > 2/3 of the frame)

Not re-tested on 2026-10-22. The latency problem above still applies at close range. Several geometry and detection effects add to it:

1. **Clipped boxes move the centroid.** When the person fills the frame, YOLO clips the box at the image edges. The box centre is then the midpoint of *what is visible*, not of the person. When the pan moves, a different part of the body is clipped, so the centroid shifts along with the camera instead of staying fixed to the person. That breaks the assumption that `error = bearing − pan`, and the loop can settle off-centre or chase its own motion.
2. **Box jitter grows with box size.** Arm movement, posture changes and partial-body detections change a near-full-frame box by tens of pixels. At `fx ≈ 812 px`, 50 px is about 3.5°. That is comparable to the hold band, so noise alone repeatedly triggers moves out of hold, and each move is then amplified by the latency above.
3. **Parallax from the camera offset.** The lens sits 6.65 cm in front of the pan axis (`camera_forward_offset_m`). For a target at range `r`, the error seen by the camera is magnified by about `r / (r − 0.0665)`: ×1.07 at 1 m, ×1.15 at 0.5 m and ×1.28 at 0.3 m. This raises the effective loop gain exactly where the loop is already close to its stability limit.
4. **The chosen detection can switch.** `select_target_centroid()` takes the highest-confidence box in each frame and ignores the tracker ID, even though `persist=True` is set. At close range YOLO often outputs overlapping full-body and partial boxes, and switching between them produces jumps of several degrees.
5. **The fisheye edges are less accurate.** The edges of a box this large lie in the strongly distorted part of the lens, where calibration residuals are largest. This is a smaller effect than items 1–4.

---

## 5. Next steps

The steps are ordered so that each one can be validated on its own with the same stand-still test.

### Step 1: Always process the newest frame, faster

**Done:** background grabber thread. [`waveshare_camera.py`](../../ugv-follower/src/ugv_follower/perception/waveshare_camera.py) runs a daemon thread that calls `cap.read()` continuously and publishes each frame as a frozen `CameraFrame(image, capture_time_monotonic, seq)`. `get_frame()` returns the newest one under a lock without blocking on I/O. `capture_time_monotonic` is the `t_capture` that Step 2 needs. `Pipeline._read_latest_frame()` logs `camera frame seq=… age=…` every cycle. Tests are in `tests/test_waveshare_camera.py`. Validated on the rover (§3).

**Remaining:**
- [ ] **Fix the camera frame rate.** Run `v4l2-ctl -d /dev/video0 --list-formats-ext`. If MJPG supports 1080p at 30 fps, call `cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))` **before** setting width and height. Log the negotiated FOURCC, resolution and FPS after opening. Re-run and check that `seq` advances at about 30/s and frame age drops below about 35 ms.
- [ ] **Remove full-resolution work from the control path.** Run `results[0].plot()` and the JPEG encode on a different thread, at a reduced rate or resolution. Alternatively, capture at 1280×720 or 640×480: YOLO resizes to 640 anyway, and the intrinsics can be scaled. Log inference time per cycle to see how much of the ~0.46 s grab → command time each part takes.
- [ ] **Consider exporting the model to NCNN or ONNX** for the Pi, which typically runs 2–3× faster than the `.pt` model on CPU.
- [ ] **Check CPU use with `top` once the camera runs at 30 fps.** The grabber decodes every frame, and at 30 fps most are thrown away. If that decode measurably slows inference, lower the camera `fps` in config first. The alternative, `cap.grab()` on the grabber thread and `cap.retrieve()` on demand, needs a lock across both calls because `cv2.VideoCapture` is not thread-safe, can stall the control loop by up to one frame period, and must take the timestamp from the `grab()` being decoded.
- Goal: grab-to-command latency under about 150 ms at 5 Hz or faster.

### Step 2: Time-align the base with the frame (the core controller fix)

Stop adding an old error to a new base. Instead:

1. Keep a short **pan history** in `PanTelemetryPoller`, holding the last ~2 s of `(t, pan_deg)` samples timestamped when each reply arrives. Add `pan_at(t)`, which interpolates between samples. Telemetry now arrives every ~50 ms with no timeouts, so the history is dense enough.
2. In `update()`, compute the target's **absolute bearing in the rover frame at the moment of capture**:
   `target_bearing = pan_at(t_capture) + corrected_error`
3. Filter `target_bearing` with a simple EMA or alpha-beta filter, then command `pan_cmd = filtered_target_bearing`. Apply the hold band and slew cap to `|pan_cmd − last_cmd|` rather than to the raw image error.

For a stationary target, `target_bearing` stays constant however far the servo has moved since the frame was captured, so there is no double counting. Latency then only delays the response; it can't destabilise the loop. In the same simulation, this version settled at 20° with no limit cycle at both 0.7 s and 0.9 s of latency.

### Step 3: Fix the close-range behaviour

- **Handle clipped boxes.** When `x1 ≤ margin` or `x2 ≥ W − margin`, treat the horizontal centroid as unreliable. Either hold, or use a reference that doesn't depend on clipping, such as the head or torso. A pose model's shoulder or neck keypoints, or the box top-centre, are good options at close range.
- **Lock onto the track.** With `persist=True`, select the box by **track ID**, and switch only when the current track is lost.
- **Make the gain or band depend on box size.** When the box is wider than about 50% of the frame, lower the gain and widen the hold band, because width is a rough proxy for range. LiDAR range is better once it is fused.
- **Correct for parallax.** Once a range estimate exists (LiDAR or box height), apply `error_pivot ≈ error_cam × (r − 0.0665) / r`.

### Step 4: Retune

- After Steps 1–2, re-check `kp` (start at 0.5–0.7), and recompute the slew cap from the real `dt`. The hold band can probably shrink to about ±2°.
- **Interim option, if you want less ringing before Steps 1–2:** set `tracking_gain_kp: 0.5`. At a one-cycle delay that gives `K ≈ 0.46` and |z| ≈ 0.68, which damps much faster, at the cost of slower acquisition. Scale the hysteresis thresholds with it (for example `enter: 1.7`, `exit: 2.5`): the hold band applies to the *scaled* error, so keeping the current 3.0/4.5 at `kp=0.5` would let the pan settle up to about 9° off-centre.
- **Acceptance test:** stand still at 1.5 m and at 0.5 m for 60 s. The pan command's peak-to-peak range should be under about 3°, with no hold-exit/re-enter cycles caused only by noise.
