# UGV Drive Calibration Investigation Log

Purpose: keep a single running record of what has been tested, what has been learned, and what to test next for poor rover yaw calibration performance.

## Current Status (2026-04-11)

### Latest calibration run (run 3, 14:00 session)
- omega_commands_rad_s: [3.5, 4.0, 4.5], command_duration_s: 0.20
- turn_rate_gain: 0.714559
- effective_track_width_m: 0.244906
- angular_dead_band_rad_s: 3.25 (unreliable — see analysis below)
- n_samples used in gain fit: 2 of 24 total rows
- fit_mae_deg: 2.9538 (good fit quality on those 2 samples, but only 2 samples)

Run analysis:
- Only CCW 3.5 (k≈0.79) and CW 3.5 (k≈0.64) passed quality gating. All other gain steps were flag=1 because the target had left the frame by step 2-3 of each gain block.
- The target was visible only for steps 1–2, 4–5, 7, and 20 (out of 24).
- CCW dead-band data (steps 7–15) is corrupt: after step 8 the target was off frame and the template matcher returned background noise. Readings of −72.56° and +15.45° at low omega are not real.
- CW dead-band (steps 16–24): steps 16–17 stalled cleanly (target visible), step 18 barely moved (−4.54°), step 19 moved (−23.30°). The +121.42° at step 22 is template noise. CW threshold appears to be around 4.0–4.5 rad/s.
- The 3.25 rad/s dead-band result derives from noisy CCW data and should not be used.

### Previous calibration run (run 2, 13:29 session)
- omega_commands_rad_s: [5.0, 6.0, 7.0], command_duration_s: 0.10
- turn_rate_gain: 0.56212 (fit MAE: 11.8062°)
- n_samples: 2 of 20
- Only CCW 5.0 and CW 5.0 passed; CW 5.0 delivered only 15% (template tracking confusion)

### Trend
- The gain fit quality (MAE) is improving with each run: run 3 MAE=2.95° is much better than run 2 MAE=11.8°.
- But n_samples remains stuck at 2 because the template goes stale after 1–2 rotations per block.
- The primary blocker is no longer the physical drivetrain — it is the template tracking architecture.

## Confirmed Findings

### 1) Single template per sweep is the primary blocker
- The template is captured once at sweep start and reused for all 24 steps across 4 blocks.
- After 1–2 rotations within any block, the target has shifted far enough that template match quality degrades (flag=1) or the matcher latches onto background features (garbage deltas).
- This limits the gain fit to the first usable step per block (1× CCW, 1× CW = 2 samples maximum).
- Recapturing the template at each block boundary (immediately after each `/confirm`) would remove this constraint entirely.

### 2) Directional dead-band asymmetry is confirmed
- CCW threshold: approximately 3.25 rad/s (crossover between 3.0 stalling and 3.5 moving at 0.2s duration)
- CW threshold: approximately 4.0–4.5 rad/s (stalls at 4.5, barely moves at 4.0)
- The ~1 rad/s gap between directions is consistent across multiple runs and aligns with the manual block-direction tests.

### 3) k≈0.65–0.79, with directional asymmetry in delivery
- CCW at 3.5 rad/s × 0.2s: delivered 32.6° vs 40.1° expected (k≈0.79)
- CW at 3.5 rad/s × 0.2s: delivered 26.5° vs 40.1° expected (k≈0.64)
- The combined OLS k=0.714 is a reasonable estimate, but the directional split confirms differential drivetrain behaviour.

### 4) Block-direction sweep ordering is necessary
- Confirmed from earlier manual testing. Alternating CW/CCW within a run produces inconsistent results due to drivetrain hysteresis and stick-slip near threshold.
- Block ordering (all CCW then all CW) is now baked into the calibration procedure.

### 5) Physical dead-band is load-dependent
- Lifted-wheel tests showed consistent response well below these thresholds.
- On-ground in-place rotation requires overcoming surface scrub, which shifts the effective dead-band above the lifted value.

### 6) Camera preflight contention causes stream freeze on back-to-back runs
- `fuser -k` kills pipewire/wireplumber, which can restart and re-grab `/dev/video0` before the next run begins.
- Waiting 30–60 seconds between runs avoids this.

## Platform Context (Waveshare UGV Rover)

From product documentation:
- 6 wheels total, 4 drive wheels
- approximate mass with PT kit around 2.19 kg
- supports in-situ rotation (minimum turning radius 0 m)
- sub-controller uses closed-loop speed control

Why this matters:
- 6-wheel geometry with only 4 driven wheels increases scrub during in-place turns on some surfaces.
- Waveshare documentation indicates the rear motor interface may not use rear encoder feedback, so "closed-loop speed control" may be partial, contributing to directional asymmetry under load.

## Working Hypotheses (ranked)

1. **Template recapture at each block boundary** will unlock more than 2 usable gain samples and is the highest-priority code change.
2. **CW dead-band is genuinely ~1 rad/s higher than CCW** due to left/right drivetrain imbalance. This is a real mechanical asymmetry, not a measurement artefact.
3. **0.2s pulse duration is in the acquisition transient** — delivery ratio varies more than it should for a steady-state kinematic model. Longer pulses (0.3–0.4s) may give more repeatable angular delivery per omega once the template recapture issue is resolved.
4. Battery or power delivery under load may add run-to-run variation, particularly for CW which operates closer to its threshold.

## Completed Tests

### Completed: Visual-target isolation
- Outcome: did not recover calibration quality by itself.

### Completed: Block-direction ordering vs alternating
- Outcome: block-direction is clearly more repeatable; alternating direction produces inconsistent results.

### Completed: Dead-band threshold mapping
- Outcome: CCW threshold ~3.25 rad/s, CW threshold ~4.0–4.5 rad/s. In-block non-monotonic pattern at low omega is now understood to be template noise, not real rover response.

### Completed: Multi-session parameter tuning
- Outcome: duration=0.2s, omega=[3.5, 4.0, 4.5] produces reliable CCW 3.5 and CW 3.5 gain readings. Higher omega steps fail flag gating due to template staleness.

## Next Steps (priority order)

### 1) Code change: recapture template at each block boundary
Goal: allow more than 2 reliable gain samples per sweep.

What to change:
- After each recenter `/confirm`, recapture the template from the current frame before resuming the next block.
- The template passed into the sweep step functions should reflect the rover's current position, not the original start position.

Expected outcome:
- Up to 3 gain samples per direction per sweep (or however many omega commands are configured) instead of just 1.
- Dead-band CCW data should become reliable instead of template noise past step 1.

### 2) After template recapture: extend gain omega range upward
Goal: get at least 4–6 gain samples across a useful range.

Candidate parameters (tune after code fix):
- omega_commands_rad_s: [3.5, 4.5, 5.5] (CCW) — note CW may need higher floor
- command_duration_s: 0.20s initially, consider 0.25s for more stable delivery
- Dead-band probe: keep [5.0, 4.5, 4.0, 3.5, 3.0] once template recapture is in place

### 3) Battery state sensitivity test (optional, lower priority)
Goal: detect voltage sag influence on dead-band threshold.

Procedure:
- Run one fixed block-direction yaw test at high state-of-charge and at lower state-of-charge.
- Keep surface and payload constant.

Decision rule:
- Significant threshold shift at lower charge indicates power-limited behaviour.

## Open Questions

1. Does battery state materially shift the CW dead-band threshold above 4.5 rad/s?
2. Once template recapture is implemented, will the gain fit stabilise across multiple runs or will run-to-run variance remain high?

## Notes for Future Updates

- Keep this file updated after each test session.
- Add date/time, test conditions, and concise measured outcomes.
- Avoid changing multiple variables in the same run unless it is an A/B test.