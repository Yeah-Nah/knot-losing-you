# UGV Drive Calibration Investigation Log

Purpose: keep a single running record of what has been tested, what has been learned, and what to test next for poor rover yaw calibration performance.

## Current Status (2026-04-19)

### Latest calibration run (run 6, 2026-04-19 06:20 session, aborted)
- omega_commands_rad_s: [4.5, 5.0, 5.5], command_duration_s: 0.20
- dead_band_omega_steps_rad_s: [5.0, 4.0, 3.0, 2.5]
- Run did not complete (aborted during transition to dead-band CCW; no final JSON result saved)

Run analysis:
- Gain-floor change is applied (3.5 removed). Early gain samples are stronger than prior run but still under-deliver vs expected.
- Out-of-frame drift still occurs within gain blocks: at step 3 the center marker left frame; at step 6 it was at the left edge.
- Low confidence aligns with edge/distortion conditions: score_after dropped to ~0.47 on flagged 5.5 steps (both directions), with sign failure on CCW 5.5.
- Stream/camera reliability is now a hard blocker: run crashed at dead-band CCW start with `Failed to capture frame for template extraction` and V4L2 `errno=19 (No such device)`.
- This confirms two active blockers: (1) target retention/match robustness at frame edges, and (2) capture-device instability during recenter/template recapture.

### Previous calibration run (run 5, 2026-04-19 05:37 session)
- omega_commands_rad_s: [3.5, 4.0, 4.5, 5.0], command_duration_s: 0.20
- dead_band_omega_steps_rad_s: [5.0, 4.0, 3.0, 2.5]
- turn_rate_gain: 0.385975
- effective_track_width_m: 0.453397
- angular_dead_band_rad_s: 3.5
- n_samples used in gain fit: 4 of 16 total rows
- fit_mae_deg: 4.7026

Run analysis:
- Near-stall gate is active: low-motion gain steps are now rejected (e.g., CW 3.5 flagged; CW 4.5 with 0° flagged).
- Mid-block recenter/template recapture is active and block-agnostic (observed in gain and dead-band blocks).
- Data quality improved versus run 4: n_samples 3→4, total rows 24→16, MAE 10.16°→4.70°.
- Dead-band result (3.5 rad/s) is now plausible and aligns better with prior directional threshold observations.
- Remaining issue: gain is still low (k=0.386) and directional asymmetry remains high (k_pos=0.2838, k_neg=0.4669; asymmetry warning).
- Some movement steps still appear to be discarded by confidence/quality checks.
- Minor capture instability remains (`cap.read()` failures during one CW dead-band step).

### Previous calibration run (run 4, 2026-04-19 04:41 session)
- omega_commands_rad_s: [3.5, 4.0, 4.5], command_duration_s: 0.20
- turn_rate_gain: 0.260056 (**unreliable — see analysis below**)
- effective_track_width_m: 0.672932 (unreliable)
- angular_dead_band_rad_s: 1.75 (**unreliable — see analysis below**)
- n_samples used in gain fit: 3 of 24 total rows
- fit_mae_deg: 10.1595 (worse than run 3)

Run analysis:
- Template recapture was working, but near-stall pass-through and long blocks without mid-block recenter still corrupted results.

### Previous calibration run (run 3, 2026-04-11 14:00 session)
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
- The two implemented fixes remain effective (near-stall gate + block-agnostic mid-block recenter).
- Raising gain floor to [4.5, 5.0, 5.5] removed near-threshold input, but out-of-frame drift still appears by step 3/6 in practice.
- Primary blocker is now capture robustness: run 6 terminated on template recapture due to camera device loss (`No such device`).
- Secondary blocker remains match robustness near frame edges (low `score_after` on high-omega edge cases).

## Confirmed Findings

### 1) Near-stall gate is effective
- Run 5 shows low-motion gain commands are now rejected instead of polluting OLS.

### 2) Mid-block recenter/template recapture is effective and block-agnostic
- Run 5 logs show mid-block recenter plus template recapture in both gain and dead-band blocks.

### 3) Reduced dead-band probe density improved reliability
- Using [5.0, 4.0, 3.0, 2.5] reduced run length and produced a plausible dead-band result (3.5 rad/s).

### 4) Directional dead-band asymmetry is still present
- CCW threshold: approximately 3.25 rad/s (crossover between 3.0 stalling and 3.5 moving at 0.2s duration)
- CW threshold: approximately 4.0–4.5 rad/s (stalls at 4.5, barely moves at 4.0)
- The ~1 rad/s gap between directions is consistent across multiple runs and aligns with the manual block-direction tests.

### 5) Gain fit is improving but still asymmetric
- Run 5 fit improved materially versus run 4, but gain remains low (k=0.386) with asymmetry warning (k_pos=0.2838, k_neg=0.4669).

### 6) Residual tracking robustness issue remains
- Some steps with visible movement still fail quality/confidence gating; undistorted matching is a focused next investigation.

### 7) Camera capture stability is now a hard blocker
- Run 6 aborted at dead-band CCW start: template recapture failed with `Failed to capture frame for template extraction` and V4L2 `errno=19 (No such device)`.
- Calibration cannot complete until capture-device dropouts during recenter/template extraction are handled.

### 8) Block-direction sweep ordering is necessary
- Confirmed from earlier manual testing. Alternating CW/CCW within a run produces inconsistent results due to drivetrain hysteresis and stick-slip near threshold.
- Block ordering (all CCW then all CW) is now baked into the calibration procedure.

### 9) Physical dead-band is load-dependent
- Lifted-wheel tests showed consistent response well below these thresholds.
- On-ground in-place rotation requires overcoming surface scrub, which shifts the effective dead-band above the lifted value.

### 10) Camera preflight contention causes stream freeze on back-to-back runs
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

1. **Capture-device instability during recenter/template extraction is the current blocker.** Device loss (`No such device`) aborts the sweep before completion.
2. **Edge-of-frame distortion is depressing match confidence on high-omega steps.** Undistorted-frame matching should reduce false rejects.
3. **Real drivetrain asymmetry remains under load.** Even with improved gating and sweep structure, delivery remains asymmetric.

## Completed Tests

### Completed: Visual-target isolation
- Outcome: did not recover calibration quality by itself.

### Completed: Block-direction ordering vs alternating
- Outcome: block-direction is clearly more repeatable; alternating direction produces inconsistent results.

### Completed: Dead-band threshold mapping
- Outcome: CCW threshold ~3.25 rad/s, CW threshold ~4.0–4.5 rad/s. In-block non-monotonic pattern at low omega is now understood to be template noise, not real rover response.

### Completed: Multi-session parameter tuning
- Outcome: duration=0.2s, omega=[3.5, 4.0, 4.5] produces reliable CCW 3.5 and CW 3.5 gain readings in run 3. These omega values are now confirmed too low — 3.5 rad/s is at or below the dead-band and should be removed from the gain sweep.

### Completed: Template recapture at block boundary
- Outcome: implemented and confirmed working. n_samples increased from 2 to 3. Did not fully resolve gain quality because near-stall readings at 3.5 rad/s slipped through quality gating.

### Completed: Near-stall gate and block-agnostic mid-block recenter
- Outcome: implemented and confirmed in run 5. n_samples increased to 4/16, MAE improved to 4.70°, and dead-band estimate became plausible (3.5 rad/s).

## Next Steps (priority order)

### 1) Stabilize camera capture during recenter/template extraction
Goal: prevent sweep aborts from camera-device dropouts.

What to change:
- Add retry/reopen handling around template recapture when frame capture fails.
- Add a short guarded recovery path when V4L2 returns transient errors before hard-failing the sweep.

Expected outcome:
- Calibration run continues through dead-band blocks instead of terminating at recenter transitions.

### 2) A/B test undistorted-frame template matching
Goal: reduce low-confidence rejects when target moves toward frame edges.

What to investigate:
- Compare one run with raw-frame template matching vs one run with undistorted-frame matching at the same motion settings.

Expected outcome:
- Higher `score_after` and fewer false rejects on high-omega edge cases.

### 3) Tune confidence/quality gates after undistortion result
Goal: recover valid moving steps without re-admitting bad data.

What to investigate:
- Re-check which gate rejects moving steps (match score, sign consistency, frame quality) after undistortion A/B.

Expected outcome:
- Higher usable sample count with stable fit quality.

### 4) Re-run gain sweep with current [4.5, 5.0, 5.5] once capture is stable
Goal: get a full completed run and re-evaluate k symmetry.

What to do:
- Keep current gain floor and dead-band steps unchanged for one clean confirmation run.

Expected outcome:
- Comparable completed dataset to isolate remaining asymmetry from measurement artefact.

## Open Questions

1. Does battery state materially shift the CW dead-band threshold above 4.5 rad/s?
2. Does undistorted-frame matching materially improve edge-step confidence and retention?
3. After camera recovery handling, do runs complete reliably without manual abort/restart?

## Notes for Future Updates

- Keep this file updated after each test session.
- Add date/time, test conditions, and concise measured outcomes.
- Avoid changing multiple variables in the same run unless it is an A/B test.
- Record the full JSON result block and key log lines alongside each run analysis.