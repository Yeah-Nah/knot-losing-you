# UGV Drive Calibration Investigation Log

Purpose: keep a single running record of what has been tested, what has been learned, and what to test next for poor rover yaw calibration performance.

## Current Status (2026-04-20)

### Latest stable calibration run (run 7, 2026-04-19 10:33 session)
- omega_commands_rad_s: [4.5, 5.0, 5.5, 5.0, 4.5], command_duration_s: 0.20
- dead_band_omega_steps_rad_s: [4.0, 3.0, 2.5]
- turn_rate_gain: 0.378166
- effective_track_width_m: 0.46276
- angular_dead_band_rad_s: null
- n_samples used in gain fit: 6 of 16 total rows
- fit_mae_deg: 1.5773

Run analysis:
- Run completed end-to-end (16 rows) with no template-recapture crash.
- Best fit quality so far: n_samples increased to 6 and MAE dropped to 1.58°.
- Gain remains near prior level (0.378 vs 0.386 in run 5), suggesting a stable but still low effective turn-rate gain.
- `analyse_runs` excluded 2 sign-inconsistent gain samples; opposite-direction events are still present.
- Undistorted matching alone previously did not improve retention; lowering `min_match_score` to 0.45 had the larger impact.
- Dead-band was not estimated (`null`) because all tested dead-band commands were classified as moved.

### Latest diagnostic run (2026-04-20 contention-isolation session)
- Test condition: stream OFF; manual `systemctl --user stop pipewire wireplumber` before run.
- Sweep outcome: completed end-to-end (16 rows) with no camera-dropout failure.
- Save outcome: analysis failed with only 1 good gain sample, so no usable turn-rate fit was produced.

Run analysis:
- Reproduced earlier dropout failures with stream OFF before this test, so MJPEG stream load is not the primary crash trigger.
- During the contention-isolation run, the sweep completed fully, strongly confirming that camera-holder contention is the primary crash root cause.
- `pipewire` and `wireplumber` still reappeared during the run because `pipewire.socket` can reactivate them after the service is stopped.
- The remaining problem in this run was data quality, not camera stability: most gain rows were rejected by quality/sign checks, leaving only 1 usable sample.
- `GET /abort` was separately verified to return HTTP 200 with `{\"status\": \"aborted\"}`; the abort endpoint is functioning.

### Previous calibration run (run 6, 2026-04-19 06:20 session, aborted)
- omega_commands_rad_s: [4.5, 5.0, 5.5], command_duration_s: 0.20
- dead_band_omega_steps_rad_s: [5.0, 4.0, 3.0, 2.5]
- Run did not complete (aborted during transition to dead-band CCW; no final JSON result saved)

Run analysis:
- Run crashed at dead-band CCW start with `Failed to capture frame for template extraction` and V4L2 `errno=19 (No such device)`.
- This prompted the camera recovery/reopen handling work.

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
- Run 7 completed without the run 6 camera/device-dropout failure.
- Usable gain data improved materially (6 samples, MAE 1.58°), primarily after lowering match confidence.
- Stream-disabled failure reproduction showed that stream resolution/load is not the dominant cause of the freeze/dropout behavior.
- Contention-isolation completion showed the dominant crash path is camera-device ownership being reclaimed by `pipewire`/`wireplumber`.
- Main uncertainty has shifted back to two separate issues: camera-holder contention for crash prevention, and tracking/data quality for fit quality.
- Dead-band threshold remains unresolved in current settings because no stall crossover was observed.

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

### 7) Camera recovery path improved run completion
- Run 7 completed through all 16 steps without the template-recapture crash seen in run 6.

### 8) Lower confidence threshold had larger effect than undistortion alone
- Undistorted-only run did not improve match count; lowering `min_match_score` to 0.45 increased retained samples.

### 9) Dead-band is not observable with current probe range
- With [4.0, 3.0, 2.5], all steps were classified as moved, so `angular_dead_band_rad_s` stayed null.

### 10) Block-direction sweep ordering is necessary
- Confirmed from earlier manual testing. Alternating CW/CCW within a run produces inconsistent results due to drivetrain hysteresis and stick-slip near threshold.
- Block ordering (all CCW then all CW) is now baked into the calibration procedure.

### 11) Physical dead-band is load-dependent
- Lifted-wheel tests showed consistent response well below these thresholds.
- On-ground in-place rotation requires overcoming surface scrub, which shifts the effective dead-band above the lifted value.

### 12) Camera-holder contention is the primary crash root cause
- Camera-dropout failures reproduced even with the stream OFF, so MJPEG load is not the primary trigger.
- When the sweep was run under contention isolation, it completed end-to-end without the `errno=19 (No such device)` failure.

### 13) `pipewire`/`wireplumber` can respawn during a run
- Stopping `pipewire.service` alone is insufficient because `pipewire.socket` can reactivate it.
- Mid-run `fuser` output showed `pipewire` and `wireplumber` holding `/dev/video0` again after the run had already started.

### 14) The abort endpoint is not the blocker
- `GET /abort` was verified separately and returned HTTP 200 with `{"status": "aborted"}` during a failed-run test.
- The earlier inability to abort was not reproduced in the direct curl test.

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

1. **Reopen handling still needs holder eviction.** The startup preflight kills existing holders, but the reopen path currently retries `cap.open(...)` without re-running camera preflight.
2. **Confidence threshold is now the dominant fit-quality lever once camera contention is controlled.** Lowering `min_match_score` increases retention but may admit borderline/sign-inconsistent samples.
3. **Dead-band probe range is too high to find crossover.** Lower omega probes are needed to identify a moved/stall boundary.
4. **Real drivetrain asymmetry remains under load.** Even with better completion and retention, delivery remains asymmetric.

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

### Completed: Stream-off crash isolation
- Outcome: camera-dropout failure still occurred with the stream disabled, so MJPEG stream load is not the primary crash source.

### Completed: Camera-holder contention isolation
- Outcome: the sweep completed end-to-end when `pipewire`/`wireplumber` interference was suppressed, confirming holder contention as the primary crash root cause.

## Next Steps

### Reopen-path preflight fix
- Status: completed.
- Outcome: re-running camera preflight in the reopen path fixed the freezing issue.

Additional next steps are intentionally deferred for now pending a different approach.

## Open Questions

1. Does the reopen-path preflight fix eliminate camera-dropout failures without requiring manual service/socket intervention?
2. What `min_match_score` gives the best retention/quality trade-off with undistorted matching enabled once camera contention is controlled?
3. At what omega does moved=False first appear in each direction with the extended dead-band probe?
4. Does battery state materially shift the CW dead-band threshold above 4.5 rad/s?

## Notes for Future Updates

- Keep this file updated after each test session.
- Add date/time, test conditions, and concise measured outcomes.
- Avoid changing multiple variables in the same run unless it is an A/B test.
- Record the full JSON result block and key log lines alongside each run analysis.
