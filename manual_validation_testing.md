# UGV Drive Calibration Investigation Log

Purpose: keep a single running record of what has been tested, what has been learned, and what to test next for poor rover yaw calibration performance.

## Current Status (2026-04-19)

### Latest calibration run (run 4, 2026-04-19 04:41 session)
- omega_commands_rad_s: [3.5, 4.0, 4.5], command_duration_s: 0.20
- turn_rate_gain: 0.260056 (**unreliable — see analysis below**)
- effective_track_width_m: 0.672932 (unreliable)
- angular_dead_band_rad_s: 1.75 (**unreliable — see analysis below**)
- n_samples used in gain fit: 3 of 24 total rows
- fit_mae_deg: 10.1595 (worse than run 3)

Run analysis:
- Template recapture is confirmed working (log shows "Template recaptured after recenter" before each of the 3 blocks). n_samples improved from 2 to 3 compared to prior runs.
- **Gain sweep — near-stall readings are corrupting the fit:**
  - CCW 3.5 rad/s: Δ=+1.58°, expected=+40.11°, flag=0. This is below the noise floor (2.0°) and is a near-stall, not a valid rotation measurement. It should have been flagged but slipped through quality gating.
  - CW 3.5 rad/s: Δ=−3.17°, expected=−40.11°, flag=0. Also a near-stall.
  - CW 4.0 rad/s: Δ=−26.96°, expected=−45.84°, flag=0. Genuine delivery, k≈0.588. The only reliable sample in this run.
  - CCW 4.0: Δ=+34.27°, flag=1; CCW 4.5: Δ=−28.89° (sign wrong), flag=1; CW 4.5: flag=1 — all went out of frame.
  - The three flag=0 samples include two near-stall readings, so the OLS fit is dragged toward k≈0.26, which is not a true gain estimate. k≈0.588 from the single reliable CW 4.0 step is more plausible and consistent with run 3's k≈0.65–0.79 range.
- **Near-stall pass-through is a new identified bug:** the quality gate uses a flag based on something other than absolute Δ magnitude. Readings below the 2.0° noise floor (or a minimum fraction of expected) are not being flagged as stalls, allowing them to corrupt the OLS gain fit.
- **Dead-band sweep — target going out of frame within blocks:**
  - CCW block (9 steps, 5.0 down to 1.0 rad/s, no mid-block recentre): multiple sign-flipped and implausible readings (CCW 4.0: −31.76°, CCW 2.0: −40.74°) are template noise from the target leaving frame mid-block. Steps accumulate ~15–40° per step with no recentre, so after 3–4 steps the target has drifted far enough to cause matcher failure.
  - CW block (9 steps): CW 4.5: +76.56° (sign wrong), CW 2.0: +96.13° (sign and magnitude wrong) are obvious template noise from out-of-frame drift.
  - The 1.75 rad/s dead-band result is derived from CW 1.5 rad/s stalling (Δ=−0.41°), which may be correct in isolation, but the surrounding data is too noisy to trust. This conflicts with the confirmed ~3.25–4.5 rad/s threshold from run 3 and earlier runs — the difference likely reflects the CW block starting with the template already stale from earlier steps.
- **Gain omega range too low:** 3.5 rad/s is at or below the confirmed dead-band for both directions, so including it in the gain sweep is noise. The gain omega floor should be set above the known dead-band.
- **Checkerboard distortion:** The visible marker is strongly distorted by the fisheye lens. Template matching is performed on the distorted frame (or a mapped version of it), and the fisheye warp means the checkerboard's appearance changes significantly as the rover rotates and the board moves across the image frame. This is a plausible contributor to template match degradation mid-block, particularly for large rotation steps.
- **Imbalance between gain and dead-band step counts:** There are 6 gain steps (3 per direction) and 18 dead-band steps (9 per direction). The dead-band sweep is 3× longer, consumes more run time, and produces more out-of-frame failures. Reducing dead-band probe density or adding mid-block recentres would improve data quality. Given the gain sweep is still the primary calibration output, it should have comparable or more steps.

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
- Template recapture (implemented before run 4) successfully increased n_samples from 2 to 3 for the gain sweep.
- However, run 4's MAE (10.16°) is worse than run 3's (2.95°) because two near-stall readings at 3.5 rad/s slipped past quality gating and corrupted the OLS fit.
- Near-stall pass-through is now the primary blocker for gain accuracy: the flag logic does not reject readings where |Δ| is below the noise floor or below a minimum meaningful rotation.
- Dead-band sweep quality remains poor due to insufficient recentres: 9 consecutive steps per block accumulate enough rotation to push the target out of frame by step 3–5.
- The primary blocker is no longer the single-template architecture — it is (1) the missing near-stall magnitude gate in the quality flag logic, and (2) the lack of mid-block recentres in the dead-band sweep.

## Confirmed Findings

### 1) Near-stall readings are not being flagged by quality gating
- Run 4 shows CCW 3.5 rad/s delivering only 1.58° (below the 2.0° noise floor) and CW 3.5 delivering only 3.17°, both with flag=0.
- These near-stall readings are included in the OLS gain fit, dragging the result to k≈0.26, far below the physically plausible range (k≈0.60–0.79 from prior runs).
- The quality flag logic needs an explicit minimum-delta check: if |Δ_corrected| < noise_floor (or < some minimum fraction of expected), the step should be flagged as a stall, not accepted as a valid rotation measurement.

### 2) Template recapture at block boundaries is now in place and partially effective
- Template recapture at each `/confirm` was the highest-priority change from run 3. It is confirmed working (log shows "Template recaptured after recenter" before each block).
- n_samples for gain sweep increased from 2 to 3 — CW 4.0 rad/s now passes (previously out-of-frame after step 1).
- The remaining limitation is the near-stall pass-through bug (finding #1), not the template architecture.

### 3) Dead-band sweep has too many steps per block without recentre
- Each dead-band block runs 9 steps (5.0 down to 1.0 rad/s) continuously. Each step can deliver 5–47° of rotation. After 3–5 steps the target can drift significantly off-centre, causing the matcher to either latch onto background features or lose the target entirely.
- The nonsensical readings (CW 4.5: +76.56°, CW 2.0: +96.13°, CCW 4.0: −31.76° sign-wrong, CCW 2.0: −40.74° sign-wrong) are consistent with template noise after out-of-frame drift.
- A mid-block recentre (after approximately 4 steps) would prevent this.

### 4) Gain omega range floor must be above the dead-band
- Steps at 3.5 rad/s are at or below the confirmed dead-band for both directions (~3.25 rad/s CCW, ~4.0–4.5 rad/s CW).
- Including such steps in the gain sweep produces near-stall readings that corrupt the fit. The gain omega floor should be set to at least 4.5–5.0 rad/s (above the highest confirmed directional threshold) so all gain steps are in the linear regime.

### 5) Fisheye distortion may degrade template matching for the checkerboard marker
- The checkerboard is visibly distorted on the fisheye stream. The template is captured from the distorted frame, so initial match quality is reasonable, but as the board moves across the frame during rotation, the distortion pattern at the board's new position differs from the captured template, degrading match quality.
- Using the undistorted (remapped) frame for template capture and matching would give spatially-consistent appearance across board positions and likely improve match stability.

### 6) Single template per sweep is now resolved
- Previously the template was captured once at sweep start and reused for all 24 steps. Template recapture at block boundaries is now implemented and working.

### 7) Directional dead-band asymmetry is confirmed
- CCW threshold: approximately 3.25 rad/s (crossover between 3.0 stalling and 3.5 moving at 0.2s duration)
- CW threshold: approximately 4.0–4.5 rad/s (stalls at 4.5, barely moves at 4.0)
- The ~1 rad/s gap between directions is consistent across multiple runs and aligns with the manual block-direction tests.

### 8) k≈0.58–0.79, with directional asymmetry in delivery
- The single reliable gain sample from run 4 (CW 4.0: k≈0.588) is consistent with the run 3 range (k≈0.64–0.79).
- The combined OLS k=0.714 from run 3 (2 clean samples) and k≈0.588 from run 4 CW 4.0 both fall in the plausible range.
- The k=0.26 output from run 4 is an artefact of near-stall pass-through and should be discarded.

### 9) Block-direction sweep ordering is necessary
- Confirmed from earlier manual testing. Alternating CW/CCW within a run produces inconsistent results due to drivetrain hysteresis and stick-slip near threshold.
- Block ordering (all CCW then all CW) is now baked into the calibration procedure.

### 10) Physical dead-band is load-dependent
- Lifted-wheel tests showed consistent response well below these thresholds.
- On-ground in-place rotation requires overcoming surface scrub, which shifts the effective dead-band above the lifted value.

### 11) Camera preflight contention causes stream freeze on back-to-back runs
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

1. **Near-stall magnitude gate is the highest-priority code fix.** Rejecting gain-step readings where |Δ_corrected| < noise_floor (or < ~10% of expected) will prevent stall artefacts from corrupting the OLS fit. This single change should recover the gain estimate to ~k=0.60–0.79 range.
2. **Mid-block recentre in dead-band sweeps** will prevent the 9-step CW/CCW blocks from accumulating enough rotation to push the target out of frame. Adding a recentre pause after ~4 steps per block (splitting into two 4-5 step sub-blocks) should dramatically reduce noise readings.
3. **Gain omega floor must be raised above the highest directional dead-band (~4.5–5.0 rad/s)** so all gain sweep steps fall in the linear regime and none are near-stall inputs.
4. **Template matching on undistorted frame** may improve match stability across the sweep by removing the position-dependent fisheye warp from the appearance model. Worth testing after the higher-priority fixes are validated.
5. **CW dead-band is genuinely ~1 rad/s higher than CCW** due to left/right drivetrain imbalance. This is a real mechanical asymmetry, not a measurement artefact.
6. Battery or power delivery under load may add run-to-run variation, but this is lower priority until the measurement issues are resolved.

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

## Next Steps (priority order)

### 1) Code fix: add near-stall magnitude gate to quality flag logic
Goal: prevent stall artefacts from being included in the OLS gain fit.

What to change:
- In the gain step quality check, reject (flag=1) any step where |Δ_corrected| < noise_floor_deg (currently 2.0°) OR where |Δ_corrected| < min_fraction × |expected| (e.g., min_fraction=0.1).
- This ensures that a step where the rover barely twitched does not count as a valid rotation measurement regardless of template match quality.

Expected outcome:
- Run 4's two near-stall readings (CCW 3.5: 1.58°, CW 3.5: 3.17°) would be correctly flagged.
- The gain fit would be based only on CW 4.0 (k≈0.588) in run 4, which is far more plausible than k=0.26.

### 2) Code fix: add mid-block recentre to dead-band sweep
Goal: prevent target from leaving frame within a 9-step dead-band block.

What to change:
- Split each 9-step dead-band block (CCW or CW) into two sub-blocks of ~4–5 steps, with a `/confirm` recentre pause and template recapture between them.
- Alternatively, trigger an automatic recentre prompt if the measured Δ accumulation within a block exceeds a threshold (e.g., cumulative |Δ| > 60°).

Expected outcome:
- Dead-band readings at lower omega (where the target has drifted furthest from the previous recenter) become reliable instead of template noise.
- The reported dead-band threshold should converge toward the known ~3.25–4.5 rad/s directional range rather than the artefact 1.75 rad/s from run 4.

### 3) Parameter change: raise gain sweep omega floor above dead-band
Goal: ensure all gain sweep steps are in the linear regime.

What to change:
- Set omega_commands_rad_s floor to at least 5.0 rad/s (above confirmed CW threshold of 4.0–4.5 rad/s).
- Candidate range: [5.0, 6.0, 7.0] or [5.0, 5.5, 6.0] depending on how far the rover travels in 0.2s at higher omega.
- Consider increasing command_duration_s to 0.25–0.30s for more stable per-step angular delivery once the omega range is above dead-band.

Expected outcome:
- All 3 CCW and all 3 CW gain steps produce valid, non-stall readings.
- OLS fit uses 4–6 samples, giving a more reliable gain estimate.

### 4) Investigation: template matching on undistorted frame
Goal: determine whether fisheye distortion is materially degrading template match quality mid-sweep.

What to investigate:
- Check whether capture and matching is performed on the raw fisheye frame or on the undistorted (remapped) frame.
- If raw: switch template capture and matching to the undistorted frame (or at minimum a linearised ROI). The board's appearance is more spatially stable in undistorted coordinates as the rover turns.
- If already undistorted: this factor is ruled out.

Expected outcome:
- Reduced mid-block template degradation for large rotation steps, allowing more steps per block to pass flag gating.

### 5) Battery state sensitivity test (optional, lower priority)
Goal: detect voltage sag influence on dead-band threshold.

Procedure:
- Run one fixed block-direction yaw test at high state-of-charge and at lower state-of-charge.
- Keep surface and payload constant.

Decision rule:
- Significant threshold shift at lower charge indicates power-limited behaviour.

## Open Questions

1. Does battery state materially shift the CW dead-band threshold above 4.5 rad/s?
2. Once the near-stall gate and mid-block recentre are implemented, will the gain fit stabilise across multiple runs at a consistent k value?
3. Is template matching performed on the raw fisheye frame or on a remapped/undistorted frame? If raw, is the distortion warp contributing to mid-sweep match failure?
4. Is the reported dead-band of 1.75 rad/s (run 4) entirely an artefact of out-of-frame template noise, or has the mechanical threshold genuinely shifted compared to earlier runs?

## Notes for Future Updates

- Keep this file updated after each test session.
- Add date/time, test conditions, and concise measured outcomes.
- Avoid changing multiple variables in the same run unless it is an A/B test.
- Record the full JSON result block and key log lines alongside each run analysis.