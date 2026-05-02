# Pan Control Stability Theory: Stops, Curves, and Why They Prevent Overshoot

## 1. Learning Goals
By the end of this document, you should be able to:
1. Explain why camera-servo tracking can oscillate even when object detection is correct.
2. Model a pan loop as a sampled-data control system with delay, nonlinearity, and hysteresis.
3. Justify practical stabilizers such as gain reduction, slew limits, filtering, deadband hysteresis, dwell logic, and nonlinear servo compensation.
4. Design a tuning workflow from first principles rather than trial-and-error.

## 2. System Context and Problem Statement
### What it is
A pan-tracking loop takes image error (target offset from image center), converts it to angular error, and sends a pan command to recenter the target.

### Theory
The pan loop is a closed-loop dynamical system with:
1. Sampling: commands are updated at discrete intervals.
2. Delay: camera exposure, inference, communication, and actuator response all add latency.
3. Actuator dynamics: finite speed, inertia, friction, backlash.
4. Measurement noise: detection jitter, quantization, lens effects.

If the controller reacts too aggressively to delayed measurements, commands overshoot the center and repeatedly reverse sign, creating a limit cycle (persistent left-right swinging).

## 3. Root Cause Model for Overshoot
### What it is
Overshoot appears when each control update assumes the previous correction has already settled physically, but the pan axis is still moving.

### Theory
A simplified discrete-time model is:

$$
e_k = \theta^* - \theta_k
$$

$$
u_k = u_{k-1} + \Delta u_k
$$

If the update uses full error each tick,

$$
\Delta u_k = e_k
$$

the loop behaves like aggressive accumulation in command space. With delay and hysteresis, the measured error can flip sign after a delayed response, and the system over-corrects in the opposite direction. Repetition yields oscillation.

| Symbol | Meaning |
|--------|---------|
| $e_k$ | Angular heading error at time step $k$ — difference between desired and actual pan angle |
| $\theta^*$ | Target pan angle setpoint (desired physical angle, usually 0° meaning centred) |
| $\theta_k$ | Actual physical pan angle at step $k$ |
| $u_k$ | Pan command sent to the servo at step $k$ |
| $u_{k-1}$ | Pan command from the previous step |
| $\Delta u_k$ | Incremental change added to the command at step $k$ |

## 4. Stop/Curve 1: Proportional Gain Below Unity
### What it is
Scale heading correction by a gain:

$$
\Delta u_k = K_p e_k, \quad 0 < K_p < 1
$$

| Symbol | Meaning |
|--------|---------|
| $\Delta u_k$ | Pan command increment at step $k$ |
| $K_p$ | Proportional gain — a scalar in $(0, 1)$ that reduces the fraction of error applied per cycle |
| $e_k$ | Heading error at step $k$ |

### Theory
Reducing effective gain increases stability margin in delayed systems:
1. Smaller steps give the plant time to respond before the next update.
2. Near-center overshoot is reduced.
3. Convergence becomes smoother rather than bang-like.

Practical note: start conservatively (for example, equivalent behaviour around 0.25 to 0.5) and increase only until response is acceptable without sustained oscillation.

## 5. Stop/Curve 2: Per-Update Slew-Rate (Delta) Limit
### What it is
Constrain command increment each tick:

$$
\Delta u_k = \operatorname{clip}(K_p e_k, -\Delta_{\max}, \Delta_{\max})
$$

| Symbol | Meaning |
|--------|---------|
| $\Delta u_k$ | Pan command increment at step $k$ |
| $K_p$ | Proportional gain |
| $e_k$ | Heading error at step $k$ |
| $\Delta_{\max}$ | Maximum permitted command change per cycle — the slew rate cap |
| $\operatorname{clip}(x, a, b)$ | Clamp $x$ to the range $[a, b]$; returns $a$ if $x < a$, $b$ if $x > b$, otherwise $x$ |

### Theory
A slew limiter is a nonlinear safety stop against impulsive corrections:
1. Suppresses large jumps caused by transient detection noise.
2. Caps acceleration demand on the servo.
3. Reduces injected energy into oscillatory modes.

This is the motion-control analogue of rate limiting used in aerospace and robotics.

## 6. Stop/Curve 3: Error Low-Pass Filtering
### What it is
Filter measured heading error before control:

$$
e_k^f = \alpha e_k + (1-\alpha)e_{k-1}^f, \quad 0 < \alpha < 1
$$

| Symbol | Meaning |
|--------|---------|
| $e_k^f$ | Filtered heading error at step $k$ |
| $\alpha$ | Filter coefficient in $(0, 1)$ — controls the blend between new measurement and previous estimate; smaller values give heavier smoothing |
| $e_k$ | Raw (unfiltered) heading error at step $k$ |
| $e_{k-1}^f$ | Filtered error from the previous step |

### Theory
Detection jitter is largely high-frequency disturbance. Filtering:
1. Attenuates high-frequency noise.
2. Reduces command chatter and unnecessary reversals.
3. Improves effective signal-to-noise ratio for control decisions.

Tradeoff: smaller $\alpha$ increases smoothing but also adds lag. Too much filtering can itself destabilize a delayed loop.

## 7. Stop/Curve 4: Deadband with Hysteresis (Two-Threshold Stop)
### What it is
Use two thresholds:
1. Enter-hold threshold (smaller magnitude).
2. Exit-hold threshold (larger magnitude).

When in hold state, do not move until error crosses the larger boundary.

### Theory
A single deadband still chatters if noise repeatedly crosses one boundary. Hysteresis adds state memory:
1. Prevents rapid on/off switching near center.
2. Improves robustness to measurement jitter and backlash.
3. Creates a stable quiet zone around setpoint.

This mirrors Schmitt-trigger hysteresis in electronics, adapted to control logic.

## 8. Stop/Curve 5: Direction-Aware Servo Curve Compensation
### What it is
Use calibrated nonlinear inverse maps from desired physical angle to commanded angle, with separate branches for positive-going and negative-going motion.

### Theory
Real servos are not perfectly linear:
1. Gain varies across angle range.
2. Friction and backlash introduce hysteresis.
3. Forward and reverse sweeps follow different input-output paths.

If software assumes command angle equals physical angle, model mismatch behaves like a persistent disturbance. Compensation reduces systematic error by:
1. Computing desired physical pan.
2. Mapping through inverse calibration curve.
3. Selecting branch by motion direction.

This is nonlinear feedforward compensation layered on top of feedback.

## 9. Stop/Curve 6: Reversal Dwell (Sign-Change Hold)
### What it is
When error sign flips, briefly hold or strongly attenuate opposite-direction motion for a small number of cycles before allowing full reversal.

### Theory
Frequent sign flips excite backlash and stiction transitions. A dwell window:
1. Prevents immediate ping-pong reversals from delayed observations.
2. Allows mechanical transients to settle.
3. Reduces wear and command chatter.

This is a hybrid control element combining logic and continuous control.

## 10. Stop/Curve 7: Servo Speed and Acceleration Shaping
### What it is
Use motion-profile limits (speed and acceleration constraints) inside the actuator command path, instead of unconstrained position jumps.

### Theory
Even stable outer-loop setpoints can overshoot if the actuator executes aggressive internal transients. Profile shaping:
1. Reduces inertial overshoot.
2. Improves repeatability under load.
3. Makes plant dynamics more predictable for outer-loop tuning.

Conceptually, this forms a cascade: outer vision loop provides setpoints; inner actuator profile enforces smooth trajectories.

## 11. Unified Control Law (Conceptual)
### What it is
A practical composite law combining filtering, gain scaling, slew limit, hysteresis logic, and saturation.

### Theory

$$
e_k^f = \alpha e_k + (1-\alpha)e_{k-1}^f
$$

If hold logic is active, set $\Delta u_k = 0$. Otherwise,

$$
\Delta u_k = \operatorname{clip}(K_p e_k^f, -\Delta_{\max}, \Delta_{\max})
$$

$$
u_k = \operatorname{clip}(u_{k-1} + \Delta u_k, u_{\min}, u_{\max})
$$

| Symbol | Meaning |
|--------|---------|
| $e_k^f$ | Filtered heading error at step $k$ |
| $\alpha$ | Low-pass filter coefficient |
| $e_k$ | Raw heading error at step $k$ |
| $e_{k-1}^f$ | Filtered error from the previous step |
| $\Delta u_k$ | Pan command increment at step $k$ |
| $K_p$ | Proportional gain |
| $\Delta_{\max}$ | Per-cycle slew rate cap |
| $\operatorname{clip}(x, a, b)$ | Clamp $x$ to $[a, b]$ |
| $u_k$ | Pan command sent at step $k$ |
| $u_{k-1}$ | Pan command from the previous step |
| $u_{\min},\, u_{\max}$ | Hardware pan travel limits (e.g. $-45°$ and $+45°$) |

Then apply direction-aware inverse servo-curve mapping before transmission.

This architecture combines linear-control intuition with nonlinear safeguards needed for real hardware.

## 12. Stability Intuition (University Perspective)
### What it is
A qualitative frequency-domain and nonlinear-systems interpretation.

### Theory
1. Gain reduction and filtering reduce high-frequency loop gain.
2. Slew limits and dwell impose bounded nonlinear action.
3. Hysteresis suppresses boundary chatter via memory.
4. Curve compensation reduces plant uncertainty and directional bias.

Together these increase practical stability margin and suppress limit cycles in delayed, non-ideal electromechanical loops.

## 13. Tuning Methodology (Lab Style)
### What it is
A structured workflow for identifying stable settings.

### Theory
1. Start conservative: low gain, moderate hysteresis, tight slew limit.
2. Verify monotonic convergence from large initial offsets.
3. Increase responsiveness gradually:
   1. raise gain first,
   2. relax slew limit second,
   3. tighten deadband last.
4. Add filtering only as needed to suppress jitter.
5. Validate both directions independently to capture hysteresis asymmetry.
6. Test across target distances and lighting conditions.

Success criterion: no sustained sign-flip oscillation near center over repeated trials.

## 14. Common Failure Signatures
### What it is
Observed motion patterns that diagnose which mechanism is missing or mis-tuned.

### Theory
1. Rapid left-right ping-pong near center: gain too high and/or deadband too narrow.
2. Sluggish delayed convergence: gain too low or filtering too strong.
3. Different behaviour left versus right: uncorrected servo hysteresis/nonlinearity.
4. Large jumps after noisy detections: missing or weak delta clamp.
5. Chatter at deadband edge: single-threshold deadband where hysteresis is required.

## 15. Summary
### What it is
Core takeaway for engineering practice.

### Theory
Pan oscillation in vision-based tracking is usually a control-dynamics issue, not an object-detection issue. Robust performance comes from layered design:
1. Reduce effective gain.
2. Limit per-tick command change.
3. Filter high-frequency measurement noise.
4. Use deadband hysteresis.
5. Compensate actuator nonlinearity and directional asymmetry.
6. Shape reversals and actuator motion profile.

This layered approach converts unstable target chasing into stable convergence with predictable behaviour.
