# Closed vs Open Loop Control - Reference

Covers the theory and control-flow of open-loop and closed-loop control for the Waveshare UGV Rover
used in this project.
Focus: what each loop type assumes, how error propagates, where calibration and command shaping fit,
and why Phase 2 drive calibration (performed with shaping disabled) is still valid for the Phase 3
closed-loop follower architecture.

---

## 1. The Core Idea

A control loop answers one question repeatedly:

"Given what I want, and given what the system is currently doing, what command should I send now?"

The difference between open and closed loop is whether that second input (what the system is doing now)
is used.

- Open loop: command is computed from the reference only.
- Closed loop: command is computed from reference minus measured output (the error).

In symbols, with reference $r$, measured output $y$, and command $u$:

- Open loop: $u = f(r)$
- Closed loop: $u = f(r - y)$

That one subtraction ($r - y$) is the entire reason closed-loop systems can reject model error,
disturbances, and drift.

---

## 2. Open-Loop Control

### Definition

Open-loop control sends commands without using runtime output feedback to correct mistakes.

For the rover, a simple open-loop turn experiment is:

1. Choose an angular velocity command $\omega_c$.
2. Apply it for a fixed duration $T$.
3. Assume the turn angle is $\Delta\theta = \omega_c T$.

This assumption is only exact if the actuator/path model is exact.

### Why Open Loop Is Still Useful

Open loop is the right format for calibration experiments because it isolates plant behaviour.

In Phase 2 drive calibration, the rover is intentionally run in open loop to estimate:

- turn-rate gain $k = \omega_a / \omega_c$
- effective track width $W_{\text{eff}} = W_{\text{nom}} / k$
- angular dead-band $\omega_{\text{dead}}$

Those are plant parameters. Measuring them directly is easier when no outer correction loop hides the raw actuator response.

### Open-Loop Weakness

If any assumption is wrong, error accumulates:

$$
\Delta\theta_{\text{actual}} = k\,\omega_c T
$$

If $k \neq 1$, every command is biased by the same factor.

---

## 3. Closed-Loop Control

### Definition

Closed-loop control uses measured output to continually correct commands.

A standard form is:

$$
e(t) = r(t) - y(t), \qquad u(t) = C\big(e(t)\big)
$$

where $C(\cdot)$ is the controller (often proportional or PI/PID).

### Why It Works

If the rover under-turns, the measured error remains non-zero, so the controller keeps commanding
correction until the error is reduced. Disturbances are not "predicted away"; they are measured and corrected.

In transfer-function form (single-input/single-output):

- Plant: $P(s)$
- Controller: $C(s)$

Closed-loop reference-to-output map:

$$
\frac{Y(s)}{R(s)} = \frac{C(s)P(s)}{1 + C(s)P(s)}
$$

Sensitivity to plant/model error is reduced by loop gain $C(s)P(s)$, which is why closed-loop control is robust compared with open loop.

---

## 4. Control Flow In This Project

### Phase 2 (Calibration)

Drive calibration is intentionally done with command shaping disabled.

Flow:

1. Script commands known $\omega_c$ profiles.
2. `UGVController.move()` maps $(v,\omega)$ to wheel speeds via differential kinematics.
3. Commands are sent directly to hardware.
4. Measured turn outcomes are used to identify $k$, $W_{\text{eff}}$, and dead-band.

This is an identification experiment, not the final follower control architecture.

### Phase 3 (Follower Runtime)

Phase 3 is planned as a cascaded closed-loop system:

1. Inner visual loop keeps the target near image center by driving pan.
2. Outer rover loop maps pan-angle error to chassis angular velocity command.
3. Rover motion changes the observed pan error, closing the loop.

Representative outer-loop law:

$$
\omega_c = k_P\,\phi_{\text{pan}}
$$

where $\phi_{\text{pan}}$ is pan-angle error and $k_P$ is a controller gain.

Because $\phi_{\text{pan}}$ is measured continuously, the system self-corrects if a single actuator command
is imperfect.

---

## 5. Where The Command Shaper Sits

The command shaper in `command_shaper.py` is an actuator-path conditioning block between
high-level commands and motor commands.

It adds:

- reversal cool-down (zero crossing + dwell)
- per-tick rate limiting

So the runtime path is effectively:

$$
\omega_c \;\to\; \text{kinematics} \;\to\; S \;\to\; P \;\to\; y
$$

with shaper dynamics $S$ and physical rover dynamics $P$.

Important point: the shaper mainly changes transients (how quickly commands are applied), not the intended steady command value.
For step or slowly varying commands, it behaves like a smoothing element with near-unit DC gain.

---

## 6. Why Calibration Without Shaper Is Still Valid For Phase 3

### Statement

Calibrating drive in Phase 2 with shaping disabled does not invalidate Phase 3 closed-loop control with shaping enabled.

### Reason 1: Calibration Targets Plant Gain/Bias

Phase 2 calibration estimates drivetrain kinematic bias ($k$, $W_{\text{eff}}$, dead-band). Those are properties of the rover/ground interaction,
not properties of the outer feedback architecture.

Updating `sensor_config.yaml` with $W_{\text{eff}}$ corrects the core $(v,\omega) <-> (v_L,v_R)$ mapping at the model root.

### Reason 2: Closed Loop Corrects Residual Command-Path Error

With feedback, any residual mismatch from shaping-induced lag appears as temporary tracking error,
which drives corrective commands on subsequent cycles.

In other words:

- Open loop would suffer direct transient distortion from the shaper.
- Closed loop measures the result and compensates online.

### Reason 3: Shaper Is A Stability/Hardware-Protection Layer

The shaper's role is to protect hardware and smooth aggressive command transitions.
It is not intended to redefine rover kinematics.

As long as shaper tuning is reasonable (update rate and ramp not excessively restrictive),
its main effect is additional phase lag and reduced jerk, which is handled by controller tuning
($k_P$ and any future derivative/integral terms), not by redoing geometric calibration.

### Practical Consequence

You calibrate geometry/actuator bias once (Phase 2), then tune closed-loop gains in Phase 3 with shaping enabled.
These are complementary tasks:

- Calibration fixes plant scale and thresholds.
- Closed-loop tuning sets responsiveness and damping with runtime dynamics (including shaper lag).

---

## 7. Simple Mental Model

Use this separation:

1. "Is my physics/model scale right?" -> calibration ($W_{\text{eff}}$, dead-band, turn gain).
2. "Is my runtime response smooth/stable/fast enough?" -> controller + shaper tuning.

If heading tracking is biased in one direction over many runs, revisit calibration.
If heading tracks correctly but feels sluggish or oscillatory, retune loop gains and/or shaper limits.

---

## 8. Summary

Open loop is ideal for identifying rover actuator and kinematic parameters because it exposes raw behaviour.
Closed loop is ideal for live following because it continuously corrects error.

In this project, Phase 2 drive calibration without shaping and Phase 3 closed-loop following with shaping are compatible by design:

1. Phase 2 identifies plant-scale parameters ($k$, $W_{\text{eff}}$, dead-band).
2. Phase 3 uses feedback to reject residual runtime mismatch.
3. The command shaper improves command quality and drivetrain safety, mainly affecting transients.

So the calibration remains relevant and useful in Phase 3; you do not lose it by enabling the shaper in the live pipeline.
