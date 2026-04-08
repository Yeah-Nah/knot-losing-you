# Rover Drive Calibration — Reference

Covers the theory and mathematics of rover drive calibration for the Waveshare UGV Rover 6WD
used in Phase 2 of the project.
Focus: what the differential drive kinematic model assumes, why real hardware violates those
assumptions, the mathematics of turn rate gain and dead-band, and what the calibration produces.

---

## 1. The Problem

### What the Controller Assumes

The `UGVController.move()` method takes two high-level velocity commands — a **linear velocity**
$v$ (m/s, forward positive) and an **angular velocity** $\omega$ (rad/s, left/counter-clockwise
positive) — and converts them to individual left and right wheel speed commands:

$$v_L = v - \frac{\omega W}{2}, \qquad v_R = v + \frac{\omega W}{2}$$

where $W$ is the **track width** — the distance between the left and right wheel contact points.
These left and right wheel speeds are sent directly to the rover's sub-controller as the `L` and
`R` fields of the JSON command.

The implicit assumption baked into this calculation is that **the wheel speeds sent to the motor
controller exactly determine the angular velocity of the chassis**. Specifically, if the sub-
controller is told to drive $v_L$ and $v_R$ for $T$ seconds, the chassis will rotate by exactly:

$$\Delta\theta = \omega \cdot T = \frac{v_R - v_L}{W} \cdot T$$

### Why This Assumption Fails in Practice

Real differential-drive robots violate this in several predictable ways:

- **Track width uncertainty.** The value of $W$ in `sensor_config.yaml` is approximate. The
  effective track width depends on the exact contact geometry of all six wheels and is not
  precisely measurable from a data sheet.

- **Motor dead-band.** Below a minimum commanded wheel speed, static friction prevents the wheels
  from rotating at all. A differential command that should produce slow rotation just stalls
  both sides, and no rotation occurs.

- **Motor gain asymmetry.** The left and right motor assemblies, while nominally identical, are not
  electrically or mechanically identical. One side may respond more aggressively than the other at
  the same commanded value, causing the rover to drift even when equal speeds are commanded.

- **Sub-controller non-linearity.** The ESP32 sub-controller maps received `L`/`R` values to PWM
  duty cycles. This mapping may not be perfectly linear across the full command range.

The result of all these effects combined is that the angular velocity the rover actually achieves
is a distorted and thresholded version of the angular velocity commanded. Before Phase 3 builds a
control loop that relies on the rover turning by a predictable amount, these effects must be
characterised.

---

## 2. Differential Drive Kinematics

### The Unicycle Model

A differential-drive vehicle can be idealised as a **unicycle** — a single point with a position
and a heading. Under this model, the wheel speeds and chassis motion are related by two
simultaneous equations:

$$v = \frac{v_R + v_L}{2}, \qquad \omega = \frac{v_R - v_L}{W}$$

The first equation says the chassis moves forward at the average wheel speed. The second says the
chassis rotates at a rate equal to the speed difference divided by the track width.

Inverting these equations gives the control law used in the implementation:

$$v_L = v - \frac{\omega W}{2}, \qquad v_R = v + \frac{\omega W}{2}$$

This is the **kinematic model** of the rover. It assumes rigid wheels, no slip, a flat surface,
and perfect motor response. Each of these assumptions introduces error in practice; the drive
calibration quantifies the most important of them.

### Track Width as the Critical Parameter

Of all the parameters in this model, $W$ has the largest leverage on turn accuracy. Observe that
the differential speed — the quantity that actually causes rotation — is:

$$v_R - v_L = \omega W$$

If $W$ in the software is wrong by a factor, the differential speed sent to the motors is wrong by
the same factor, and the rover turns at the wrong rate by exactly that factor. There is no other
parameter in the model that can absorb this error.

Concretely: if the true wheel-contact separation is 280 mm but the config says 300 mm, then the
software will command:

$$v_R - v_L = \omega \times 0.3$$

while the rover actually needs:

$$v_R - v_L = \omega \times 0.28$$

to turn at that rate. The rover will systematically over-turn by a factor of $300/280 \approx 1.07$
— a 7% error on every turn, in the same direction, on every run.

### Heading Drift Over Time

In the follower application, turn errors accumulate. If the rover makes a series of small
corrective turns to track a person walking in a gentle arc, each individual turn error is small,
but they compound in the same direction. The calibrated gain eliminates the systematic component
of this accumulation.

---

## 3. The Turn Rate Gain

### Defining the Gain

Let $\omega_c$ be the commanded angular velocity and $\omega_a$ be the angular velocity the rover
actually achieves. The **turn rate gain** $k$ is the ratio of actual to commanded:

$$k = \frac{\omega_a}{\omega_c}$$

An ideal rover has $k = 1$. A rover that under-turns has $k < 1$; one that over-turns has $k > 1$.

### Relating Gain to Effective Track Width

The commanded angular velocity produces a differential wheel speed:

$$v_R - v_L = \omega_c \cdot W_\text{nom}$$

The rover's actual angular velocity is determined by how that differential speed interacts with
the true physical geometry. Treating the ground-truth geometry as an **effective track width**
$W_\text{eff}$:

$$\omega_a = \frac{v_R - v_L}{W_\text{eff}} = \frac{\omega_c \cdot W_\text{nom}}{W_\text{eff}}$$

Comparing with the definition of $k$:

$$k = \frac{W_\text{nom}}{W_\text{eff}}$$

So the turn rate gain and the effective track width are two descriptions of the same fact.
Rearranging:

$$\boxed{W_\text{eff} = \frac{W_\text{nom}}{k}}$$

The calibration measures $k$ from data. The result is then used to update $W$ in
`sensor_config.yaml`, which corrects the kinematics model at its root rather than applying a
downstream scalar fudge factor.

### Measuring $k$ from a Single Run

The simplest measurement: command the rover to rotate in place at angular velocity $\omega_c$
with no linear motion ($v = 0$) for a fixed duration $T$. The expected angle turned is:

$$\Delta\theta_\text{expected} = \omega_c \cdot T$$

The actual angle turned $\Delta\theta_\text{actual}$ is then measured physically. The gain
estimate from this single run is:

$$\hat{k} = \frac{\Delta\theta_\text{actual}}{\Delta\theta_\text{expected}} = \frac{\Delta\theta_\text{actual}}{\omega_c \cdot T}$$

Using $v = 0$ (pure in-place rotation) is deliberate. When both wheels rotate at equal magnitude
in opposite directions, linear motion and rotational motion are decoupled. Any error in the
measured angle is entirely attributable to the rotational model error, not to any linear drift.

---

## 4. Dead-Band

### What Dead-Band Means

Dead-band is the range of small commands that produce no observable output. In a mechanical system,
it arises from static friction: the motor must exert enough torque to break the contact friction at
the wheel before the platform begins to move. Below this threshold, a command is issued but
nothing happens.

For a differential-drive rover, the relevant dead-band is in *angular velocity* — specifically,
the minimum $|\omega_c|$ (at $v = 0$) below which the rover does not rotate. Since the wheel
speed commanded to each side is $\pm \omega_c W/2$, an equivalent statement is that the
individual wheel speed $|v_R| = |v_L| = \omega_c W/2$ must exceed the static friction threshold.

Formally, the dead-band is the threshold $\omega_\text{dead}$ such that:

$$
\omega_a = \begin{cases} 0 & |\omega_c| < \omega_\text{dead} \\ k(\omega_c - \omega_\text{dead} \cdot \text{sign}(\omega_c)) & |\omega_c| \geq \omega_\text{dead} \end{cases}
$$

Below $\omega_\text{dead}$, the command is entirely absorbed by friction and no rotation occurs.
Above it, the rover begins to turn (with the linear region offset by $\omega_\text{dead}$).

### Why Dead-Band Matters for Control

A controller that does not know the dead-band will issue small corrective commands that silently do
nothing. For example, if the rover is 2° misaligned and the proportional gain produces
$\omega_c = 0.05$ rad/s, but the dead-band is $\omega_\text{dead} = 0.08$ rad/s, then the
command is below the threshold and the rover stays misaligned indefinitely.

Conversely, a controller that knows $\omega_\text{dead}$ can apply a minimum-kick: any non-zero
command is augmented to at least $\omega_\text{dead}$ before being sent to the motors, ensuring
that every intended correction results in actual motion.

### Identifying the Dead-Band

Dead-band identification does not require precise angle measurement. It requires only a binary
outcome: *did the rover move, or not?*

The procedure is to command a series of decreasing angular velocity magnitudes, starting above
the expected threshold. For each command, observe whether the rover rotates. The dead-band
$\omega_\text{dead}$ is the crossover point between commands that produce motion and commands
that do not:

$$\omega_\text{dead} \approx \frac{\omega_\text{last-moves} + \omega_\text{first-stalls}}{2}$$

Because static friction is not perfectly repeatable — it varies with surface, temperature, and
how recently the robot moved — the dead-band has a stochastic component. It is most useful as a
conservative estimate: record the largest $|\omega_c|$ that reliably stalls the rover, and treat
that as the dead-band threshold used in Phase 3.

---

## 5. Fitting the Turn Rate Gain From Multiple Samples

### Why Multiple Samples

A single measurement of $\hat{k}$ carries the full measurement error of that one run — human
reading error in the physical angle, uncertainty in the timing, surface unevenness, and so on.
With $N$ measurements at different commanded angular velocities, the estimate improves and outliers
can be identified.

### The Linear Model

For a set of $N$ in-place rotation runs, each at commanded rate $\omega_{c,i}$ for time $T$, the
measured angle is modelled as:

$$\Delta\theta_i = k \cdot (\omega_{c,i} \cdot T) + \epsilon_i$$

where $\epsilon_i$ is measurement noise. This is a linear model in the single unknown $k$, with
no intercept term. The no-intercept constraint is correct: at zero commanded angular velocity,
the expected angle turned is zero regardless of $k$.

### Least-Squares Estimate

Minimising $\sum_i \epsilon_i^2$ with respect to $k$ yields the **least-squares** estimate:

$$\hat{k} = \frac{\displaystyle\sum_{i=1}^{N} \Delta\theta_i \cdot (\omega_{c,i} \cdot T)}{\displaystyle\sum_{i=1}^{N} (\omega_{c,i} \cdot T)^2}$$

This is a regression through the origin. The denominator weights each run by the square of the
expected angle, meaning larger-angle runs (higher $\omega_c$ or longer $T$) have more influence on
the estimate — as they should, since they carry less fractional measurement error.

The residual for run $i$ is:

$$r_i = \Delta\theta_i - \hat{k} \cdot \omega_{c,i} \cdot T$$

Large residuals indicate runs where the surface, initial conditions, or measurement were anomalous
and should be excluded or rerun.

### Practical Angle Range

Larger rotation angles produce better estimates because the relative measurement error is smaller.
If a human measurement has an angular resolution of roughly ±2°, then:

| Rotation angle | Relative error |
|----------------|---------------|
| 10° | ±20% |
| 45° | ±4.4% |
| 90° | ±2.2% |
| 180° | ±1.1% |

Targeting angles in the range 90°–180° per run is a practical sweet spot: large enough that
measurement noise is small relative to the signal, but small enough to be completed on a
reasonably sized floor space. For a 2 s command duration, these angles correspond to angular
velocity commands of roughly 0.8–1.6 rad/s.

### Asymmetry

If positive $\omega_c$ (counter-clockwise) and negative $\omega_c$ (clockwise) produce noticeably
different gains, the motor assembly or friction is asymmetric. In that case, fit separate gain
values $k_+$ and $k_-$ using only positive and only negative commands respectively. The Phase 3
controller then applies the appropriate gain depending on the sign of the angular velocity
command.

---

## 6. Propagation of Error Into Phase 3

### How the Gain Is Used at Runtime

The Phase 3 outer control loop computes a desired chassis angular velocity from the pan angle:

$$\omega_c = k_P \cdot \phi_\text{pan}$$

This command is passed to `UGVController.move()`, which applies the kinematic model to compute
wheel speeds. If $W$ has been corrected to $W_\text{eff}$, then the differential wheel speed
sent to the motors is already correct and $k \approx 1$ at runtime. The correction is
absorbed into the kinematic model itself, which is the cleanest approach.

### Systematic Bias Without Calibration

If the gain error is not corrected and $k \neq 1$, then the chassis turns at $k$ times the
intended rate on every command. This is a **systematic bias** — it does not average out over time.

For a proportional controller with gain $k_P$, the effective loop gain seen by the closed-loop
system is $k_P \cdot k$ rather than $k_P$. If $k > 1$ the loop is effectively over-tuned and may
oscillate; if $k < 1$ it is under-tuned and responds sluggishly. The calibration removes the
ambiguity between "the gain is wrong" and "the controller is mis-tuned".

### Dead-Band at Runtime

In Phase 3, the rover should not receive commands below $\omega_\text{dead}$ unless the intended
command is genuinely zero. Any continuous-valued proportional command below the dead-band threshold
should either be zeroed (if the error is small enough that no turn is needed) or raised to
$\pm \omega_\text{dead}$ (if a correction is needed). This two-regime treatment is only possible
if $\omega_\text{dead}$ has been measured.

---

## 7. What the Calibration Produces

The calibration writes a new `ugv_drive` section to `sensor_config.yaml` containing:

```yaml
ugv_drive:
  calibration_surface: "smooth concrete"   # Description of the surface used
  effective_track_width_m: null            # W_eff = W_nom / k (metres)
  turn_rate_gain: null                     # k = omega_actual / omega_commanded (dimensionless)
  turn_rate_gain_pos: null                 # k for positive omega (ccw) — null if symmetric
  turn_rate_gain_neg: null                 # k for negative omega (cw) — null if symmetric
  angular_dead_band_rad_s: null            # Minimum |omega| that overcomes static friction
  n_samples: null                          # Number of rotation runs used in the fit
  fit_mae_deg: null                        # Mean absolute residual of the fit (degrees)
```

The primary product is `effective_track_width_m`, since it corrects the kinematic model at its
root. The turn rate gain is also stored explicitly so the relationship to the nominal track width
is preserved and the calibration can be re-interpreted if the config value is ever changed.

The `calibration_surface` field is a human-readable note. The dead-band depends on the surface —
smooth concrete has a lower dead-band than rough asphalt. The field records which surface the
measurement was taken on, so the estimate can be treated appropriately if the rover is used on
a different surface.

---

## 8. Assumptions and Limitations

This calibration assumes:

- **The surface is flat.** On an inclined surface, gravity assists or resists rotation depending
  on the direction of turn, and the measured $k$ absorbs both friction and gravitational effects.
  The resulting $W_\text{eff}$ would only be valid on that incline angle.

- **Pure in-place rotation.** All measurements use $v = 0$. The kinematic model couples the turn
  rate to track width only in the differential mode. Straight-line speed calibration (which would
  reveal absolute wheel speed accuracy) is not performed here.

- **Static measurement, not dynamic.** The calibration measures the net rotation after a fixed
  time window. It does not model acceleration, deceleration, or the transient behaviour during
  command transitions. The Phase 3 controller issues commands at a regular cycle rate, so the
  steady-state gain is the relevant quantity.

- **No wheel slip model.** At high commands on low-friction surfaces, wheels may slip and the
  effective gain changes. The calibration should be performed at command levels representative of
  normal operation, not at maximum speed.

The calibration is intentionally simple. It is not a full system-identification exercise. It
identifies one number ($k$, or equivalently $W_\text{eff}$) and one threshold
($\omega_\text{dead}$) — the two parameters that have the most significant and direct effect on
the Phase 3 angular control loop.

---

## 9. Summary

Rover drive calibration is a kinematic model-identification problem. The unicycle model maps
commanded linear and angular velocities to left and right wheel speeds using the track width $W$.
If $W$ is wrong, or if there is a dead-band below which no rotation occurs, commands issued by
the Phase 3 controller will not produce the intended chassis motion.

The calibration measures two things:

1. **Turn rate gain** $k = \omega_\text{actual} / \omega_\text{commanded}$, from which the
   effective track width is recovered as $W_\text{eff} = W_\text{nom} / k$.

2. **Angular dead-band** $\omega_\text{dead}$: the minimum angular velocity command magnitude
   that overcomes static friction and produces observable rotation.

Both are measured by commanding in-place rotations at $v = 0$ and observing the resulting heading
change. The fit uses ordinary least squares over multiple runs at different command magnitudes.
Results are stored in `sensor_config.yaml` and consumed directly by the Phase 3 angular control
loop.
