# Pan-Tilt Servo Curve Calibration — Reference

Covers the theory and mathematics of pan-axis servo curve calibration for the Waveshare
pan-tilt module used in Phase 2 of the project.
Focus: what problem is being solved, what servo curve calibration means, the mathematics of the
calibration, and a high-level implementation approach.

---

## 1. The Problem

### From Commanded Motion to Physical Motion

In control design, it is tempting to treat an actuator command as if it were a direct physical
quantity. For example, one might assume that commanding a pan angle of $10^\circ$ causes the
camera to rotate by exactly $10^\circ$.

Real low-cost servo-driven pan-tilt systems do not behave this way. The commanded value sent to
the controller and the actual physical angle reached by the mechanism are related, but they are
not identical. The discrepancy comes from several sources:

- nonlinearity in the servo and linkage geometry
- static friction and stiction near zero motion
- mechanical backlash and gear slack
- asymmetric behaviour between leftward and rightward motion
- saturation near the ends of travel

As a result, the actuator has an unknown input-output law:

$$
\phi_{\text{actual}} = f(c)
$$

where:

- $c$ is the commanded pan value sent to the actuator
- $\phi_{\text{actual}}$ is the true physical pan angle reached after the mechanism settles

The purpose of servo curve calibration is to identify this function well enough that later control
algorithms can reason in physical angles rather than in raw command units. I.e. Then your tracking
algorithm can ask for 10∘ physically, and the calibration layer picks the right command value to
achieve it.

### Why This Matters

Any vision-guided controller that steers using pan angle implicitly assumes a relationship between
command and motion. If that relationship is wrong, then the controller is built on a false model.

This leads to predictable failures:

- under-correction in regions where the actuator is less responsive than assumed
- over-correction in regions where the actuator is more responsive than assumed
- poor centring performance near zero because small commands do not move the mechanism at all
- inconsistent behaviour depending on the direction from which the setpoint was approached

Servo curve calibration is therefore a model-identification problem: determine the static mapping
between command and achieved angle, together with the limits and nonlinear effects that matter for
control.

---

## 2. What Servo Curve Calibration Is

### A Static Actuator Calibration

Servo curve calibration is the process of experimentally measuring the actuator's static response.
The word static is important: the aim is not to model transient motor dynamics, overshoot, or
high-frequency vibration. Instead, the actuator is commanded to a sequence of values, allowed to
settle, and then measured at equilibrium.

The calibration outputs a map of the form:

$$
f : c \mapsto \phi
$$

where each commanded value $c$ is associated with a measured settled angle $\phi$.

The inverse map is often even more useful:

$$
f^{-1} : \phi_{\text{desired}} \mapsto c
$$

because control laws usually compute a desired physical angle and then need to convert that angle
into a hardware command.

### Ideal Versus Real Behaviour

An ideal actuator would satisfy:

$$
\phi_{\text{actual}} = c
$$

or perhaps a simple scaled relation such as:

$$
\phi_{\text{actual}} = kc
$$

for some constant gain $k$.

Real servo systems are better described as monotonic but imperfect nonlinear functions. A typical
response curve contains:

- a flat region around zero due to dead-band
- an approximately linear central region
- compression or saturation near travel limits
- possibly different forward and reverse curves due to backlash

This means servo calibration is not just about estimating one gain. It is about identifying the
shape of the full response curve.

---

## 3. A Mathematical Model of the Actuator

### The Forward Map

The simplest useful mathematical description is:

$$
\phi = f(c)
$$

with the assumptions that:

- $f$ is approximately monotonic over the usable range
- $f$ may be nonlinear
- $f(0)$ should be near zero after proper mechanical zeroing, but not necessarily exactly zero

Given a finite set of calibration samples $(c_i, \phi_i)$, the function is not known in closed
form. It is inferred from data.

### The Inverse Map

For control, the inverse relation is often the practical object of interest:

$$
c = f^{-1}(\phi)
$$

This inverse exists only if the forward map is single-valued (for a given command c, there is
exactly one output angle) and monotonic over the operating range (the curve never turns around.
I.e. the camera doesn't pan back in the opposite direction it started). If the mechanism exhibits
strong hysteresis, then the same command may produce different angles depending on motion
direction, and the actuator is better described by two conditional maps:

$$
\phi = f_{+}(c) \quad \text{for motion approached from one side}
$$

$$
\phi = f_{-}(c) \quad \text{for motion approached from the other side}
$$

Hysteresis is when the output depends not just on the current input, but on the history —
specifically, which direction you arrived from. If this is the case then the inverse doesn't
exist because the singled-value condition doesn't hold. A single command c depending on the
direction you're coming from, results in two different angles.

### Piecewise-Linear Representation

In many engineering systems, especially with low-cost actuators, the most practical model is not a
high-order polynomial but a piecewise-linear interpolation over measured samples:

$$
f(c) \approx \text{interp}\big((c_1, \phi_1), (c_2, \phi_2), \dots, (c_n, \phi_n)\big)
$$

This approach has several advantages:

- it preserves the measured data directly
- it is easy to inspect and debug
- it avoids oscillatory behaviour associated with polynomial overfitting
- it is straightforward to invert numerically if the samples are monotonic

For calibration of a small pan servo, this is usually the correct first model.

---

## 4. Dead-Band, Backlash, and Saturation

### Dead-Band

Dead-band is the range of small commands that do not produce observable motion.

If the measurement noise floor is $\epsilon_\phi$, then a practical dead-band threshold is the
smallest command magnitude satisfying:

$$
|f(c)| > \epsilon_\phi
$$

repeatably after settling.

In general, positive and negative dead-bands are not identical, so it is better to estimate:

$$
c_{\text{dead},+}, \qquad c_{\text{dead},-}
$$

separately.

### Backlash and Hysteresis

Backlash occurs when there is mechanical slack in gears or linkages. In that case, the measured
angle depends not only on the current command but also on the direction from which the command was
reached.

Mathematically, the actuator is then not a memoryless function of command alone. It depends on the
history of motion:

$$
\phi_k = f(c_k, \text{history})
$$

In practice, this is usually handled by measuring separate forward and reverse sweeps and either:

- storing separate calibration curves, or
- enforcing a fixed approach direction during runtime setpointing

### Saturation

Near the ends of travel, additional command may produce little or no extra angle. This defines the
usable operating interval:

$$
\phi \in [\phi_{\min}, \phi_{\max}]
$$

Outside this interval, the inverse map is either undefined or unreliable.

---

## 5. Measuring Angle with the Camera

### Turning the Camera into an Angle Sensor

The pan mechanism needs an external reference in order to be calibrated. In this project, that
reference can be obtained from the camera itself after intrinsic calibration.

Place a stationary target in front of the pan-tilt unit. If the target is fixed in the world,
then changing the pan angle causes the target to shift horizontally in the image. That shift can
be converted into a viewing angle using the intrinsic parameters.

This turns the camera into an instrument for measuring pan angle. Note, we've USING THE CAMERA
TO ESTIMATE THE PAN ANGLE. So the previous work that was done on intrinsic calibration, was to
work out if something appears on a particular pixel, where is it physically in the real world
relative to the camera. Basically the intrinsic calibration accounts for the distortion caused
by the lens. So now we can measure the change in angles on a static object to estimate the
pan angle changes caused by different commands sent to the servo.

### The Pinhole Relation

From the pinhole camera model, the horizontal image coordinate satisfies:

$$
u = f_x \frac{x_C}{z_C} + c_x
$$

where:

- $u$ is the horizontal image coordinate of the target centroid
- $f_x$ is the focal length in pixels
- $c_x$ is the principal point
- $(x_C, z_C)$ describe the target ray in the camera frame

Rearranging gives:

$$
\frac{x_C}{z_C} = \frac{u - c_x}{f_x}
$$

The ratio $x_C / z_C$ is the tangent of the target's horizontal bearing relative to the optical
axis, so the image-derived viewing angle is:

$$
\alpha = \arctan\!\left(\frac{u - c_x}{f_x}\right)
$$

If the target was initially centred when the pan axis was at mechanical zero, then after a pure
pan motion the actual pan angle is approximately the negative of the observed image angle:

$$
\phi_{\text{actual}} \approx -\arctan\!\left(\frac{u - c_x}{f_x}\right)
$$

The sign convention depends on the actuator's positive direction and must be confirmed
experimentally, but the magnitude relation follows directly from geometry.

### Zero Condition

The calibration requires a known reference configuration. The natural zero condition is:

$$
u = c_x
$$

meaning that the stationary target lies on the optical axis when the commanded pan value is zero.

Under this condition:

$$
\phi_{\text{actual}} = 0
$$

and all subsequent measured angles are relative to a physically meaningful reference.

---

## 6. The Calibration Data Set

### Sample Pairs

Suppose the actuator is commanded to a sequence of values:

$$
c_1, c_2, \dots, c_n
$$

For each command, after allowing the mechanism to settle, the target is measured in the image and
converted into an achieved angle:

$$
\phi_i = -\arctan\!\left(\frac{u_i - c_x}{f_x}\right)
$$

This yields the calibration data set:

$$
\mathcal{D} = \{(c_i, \phi_i)\}_{i=1}^{n}
$$

The data set is the primary experimental output of the calibration. Any fitted model is secondary
to these measurements.

### Repeated Measurements

In practice, for each command $c_i$, several frames are collected after settling. If the measured
centroids are $u_{i1}, u_{i2}, \dots, u_{im}$, then the mean centroid can be used:

$$
\bar{u}_i = \frac{1}{m}\sum_{j=1}^{m} u_{ij}
$$

and the corresponding angle estimate is:

$$
\phi_i = -\arctan\!\left(\frac{\bar{u}_i - c_x}{f_x}\right)
$$

This reduces noise from image jitter and imperfect target detection.

### Measurement Uncertainty

If the centroid estimate has pixel uncertainty $\sigma_u$, then the angular uncertainty follows
from error propagation. Since

$$
\phi = -\arctan\!\left(\frac{u - c_x}{f_x}\right)
$$

the derivative is:

$$
\frac{\partial \phi}{\partial u} = -\frac{1}{f_x\left(1 + \left(\frac{u - c_x}{f_x}\right)^2\right)}
$$

so the first-order angular uncertainty is approximately:

$$
\sigma_\phi \approx \left|\frac{\partial \phi}{\partial u}\right|\sigma_u
$$

Near the image centre, this simplifies to:

$$
\sigma_\phi \approx \frac{\sigma_u}{f_x}
$$

which shows directly that better centroid accuracy and larger focal length both improve the angle
measurement.

---

## 7. Fitting the Servo Curve

### Goal of the Fit

The fit is not an abstract numerical exercise. Its purpose is to construct a usable actuator model
from measured samples.

There are three common options:

- linear fit
- polynomial fit
- piecewise-linear interpolation

For a low-cost pan servo, piecewise-linear interpolation is usually preferred unless the measured
data are exceptionally smooth.

### Linear Approximation

The simplest model is:

$$
\phi \approx ac + b
$$

This is easy to estimate and easy to invert, but it assumes a constant gain across the full range.
That assumption is often violated in practice, especially near the centre and near the ends.

### Polynomial Approximation

A polynomial model might take the form:

$$
\phi \approx a_0 + a_1c + a_2c^2 + a_3c^3
$$

This can represent smooth nonlinearity, but it has drawbacks:

- it may oscillate between measured points
- it may behave poorly near the interval boundaries
- inversion is more complicated

### Piecewise-Linear Approximation

If the measured samples are sorted by command, then interpolation between neighbouring samples
provides a direct model:

$$
\phi(c) \approx \phi_i + \frac{\phi_{i+1} - \phi_i}{c_{i+1} - c_i}(c - c_i)
$$

for $c \in [c_i, c_{i+1}]$.

The inverse is analogous if the angle samples are monotonic:

$$
c(\phi) \approx c_i + \frac{c_{i+1} - c_i}{\phi_{i+1} - \phi_i}(\phi - \phi_i)
$$

for $\phi \in [\phi_i, \phi_{i+1}]$.

This representation is simple, physically interpretable, and faithful to the measured data.

---

## 8. High-Level Calibration Procedure

### Step 1 — Establish Mechanical Zero

Command the pan axis to its nominal zero value and allow it to settle. Place a stationary target
so that its centroid lies on the camera principal point:

$$
u = c_x
$$

This defines the reference condition for the experiment.

### Step 2 — Sweep the Command Range

Choose a set of command values spanning the usable range of the actuator, for example from left to
right in fixed increments and then back again. The reverse sweep is valuable because it exposes
hysteresis.

### Step 3 — Measure the Settled Angle

For each commanded value:

- send the command
- wait for the mechanism to settle
- capture several image frames
- estimate the target centroid in each frame
- average the centroid measurements
- convert the average centroid to a physical pan angle

### Step 4 — Estimate Dead-Band and Limits

Use the measured samples to determine:

- the smallest positive command producing reliable motion
- the largest negative command producing reliable motion
- the minimum and maximum usable achieved angles

### Step 5 — Construct the Calibration Map

Build a forward map $c \mapsto \phi$ and, if possible, an inverse map $\phi \mapsto c$.
If hysteresis is substantial, use directional maps or adopt a fixed setpoint-approach rule.

---

## 9. What the Calibration Produces

The calibration should produce, at minimum, the following information:

- a set of measured command-angle sample pairs
- an estimate of positive and negative dead-band
- the usable pan-angle interval
- a chosen fit model or interpolation method
- optionally separate forward and reverse sweep data

Conceptually, the result is a measured actuator model that lets later control design operate in
physical angle space rather than raw hardware-command space.

---

## 10. Assumptions and Limitations

This calibration note assumes the following:

- the target remains stationary throughout the calibration
- the pan axis dominates horizontal image motion
- camera intrinsics are already known
- the mechanism is measured only after settling

The calibration is intentionally static. It does not model:

- transient response time
- overshoot and oscillation
- vibration during vehicle motion
- coupling between pan and tilt under dynamic loading

Those effects belong to dynamic actuator modelling and controller design. Servo curve calibration
is the earlier and more fundamental problem of identifying the actuator's equilibrium mapping.

---

## 11. Summary

Pan-tilt servo curve calibration is an actuator-identification problem. The aim is to determine how
commanded pan values relate to the physical pan angles actually achieved by the mechanism.

The core mathematical object is the static map:

$$
\phi = f(c)
$$

which is measured experimentally by commanding a sweep of actuator values and estimating the actual
angle from image geometry:

$$
\phi_{\text{actual}} \approx -\arctan\!\left(\frac{u - c_x}{f_x}\right)
$$

From the resulting sample set, one can estimate dead-band, identify saturation limits, detect
hysteresis, and construct an inverse map suitable for later control.

In engineering terms, the calibration replaces the fiction of perfect actuation with a measured
model of the real mechanism.
