# Coordinate System Conversion — Reference

Covers the theory and mathematics of coordinate system conversion for LiDAR scan data
used in Phase 3 of the project.
Focus: converting LiDAR polar returns to robot body-frame Cartesian coordinates, handling
angle conventions correctly, and extracting a forward arc for control and safety logic.

---

## 1. Why Coordinate Conversion Exists

### The Mismatch Between Sensor Output and Control Needs

The LiDAR naturally reports each return in **polar coordinates**:

- range $r$ (distance from the LiDAR origin)
- bearing $\theta$ (direction of that return)

So each measurement is of the form:

$$
(r, \theta)
$$

This is ideal for the sensor hardware, but most control logic is easier in a robot body frame:

- $x$: forward distance
- $y$: lateral distance

So the same point is represented as:

$$
(x, y)
$$

The coordinate conversion step translates between these two representations without changing
physical meaning. It answers: "Where is this LiDAR return relative to the rover body?"

### Why This Matters in Phase 3

Phase 3 needs geometric operations that are awkward in polar form but simple in Cartesian form:

- checking whether points lie in front of the rover (forward arc)
- finding nearest obstacle ahead
- associating LiDAR points with a camera-derived search direction

Coordinate conversion is therefore a foundational preprocessing step for all downstream
following and safety behaviour.

---

## 2. Coordinate Frames and Angle Conventions

### LiDAR Frame (Sensor Native)

Per project assumptions (see [Angular Offset Calibration](./angular_offset_calibration.md)),
the LDRobot D500 uses:

- $\theta = 0^\circ$ at LiDAR forward
- clockwise-positive angles when viewed from above
- angles typically wrapped to $[0^\circ, 360^\circ)$

This is a left-handed angular convention in the horizontal plane.

### Rover Body Frame (Control-Friendly)

For control and geometry, a common 2D body frame is:

- +$x$: forward
- +$y$: left

This is right-handed in the horizontal plane.

### Why Conventions Must Be Stated Explicitly

Coordinate conversion formulas depend on angle sign convention. If one part of the pipeline
assumes clockwise-positive and another assumes counter-clockwise-positive, the result can be a
mirrored trajectory and persistent steering errors.

In practice, most conversion bugs are not algebra mistakes. They are convention mismatches.

---

## 3. Geometry of Polar to Cartesian Conversion

### Polar Representation

For a point $P$ observed by LiDAR:

- $r \ge 0$ is radial distance
- $\theta$ is bearing from sensor forward axis

### Trigonometry Primer (From First Principles)

If trigonometry is new, think of one LiDAR return as a point connected to the origin by a
straight line of length $r$. That line and the axes form a right triangle.

```
                P(x, y)
                  *
                 /|
                / |
             r /  | y   (vertical side)
              /θ  |
             *----*
           origin  x     (horizontal side)
```

In this triangle:

- the hypotenuse is $r$
- the horizontal side is $x$
- the vertical side is $y$
- the angle at the origin is $\theta$

Now use the basic right-triangle definitions:

$$
\cos\theta = \frac{\text{adjacent}}{\text{hypotenuse}} = \frac{x}{r}
\quad\Rightarrow\quad
x = r\cos\theta
$$

$$
\sin\theta = \frac{\text{opposite}}{\text{hypotenuse}} = \frac{y}{r}
\quad\Rightarrow\quad
y = r\sin\theta
$$

That is the whole reason the conversion works. Cosine gives the horizontal "share" of the
distance $r$, and sine gives the vertical "share".

Quick sanity checks:

- if $\theta = 0^\circ$, then $\cos\theta = 1$, $\sin\theta = 0$, so $(x, y) = (r, 0)$
	(all distance is forward)
- if $\theta = 90^\circ$, then $\cos\theta = 0$, $\sin\theta = 1$, so $(x, y) = (0, r)$
	(all distance is lateral)

These checks are useful when debugging code: if your conversion does not match these edge
cases, a sign convention or angle-unit issue is likely present.

### Cartesian Representation

In a standard mathematical convention (counter-clockwise-positive):

$$
x = r\cos\theta, \qquad y = r\sin\theta
$$

But this formula assumes $\theta$ increases counter-clockwise. The D500 increases clockwise.
So the sign or angle mapping must be adjusted.

### Equivalent Ways to Handle Clockwise Angles

Two mathematically equivalent approaches are common:

1. Convert D500 bearing to a counter-clockwise angle first:

$$
\theta_{\mathrm{ccw}} = -\theta_{\mathrm{cw}}
$$

then use:

$$
x = r\cos\theta_{\mathrm{ccw}}, \qquad y = r\sin\theta_{\mathrm{ccw}}
$$

2. Keep D500 clockwise angle directly and fold sign into $y$:

$$
\boxed{x = r\cos\theta_{\mathrm{cw}}, \qquad y = -r\sin\theta_{\mathrm{cw}}}
$$

Both produce identical $(x, y)$ when the rover frame is +$x$ forward, +$y$ left.

---

## 4. Angle Wrapping and Normalization

### Why Wrapping Is Necessary

Bearings are circular quantities. For example, $-5^\circ$, $355^\circ$, and $715^\circ$
represent the same direction. Code must map angles into a canonical interval.

### What "Wrap" Means

To **wrap** an angle means to repeatedly add or subtract full turns ($360^\circ$) until the
angle lands inside a chosen interval.

Think of it like a clock face:

- 12:05 and 00:05 are the same clock direction
- similarly, $365^\circ$ and $5^\circ$ are the same geometric direction

So wrapping is not changing the direction. It is only changing how that direction is written.

For example, wrapping to $[0^\circ, 360^\circ)$ gives:

$$
365^\circ \to 5^\circ, \qquad -20^\circ \to 340^\circ
$$

Wrapping to $(-180^\circ, 180^\circ]$ gives:

$$
340^\circ \to -20^\circ, \qquad 181^\circ \to -179^\circ
$$

### Common Canonical Intervals

1. Unsigned interval:

$$
\theta \in [0^\circ, 360^\circ)
$$

2. Signed interval around forward:

$$
\theta \in (-180^\circ, 180^\circ]
$$

The signed form is often easier for forward-arc logic because "left vs right" becomes
"positive vs negative" naturally.

### Practical Wrap Formulas

For degrees, a common implementation is:

$$
\operatorname{wrap}_{[0,360)}(\theta) = \theta - 360\,\left\lfloor\frac{\theta}{360}\right\rfloor
$$

The bracket symbols $\lfloor\cdot\rfloor$ are the **floor** operator, meaning "round down to
the nearest integer."

Examples:

$$
\left\lfloor 2.7 \right\rfloor = 2, \qquad
\left\lfloor -0.3 \right\rfloor = -1, \qquad
\left\lfloor -1.9 \right\rfloor = -2
$$

In the wrap formula, $\left\lfloor\theta/360\right\rfloor$ counts how many full turns of
$360^\circ$ should be removed (or added, when negative) so the result lands in $[0^\circ,360^\circ)$.

Quick checks:

$$
\theta = 725^\circ: \quad \left\lfloor\frac{725}{360}\right\rfloor = 2
\Rightarrow 725 - 360\cdot2 = 5^\circ
$$

$$
\theta = -20^\circ: \quad \left\lfloor\frac{-20}{360}\right\rfloor = -1
\Rightarrow -20 - 360(-1) = 340^\circ
$$

Then a convenient signed wrap is:

$$
\operatorname{wrap}_{(-180,180]}(\theta) = \operatorname{wrap}_{[0,360)}(\theta + 180) - 180
$$

Equivalent modulo forms are fine in code as long as behaviour at boundaries is defined and tested.

### Minimal Angular Difference

When comparing two bearings $\theta_1$ and $\theta_2$, always compute shortest circular error,
not simple subtraction. A robust form is:

$$
\Delta\theta = \operatorname{wrap}_{(-180,180]}(\theta_1 - \theta_2)
$$

This $\Delta\theta$ is the **minimal angular difference** (also called shortest circular error):

- magnitude $|\Delta\theta|$ = smallest turn angle needed
- sign of $\Delta\theta$ = turn direction under your sign convention

### Why Simple Subtraction Fails Near 0/360

Suppose:

- current heading = $359^\circ$
- desired heading = $1^\circ$

Naive subtraction:

$$
  \theta_{\text{desired}} - \theta_{\text{current}} = 1 - 359 = -358^\circ
$$

This suggests a huge clockwise turn. But geometrically, the target is only $2^\circ$ away
across the wrap boundary.

Wrapped shortest error:

$$
\Delta\theta = \operatorname{wrap}_{(-180,180]}(1 - 359)
= \operatorname{wrap}_{(-180,180]}(-358)
= +2^\circ
$$

Now the result matches physical intuition: a small turn is enough.

### Step-by-Step Recipe (Controller Friendly)

Given a desired bearing $\theta_d$ and current bearing $\theta_c$:

1. Subtract normally: $e_0 = \theta_d - \theta_c$
2. Wrap once into signed interval: $e = \operatorname{wrap}_{(-180,180]}(e_0)$
3. Use $e$ as heading error in control logic

This guarantees $e \in (-180^\circ, 180^\circ]$, so the controller always commands the
shortest turn direction instead of accidentally requesting almost a full revolution.

Without this, a tiny mismatch near the $0^\circ/360^\circ$ boundary may appear as a nearly
$360^\circ$ error.

---

## 5. Forward Arc Extraction

### What "Forward Arc" Means

The forward arc is the subset of LiDAR points whose bearings are near rover forward.
If forward is defined as $0^\circ$, then points near $0^\circ$ are potentially relevant to
following and immediate collision checks.

### Typical Definition

Choose a half-width $\beta$ (for example $30^\circ$). Keep points satisfying:

$$
|\operatorname{wrap}_{(-180,180]}(\theta)| \le \beta
$$

This avoids special-case code at $0^\circ/360^\circ$.

### Cartesian Interpretation

In Cartesian terms, forward-arc points usually satisfy:

- $x > 0$ (in front of rover)
- and lateral displacement $|y|$ bounded relative to chosen arc width

Using both angular and Cartesian checks can improve robustness in noisy scans.

---

## 6. From Camera Heading to LiDAR Search Region

Coordinate conversion becomes most useful when tied to target search.
Phase 3 uses a LiDAR search bearing of the form:

$$
\theta_{\text{search}} = \delta_{\text{offset}} + \phi_{\text{pan}} + \delta_{\text{heading}}
$$

(then mapped into LiDAR frame; see [Angular Offset Calibration](./angular_offset_calibration.md)).

Given $\theta_{\text{search}}$, the runtime algorithm typically:

1. selects LiDAR returns within an angular tolerance window around that bearing
2. converts selected points to $(x, y)$ if needed for geometric filtering
3. picks the nearest valid range as target distance

So coordinate conversion is not an isolated math exercise. It is the bridge between raw
scan geometry and control-ready measurements.

---

## 7. Error Sources and Practical Implications

### 1. Convention Error (Most Dangerous)

If clockwise and counter-clockwise conventions are mixed, the rover can steer opposite to intent.
This error is systematic and will not average out.

### 2. Calibration Bias in Angular Terms

If $\delta_{\text{offset}}$ is biased, the LiDAR search window is shifted at every frame.
Even perfect polar-to-Cartesian algebra will then query the wrong spatial region.

### 3. Quantization and Sparsity

LiDAR returns are sampled at discrete angles, not continuous bearings. A search window that is too
narrow may contain no points even when geometry is correct.

### 4. Near-Field Translation Effects

At very short ranges, physical displacement between camera and LiDAR origins can create measurable
association error. This is one reason the project treats full 6-DOF extrinsics as an optional
higher-fidelity path for advanced phases.

---

## 8. Worked Numerical Example

Assume one LiDAR return:

- $r = 2.0\,\text{m}$
- D500 bearing $\theta_{\mathrm{cw}} = 30^\circ$
- rover frame: +$x$ forward, +$y$ left

Using the clockwise-aware conversion:

$$
x = r\cos\theta_{\mathrm{cw}} = 2.0\cos 30^\circ \approx 1.732
$$

$$
y = -r\sin\theta_{\mathrm{cw}} = -2.0\sin 30^\circ = -1.0
$$

Interpretation:

- the return is 1.732 m forward
- and 1.0 m to the right (negative $y$ because +$y$ is left)

This matches intuition for a clockwise-positive bearing.

---

## 9. Implementation Checklist for Phase 3

Before integrating conversion code into `Pipeline._main_loop()`, verify:

1. one explicit definition of rover body frame axes (+$x$, +$y$)
2. one explicit declaration of LiDAR angle convention
3. one tested angle-wrap helper used everywhere
4. one tested polar-to-Cartesian conversion function with sign conventions locked
5. forward-arc extraction built on wrapped signed-angle comparisons
6. search-window tolerance wide enough to handle angular calibration and sampling uncertainty

If these six points are satisfied, coordinate conversion becomes a reliable geometric primitive
for both following control and obstacle safety logic.

---

## 10. Summary

Coordinate system conversion in this project is the disciplined transformation:

$$
(r, \theta) \longrightarrow (x, y)
$$

under clearly defined axis and angle conventions.

The mathematics is straightforward; the engineering challenge is consistency. Once conventions,
wrapping, and calibration are handled rigorously, this step provides the geometric foundation for
Phase 3 target ranging and later obstacle-avoidance behaviour.
