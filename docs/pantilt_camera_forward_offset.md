# Pan-Axis Camera Forward Offset — Reference

Covers the theory and mathematics of how a camera mounted forward of the pan-axis rotation
centre affects camera-based pan servo calibration for the Waveshare pan-tilt module used in
Phase 2 of the project.
Focus: what problem is being introduced, why the offset matters, the geometry of the effect,
the resulting calibration error, and how to account for it in practice.

---

## 1. The Problem

### When the Camera Does Not Sit on the Pan Axis

The pan-axis servo curve calibration document treats the camera as an angle sensor. A stationary
target is placed in front of the pan-tilt, the pan servo is commanded through a range of values,
and the horizontal image motion of the target is converted into an angle.

That approach is exact only if the optical centre of the camera lies on the physical axis of
rotation of the pan mechanism. In that ideal case, when the servo rotates by an angle
$\phi$, the camera undergoes a pure rotation about its own optical centre.

In the real Waveshare pan-tilt assembly, the optical centre is usually displaced from the pan
axis. In this project the camera sits approximately 6.67 cm forward of the pan servo origin.
This means that when the pan axis rotates, the camera does not merely rotate. It also translates
sideways along an arc.

So the calibration problem changes from:

$$
\text{pure rotation} \quad \Longrightarrow \quad \text{image bearing} = \text{pan angle}
$$

to:

$$
\text{rotation + translation} \quad \Longrightarrow \quad \text{image bearing} \neq \text{pan angle}
$$

The distinction matters because the calibration is trying to identify the physical relationship
between commanded servo values and the actual angle of the pan mechanism. If the angle estimate is
biased by the camera's offset from the pan axis, then the fitted servo curve partly describes the
servo and partly describes the measurement geometry.

### Why This Matters

If the forward offset is ignored, the camera-based calibration may report an angle that differs
from the true mechanical pan angle.

This causes several practical problems:

- the fitted servo curve is biased by a geometric effect rather than only servo behaviour
- the bias depends on target distance, so the calibration is no longer universal
- a calibration performed at one distance may not match runtime behaviour at another distance
- the error grows as targets move closer to the rover
- dead-band and nonlinearity near zero can be misinterpreted if the geometry is not modelled

The core issue is that the camera is no longer measuring a pure rotation. It is measuring the
bearing to a fixed target from a moving viewpoint.

---

## 2. Physical Picture

### Top-Down Geometry

The effect is easiest to understand in a top-down view of the pan mechanism.

- let $O$ be the pan-axis origin, so 0,0 as coordinates
- let $C$ be the optical centre of the camera
- let $d$ be the fixed forward offset from $O$ to $C$
- let $T$ be a stationary target used during calibration
- let $\phi$ be the true pan rotation angle of the servo
- let $\alpha$ be the target bearing measured by the camera from the image

At zero pan, the target is placed directly in front of the pan axis and centred in the image.
The camera optical centre lies a distance $d$ forward of the pan origin.

```
Top-down view at zero pan:

          T = stationary target
          |
          |  L
          |
          C = camera optical centre
          |
          |  d
          |
          O = pan-axis origin
```

After a pan rotation by $\phi$, the camera moves on a circular arc of radius $d$ around the pan
origin. The optical axis also rotates by $\phi$. The line of sight from the camera to the target
changes because both the viewing direction and the camera position have changed.

This is the reason the image-derived bearing is not exactly equal to the pan angle.

### Pure Rotation Versus Offset Rotation

If the camera were located exactly at the pan origin, then a rotation by $\phi$ would simply turn
the optical axis by $\phi$ and the target's observed bearing would satisfy:

$$
\alpha = -\phi
$$

If the camera is offset forward by $d > 0$, then the measured bearing becomes a function of both
the rotation angle and the target distance.

In other words, the measured angle is no longer a property of the servo alone. It is a property of
the servo-plus-measurement-geometry.

---

## 3. Coordinate Model

### Defining the Horizontal Plane

Work in a 2D top-down plane.

- take the pan origin $O$ as the coordinate origin
- let the positive forward direction be the world $Z$ axis
- let the positive rightward direction be the world $X$ axis
- let the target at zero pan lie at

$$
T = \begin{bmatrix} 0 \\ L \end{bmatrix}
$$

where $L$ is the target distance measured from the pan origin along the zero-pan forward axis

- let the camera optical centre be offset forward from the pan origin by

$$
C_0 = \begin{bmatrix} 0 \\ d \end{bmatrix}
$$

at zero pan

- after a pan rotation $\phi$, the camera centre (where the centre of the camera moves to) becomes

$$
C(\phi) = \begin{bmatrix} d\sin\phi \\ d\cos\phi \end{bmatrix}
$$

The camera optical axis also rotates by $\phi$.

### Relative Target Position in the Camera Frame

The target position relative to the camera centre is:

$$
T - C(\phi) = \begin{bmatrix} -d\sin\phi \\ L - d\cos\phi \end{bmatrix}
$$

Expressing this vector in the rotated camera frame gives the horizontal coordinates of the target
relative to the camera optical axis:

$$
x_C = -L\sin\phi
$$

$$
z_C = L\cos\phi - d
$$

The image-derived horizontal bearing is therefore:

$$
\alpha = \arctan\!\left(\frac{x_C}{z_C}\right)
= -\arctan\!\left(\frac{L\sin\phi}{L\cos\phi - d}\right)
$$

This is the exact horizontal relation for a target initially centred at zero pan.

---

## 4. The Forward Relation

### From True Pan Angle to Measured Image Bearing

The true servo rotation is the mechanical angle $\phi$. The camera does not observe $\phi$
directly. It observes the target bearing $\alpha$ in the image.

With forward camera offset $d$, the relationship is:

$$
\boxed{\alpha = -\arctan\!\left(\frac{L\sin\phi}{L\cos\phi - d}\right)}
$$

This equation makes the key dependence explicit:

- if $d = 0$, then $\alpha = -\phi$
- if $d > 0$, then $\alpha$ depends on both $\phi$ and $L$

This means the same servo angle can produce different measured image bearings at different target
distances.

### Consequence for Calibration

The original servo curve calibration note uses the approximation:

$$
\phi_{\text{actual}} \approx -\arctan\!\left(\frac{u - c_x}{f_x}\right)
$$

That approximation is correct only when the camera is effectively rotating about its own optical
centre. With forward offset, the image quantity

$$
\alpha = \arctan\!\left(\frac{u - c_x}{f_x}\right)
$$

is still the observed viewing angle, but it is no longer equal to $-\phi$.

So if the offset is ignored, the calibration data set becomes:

$$
\mathcal{D}_{\text{biased}} = \{(c_i, -\alpha_i)\}
$$

instead of the true set:

$$
\mathcal{D}_{\text{true}} = \{(c_i, \phi_i)\}
$$

---

## 5. The Inverse Relation

### Recovering True Pan Angle From a Measured Bearing

For calibration, the practical task is usually the inverse problem:

- command a servo value $c_i$
- measure image bearing $\alpha_i$
- infer the true mechanical angle $\phi_i$

Starting from

$$
\alpha = -\arctan\!\left(\frac{L\sin\phi}{L\cos\phi - d}\right)
$$

the inverse relation is:

$$
\boxed{\phi = -\alpha + \arcsin\!\left(\frac{d}{L}\sin\alpha\right)}
$$

This equation reduces correctly to

$$
\phi = -\alpha
$$

when $d = 0$.

### Full Measurement Chain

The camera still measures pixel coordinates in the image. So the full calibration chain becomes:

1. Measure the target centroid horizontal coordinate $u$
2. Convert pixel position to image bearing using the intrinsic matrix:

$$
\alpha = \arctan\!\left(\frac{u - c_x}{f_x}\right)
$$

3. Correct for the camera's forward offset:

$$
\phi = -\alpha + \arcsin\!\left(\frac{d}{L}\sin\alpha\right)
$$

4. Use the corrected $\phi$ values to fit the servo curve

$$
\phi = f(c)
$$

This is the correct calibration workflow when the camera is measurably displaced from the pan
axis and the calibration target distance $L$ is known.

### What Happens if $L$ is Unknown

If the target distance is not known, then the inverse problem is underdetermined.

The measured image bearing $\alpha$ alone is not enough to recover the true pan angle because
many different combinations of $\phi$ and $L$ can produce the same observed bearing.

So if $L$ is unknown, one of the following must be true:

- the target is far enough away that the offset effect is negligible
- the distance must be measured during calibration
- some independent angle reference must be used instead of relying on image geometry alone

---

## 6. Approximation for Small Offsets

### First-Order Error Formula

When the offset ratio $d/L$ is small, the correction term can be simplified using
$\arcsin(x) \approx x$ for small $x$:

$$
\phi \approx -\alpha + \frac{d}{L}\sin\alpha
$$

Equivalently, the error made by the naive estimate $\phi_{\text{naive}} = -\alpha$ is:

$$
\phi_{\text{naive}} - \phi \approx -\frac{d}{L}\sin\alpha
$$

or, approximately in terms of the true servo angle,

$$
(-\alpha) - \phi \approx \frac{d}{L}\sin\phi
$$

This shows the main trends clearly:

- the error grows linearly with the forward offset $d$
- the error shrinks as the target distance $L$ increases
- the error is near zero at $\phi = 0$
- the error grows with pan angle magnitude

### Interpretation

For a camera mounted forward of the pan origin, the naive image-only method generally reports a
larger angle magnitude than the true pan rotation.

That is, for moderate pan angles,

$$
|\alpha| > |\phi|
$$

so simply taking $-\alpha$ tends to overestimate how far the servo has actually rotated.

---

## 7. Numerical Example

### Example Using the Project Geometry

Take:

- forward camera offset $d = 0.0667$ m
- calibration target distance $L = 1.0$ m
- true pan angle $\phi = 30^\circ$

Then the image bearing predicted by the exact model is:

$$
\alpha = -\arctan\!\left(\frac{1.0\sin 30^\circ}{1.0\cos 30^\circ - 0.0667}\right)
$$

$$
\alpha \approx -32.0^\circ
$$

So a naive calibration would infer

$$
\phi_{\text{naive}} = -\alpha \approx 32.0^\circ
$$

even though the true mechanical angle is only

$$
\phi = 30.0^\circ
$$

The error is therefore about $2^\circ$ at 1 m distance.

### Distance Dependence

For the same 6.67 cm forward offset, the approximate error at $30^\circ$ pan is:

| Target distance $L$ | Approximate error |
|---------------------|-------------------|
| 3.0 m | $\approx 0.65^\circ$ |
| 2.0 m | $\approx 1.0^\circ$ |
| 1.0 m | $\approx 2.0^\circ$ |
| 0.5 m | $\approx 4.3^\circ$ |

This table shows why the offset may seem unimportant at long range but become significant at
close range.

---

## 8. Effect on the Servo Curve Calibration

### What Changes and What Does Not

The forward offset does **not** change the physical behaviour of the servo itself.

It does **not** create dead-band, backlash, or saturation. Those are mechanical properties of the
actuator and linkage.

What the offset changes is the measurement model used to estimate pan angle from the camera.

Therefore:

- the raw servo still has some true curve $\phi = f(c)$
- the camera-based estimate of $\phi$ becomes biased if the offset is ignored
- the fitted calibration curve may appear to have extra gain or nonlinearity caused by geometry

In other words, the offset corrupts the identification stage, not the servo itself.

### Distance-Dependent Calibration Bias

This is the most important practical consequence.

Because the measurement error depends on $L$, a calibration obtained at one distance is not
strictly valid at another distance if the offset is ignored.

For example:

- calibrating with a target at 3 m produces only a small bias
- calibrating with a target at 0.5 m can produce several degrees of error
- using the same servo curve at runtime across multiple distances then mixes actuator effects and
  geometric effects

This is why a close-range calibration target can unintentionally fit the geometry of the rig more
than the true servo behaviour.

### Calibration-Time Correction Versus Runtime Correction

It is important to separate two different uses of this geometry model.

1. **Calibration-time use**: correcting for forward offset during servo calibration improves the
  estimate of the true mechanical pan angle $\phi$ for each commanded servo value $c$.
  This gives a cleaner estimate of the actuator map $\phi = f(c)$.
2. **Runtime use**: when tracking a live target, the camera is still physically offset from the
  pan origin. So converting image measurements into a target angle relative to the pan origin
  (or rover frame) still requires geometry that accounts for the same forward displacement,
  especially at close range.

Therefore, applying the correction during calibration does not automatically remove runtime
bearing error for close targets. It improves actuator identification. Runtime steering and target
bearing estimation still need their own offset-aware conversion from camera observation to pan-axis
or rover-frame angle.

---

## 9. Practical Calibration Strategies

### Strategy 1 — Use a Distant Target

The simplest practical approach is to place the calibration target far enough away that
$d/L \ll 1$.

This makes the correction term small and restores the approximation:

$$
\phi \approx -\alpha
$$

This is usually the right first approach when:

- the calibration space allows a long target distance
- sub-degree accuracy is not required
- the goal is a robust first servo map rather than a metrology-grade measurement

### Strategy 2 — Measure Target Distance and Correct the Model

If the target distance $L$ is known during calibration, then the exact inverse relation can be
used directly:

$$
\phi = -\alpha + \arcsin\!\left(\frac{d}{L}\sin\alpha\right)
$$

This produces a calibration that better reflects the true pan-axis rotation rather than the image
bearing observed from an offset camera.

### Strategy 3 — Use an Independent Angle Reference

Another option is to avoid the ambiguity entirely by measuring pan angle with something other than
the camera image. For example:

- a mechanical protractor or angle scale attached to the pan mechanism
- a laser line projected onto a distant wall with known geometry
- a fiducial target with externally measured pose
- an encoder or high-confidence servo feedback if such hardware is available

This approach separates actuator calibration from camera geometry.

---

## 10. Assumptions and Limitations

This note assumes the following:

- the analysis is restricted to the horizontal plane
- the target is stationary during calibration
- the target is initially centred when the pan mechanism is at zero
- the forward offset $d$ is known and fixed
- the target lies on the zero-pan forward ray of the pan origin at the start of the experiment

This note does not model:

- vertical offset or tilt-axis coupling
- camera pitch and roll misalignment relative to the pan axis
- dynamic effects while the servo is still moving
- uncertainty in target distance $L$

If those effects are large, then a more general extrinsic geometric model is required.

---

## 11. Summary

When the camera optical centre sits forward of the pan-axis origin, the camera does not undergo a
pure rotation during pan motion. It undergoes a rotation about the servo origin and a translation
along a circular arc.

As a result, the image-derived target bearing is not equal to the true mechanical pan angle.
Instead, for a target at distance $L$ and a forward camera offset $d$, the exact relation is:

$$
\alpha = -\arctan\!\left(\frac{L\sin\phi}{L\cos\phi - d}\right)
$$

and the corresponding inverse used for calibration is:

$$
\phi = -\alpha + \arcsin\!\left(\frac{d}{L}\sin\alpha\right)
$$

The effect becomes small when the target is far away, but can introduce several degrees of bias at
close range. Therefore, if accurate camera-based pan calibration is required, the forward offset
must either be made negligible by using a distant target, or explicitly included in the
measurement model.

Accounting for forward offset during calibration improves the estimate of true servo pan angle for
each command, but this is only part of the full problem. During runtime tracking, if the system
needs the target angle relative to the pan origin (for steering or LiDAR search), the same
forward-displacement geometry still needs to be handled in the runtime angle-conversion step.