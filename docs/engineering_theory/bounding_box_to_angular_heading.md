# Bounding Box to Angular Heading - Reference

Covers the theory and mathematics of converting a detection bounding box from the camera image
into a horizontal angular heading error used by the Phase 3 follower pipeline.
Focus: converting pixel displacement to angle using camera intrinsics, handling fisheye caveats,
and producing a control-friendly heading error for pan-tilt and rover loops.

---

## 1. Why This Transformation Exists

### The Representation Gap

The detector output is in image coordinates: a bounding box with pixel location and size.
Control logic, however, needs geometric quantities in angular units (degrees or radians).

So the core problem is:

- perception gives: pixel offset from image centre
- controller needs: heading error relative to camera forward axis

The spatial transformation bridges these two spaces.

### What This Enables in Phase 3

Once heading error is available, the pipeline can:

- drive the pan servo to re-centre the target (inner loop)
- compute LiDAR search bearing together with calibrated offsets
- generate rover turn commands from physically meaningful angular error

For the LiDAR bearing composition step, see:
[Angular Offset Calibration](./angular_offset_calibration.md).

---

## 2. Prerequisites and Linked Theory

This note intentionally avoids re-deriving concepts already documented elsewhere.
Use these references for the full background:

- Camera intrinsics, pinhole projection, and fisheye model:
  [Intrinsic Camera Calibration](./intrinsic_calibration.md)
- Angle conventions, wrapping, and coordinate-sign hygiene:
  [Coordinate System Conversion](./coordinate_system_conversion.md)
- LiDAR-pan offset and runtime bearing composition:
  [Angular Offset Calibration](./angular_offset_calibration.md)
- Why heading error is fed back in closed loop:
  [Closed vs Open Loop Control](./closed_vs_open_loop_control.md)

---

## 3. Problem Setup and Notation

Assume a detected target bounding box with horizontal centre pixel coordinate $u$.

Define:

- $u$: bounding-box centroid x-coordinate in pixels
- $c_x$: principal point x-coordinate from intrinsic matrix $K$
- $f_x$: focal length in pixels from $K$
- $\Delta u = u - c_x$: horizontal pixel offset from optical axis
- $\delta_{\text{heading}}$: horizontal heading error (camera frame)

The intrinsic matrix term usage comes directly from calibration output in
`sensor_config.yaml` via the model in [Intrinsic Camera Calibration](./intrinsic_calibration.md).

---

## 4. Derivation: Pixel Offset to Heading Error

### Pinhole Relation Along the Horizontal Axis

From the pinhole model:

$$
\frac{X}{Z} = \frac{u - c_x}{f_x} = \frac{\Delta u}{f_x}
$$

where $X$ is lateral displacement and $Z$ is forward depth in the camera frame. I.e. X and Z are measurements
of distance from the camera body. The lowercase letters are measured from the sensor.

The horizontal viewing angle relative to optical axis is therefore:

$$
\delta_{\text{heading}} = \arctan\!\left(\frac{\Delta u}{f_x}\right)
$$

This is the core spatial transformation used in Phase 3.

### Small-Angle Approximation (Useful Intuition)

The argument to $\arctan$ is the ratio $\Delta u / f_x$. When the pixel offset is much smaller
than the focal length:

$$
|\Delta u| \ll f_x \quad \Rightarrow \quad \left|\frac{\Delta u}{f_x}\right| \ll 1
$$

This means the ratio is a small number close to zero. The Taylor series expansion of $\arctan$
around zero is:

$$
\arctan(x) = x - \frac{x^3}{3} + \frac{x^5}{5} - \cdots
$$

When $|x| \ll 1$, all higher-order terms ($x^3$, $x^5$, ...) become negligible compared to
the first term. So the full series collapses to just its first term:

$$
\arctan(x) \approx x \quad \text{when } |x| \ll 1
$$

Substituting $x = \Delta u / f_x$:

$$
\delta_{\text{heading}} = \arctan\!\left(\frac{\Delta u}{f_x}\right) \approx \frac{\Delta u}{f_x}
$$

In geometric terms: when the target is close to image centre, the angle is small, and for small
angles the ratio of sides of a right triangle is already a good approximation of the angle in
radians. The full $\arctan$ only becomes noticeably different when the target is far off-centre.

This approximation is useful for reasoning, but runtime should use the full $\arctan$ form.

---

## 5. Tilt Correction Used in This Project

Your Phase 3 plan includes a tilt-dependent correction term:

$$
\delta_{\text{heading, corrected}}
= \arctan\!\left(\frac{\Delta u}{f_x}\right)\cos\psi_{\text{tilt}}
$$

where $\psi_{\text{tilt}}$ is camera tilt angle from horizontal.

Interpretation:

- as tilt increases, horizontal image displacement corresponds to a reduced horizontal world
  heading component
- multiplying by $\cos\psi_{\text{tilt}}$ projects the optical-frame error back onto the
  horizontal plane used for rover steering and LiDAR bearing search

This is a practical planar correction used by the project architecture.

---

## 6. Fisheye Lens Caveat and Undistortion

The Waveshare camera uses a wide-angle/fisheye lens. Raw pixels near image edges are distorted,
so geometric angle mapping from raw $u$ is biased unless distortion is handled.

Two valid approaches:

1. Undistort image points first (preferred)
2. Restrict operation near image centre where distortion is weaker

For full fisheye model details and why this matters, see
[Intrinsic Camera Calibration](./intrinsic_calibration.md).

---

## 7. Sign Conventions and Frame Hygiene

The formula gives heading error in the camera optical frame. To avoid mirrored steering bugs,
keep conventions explicit:

- define whether positive $\delta_{\text{heading}}$ means target right or left in image
- keep one angle unit internally (radians preferred), convert once at interfaces
- wrap/compress angles only when needed at subsystem boundaries

For canonical angle wrapping patterns, see
[Coordinate System Conversion](./coordinate_system_conversion.md).

---

## 8. Runtime Algorithm (Phase 3 View)

Per frame, the transformation step is:

1. Read detected bounding box for the target class.
2. Compute centroid $u$.
3. Compute $\Delta u = u - c_x$.
4. Compute $\delta_{\text{heading}} = \arctan(\Delta u / f_x)$.
5. Apply tilt correction if tilt is non-zero.
6. Pass heading error to:
   - pan inner loop (recentre target), and
   - LiDAR bearing composition path (with calibrated offsets).

This keeps perception output directly usable by control.

---

## 9. Worked Example

Suppose:

- image width: 640 px
- principal point: $c_x = 320$ px
- focal length: $f_x = 300$ px
- detected centroid: $u = 380$ px

Then:

$$
\Delta u = 380 - 320 = 60
$$

$$
\delta_{\text{heading}} = \arctan\!\left(\frac{60}{300}\right)
= \arctan(0.2) \approx 0.197\ \text{rad} \approx 11.3^\circ
$$

If camera tilt is $\psi_{\text{tilt}} = 20^\circ$:

$$
\delta_{\text{heading, corrected}} \approx 11.3^\circ \cdot \cos(20^\circ)
\approx 10.6^\circ
$$

So the target is right of centre by roughly 11 degrees (or 10.6 degrees with tilt projection).

---

## 10. Common Failure Modes (And Why)

- Wrong $f_x$ or $c_x$ loaded: heading scale or zero-point is biased.
- Distortion ignored at image edges: heading error drifts with radial position.
- Sign convention mismatch: rover steers away from target.
- Mixed degrees/radians: unstable or sluggish control response.
- Inconsistent frame definitions between camera, pan, and LiDAR paths: systematic bearing bias.

Most failures are convention/configuration problems, not algebra errors.

---

## 11. Summary

Bounding-box to angular-heading conversion is the key perception-to-control transform in Phase 3.
It converts image displacement into a physically meaningful steering signal via:

$$
\delta_{\text{heading}} = \arctan\!\left(\frac{u-c_x}{f_x}\right)
$$

With proper intrinsics, distortion handling, and sign discipline, this gives a robust heading
error suitable for pan re-centring, LiDAR association, and rover turn control.
