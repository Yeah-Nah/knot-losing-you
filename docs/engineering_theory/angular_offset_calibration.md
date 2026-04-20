# Angular Offset Calibration — LiDAR ↔ Pan-Tilt

Covers the theory and mathematics of the single-scalar angular offset calibration between
the LDRobot D500 LiDAR and the Waveshare pan-tilt camera required in Phase 2 of the project.

---

## 1. The Problem

### Two Forward Axes, One Chassis

The LiDAR and the pan-tilt camera are both physically mounted to the same UGV chassis.
In an ideal world they would both have the same "forward" direction — the LiDAR's 0° bearing
and the pan-tilt's optical axis at zero command would point in exactly the same direction.

In practice this is never the case. Manufacturing tolerances in the chassis, the bracket that
holds the pan-tilt, and the LiDAR mount all introduce small angular differences. When you
command `set_pan_tilt(0, 0)`, the camera looks in some direction. When the LiDAR reports a
return at 0°, it is measuring in a slightly different direction. Neither sensor knows about
the other — they each report angles and bearing in their own private reference frame.

**Angular offset calibration** measures the single horizontal angle $\delta_\text{offset}$
between those two reference frames. Once known, any bearing computed in the pan-tilt optical
frame can be converted to a LiDAR bearing with a simple addition, and vice versa.

### How This Differs From the Full Extrinsic Calibration

The [extrinsic calibration](extrinsic_calibration.md) document describes the general-case
approach: measuring the full 6-DOF rigid-body transform $T_C^L$ (a $4 \times 4$ matrix
encoding 3D rotation and 3D translation) between the LiDAR and the camera. That approach
projects LiDAR returns onto image pixels and is needed for sensor fusion validation in
Phase 4.

The angular offset calibration is a **restricted special case** of the same underlying
geometry. It exploits two facts about this specific application:

1. The LiDAR scans a single horizontal plane. Every return has the same height ($z_L = 0$).
2. The follower application only needs to know *which horizontal angle* to search in the
   LiDAR scan for a target that the camera is already tracking. No 3D projection into pixel
   coordinates is required at runtime.

These two facts reduce the 6-DOF extrinsic problem to a single scalar: the horizontal
(yaw) angular difference between the two sensors' forward axes. The full $T_C^L$ machinery
is not needed because we never need to project a 3D LiDAR point onto the image — we only
need to convert a *bearing* from one angular frame to another.

---

## 2. Coordinate Frames

### LiDAR Frame (L) — ⚠ Left-Handed

The LDRobot D500 uses a **left-handed** coordinate frame with angles increasing
**clockwise** when viewed from above.

- **Origin**: centre of the D500's rotating mirror
- **X**: forward — the direction corresponding to 0° in the scan data
- **Y**: right — positive Y is to the right of the forward axis (clockwise = positive angle)
- **Z**: straight up, perpendicular to the scan plane
- **Angle convention**: angles reported by the D500 run from 0° to 360°, increasing clockwise.
  $\theta = 0°$ is straight ahead (+X), $\theta = 90°$ is to the right (+Y), $\theta = 270°$
  (equivalently $-90°$) is to the left.

The "forward arc" centred on 0° is the region directly in front of the rover.

### Pan-Tilt Optical Frame (P)

The pan-tilt optical frame is defined by the camera lens:

- **Origin**: the optical centre of the Waveshare RGB lens
- **Zero bearing**: the direction the optical axis points when `set_pan_tilt(0, 0)` is
  commanded and the servo has settled — this is the **pan-tilt mechanical zero**
- **Positive pan direction**: looking at the camera from behind, commanding a positive pan
  angle turns the camera to the right (consistent with the rover's right)

### The Offset

The angle $\delta_\text{offset}$ is defined as the pan-tilt mechanical zero direction
expressed in the rover's reference frame (where rover forward = 0°):

$$\delta_\text{offset} = \theta_\text{rover}\big|_{\phi_\text{pan}=0,\;\delta_\text{heading}=0}$$

In other words: if the camera is at zero pan and a target is perfectly centred in the image,
what angle does the rover's compass read for that target? That is $\delta_\text{offset}$
(after subtracting the LiDAR mounting offset).

If the sensors were perfectly aligned, $\delta_\text{offset} = 0°$ (relative to rover forward).
In practice it will be a small non-zero value (typically a few degrees) caused by mounting
imprecision.

```
Top-down view of rover chassis (both angles in rover frame):

         Rover forward = 0°
              ↑
              |
              |  δ_offset (pan-tilt zero in rover frame)
              |↗
              |— LiDAR 0° in rover frame (= mounting_offset_deg)
              |
         Pan-tilt 0° (optical axis at zero command)

  δ_offset is the angle from rover forward to the pan-tilt zero.
```

---

## 3. Why a Single Scalar is Sufficient

### The 6-DOF Problem Reduced

A general rigid-body transform between two sensors in 3D space has **6 degrees of freedom**:
3 rotational (roll, pitch, yaw) and 3 translational (forward/back, left/right, up/down).
The full extrinsic calibration solves for all 6. This calibration solves for only 1.

The reduction is valid because of three assumptions that hold for this application:

**Assumption 1 — Planarity.** The D500 scans a single horizontal plane. All LiDAR returns
have $z_L = 0$. Roll and pitch misalignments between the LiDAR and the pan-tilt would rotate
returns out of the horizontal plane — but since there are no returns above or below the plane
to begin with, roll and pitch have no observable effect on the scan data. For the purpose of
bearing computation, they are irrelevant.

**Assumption 2 — Bearing, not position.** The Phase 3 formula needs to know *which bearing*
in the LiDAR scan corresponds to the camera's line of sight. It does not need to know the
3D position of the LiDAR origin relative to the camera origin. The translational components
of $T_C^L$ (how far apart the two sensors are physically) shift the exact range to a target
but do not shift its bearing at any non-trivial distance. At 1 m range, a 10 cm lateral
sensor offset introduces only a ~5.7° bearing error — and at 2 m it halves to ~2.9°.

> **Note:** For very close targets (< 0.5 m), the translation offset becomes significant.
> This calibration approach is designed for the typical following distance of 1–3 m.
> If sub-metre following is required, the full $T_C^L$ from
> [extrinsic_calibration.md](extrinsic_calibration.md) should be used instead.

**Assumption 3 — Horizontal scan height match.** The LiDAR scan plane should pass through
the scene at approximately the same height as the centroid of the target in the camera's field
of view. This is a mounting concern rather than a calibration concern — if the LiDAR is
mounted very high or very low relative to the pan-tilt, the scan plane may not intercept a
standing person's torso at any practical following distance.

Under these three assumptions, the only geometrically meaningful parameter left is the
horizontal (yaw) angular difference between the two sensors' forward axes — a single scalar.

### When These Assumptions Break Down

| Situation | Consequence | Resolution |
|-----------|-------------|------------|
| Pan-tilt has a significant mechanical roll offset (bracket bent) | The optical axis sweeps an arc out of the horizontal plane as pan angle increases | Full $T_C^L$ calibration |
| Following distance < 0.5 m | Sensor translation offset causes significant bearing error | Full $T_C^L$ calibration |
| LiDAR scan plane misses the target entirely | No return to match; calibration fails | Adjust LiDAR mounting height |

---

## 4. The Phase 3 Search Formula

### Where $\delta_\text{offset}$ Lives at Runtime

In Phase 3, the pipeline needs to find the distance to the tracked target using the LiDAR.
The target is visible in the camera image at some pixel position, and the pan-tilt has been
commanded to some angle $\phi_\text{pan}$ to keep it roughly centred. The search bearing
is computed in two steps:

1. **Compute target bearing in rover frame:**
   $$\theta_\text{rover} = \delta_\text{offset} + \phi_\text{pan} + \delta_\text{heading}$$
   where all three terms are in rover frame (rover forward = 0°).

2. **Convert to LiDAR frame:**
   $$\boxed{\theta_\text{search} = (\theta_\text{rover} - \text{lidar.mounting\_offset\_deg}) \bmod 360°}$$
   This final value is the LiDAR bearing (in the LiDAR's native angle convention) at which
   to search for the target.

Each term corrects for a different source of angular displacement:

| Term | Meaning | Units | Source |
|------|---------|-------|--------|
| $\delta_\text{offset}$ | Fixed mounting offset in rover frame — pan-tilt mechanical zero relative to rover forward | degrees | This calibration (stored in rover frame) |
| $\phi_\text{pan}$ | Current pan servo angle — how far the camera has been turned from its zero | degrees | Pan-tilt servo curve calibration |
| $\delta_\text{heading}$ | Residual heading error — how far the target's pixel centroid is from the image centre, expressed as an angle | degrees | Computed from bounding box + intrinsic matrix $K$ |
| $\text{lidar.mounting\_offset\_deg}$ | Direction the LiDAR's 0° axis points in rover frame | degrees | sensor_config.yaml |

### Propagation of $\delta_\text{offset}$ Error

If $\delta_\text{offset}$ is measured incorrectly by $\varepsilon$ degrees, then
$\theta_\text{search,lidar}$ is off by $\varepsilon$ degrees at every single frame, regardless
of pan angle or target position. This is a **systematic bias** — it does not average out
over time. Its effect on range accuracy depends on target distance $r$:

$$\text{positional error} = r \cdot \tan\varepsilon \approx r \cdot \varepsilon \quad (\varepsilon \text{ in radians})$$

At 2 m range, a 3° error in $\delta_\text{offset}$ ($\varepsilon \approx 0.052$ rad)
causes a ~10 cm positional error in the LiDAR search. For a 1° error, the error is ~3.5 cm.
Whether this matters depends on the width of the LiDAR search window used in Phase 3 —
the search window must be wider than the calibration error or the target will be missed.

---

## 5. Deriving $\delta_\text{offset}$ at Calibration Time

### Setting Up Zero Conditions

The calibration procedure creates conditions where $\phi_\text{pan} = 0$ and
$\delta_\text{heading} = 0$. It then measures the LiDAR bearing of the target, converts
that bearing to the rover frame by applying the `lidar.mounting_offset_deg`, and stores
the result:

$$\delta_\text{offset} = (\theta_\text{lidar} + \text{lidar.mounting\_offset\_deg}) \bmod 360°$$

where $\theta_\text{lidar}$ is the median of the target cluster in the LiDAR's native
frame.

When both pan and heading conditions hold, this direct calculation yields $\delta_\text{offset}$.
No minimisation, no fitting, no iterative solver is required. The calibration is a single
direct measurement under controlled conditions.

### Ensuring $\phi_\text{pan} = 0$

This is straightforward: command `set_pan_tilt(0, 0)` and wait for the servo to settle
(typically 0.5–1 second). The servo is at its firmware-defined zero position. All
subsequent rover drive calibration steps use this same zero as their reference.

### Ensuring $\delta_\text{heading} = 0$

This is the physically demanding part: you must position the calibration target so that
it appears centred on the camera's horizontal optical axis. The precise condition is:

$$u_\text{target} = c_x$$

where $u_\text{target}$ is the horizontal pixel coordinate of the target's centroid in the
image and $c_x$ is the principal point (image centre column) from intrinsic calibration
($c_x \approx 672$ px for the Waveshare RGB at 1280×720). How to achieve this in practice
is covered in Section 7.

---

## 6. The Heading Error Term $\delta_\text{heading}$

This section derives the heading error formula used at runtime and used (zeroed out) during
calibration. Understanding it clarifies why centring the target on $u = c_x$ is the correct
zero condition.

### From Pixel Offset to Angle

The intrinsic matrix $K$ from Phase 2 maps a 3D camera-frame point
$(x_C, y_C, z_C)^\top$ to an image pixel $(u, v)$:

$$u = f_x \cdot \frac{x_C}{z_C} + c_x, \qquad v = f_y \cdot \frac{y_C}{z_C} + c_y$$

Rearranging for the horizontal component and letting $\Delta u = u - c_x$ be the pixel
offset from the image centre:

$$\frac{x_C}{z_C} = \frac{u - c_x}{f_x} = \frac{\Delta u}{f_x}$$

The ratio $x_C / z_C$ is the tangent of the horizontal angle between the optical axis and
the ray toward the target:

$$\delta_\text{heading,raw} = \arctan\!\left(\frac{\Delta u}{f_x}\right)$$

### Tilt Correction

When the pan-tilt has a non-zero tilt angle $\psi_\text{tilt}$, the image plane is no longer
perpendicular to the horizontal. A horizontal pixel offset $\Delta u$ then corresponds to a
slightly smaller true horizontal bearing change because part of the apparent lateral motion
is in the vertical direction. The correction multiplies by $\cos\psi_\text{tilt}$:

$$\delta_\text{heading} = \arctan\!\left(\frac{\Delta u}{f_x}\right) \cdot \cos\psi_\text{tilt}$$

At $\psi_\text{tilt} = 0$ (tilt at centre), $\cos 0 = 1$ and the correction vanishes.
For small tilt angles (< 20°), the correction is less than 6% of $\delta_\text{heading}$ and
may be omitted without significant error. For large downward tilts on steep terrain,
the correction becomes important.

### Why $\delta_\text{heading} = 0$ at Calibration

When the target is centred horizontally ($u_\text{target} = c_x$, so $\Delta u = 0$):

$$\delta_\text{heading} = \arctan\!\left(\frac{0}{f_x}\right) \cdot \cos\psi_\text{tilt} = \arctan(0) \cdot \cos\psi_\text{tilt} = 0°$$

regardless of tilt angle. This is why centring on $c_x$ (not on the pixel-count midpoint
$\text{width}/2$) is the correct condition. The principal point $c_x$ is the pixel through
which the true optical axis passes — the lens distortion model is defined around this point.
Centring on the raw pixel-count midpoint instead would introduce a bias equal to the offset
between $c_x$ and the image half-width (for the Waveshare RGB: $c_x \approx 672$ vs
$1280/2 = 640$, a 32 px difference, which at $f_x = 542$ corresponds to a ~3.4° error in
$\delta_\text{offset}$).

---

## 7. Placing the Target at Zero Bearing

### The Camera-Guided Method

"Pan-tilt zero bearing" is defined as the direction the camera looks when
`set_pan_tilt(0, 0)` is commanded. A target is at that bearing if and only if it appears
at $u = c_x$ in the camera image. The procedure uses the camera itself as the measurement
instrument, making it self-referencing — no external ruler, string line, or laser level is
needed.

**Procedure:**

1. Boot the rover and command `set_pan_tilt(0, 0)`. Wait 1–2 seconds for the servo to settle.
2. Open a live camera feed on the Pi (e.g. via an SSH X11 session or a small OpenCV
   display script).
3. Draw a vertical guide line at $x = c_x = 672$ pixels on the display overlay.
4. Place the calibration target (a flat-faced box or a sheet of cardboard) roughly in
   front of the rover at 1.5–2 m distance.
5. Physically slide the target left or right until its face is bisected by the guide line.
6. The target is now at the pan-tilt zero bearing. Proceed to the LiDAR scan step.

### Angular Error vs Distance

A small lateral positioning error $\varepsilon_\text{lat}$ at distance $d$ produces an
angular error in $\delta_\text{offset}$:

$$\varepsilon_\text{angular} = \arctan\!\left(\frac{\varepsilon_\text{lat}}{d}\right) \approx \frac{\varepsilon_\text{lat}}{d} \quad \text{(small angle, radians)}$$

| Distance | 1 cm lateral error | 5 cm lateral error |
|----------|-------------------|-------------------|
| 0.5 m | 1.15° | 5.7° |
| 1.0 m | 0.57° | 2.9° |
| 2.0 m | 0.29° | 1.4° |

**Place the target as far away as the space allows.** At 2 m, even a sloppy 5 cm
positioning error produces only 1.4° of angular offset error — well within the D500's
~1° angular resolution. At 0.5 m, the same error is 5.7°, which would make the sensor
fusion unreliable.

### Target Properties

The target should:

- Present a **flat, perpendicular face** to the LiDAR — angled surfaces move the apparent
  range and bearing of the returned centroid
- Be **tall enough** to intersect the LiDAR scan plane — the D500 scans at its mounting
  height; a flat piece of paper lying on the floor will not be visible
- Be **narrow enough** to produce a compact LiDAR cluster — a wide wall gives a diffuse
  cluster and makes the angular centroid harder to estimate; a box 15–30 cm wide at 2 m
  subtends ~4–9° and produces a clean, identifiable cluster

A cardboard box (roughly 20 cm wide) stood upright at 2 m is ideal.

---

## 8. Identifying the LiDAR Return

### Scan Accumulation

The D500 streams 12-point packets continuously. A single packet covers only ~30° of arc.
For a reliable measurement you should accumulate enough packets to assemble at least one
complete 360° scan, though in practice 3–5 full rotations' worth of data (a few seconds)
is better because the scan head spins at ~10 Hz and individual returns contain noise.

Accumulate all returns in the expected bearing range (e.g. 330°–30°, the ±30° forward arc)
with a range filter that keeps only returns consistent with the target's known distance
(e.g. $d - 0.2\,\text{m}$ to $d + 0.2\,\text{m}$, where $d$ is the measured or estimated
placement distance).

### Cluster Extraction

After range filtering, the remaining returns form a cluster corresponding to the target face.
Extract the angular centroid:

$$\delta_\text{offset} = \text{median}\!\left(\{\theta_i : r_i \in [d_\text{min},\, d_\text{max}]\}\right)$$

Median is preferred over mean because a single spurious return (reflections from the floor
or objects behind the target) will not shift the median, whereas it would bias the mean.

### Handling the 0°/360° Wrap

The D500 reports angles in $[0°, 360°)$. If the target is slightly to the left of dead
ahead, the cluster may straddle the wrap boundary — some returns at e.g. 358°, 359° and
others at 0°, 1°. A naive mean of $\{358, 359, 0, 1\}$ gives $179.5°$, which is completely
wrong.

Before computing the centroid, convert angles to a signed range $(-180°, +180°]$:

$$\theta_\text{signed} = \begin{cases} \theta & \text{if } \theta \leq 180° \\ \theta - 360° & \text{if } \theta > 180° \end{cases}$$

Then compute the median on the signed values. If the result is negative (target slightly
to the left), $\delta_\text{offset}$ is a small negative number in the signed convention.
When storing in `sensor_config.yaml`, adopt a consistent convention (see Section 11).

### D500 Angular Resolution

The D500 operates at approximately 10 Hz rotation speed and streams ~4500 points per second.
At 10 Hz, one full revolution contains ~450 points, giving an average angular spacing of
$360° / 450 \approx 0.8°$ per point. This is the hard floor on the achievable precision
of $\delta_\text{offset}$ — no amount of careful target placement can exceed it.

In practice, accumulating many scans and taking the median of several hundred returns in
the cluster smooths quantisation noise and achieves an effective precision closer to 0.3°
by averaging over slightly varying packet timings and scan speeds.

---

## 9. Error Analysis

### Sources of Error

| Source | Typical magnitude | Type |
|--------|------------------|------|
| D500 angular resolution | ~0.8° per point | Quantisation (reducible by averaging) |
| Target centring error (pixel) | 1–5 px → 0.1–0.5° at $f_x = 542$ | Random |
| Target lateral placement error | 0.3–1.4° (at 2 m, 1–5 cm error) | Systematic (reducible by care) |
| Servo settling — residual vibration after `set_pan_tilt(0, 0)` | < 0.5° | Random |
| Rover not on flat surface (causes pan-tilt tilt) | 0–1° | Systematic (eliminate by flat surface) |

### Total Expected Error Budget

Adding representative values in quadrature:

$$\sigma_{\delta_\text{offset}} \approx \sqrt{0.3^2 + 0.2^2 + 0.3^2 + 0.2^2} \approx 0.5°$$

A well-executed calibration on a flat surface with the target at 2 m therefore achieves
$\delta_\text{offset}$ to within roughly ±0.5°.

### What is Acceptable

The Phase 3 search window around $\theta_\text{search}$ is typically ±5–10°. A 0.5°
calibration error is comfortably inside this window and will not cause missed matches.
A 3° error is marginal. A 5°+ error will reliably miss returns for targets at angles
near the edge of the search window.

---

## 10. Assumptions and Limitations

### Planarity

The critical assumption is that the pan-tilt forward axis at zero pan *lies in the same
horizontal plane as the LiDAR scan*. This holds when:

- The platform is level (no roll or pitch of the chassis)
- The pan-tilt bracket has no significant mechanical pitch or roll offset

If the pan-tilt bracket is bent or shimmed so that the zero-pan axis points upward or
downward by several degrees, then as the pan angle increases, the optical axis sweeps a
cone (not a horizontal plane). The LiDAR bearing of a target at constant height will then
change as a function of pan angle in a way that a single scalar offset cannot fully
describe. In this case the full $T_C^L$ approach from the
[extrinsic calibration document](extrinsic_calibration.md) is required.

**Test for this:** after calibration, command pan angles of ±20°, ±40° and verify that
$\theta_\text{search} = \delta_\text{offset} + \phi_\text{pan}$ correctly predicts the
LiDAR bearing of a stationary target at each angle. If the residuals grow with pan angle,
a conical sweep is occurring.

### Close-Range Limitation

As noted in Section 3, the translation offset between the LiDAR and pan-tilt origins
introduces a bearing error that is negligible at typical following distances (1–3 m) but
becomes significant at very close range (< 0.5 m). The system should enforce a minimum
following distance that keeps this error within acceptable bounds.

### Static Calibration

$\delta_\text{offset}$ is a property of the physical mounting. It does not change during
operation. However, it will change if:

- The pan-tilt is removed and reattached
- The LiDAR is removed and reattached
- The chassis receives a significant impact that shifts a mounting bracket

The calibration should be re-run any time the sensor hardware is disturbed.

---

## 11. `sensor_config.yaml` — Result Structure

The calibration produces a single scalar. It is stored under the `extrinsic` key alongside
the sensor intrinsics:

```yaml
extrinsic:
  # Direction the pan-tilt mechanical zero points, expressed in the rover's reference
  # frame (rover forward = 0°). Computed by converting the LiDAR cluster bearing to
  # rover frame via (lidar_bearing + lidar.mounting_offset_deg) % 360.
  # Positive value: pan-tilt zero points to the right of rover forward.
  # Negative value: pan-tilt zero points to the left of rover forward.
  # Units: degrees. Range: (-180, +180].
  lidar_to_pantilt_offset_deg: null   # set after calibration
```

### Sign Convention

The signed $(-180°, +180°]$ range is preferred over the raw $[0°, 360°)$ convention because:

- It makes the physical meaning immediately readable: positive = pan-tilt looks right of
  the LiDAR, negative = pan-tilt looks left
- The Phase 3 formula $\theta_\text{search} = \delta_\text{offset} + \phi_\text{pan} + \delta_\text{heading}$
  works correctly with signed arithmetic including modular reduction to $[0°, 360°)$ at the
  end

### Loading in Python

```python
import math

lidar_offset_deg: float = cfg.extrinsic.lidar_to_pantilt_offset_deg
lidar_mounting_offset_deg: float = cfg.lidar.mounting_offset_deg

def compute_search_bearing(
    rover_offset_deg: float,
    pan_angle_deg: float,
    delta_heading_deg: float,
    lidar_mounting_offset_deg: float,
) -> float:
    """Return LiDAR search bearing in [0, 360) degrees.

    Converts from rover frame back to LiDAR frame by adding mounting offset.
    """
    rover_bearing = rover_offset_deg + pan_angle_deg + delta_heading_deg
    lidar_bearing = (rover_bearing + lidar_mounting_offset_deg) % 360.0
    return lidar_bearing
```

---

## 12. Verification

### Sanity Check at Multiple Pan Angles

After measuring $\delta_\text{offset}$, verify it with the following procedure:

1. Keep the calibration target at the same position (it has not moved).
2. Command the pan-tilt to several different pan angles: $-30°, -15°, 0°, +15°, +30°$.
3. At each angle, record the **LiDAR bearing** of the target cluster (in the LiDAR's native frame).
4. Compute the **predicted LiDAR bearing** using:
   $$\theta_\text{pred} = (\delta_\text{offset} + \phi_\text{pan}) - \text{lidar.mounting\_offset\_deg} \pmod{360°}$$
5. Compare predicted vs observed.

A good calibration produces residuals that are:
- Small (< 1°) across all pan angles
- Flat (not growing with pan angle)

If residuals are small and flat, $\delta_\text{offset}$ is correct and the planarity
assumption holds. If residuals grow systematically with pan angle, the pan-tilt bracket has
a pitch/roll offset and the scalar model is insufficient.

### Acceptance Criterion

| Residual (max across tested pan angles) | Action |
|----------------------------------------|--------|
| < 1° | Accept — proceed to servo curve calibration |
| 1°–3° | Accept with wide search window in Phase 3 (±7°+) |
| > 3° flat | Re-run calibration — likely a centring or scan error |
| > 3° growing with pan | Pan-tilt bracket is mechanically offset — investigate mounting |

---

## 13. Phase Dependencies

### Requires

**Intrinsic calibration must be completed first.** The values $c_x$ (principal point) and
$f_x$ (focal length) from the Waveshare RGB intrinsic calibration are needed for two reasons:

1. The guide line drawn on the live display must be at $x = c_x$, not at the raw pixel
   midpoint. Using the wrong column would introduce a systematic bias of several degrees
   (see Section 6).
2. The $\delta_\text{heading}$ formula at runtime requires $f_x$ and $c_x$.

If intrinsic calibration has not been run, do not proceed.

### Enables

Once $\delta_\text{offset}$ is stored in `sensor_config.yaml`:

- **Pan-tilt servo curve calibration** (Phase 2, next step) can proceed — it uses the same
  experimental setup (target in front, camera feed visible) and builds on the zero bearing
  established here
- **Phase 3 runtime code** can implement the full search bearing formula
  $\theta_\text{search} = \delta_\text{offset} + \phi_\text{pan} + \delta_\text{heading}$

### Sequencing

```
Intrinsic calibration (K, D, cx, fx)
        ↓
Angular offset calibration (δ_offset)
        ↓
Pan-tilt servo curve calibration (command → degrees mapping)
        ↓
Rover drive calibration (turn rate, dead-band)
        ↓
Phase 3 runtime
```
