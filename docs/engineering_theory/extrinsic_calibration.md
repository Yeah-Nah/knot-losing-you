# Extrinsic Calibration — Reference

Covers the theory and mathematics of the LiDAR ↔ camera extrinsic transform required in
Phase 2 of the project.
Sensors: **LDRobot D500 LiDAR** and **OAK-D Lite colour camera**.

---

## 1. The Problem

### Two Sensors, Two Worlds

After intrinsic calibration you have a well-characterised camera — you know exactly how a 3D
point in the camera's own coordinate frame projects onto the image. But the LiDAR and the camera
are two separate physical devices bolted to the same chassis, each looking at the world from
its own origin with its own axes pointing in its own directions.

When the LiDAR reports a return at distance $r$ and bearing $\theta$, and the camera
simultaneously sees a person at pixel $(u, v)$, there is no automatic answer to the question:
*"Are these the same object?"* The two sensors speak different geometric languages, and their
origins are physically offset from one another.

**Extrinsic calibration** is the process of measuring the rigid-body spatial relationship
between the two sensors — once measured, any point reported in one sensor's coordinate frame
can be exactly expressed in the other's.

### What "Extrinsic" Means

The word distinguishes this transform from the *intrinsic* parameters, which describe the
internal geometry of a single sensor. Extrinsic parameters describe geometry that is
*external to both sensors* — specifically, how they are positioned and oriented relative to
one another in the physical world.

Crucially: once the sensors are mechanically fixed to the chassis, **the extrinsic transform
is a constant**. It never changes during operation (unless a sensor is physically moved or
knocked). You measure it once offline and store it in `sensor_config.yaml`.

---

## 2. Coordinate Frames

Before the maths, you need to be precise about what a *coordinate frame* is and which frames
are involved.

A **coordinate frame** is an origin point plus three mutually perpendicular axes (X, Y, Z)
with an agreed handedness. The camera frame and robot body frame in this project are
right-handed. The D500 LiDAR is an exception — its native frame is **left-handed** (angles
increase clockwise when viewed from above, placing positive Y to the right of the forward
axis). This handedness difference is not a problem: $R$ in $T_C^L$ absorbs it as part of the
frame-to-frame rotation, the same way it absorbs any other axis misalignment.

### The Three Frames in Play

```
                             ^ Z (up, out of scan plane)
                             |
  LiDAR origin (L)           |        Camera origin (C)
       o---> X (forward)          o---> X (right in image)
       |                          |
       v Y (right) [left-handed]  v Y (down in image)
                                  .
                              Z (forward, out of lens)
```

**LiDAR frame (L): ⚠ left-handed**
- Origin: the centre of the D500's rotating mirror
- X: forward (aligned with the rover's forward direction when mounted centrally)
- Y: **right** — angles increase clockwise when viewed from above; positive Y is to the right
  of the forward axis. This makes the frame left-handed (X forward, Y right, Z up).
- Z: straight up (perpendicular to the horizontal scan plane)
- Measurements live in the XY plane ($Z = 0$ for all returns — the D500 is a planar scanner)

**Camera frame (C):**
- Origin: the optical centre of the OAK-D Lite colour lens
- X: right (as seen through the lens)
- Y: down (image convention — matches pixel row direction)
- Z: forward along the optical axis (out of the lens, into the scene)
- This is the frame you already know from intrinsic calibration — $K$ lives here

**Robot body frame (B):**
- A convenience frame centred on the rover chassis
- Used in Phase 3/4 for control signals; not needed for the calibration itself
- Both sensor frames relate to it, but the direct LiDAR ↔ camera transform is what matters
  for sensor fusion

The goal of extrinsic calibration is to measure the transform from L to C precisely.

---

## 3. Rigid-Body Transforms

### What "Rigid-Body" Means

A rigid-body transform preserves distances and angles — it is a **rotation followed by a
translation** (or equivalently a translation followed by a rotation — the order matters,
but rotation-then-translation is the standard convention). No scaling, no shearing, no
bending.

This is appropriate here because the sensors are rigid objects fixed in place. The spatial
relationship between them is a pure rotation (how one sensor is *tilted* relative to the
other) plus a pure translation (how far apart their origins are in 3D space).

### Applying the Transform

Let $\mathbf{p}_L = (x_L,\ y_L,\ z_L)^\top$ be a 3D point expressed in the LiDAR frame.
The same point expressed in the camera frame is:

$$\mathbf{p}_C = R\,\mathbf{p}_L + \mathbf{t}$$

where:
- $R$ is a $3 \times 3$ **rotation matrix** describing how the LiDAR's axes are oriented
  relative to the camera's axes
- $\mathbf{t}$ is a $3 \times 1$ **translation vector** describing the LiDAR origin's
  position in the camera frame

Although written as a single vector equation, this is a system of **three scalar equations**
— one per spatial dimension — computed simultaneously:

$$x_C = r_{11}x_L + r_{12}y_L + r_{13}z_L + t_x$$
$$y_C = r_{21}x_L + r_{22}y_L + r_{23}z_L + t_y$$
$$z_C = r_{31}x_L + r_{32}y_L + r_{33}z_L + t_z$$

Each output coordinate is a weighted sum of *all three* input coordinates — which is why a
pure yaw rotation (rotating around Z) changes both $x_C$ and $y_C$ together, not just one
of them in isolation.

The system involves 6 independent numbers total: $R$ is a $3 \times 3$ matrix (9 entries)
and $\mathbf{t}$ is a 3-vector, but the constraints on a rotation matrix reduce $R$'s 9
entries to only 3 free values (roll, pitch, yaw). Add the 3 translation components and you
get the **6 degrees of freedom** of a rigid-body transform in 3D space.

---

## 4. The Rotation Matrix $R$

### Geometric Meaning

$R$ is a $3 \times 3$ matrix where each column is one of the LiDAR's unit axis vectors,
expressed in camera-frame coordinates:

$$R = \begin{bmatrix} | & | & | \\ \hat{x}_L & \hat{y}_L & \hat{z}_L \\ | & | & | \end{bmatrix}$$

where $\hat{x}_L$, $\hat{y}_L$, $\hat{z}_L$ are the LiDAR's three axis directions written
in the camera's coordinate language. Column 1 answers *"which way does LiDAR-forward
point, in camera coordinates?"*, column 2 answers the same for LiDAR-right, column 3 for
LiDAR-up.

This column interpretation gives the cleanest geometric reading of the multiplication
$R\,\mathbf{p}_L$: it is a weighted sum of the LiDAR's axes expressed in camera
coordinates —

$$R\,\mathbf{p}_L = x_L\underbrace{\begin{bmatrix}r_{11}\\r_{21}\\r_{31}\end{bmatrix}}_{\hat{x}_L\text{ in cam coords}} + y_L\underbrace{\begin{bmatrix}r_{12}\\r_{22}\\r_{32}\end{bmatrix}}_{\hat{y}_L\text{ in cam coords}} + z_L\underbrace{\begin{bmatrix}r_{13}\\r_{23}\\r_{33}\end{bmatrix}}_{\hat{z}_L\text{ in cam coords}}$$

The point is $x_L$ steps along LiDAR-forward, plus $y_L$ steps along LiDAR-right, plus
$z_L$ steps along LiDAR-up — but each of those "steps" is now measured using the camera's
own axis directions.

If the two sensors were perfectly aligned (parallel axes, same orientation), each LiDAR
axis would coincide with the matching camera axis, $R$ would be the $3 \times 3$ identity
matrix $I$, and $R\,\mathbf{p}_L = \mathbf{p}_L$ — no change.

In practice the sensors will be slightly misaligned in pitch, roll, and yaw. $R$ captures
all three angular differences simultaneously.

### Properties of $R$

A rotation matrix is not just any $3 \times 3$ matrix. It belongs to the **special orthogonal
group** $SO(3)$, which means it satisfies two constraints:

$$R^\top R = I \qquad \text{(columns are mutually orthonormal)}$$
$$\det(R) = +1 \qquad \text{(right-handed — no reflection)}$$

These constraints are important: a $3 \times 3$ matrix has 9 entries, but satisfying these
constraints leaves only **3 degrees of freedom** (roll, pitch, yaw). The constraints also
mean that the inverse of a rotation matrix is just its transpose:

$$R^{-1} = R^\top$$

This is computationally cheap and numerically exact — a property you will use when inverting
the transform.

### The Three Angles

Although $R$ is stored and applied as a matrix, it can equivalently be described by three
rotation angles. Two common parameterisations:

**Euler angles** (roll $\phi$, pitch $\theta$, yaw $\psi$) — intuitive but suffer from
gimbal lock and are order-dependent (ZYX, XYZ, etc. give different results).

**Rodrigues vector** $\mathbf{r}$ — yes, you really can compress $R$'s 9 entries into just
3 numbers. Here's why that works.

**Euler's Rotation Theorem** states that any orientation change in 3D — no matter how
complex, including combined yaw + pitch + roll — is always equivalent to a *single rotation
around a single axis by a single angle*. There is always exactly one such axis.

Note the distinction: the three *coordinate* axes (X, Y, Z) are a property of the frame;
the *rotation axis* is a different concept — it is some arbitrary direction in 3D space
around which the entire rotation can be described as one turn. It need not align with any
coordinate axis.

The Rodrigues vector encodes both pieces of information in one 3-vector:

$$\mathbf{r} = \theta\,\hat{\mathbf{u}}$$

where $\hat{\mathbf{u}}$ is a plain unit vector (length = 1) that points along the rotation
axis, and $\theta$ is the rotation angle in radians. Multiplying $\hat{\mathbf{u}}$ by the
scalar $\theta$ stretches it to length $\theta$ without changing its direction — so the
resulting 3-vector $\mathbf{r}$ simultaneously carries **both** pieces of information:

| Property | What it encodes | How to recover it |
|----------|-----------------|-------------------|
| **Direction** of $\mathbf{r}$ | The line in 3D space to rotate around — need not align with X, Y, or Z | $\hat{\mathbf{u}} = \mathbf{r} / \|\mathbf{r}\|$ |
| **Magnitude** $\|\mathbf{r}\|$ | How far to rotate around that line (radians) | $\theta = \|\mathbf{r}\|$ |

The rotation axis is *not* necessarily one of the coordinate axes (X, Y, Z). Those are just
the named axes of a particular frame. The rotation axis is an arbitrary direction in 3D space
— the one unique line around which the entire combined misalignment can be expressed as a
single turn. For pure misalignments it happens to coincide with a coordinate axis (pure yaw
→ Z, pure pitch → Y, pure roll → X), but any combination of misalignments produces a
diagonal axis that doesn't align with any of them.

Note also that $\hat{\mathbf{u}}$ has **no fixed location in space** — it is a pure
direction, like a compass bearing, not a ray anchored to the camera origin or any other
physical point. This is precisely why $R$ and $\mathbf{t}$ are kept separate in
$\mathbf{p}_C = R\,\mathbf{p}_L + \mathbf{t}$: $R$ captures **orientation only** (how the
frames are rotated relative to each other, with no location involved), and $\mathbf{t}$
captures **position only** (where the LiDAR origin physically sits in the camera frame).
You can rotate without moving, and move without rotating — they are geometrically
independent, and the transform reflects that separation.

Finally, $\hat{\mathbf{u}}$ is **frame-independent**: it has the same numerical components
whether you express it in LiDAR coordinates or camera coordinates. Mathematically,
$\hat{\mathbf{u}}$ is the eigenvector of $R$ with eigenvalue 1 —

$$R\hat{\mathbf{u}} = \hat{\mathbf{u}}$$

— meaning it is the one direction the rotation *leaves unchanged*. A direction that is
unchanged by a rotation looks identical from either side of that rotation, so neither frame
has a privileged claim on it. This is what makes the Rodrigues vector a clean, neutral
description of the rotation itself, rather than a description tied to one sensor's
perspective.

Concretely: if the LiDAR is yawed 0.1 rad (≈ 5.7°) to the right, the rotation axis is
straight up (Z), so:

$$\mathbf{r} = 0.1 \times \begin{bmatrix}0\\0\\1\end{bmatrix} = \begin{bmatrix}0\\0\\0.1\end{bmatrix}$$

The vector points straight up (→ axis is Z), length is 0.1 (→ angle is 0.1 rad). If the
yaw were 0.2 rad instead, the direction would be identical but the length would be 0.2. A
combined yaw and pitch would produce a diagonally-pointing vector whose length gives the
single equivalent angle.

<img src="../other/images/Euler_AxisAngle.png" alt="Test" width="300">

<img src="../other/images/euler_image.png" alt="Test" width="300">

Concrete examples for this project:

| Sensor misalignment | Rodrigues vector |
|---------------------|------------------|
| Pure yaw (0.1 rad right) | $(0,\ 0,\ 0.1)$ — points along Z (up), length 0.1 |
| Pure pitch (0.05 rad forward tilt) | $(0,\ 0.05,\ 0)$ — points along Y, length 0.05 |
| Combined yaw + pitch | $(r_x,\ r_y,\ r_z)$ — diagonal direction, length = total equivalent angle |

The 9 entries of $R$ cannot all vary freely — 6 are consumed by the orthonormality
constraints $R^\top R = I$, leaving exactly 3 free values. The Rodrigues vector captures
precisely those 3 free values.

No singularities exist (unlike Euler angles), it is compact, and it is what OpenCV uses
internally. Convert to/from a full $3 \times 3$ matrix with `cv2.Rodrigues()`.

---

## 5. The Translation Vector $\mathbf{t}$

$\mathbf{t}$ is a 3-vector $(t_x,\ t_y,\ t_z)^\top$ in metres, expressed in the camera
frame, pointing from the camera origin to the LiDAR origin.

Physically this is just the 3D offset between the two sensor mounting points. For a compact
chassis like the Waveshare UGV Rover, expect values of a few centimetres in each axis.

Note the direction convention: $\mathbf{t}$ describes where the LiDAR origin is *in camera
coordinates*, not where the camera origin is in LiDAR coordinates. Consistent with the sense
of the transform $\mathbf{p}_C = R\,\mathbf{p}_L + \mathbf{t}$.

---

## 6. Homogeneous Coordinates and the $4 \times 4$ Matrix

### Why Homogeneous Coordinates?

The transform $\mathbf{p}_C = R\,\mathbf{p}_L + \mathbf{t}$ involves both a matrix
multiplication ($R$) and a vector addition ($\mathbf{t}$). This is inconvenient when
chaining multiple transforms — you would need to carry the additions separately.

**Homogeneous coordinates** fold both operations into a single matrix multiplication by
augmenting the 3-vectors with a fourth component equal to 1:

$$\tilde{\mathbf{p}} = \begin{bmatrix} x \\ y \\ z \\ 1 \end{bmatrix}$$

### The $4 \times 4$ Homogeneous Transform

The extrinsic transform is then expressed as a single $4 \times 4$ matrix:

$$T_C^L = \begin{bmatrix} R & \mathbf{t} \\ \mathbf{0}^\top & 1 \end{bmatrix} = \begin{bmatrix} r_{11} & r_{12} & r_{13} & t_x \\ r_{21} & r_{22} & r_{23} & t_y \\ r_{31} & r_{32} & r_{33} & t_z \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

Applied to a point:

$$\begin{bmatrix} \mathbf{p}_C \\ 1 \end{bmatrix} = T_C^L \begin{bmatrix} \mathbf{p}_L \\ 1 \end{bmatrix}$$

which expands to exactly $\mathbf{p}_C = R\,\mathbf{p}_L + \mathbf{t}$ — identical to before,
but now expressed as one matrix-vector product. This is the form stored in `sensor_config.yaml`
and used throughout the codebase.

### Notation Convention

The superscript/subscript notation $T_C^L$ is read as *"transform from frame L to frame C"*
— it takes a point expressed in $L$ and gives you the same point expressed in $C$. The
planning document calls this $T_\text{cam}^\text{lidar}$; they are the same thing.

### Inverting the Transform

To go from camera frame back to LiDAR frame:

$$(T_C^L)^{-1} = T_L^C = \begin{bmatrix} R^\top & -R^\top\mathbf{t} \\ \mathbf{0}^\top & 1 \end{bmatrix}$$

No numerical matrix inversion is needed — the closed-form inverse exploits $R^{-1} = R^\top$
and costs only a transposition and a matrix–vector multiply.

### Chaining Transforms

The payoff of homogeneous form is that transforms chain by simple matrix multiplication.
If you later add a body frame (B) with known transforms $T_B^C$ and $T_B^L$, then:

$$T_C^L = T_C^B \cdot T_B^L$$

This associativity is used in Phase 4 when fusing sensors through a common reference frame.

---

## 7. The Full Projection Pipeline

Combining the extrinsic transform with the intrinsic matrix $K$ (from Phase 2) gives the
complete pipeline from a raw LiDAR measurement to an image pixel:

### Step 1 — Polar to Cartesian (LiDAR frame)

The D500 reports each return as a range $r$ (mm) and bearing angle $\theta$ (degrees,
measured clockwise from the sensor's forward axis — consistent with the left-handed frame).
Convert to LiDAR-frame Cartesian coordinates:

$$x_L = r \cos\theta, \qquad y_L = r \sin\theta, \qquad z_L = 0$$

With the left-handed convention: $\theta = 0°$ is straight ahead (+X), $\theta = 90°$ is to
the right (+Y), and $\theta = -90°$ (or 270°) is to the left (−Y). The formula is unchanged
from the standard polar conversion — the handedness is already encoded in the axis definitions.

$z_L = 0$ always — the D500 scans a single horizontal plane. This is the Phase 3 polar→Cartesian
conversion; the extrinsic transform operates on its output.

### Step 2 — LiDAR frame to Camera frame

Apply the extrinsic transform:

$$\begin{bmatrix} x_C \\ y_C \\ z_C \\ 1 \end{bmatrix} = T_C^L \begin{bmatrix} x_L \\ y_L \\ 0 \\ 1 \end{bmatrix}$$

After this step the point is expressed in the same 3D coordinate system as the camera model.
$z_C > 0$ means the point is in front of the camera (required for a valid projection).

### Step 3 — Camera frame to image pixels

Apply the intrinsic matrix $K$:

$$s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = K \begin{bmatrix} x_C \\ y_C \\ z_C \end{bmatrix} = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} x_C \\ y_C \\ z_C \end{bmatrix}$$

where $s = z_C$ is the depth. The final pixel coordinates are:

$$u = f_x \cdot \frac{x_C}{z_C} + c_x, \qquad v = f_y \cdot \frac{y_C}{z_C} + c_y$$

### The Complete Chain

$$\underbrace{(r,\,\theta)}_{\substack{\text{LiDAR} \\ \text{polar}}} \xrightarrow{\;\text{polar} \to \text{Cartesian}\;} \underbrace{(x_L,\,y_L,\,0)}_{\substack{\text{LiDAR} \\ \text{frame}}} \xrightarrow{\;T_C^L\;} \underbrace{(x_C,\,y_C,\,z_C)}_{\substack{\text{camera} \\ \text{frame}}} \xrightarrow{\;K\;} \underbrace{(u,\,v)}_{\substack{\text{image} \\ \text{pixel}}}$$

This pipeline is used in Phase 4 (sensor fusion validation) to check whether a LiDAR return
projects into the bounding box of a detected object.

---

## 8. The Planarity Constraint — A Fundamental Limitation

### What the D500 Cannot See

The LDRobot D500 is a 2D planar LiDAR. Its laser sweeps a single horizontal plane —
it has no physical ability to measure returns above or below that plane. In the LiDAR frame,
every return has $z_L = 0$ by construction.

This creates an **observability gap** in the extrinsic calibration:

- $t_x$ and $t_y$ (the horizontal offsets) are directly observable from LiDAR data
- $t_z$ (the vertical offset — how far above or below the camera the LiDAR sits) cannot
  be inferred from LiDAR measurements alone, because the LiDAR cannot sample points at
  varying $z_L$
- Similarly, **roll and pitch components of $R$** that rotate points out of the scan plane
  are weakly observable or unobservable from planar scan data alone

Concretely: if you tilt the LiDAR slightly in pitch (rotating returns upward), their
projected pixel positions shift, but a flat scan with all $z_L = 0$ contains almost no
information to distinguish a true pitch from a change in $t_z$. These parameters are
**coupled** in the measurement.

### Consequence for the Calibration

The unobservable components must be **measured physically** (ruler, calipers) and fixed,
rather than solved for from sensor data:

| Component | Observable from data? | Approach |
|-----------|----------------------|----------|
| $t_x$ — forward/back offset | Yes | Solve from calibration |
| $t_y$ — left/right offset | Yes | Solve from calibration |
| $t_z$ — vertical offset | No | Measure with calipers |
| $R$ yaw — horizontal rotation | Yes | Solve from calibration |
| $R$ pitch — tilt toward/away | Weakly | Fix from physical measurement |
| $R$ roll — lean left/right | Weakly | Fix from physical measurement |

This is not a failure of the method — it is a geometrically honest reflection of what a
single-plane LiDAR can and cannot observe. A 3D LiDAR (e.g. Velodyne) would have full
observability; the D500 does not.

In practice, for the follower application:
- The vertical offset $t_z$ matters for the Phase 4 projection check (whether a LiDAR
  return overlaps the bounding box in the image)
- The yaw component of $R$ is the most safety-critical: errors here cause LiDAR returns
  to be associated with the wrong angular sector in the image

---

## 9. Reprojection Error — How You Know It Worked

### Definition

The quality of an extrinsic calibration is measured by **reprojection error**: take a 3D point
whose location is known in both sensor frames, project it into the image using the estimated
$T_C^L$ and $K$, and compare the projected pixel to the actual observed pixel.

$$e_{ij} = \left\| \mathbf{p}_{ij}^\text{observed} - \hat{\mathbf{p}}\!\left(T_C^L,\, K,\, D,\, \mathbf{P}_j^L\right) \right\|_2$$

Lower is better. The RMS across all point correspondences is the single headline number
reported at the end of calibration.

### Why This Is the Right Metric

Reprojection error directly measures what you actually care about in Phase 4: *"When I take
a LiDAR return and project it into the image, does it land where it should?"* A low mean
reprojection error with no high-error outliers is your accept/reject criterion.

### Expected Values

| Result | Interpretation |
|--------|---------------|
| RMS < 3 px | Good — suitable for sensor fusion |
| RMS 3–8 px | Acceptable — minor axis misalignment |
| RMS > 8 px | Poor — likely a systematic error (wrong $t_z$, board detection failure, or wrong frame convention) |

These are wider tolerances than intrinsic calibration (< 1 px) because the LiDAR's angular
resolution (~1°) and the planarity constraint both limit achievable accuracy. A 1° angular
error at 3 m range is ~52 mm ≈ several pixels of reprojection error at typical focal lengths.

---

## 10. `sensor_config.yaml` — Result Structure

The calibration produces $T_C^L$ as a $4 \times 4$ matrix. The convention used throughout
the codebase is to store $R$ and $\mathbf{t}$ separately (easier to inspect and edit) plus
the full matrix for direct use in NumPy:

```yaml
extrinsic:
  lidar_to_camera:
    # Rotation matrix R (3x3, row-major) — maps LiDAR-frame axes to camera-frame axes
    rotation:
      - [r11,  r12,  r13]
      - [r21,  r22,  r23]
      - [r31,  r32,  r33]
    # Translation vector t (metres) — LiDAR origin expressed in camera frame [x, y, z]
    translation: [tx, ty, tz]
    # Vertical offset measured physically (t_z above) — kept explicitly for auditability
    tz_physical_m: null
    # RMS reprojection error (pixels) — filled in after calibration
    rms_reprojection_error: null
```

Loading this in Python and reconstructing $T_C^L$:

```python
import numpy as np

R = np.array(cfg.extrinsic.lidar_to_camera.rotation, dtype=np.float64)   # (3, 3)
t = np.array(cfg.extrinsic.lidar_to_camera.translation, dtype=np.float64) # (3,)

T = np.eye(4, dtype=np.float64)
T[:3, :3] = R
T[:3, 3]  = t
```

Projecting a single LiDAR return into the image:

```python
# LiDAR point in homogeneous form
p_L = np.array([x_L, y_L, 0.0, 1.0])

# Transform to camera frame
p_C = T @ p_L                    # shape (4,)
x_C, y_C, z_C = p_C[:3]

if z_C <= 0:
    pass  # behind the camera — skip

# Project to pixel
u = K[0, 0] * (x_C / z_C) + K[0, 2]
v = K[1, 1] * (y_C / z_C) + K[1, 2]
```

---

## 11. Dependencies and Phase Ordering

The extrinsic calibration depends on intrinsic calibration having been completed first:

- $K$ (the intrinsic matrix) is used in the reprojection step — the solver minimises
  reprojection error in pixel space, so correct $K$ is required to get a correct $T_C^L$
- $D$ (distortion coefficients) should be applied to undistort image coordinates before
  any point correspondence is established — distorted and undistorted pixels differ by
  up to several pixels near the frame edges

The sequencing in Phase 2 is therefore:

```
Intrinsic calibration (K, D)  →  Extrinsic calibration (T_C^L)  →  Phase 3 runtime
```

Using an incorrect or uncalibrated $K$ to estimate $T_C^L$ propagates the intrinsic error
into every LiDAR→image projection for the rest of the project.
