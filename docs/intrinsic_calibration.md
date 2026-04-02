# Intrinsic Camera Calibration — Reference

Covers the theory and mathematics of intrinsic calibration and the OpenCV fisheye checkerboard
routine used in Phase 2 of the project.
Target camera: **Waveshare RGB (pan-tilt module)**.

---

## 1. Why Intrinsic Calibration

### What Intrinsic Calibration Solves

The fundamental problem is this: **a camera lens is not a perfect optical instrument**. The
Waveshare camera uses a wide-angle fisheye lens that distorts the image — straight lines in the
real world appear curved in the photo. More importantly, the relationship between a pixel's
position on the sensor and the actual direction that ray came from is **unknown without calibration**.

Intrinsic calibration is the process of measuring and modelling these optical imperfections.
Its goal is to answer the question: "Given a pixel location $(u, v)$ in the image, what direction
in 3D space does that pixel correspond to?" The answer requires two pieces of information:

1. **The focal lengths and principal point** ($K$): how the optical axis and zoom level map the
   world onto the sensor
2. **The lens distortion coefficients** ($D$): how the real lens deviates from the ideal pinhole
   model, distorting what should-be-straight lines into curves

Without these measurements, every pixel is just a number with no geometric meaning. You cannot
reliably determine where an object is in the real world relative to the camera.

### How It Enables Downstream Computation

Once calibration is complete, you can trace rays from pixels back into the world. This underpins
every downstream phase:

**Phase 3 steering.** The follower tracks a target in the image and must steer toward it. It
computes how far off-centre the target bounding box is in pixels, then converts that pixel offset
to a physical angle using the focal length $f_x$:

$$\alpha = \arctan\!\left(\frac{\Delta u}{f_x}\right)$$

Without calibration, $f_x$ is unknown and the pixel offset is meaningless — you cannot steer.

**Phase 3 geometric computations.** The Waveshare's fisheye lens distorts the image. Before any
geometric algorithm can use pixel coordinates (e.g. to detect where a target is), those coordinates
must first be *undistorted* — mapped back to where they would appear in an ideal pinhole camera.
The distortion coefficients $D$ are the only way to compute this correction.

**Extrinsic calibration (Phase 2).** The next step measures how the LiDAR and camera are oriented
relative to each other. This solver needs the intrinsic matrix $K$ to project 3D points onto image
pixels correctly. An incorrect $K$ directly corrupts the computed transform.

**Phase 4 sensor fusion.** Any algorithm that needs to project LiDAR points onto camera images or
vice versa relies entirely on having an accurate, calibrated $K$ and $D$ from this step.

Intrinsic calibration is therefore the foundational measurement on which everything downstream is
built. The output is two objects stored in `sensor_config.yaml`:

- $K$ — a $3 \times 3$ matrix encoding the focal lengths and principal point
- $D$ — a vector of four distortion coefficients specific to the fisheye lens model

---

## 2. The Pinhole Camera Model

### Physical Intuition

The idealised camera model starts with a thought experiment: a completely dark box with a single
infinitesimally small hole in one face and a flat sensor on the opposite face. Light from a scene
point travels in a straight line through the hole and lands on the sensor. No lens, no glass —
just geometry.

```
                         sensor plane
World point              (image plane)
    P                        |
     \                       |
      \                      |  p (image of P)
       \                     |
        [optical centre] ----+---------> optical axis (Z)
       /                     |
      /                      |
     /                       |
    Q                        |  q (image of Q)
                             |
                         focal length f
```

The key property is that **all projection rays pass through a single point** — the optical centre.
The sensor sits at distance $f$ (the **focal length**) behind it.

### Step 1 — Perspective Projection

Consider a world point $P = (X, Y, Z)$ in **camera coordinates** (origin at the optical centre,
Z pointing forward along the optical axis). By similar triangles it projects to the sensor point
$p = (x, y)$:

$$x = f \cdot \frac{X}{Z}, \qquad y = f \cdot \frac{Y}{Z}$$

The division by $Z$ is what makes this **perspective projection** — objects farther away appear
smaller in proportion to their distance.

### Step 2 — Physical Units to Pixels

The sensor is measured in pixels. Each pixel has a physical width $d_x$ and height $d_y$
(metres per pixel). Dividing the metric projection by pixel size:

$$u = \frac{x}{d_x} = \frac{f}{d_x} \cdot \frac{X}{Z}, \qquad v = \frac{y}{d_y} = \frac{f}{d_y} \cdot \frac{Y}{Z}$$

Define the **focal lengths in pixels**:

$$f_x = \frac{f}{d_x}, \qquad f_y = \frac{f}{d_y}$$

These are the only form of focal length that ever appears in the camera model. You never need to
separately know the physical focal length in millimetres or the pixel pitch in micrometres —
only their ratio matters, and calibration solves for $f_x$ and $f_y$ directly.

For most cameras $d_x \approx d_y$ (square pixels), so $f_x \approx f_y$. A significant
difference between them typically indicates a calibration problem.

### Step 3 — The Principal Point

The optical axis intersects the sensor at the **principal point** $(c_x, c_y)$ in pixel
coordinates. In an ideal camera this is exactly the image centre. In reality it is slightly
off due to manufacturing tolerances — typically within 1–2% of the image dimension. Accounting
for this offset:

$$u = f_x \cdot \frac{X}{Z} + c_x, \qquad v = f_y \cdot \frac{Y}{Z} + c_y$$

---

## 3. The Intrinsic Matrix $K$

Multiplying through by $Z$ and writing in homogeneous form consolidates all four parameters into
a single matrix:

$$Z \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \underbrace{\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}}_{K} \begin{bmatrix} X \\ Y \\ Z \end{bmatrix}$$

This is the **intrinsic matrix** $K$ — the primary product of intrinsic calibration. The zeros in
the off-diagonal positions enforce the *no-skew* assumption (pixel axes are perpendicular), which
holds for every modern image sensor. The structure of $K$ is not a convention or a choice — the
shape falls out directly from the algebra of perspective projection.

$K$ is a fixed physical property of the camera. It does not change with what the camera is
pointing at, how far away objects are, or where on the chassis the camera is mounted. The only
thing that invalidates a previously measured $K$ is a physical change to the lens or sensor —
adjusting focus, changing the lens, or damaging the camera.

### What the Numbers Will Look Like

The Waveshare RGB camera is a wide-angle USB camera. Expected values at 640×480:

| Parameter | Typical value | Meaning |
|-----------|--------------|---------|
| $f_x$ | 200–400 px | Horizontal focal length (wide lens → lower value) |
| $f_y$ | 200–400 px | Vertical focal length (≈ $f_x$) |
| $c_x$ | ~320 px | Near horizontal centre |
| $c_y$ | ~240 px | Near vertical centre |

The lower focal length range compared to a narrow-angle webcam is expected — a wide-angle fisheye
lens crams a larger field of view into the same number of pixels, so the apparent focal length
in pixels is smaller.

---

## 4. Why the Standard Pinhole Model Is Not Enough

### Perspective vs. Fisheye Projection

In the ideal pinhole model, a point at angle $\theta$ from the optical axis projects to a radial
distance $r$ on the sensor given by:

$$r_\text{perspective} = f \tan\theta$$

For a narrow camera ($\theta < 30°$) this is a good model — $\tan\theta \approx \theta$, and the
images look geometrically correct. The standard OpenCV distortion model adds small polynomial
corrections around this.

The Waveshare camera uses a **wide-angle fisheye lens** with a field of view well beyond 90°.
At $\theta = 80°$, $\tan(80°) \approx 5.7$ — a point near the edge of the frame would need to
project nearly six focal lengths from the image centre to satisfy the perspective equation. That
is physically impossible on a finite sensor, and attempting to model the image as perspective
produces coefficients that have no physical meaning and misrepresent the actual geometry.

A fisheye lens deliberately departs from perspective projection to compress a wide field of view
onto a finite sensor. The relationship between angle and radial distance follows a different curve
— and the distortion model must reflect that different curve.

---

## 5. The Fisheye Projection Model

### The Equidistant Baseline

The simplest fisheye projection law is **equidistant**:

$$r = f \theta$$

Instead of mapping $\tan\theta$ (perspective), it maps $\theta$ itself — the angle from the
optical axis in radians — linearly to the radial distance on the sensor. The result is that equal
angular intervals anywhere in the field of view map to equal radial steps on the sensor, regardless
of where in the frame you are. This is why fisheye lenses are useful: the angular resolution is
uniform across the entire field of view.

### The Kannala-Brandt Model

No real lens is perfectly equidistant. OpenCV's `cv2.fisheye` module uses the
**Kannala-Brandt model** (Kannala & Brandt 2006), which adds a polynomial correction to the ideal
equidistant projection. The distorted radius $r_d$ is:

$$r_d = \theta\Bigl(1 + k_1\theta^2 + k_2\theta^4 + k_3\theta^6 + k_4\theta^8\Bigr)$$

where $\theta$ is the angle of incidence from the optical axis:

$$\theta = \arctan\!\left(\frac{\sqrt{X^2 + Y^2}}{Z}\right)$$

When all four $k$ coefficients are zero, $r_d = \theta$ — the ideal equidistant case. The
polynomial captures the real lens's deviation from ideal equidistant behaviour, with each higher
term accounting for progressively finer corrections at larger angles.

### From Distorted Radius to Pixel Coordinates

Once the distorted radius $r_d$ is known, the projected image point is scaled along the original
direction of the ray and then passed through $K$:

$$x_d = r_d \cdot \frac{X/Z}{\sqrt{(X/Z)^2 + (Y/Z)^2}}, \qquad y_d = r_d \cdot \frac{Y/Z}{\sqrt{(X/Z)^2 + (Y/Z)^2}}$$

This keeps the distorted point on the same radial line as the undistorted point — only the
radial distance changes, not the direction. The final pixel coordinates are then:

$$u = f_x \cdot x_d + c_x, \qquad v = f_y \cdot y_d + c_y$$

The complete forward projection chain for a 3D camera-frame point is therefore:

$$\underbrace{(X, Y, Z)}_{\substack{\text{camera} \\ \text{frame}}} \xrightarrow{\;\theta = \arctan\!\left(\frac{\sqrt{X^2+Y^2}}{Z}\right)\;} \underbrace{\theta}_{\substack{\text{incidence} \\ \text{angle}}} \xrightarrow{\;r_d = \theta(1 + k_1\theta^2 + \cdots)\;} \underbrace{r_d}_{\substack{\text{distorted} \\ \text{radius}}} \xrightarrow{\;K\;} \underbrace{(u, v)}_{\substack{\text{image} \\ \text{pixel}}}$$

### The Distortion Vector $D$

The calibration output for the fisheye model is:

$$D = \begin{bmatrix} k_1 & k_2 & k_3 & k_4 \end{bmatrix}$$

These are the four Kannala-Brandt coefficients stored as a flat vector. They have no direct
geometric interpretation individually — they are the best-fit polynomial coefficients that
make the model agree with the observed corner positions across all calibration images.

**Contrast with the standard OpenCV model:** The standard `cv2.calibrateCamera` uses a separate
radial + tangential polynomial $D = [k_1, k_2, p_1, p_2, k_3]$ built around the perspective
baseline. These two $D$ vectors are fundamentally different models and are not interchangeable.
Always use `cv2.fisheye.*` functions with a fisheye $K$ and $D$, and `cv2.undistort` /
`cv2.calibrateCamera` for standard-model parameters. Mixing them produces wrong results silently.

---

## 6. The Checkerboard and Zhang's Method

### Why a Checkerboard?

The calibration problem is: given a set of known 3D points and their observed 2D pixel positions,
find $K$ and $D$. The known 3D points must come from a target with **precisely known geometry**.

A checkerboard is ideal because:
- Inner corner positions are **algebraically exact** — no printing imprecision in the geometry,
  only in the overall scale
- `cv2.findChessboardCorners` + `cv2.cornerSubPix` locate corners to **sub-pixel accuracy**
  (typically ~0.1 px)
- The target is flat (planar), which enables the homography-based method below

**The board's coordinate frame.** A coordinate frame is an origin plus three axes (X, Y, Z).
For calibration, the world frame is defined as **the board itself**:
- Origin at the top-left inner corner
- X runs rightward along the top row of corners
- Y runs downward along the left column
- Z points perpendicular out of the board surface

Since the board is flat and rigid, every corner has $Z = 0$ by definition. For an 8×5
inner-corner board with 28 mm squares:

$$\{(0,0,0),\ (28,0,0),\ (56,0,0),\ \ldots,\ (196, 112, 0)\} \text{ mm}$$

These coordinates are known **exactly** before capturing a single image.

### What You Give OpenCV

For every calibration image you provide two lists:

| `object_points` | `image_points` |
|---|---|
| Where each corner **is in the world** (mm, in board frame) | Where each corner **appeared in the photo** (pixels) |
| Identical for every image — the board doesn't change | Detected by `findChessboardCorners` |

### Zhang's Planar Homography Method

OpenCV uses **Zhang's method** (Zhang 2000). A **homography** $H$ is a $3 \times 3$ projective
matrix mapping points from one flat plane to another. Since all board corners have $Z = 0$, the
third column of the full projection matrix drops out and the projection reduces to $H$:

$$H = K \begin{bmatrix} \mathbf{r}_1 & \mathbf{r}_2 & \mathbf{t} \end{bmatrix}$$

where $\mathbf{r}_1, \mathbf{r}_2$ are the first two columns of the pose rotation and $\mathbf{t}$
is the translation. The key insight is that $K$ is **constant across all images** while the
pose $[\mathbf{r}_1\ \mathbf{r}_2\ \mathbf{t}]$ varies. Each image gives 2 linear constraints on
$K$; three or more images make the system over-determined and solvable.

**Step 1 — Compute $H_i$ per image (DLT).** Each corner correspondence $(X_j, Y_j) \to (u_j, v_j)$
gives two linear equations. Stacking all $M$ corners into a matrix equation $A\mathbf{h} = \mathbf{0}$
and solving via SVD gives $H_i$ for that image.

**Step 2 — Extract $K$ constraints.** The orthonormality of $\mathbf{r}_1, \mathbf{r}_2$ provides
two constraints per image on $B = K^{-\top}K^{-1}$ (the Image of the Absolute Conic). Solving for
$B$ then Cholesky-decomposing it recovers $K$.

**Step 3 — Nonlinear refinement.** The linear solution initialises a **Levenberg-Marquardt**
optimiser that minimises total reprojection error across all images and all corners simultaneously,
refining both $K$ and $D$:

$$\min_{K,\, D,\, \{R_i,\, \mathbf{t}_i\}} \sum_{i=1}^{N} \sum_{j=1}^{M} \left\| \mathbf{p}_{ij} - \hat{\mathbf{p}}(K, D, R_i, \mathbf{t}_i, \mathbf{P}_j) \right\|^2$$

where $\hat{\mathbf{p}}$ is the full fisheye projection including the Kannala-Brandt distortion.
This nonlinear step is where the distortion coefficients converge — the linear solution cannot
recover $D$.

### Why You Cannot Use One Image

With a single image, an unknown pose, and an unknown $K$, the system is underdetermined. You could
explain the same corner observations with a large $f_x$ and a distant board, or a small $f_x$ and
a close board. Multiple images at genuinely different angles and distances eliminate this ambiguity.
Image **variety** — different angles, distances, positions in the frame — matters far more than
image count.

---

## 7. Capture Strategy

The capture strategy determines the quality ceiling. A technically perfect solver cannot
recover a good calibration from poor images.

1. **20–30 images minimum** — fewer gives the Kannala-Brandt polynomial insufficient constraints
   to converge, especially for $k_3$ and $k_4$
2. **Tilt the board** — tilt ±30–45° around both axes across different captures; varying angle is
   the single most important factor
3. **Fill the entire frame** — corners must appear near all four image edges, not just in the
   centre; distortion is strongest at the periphery and must be sampled there to be characterised
4. **Vary distance** — close, mid, and far captures; this helps constrain the scaling relationship
   between $K$ and the object-point scale
5. **No blur** — motion blur corrupts sub-pixel corner detection; hold the board still and use
   adequate lighting
6. **Keep the board flat** — any warp in a paper board biases the distortion coefficients
7. **Lock the pan-tilt to (0°, 0°)** — intrinsics are a property of the lens, not the pan/tilt
   angle, but physically the lens moves when the servo moves; calibrate with the servo at its
   operational centre position

---

## 8. `sensor_config.yaml` — Result Structure

The calibration produces $K$ and $D$ using `cv2.fisheye.calibrate` with the
`CALIB_RECOMPUTE_EXTRINSIC | CALIB_FIX_SKEW` flags. `CALIB_FIX_SKEW` constrains the off-diagonal
of $K$ to zero (the no-skew assumption), reducing the free parameters from 5 to 4.

The results are written under the `waveshare_rgb` key:

```yaml
waveshare_rgb:
  model: fisheye                 # Kannala-Brandt — use cv2.fisheye.* functions
  camera_matrix:                 # 3×3 intrinsic matrix K (row-major)
    - [fx,  0.0, cx]
    - [0.0, fy,  cy]
    - [0.0, 0.0, 1.0]
  dist_coeffs: [k1, k2, k3, k4] # Kannala-Brandt coefficients — NOT standard [k1,k2,p1,p2,k3]
  resolution: [640, 480]         # Resolution these parameters were measured at
  rms_reprojection_error: null   # RMS error (px) — filled in after calibration
```

Loading in Python and reconstructing the arrays:

```python
import numpy as np

K = np.array(cfg.waveshare_rgb.camera_matrix, dtype=np.float64)  # (3, 3)
D = np.array(cfg.waveshare_rgb.dist_coeffs,   dtype=np.float64)  # (4,)
```

Undistorting a frame for geometric computation:

```python
# Use cv2.fisheye.undistortImage — NOT cv2.undistort
undistorted = cv2.fisheye.undistortImage(frame, K, D, Knew=K)
```

Passing a fisheye $D$ to `cv2.undistort` (the non-fisheye function) will produce a wrong result
silently. The `model: fisheye` key in the YAML is the machine-readable indicator of which
function to use.

---

## 9. Reprojection Error — How You Know It Worked

### Definition

The quality of an intrinsic calibration is measured by **reprojection error**: take a corner
whose 3D board-frame position is known, project it into the image using the estimated $K$, $D$,
and the per-image pose, and compare the projected pixel to the actual detected corner position.

$$e_{ij} = \left\| \mathbf{p}_{ij}^\text{detected} - \hat{\mathbf{p}}(K, D, R_i, \mathbf{t}_i, \mathbf{P}_j) \right\|_2$$

The RMS across all corners in all images is the headline quality metric reported by
`cv2.fisheye.calibrate`.

### Acceptance Criteria

| Check | Target | Interpretation if missed |
|-------|--------|--------------------------|
| RMS reprojection error | < 1.0 px (ideally ≤ 0.5 px) | Poor coverage or blurry images |
| $c_x$ vs $W/2$ | within 10% of frame width | Wrong board size configured |
| $c_y$ vs $H/2$ | within 10% of frame height | Wrong board size configured |
| $\|f_x - f_y\| / \max$ | < 5% | Lens or mount issue |
| Per-image errors | no single image > 2× the mean | One bad image degrading the run |

### Visual Sanity Check

After calibration, `cv2.fisheye.undistortImage` applied to a test frame should make straight
lines in the scene (door frames, table edges) appear straight in the output image. Curved
straight edges are the most intuitive sign of a bad calibration.

---

## 10. Dependencies and Phase Ordering

The intrinsic calibration has no upstream measurement dependencies — it is the first calibration
step in Phase 2 and depends only on a printed checkerboard and the camera itself.

Everything downstream depends on it:

```
Intrinsic calibration (K, D)  →  Extrinsic calibration (T_C^L)  →  Phase 3 runtime
```

Specifically:
- **Extrinsic calibration** uses $K$ in its reprojection-error solver — an incorrect $K$
  propagates directly into the LiDAR-to-camera transform
- **Phase 3 heading error** uses $f_x$ from $K$ to convert pixel centroid offset to degrees
- **All geometric computation** on image coordinates requires $D$ to first undistort pixels

The `model: fisheye` field in `sensor_config.yaml` is the key runtime discriminator — any code
loading these values must select `cv2.fisheye.*` functions, not the standard `cv2` equivalents.
