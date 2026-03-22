# Intrinsic Camera Calibration — Reference

Covers the OpenCV checkerboard calibration routine used in Phase 2 of the project.
Target camera: **Waveshare RGB (pan-tilt module)**.

---

## 1. The Pinhole Camera Model

### Physical Intuition

Imagine a completely dark box with a single tiny hole (the "pinhole") in one face, and a flat
sensor on the opposite face. Light from a scene point travels in a straight line through the hole
and lands on the sensor. No lens, no glass — just geometry.

The key insight is that **all projection rays pass through a single point** — the optical centre
(also called the camera centre or projection centre). This is what makes the math tractable.

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

The sensor sits at distance $f$ (the **focal length**) behind the optical centre.

---

### Step 1 — Similar Triangles Give You the Projection

Consider a world point $P = (X, Y, Z)$ in **camera coordinates** (origin at the optical centre,
Z pointing forward along the optical axis). It projects to a point $p = (x, y)$ on the sensor.

By similar triangles on the Y axis:

$$\frac{y}{f} = \frac{Y}{Z} \implies y = f \cdot \frac{Y}{Z}$$

And identically for X:

$$x = f \cdot \frac{X}{Z}$$

This is the **ideal pinhole projection**. The division by $Z$ is what makes it **perspective
projection** — things farther away appear smaller.

---

### Step 2 — Physical Metres to Pixels

The sensor is not measured in metres — it is measured in **pixels**. Each pixel has a physical
size: $d_x$ (metres per pixel horizontally) and $d_y$ (metres per pixel vertically).

Converting the metre-valued projection to pixel coordinates:

$$u = \frac{x}{d_x} = \frac{f}{d_x} \cdot \frac{X}{Z}, \qquad v = \frac{y}{d_y} = \frac{f}{d_y} \cdot \frac{Y}{Z}$$

Now define:

$$f_x = \frac{f}{d_x}, \qquad f_y = \frac{f}{d_y}$$

These are the **focal lengths in pixels** — the only form that appears in $K$. You never need to
separately know $f$ in millimetres or $d_x$ in micrometres; their ratio is what matters, and
`cv2.calibrateCamera` solves for $f_x$ and $f_y$ directly.

For most cameras $d_x \approx d_y$ (square pixels), so $f_x \approx f_y$. A significant
difference between them usually indicates something went wrong in calibration.

---

### Step 3 — The Principal Point

The optical axis hits the sensor at the **principal point** $(c_x, c_y)$ in pixel coordinates.
In an ideal camera this is exactly the image centre. In reality it is slightly off due to
manufacturing tolerances — typically within 1–2% of the image dimension.

Accounting for this offset:

$$u = f_x \cdot \frac{X}{Z} + c_x, \qquad v = f_y \cdot \frac{Y}{Z} + c_y$$

### Conversion Chain: Steps 1 → 2 → 3

$$\underbrace{x = f \cdot \frac{X}{Z}}_{\text{Step 1: geometry (metres)}}
\xrightarrow{\div\, d_x}
\underbrace{u = \frac{f}{d_x} \cdot \frac{X}{Z} = f_x \cdot \frac{X}{Z}}_{\text{Step 2: pixel units, origin at optical axis}}
\xrightarrow{+\, c_x}
\underbrace{u = f_x \cdot \frac{X}{Z} + c_x}_{\text{Step 3: pixel units, origin at top-left}}$$

$$\underbrace{y = f \cdot \frac{Y}{Z}}_{\text{Step 1: geometry (metres)}}
\xrightarrow{\div\, d_y}
\underbrace{v = \frac{f}{d_y} \cdot \frac{Y}{Z} = f_y \cdot \frac{Y}{Z}}_{\text{Step 2: pixel units, origin at optical axis}}
\xrightarrow{+\, c_y}
\underbrace{v = f_y \cdot \frac{Y}{Z} + c_y}_{\text{Step 3: pixel units, origin at top-left}}$$

---

### Step 4 — The Intrinsic Matrix $K$

Multiply through by $Z$ and write in homogeneous form:

$$Z \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \underbrace{\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}}_{K} \begin{bmatrix} X \\ Y \\ Z \end{bmatrix}$$

This is why $K$ has that exact structure. The zeros in the off-diagonal are the "no skew"
assumption (pixel axes are perpendicular) — true for virtually all modern sensors.

---

### The Full Projection Chain

Combining the intrinsic matrix with the camera's pose in the world:

$$s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \underbrace{\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}}_{K} \underbrace{\begin{bmatrix} R & \mathbf{t} \end{bmatrix}}_{[\text{R|t}]} \begin{bmatrix} X_w \\ Y_w \\ Z_w \\ 1 \end{bmatrix}$$

Where:
- $K$ is the **intrinsic matrix** — the thing you are solving for
- $[R \mid \mathbf{t}]$ is the **extrinsic matrix** — the camera's pose relative to the world
  (differs for every photo)
- $s$ is a scalar (the depth, i.e. Z in camera coordinates)

---

### Lens Distortion — $D$

Real lenses are not perfect pinholes. OpenCV models two kinds:

**Radial distortion** (barrel/pincushion — the dominant term):

$$x_d = x(1 + k_1 r^2 + k_2 r^4 + k_3 r^6)$$

**Tangential distortion** (lens not perfectly parallel to the sensor):

$$x_{dt} = x + \bigl[2p_1 xy + p_2(r^2 + 2x^2)\bigr]$$

$$y_{dt} = y + \bigl[p_1(r^2 + 2y^2) + 2p_2 xy\bigr]$$

where $r^2 = x^2 + y^2$ in **normalised image coordinates** (after dividing by $f$ and
subtracting $c_x, c_y$). The distortion vector is:

$$D = \begin{bmatrix} k_1 & k_2 & p_1 & p_2 & k_3 \end{bmatrix}$$

---

### Why You Cannot Just Use One Image

With a single image and an unknown pose, the system is **underdetermined**. You could explain the
observed corners with a large $f_x$ and a far-away board, or a small $f_x$ and a close-up board —
the observations alone do not distinguish these. Multiple images at different distances and angles
break this ambiguity. This is also why image variety matters more than image count.

---

### What the Numbers Will Look Like for Your Camera

The Waveshare RGB camera is a typical low-cost USB webcam. At 640×480 expect roughly:

| Parameter | Typical value | Meaning |
|-----------|--------------|---------|
| $f_x$ | 500–700 px | Horizontal focal length |
| $f_y$ | 500–700 px | Vertical focal length (≈ $f_x$) |
| $c_x$ | ~320 px | Near horizontal centre |
| $c_y$ | ~240 px | Near vertical centre |
| $k_1$ | −0.3 to +0.1 | Dominant radial distortion term |

If $f_x$ comes out at 600 px and the pixel size is ~3 µm, the physical focal length is
$600 \times 0.003\text{ mm} = 1.8\text{ mm}$ — consistent with a tiny webcam lens.

---

## 2. Why a Checkerboard?

The calibration problem is: given a set of known 3D points and their observed 2D pixel positions,
find $K$ and $D$. You need a target with **precisely known geometry**.

A checkerboard (or chessboard) is ideal because:
- The inner corner positions are **algebraically exact** — no printing imprecision in the geometry,
  only in the scale
- `cv2.findChessboardCorners` + `cv2.cornerSubPix` can locate corners to **sub-pixel accuracy**
  (typically ~0.1 px)
- The target is flat (planar), which enables Zhang's method (see §3)

### The Board's Own Coordinate Frame

A coordinate frame is an origin point plus three axes (X, Y, Z). Normally you might think of the
world frame as "the room" or "the floor" — but for calibration, you define it as **the board
itself**:

- **Origin**: the top-left inner corner of the checkerboard
- **X axis**: runs along the top row of corners (rightward)
- **Y axis**: runs down the left column of corners (downward)
- **Z axis**: points straight out of the board's surface (perpendicular to it)

Since the board is a flat, rigid, physical object, every single corner on it has $Z = 0$ by
definition — they all lie in the same flat plane. A 9×6 inner-corner board with 30 mm squares
gives object points:

$$\{(0,0,0),\ (30,0,0),\ (60,0,0),\ \ldots,\ (240, 150, 0)\} \text{ mm}$$

You know these coordinates **exactly** — you measured the square size when you printed the board.

### What You Are Actually Giving OpenCV

For every calibration image you provide two lists:

| `object_points` | `image_points` |
|---|---|
| Where each corner **is in the world** (mm, in board frame) | Where each corner **appeared in the photo** (pixels) |
| Known before you even turn the camera on | Detected by `findChessboardCorners` |

The `object_points` list is **identical for every image** — the board doesn't change shape. What
changes between images is where the camera was relative to the board (or equivalently, where the
board was relative to the camera).

### What OpenCV Is Actually Solving

For each image, OpenCV asks: *"Given that these 54 corners are at those known 3D positions in the
board frame, and they appeared at these pixel locations — what was the camera's position and
orientation $(R_i, \mathbf{t}_i)$ relative to the board, and what are $K$ and $D$?"*

The translation vector $\mathbf{t}_i$ from that solution encodes where the board was in camera
space — its Z component is effectively the depth you never had to measure. OpenCV recovers it as
a byproduct of matching the 2D observations to the known 3D geometry. You never had to measure
the camera-to-board distance with a tape measure.

The key insight is that you didn't avoid the depth (Z) problem — you **sidestepped it** by
choosing a coordinate system where Z is trivially known (zero for every point), and letting the
algorithm figure out the full 3D pose as a byproduct. The board's rigid, flat, known structure
is what makes this possible. A random cluster of points floating in space at unknown depths
couldn't do this.

### Physical Target Tips

- **Print on paper, mount flat on rigid backing** (clipboard, MDF, foam board) — any warp biases
  distortion coefficients
- Foam board from a print shop is the best low-effort option
- For the Waveshare RGB camera: **lock the pan-tilt to centre (0°, 0°)** before calibrating and
  do not move it during the session; intrinsics are fixed to the lens regardless of pan/tilt angle

---

## 3. Zhang's Method — The Math

OpenCV uses **Zhang's planar homography method** (Zhang 2000).

### What is a Homography?

A **homography** is a $3 \times 3$ projective transformation matrix — defined only up to an
overall scale factor — that maps points from one **flat plane** to points in another plane. It is
the right tool here precisely because the checkerboard is flat: all corners lie in a single plane
with $Z = 0$.

$$s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = H \begin{bmatrix} X \\ Y \\ 1 \end{bmatrix}, \qquad H = \begin{bmatrix} h_{11} & h_{12} & h_{13} \\ h_{21} & h_{22} & h_{23} \\ h_{31} & h_{32} & h_{33} \end{bmatrix}$$

Because $H$ is only defined up to scale (multiplying every entry by the same constant gives an
identical mapping), it has **8 degrees of freedom** despite having 9 entries. Each point
correspondence $(X, Y) \to (u, v)$ provides 2 equations, so you need at least **4 non-collinear
point pairs** to uniquely determine $H$. A calibration image with 54 inner corners is heavily
overdetermined — a feature, not a luxury.

### The Key Factorisation

The reason homographies are so useful here is that they **factor neatly into camera intrinsics
and a per-image pose**. When $Z = 0$ on the board, the third column of $[R \mid \mathbf{t}]$
drops out of the full projection equation, reducing it to:

$$H = K \begin{bmatrix} \mathbf{r}_1 & \mathbf{r}_2 & \mathbf{t} \end{bmatrix}$$

where $\mathbf{r}_1$ and $\mathbf{r}_2$ are the first two columns of the rotation matrix $R$,
and $\mathbf{t}$ is the translation vector. Both $K$ and the per-image pose are folded into a
single measurable matrix $H$:

- $K$ is **the same in every image** — it is a physical property of the lens and sensor
- $[\mathbf{r}_1\ \mathbf{r}_2\ \mathbf{t}]$ **changes for every image** — the board had a
  different position and orientation in each shot

This is the central insight of Zhang's method: computing one $H$ per image gives you **multiple
independent windows into the same unknown $K$**. The rigid constraints that $\mathbf{r}_1$ and
$\mathbf{r}_2$ must satisfy (being orthonormal rotation columns) then let you disentangle $K$
from those views using only linear algebra.

### Step 1 — Computing Each Homography (DLT)

For each calibration image, $H$ is computed from the known 3D corner positions and their
observed pixel locations using the **Direct Linear Transform (DLT)**.

Each point correspondence $(X_j,\ Y_j) \to (u_j,\ v_j)$ produces two linear equations by
expanding the homography definition and cross-multiplying to eliminate the denominator:

$$u_j \cdot (h_{31}X_j + h_{32}Y_j + h_{33}) = h_{11}X_j + h_{12}Y_j + h_{13}$$
$$v_j \cdot (h_{31}X_j + h_{32}Y_j + h_{33}) = h_{21}X_j + h_{22}Y_j + h_{23}$$

Stacking all $M$ point pairs into a single matrix equation $A\mathbf{h} = \mathbf{0}$ (where
$\mathbf{h}$ is the 9-entry column vector of $H$'s entries) and solving via **SVD** yields
$\mathbf{h}$ as the last right singular vector — the unit-norm least-squares solution. This is
done independently for each calibration image, producing one $H_i$ per image.

### Step 2 — Extract Intrinsic Constraints from Each $H$

Let $H = [\mathbf{h}_1\ \mathbf{h}_2\ \mathbf{h}_3]$. Since $\mathbf{r}_1$ and $\mathbf{r}_2$
are orthonormal rotation columns:

$$\mathbf{r}_1 \cdot \mathbf{r}_2 = 0 \implies \mathbf{h}_1^\top K^{-\top} K^{-1} \mathbf{h}_2 = 0$$

$$\|\mathbf{r}_1\| = \|\mathbf{r}_2\| \implies \mathbf{h}_1^\top K^{-\top} K^{-1} \mathbf{h}_1 = \mathbf{h}_2^\top K^{-\top} K^{-1} \mathbf{h}_2$$

This gives **2 linear constraints per image** on the symmetric matrix $B = K^{-\top} K^{-1}$
(the Image of the Absolute Conic). $B$ has 5 unknowns (matching the 5 intrinsic parameters).
With $n \geq 3$ images: $2n \geq 6$ constraints → solve for $B$ → Cholesky decompose to
recover $K$.

### Step 3 — Nonlinear Refinement

The linear solution is just an initialisation. `cv2.calibrateCamera` runs
**Levenberg–Marquardt** to minimise total **reprojection error** across all images jointly:

$$\min_{K,\, D,\, \{R_i,\, \mathbf{t}_i\}} \sum_{i=1}^{N} \sum_{j=1}^{M} \left\| \mathbf{p}_{ij} - \hat{\mathbf{p}}(K, D, R_i, \mathbf{t}_i, \mathbf{P}_j) \right\|^2$$

where $\hat{\mathbf{p}}$ is the full projection including distortion. The RMS of this (in pixels)
is the **reprojection error** — your primary quality metric.

---

## 4. Capture Strategy

This is where most calibrations go wrong. Rules of thumb:

1. **15–30 images minimum** — fewer gives poor distortion estimates
2. **Vary the angle** — tilt 30–45° around both axes, rotate in-plane, move closer/farther
3. **Fill the frame** — corners in all quadrants, especially near the image edges where
   distortion is worst
4. **No blur** — motion blur corrupts sub-pixel corner detection; use good lighting
5. **Keep the board flat** — any warp in a printed-paper board biases the result
6. **Don't move in one plane only** — Zhang's method degenerates if all views are related by
   pure rotation

---

## 5. Implementation

Since you have decided on the **Waveshare RGB camera**, it presents as a standard USB webcam and
is accessed via `cv2.VideoCapture`, not DepthAI. The existing `CameraAccess` class is not used
for calibration.

### `sensor_config.yaml` — New Section to Add

```yaml
# Waveshare RGB (pan-tilt) camera — intrinsic calibration results
waveshare_rgb:
  camera_matrix:       # 3x3 intrinsic matrix K (row-major)
    - [fx,  0.0, cx]
    - [0.0, fy,  cy]
    - [0.0, 0.0, 1.0]
  dist_coeffs: [k1, k2, p1, p2, k3]   # OpenCV distortion vector
  resolution: [640, 480]               # Resolution these parameters were measured at
  rms_reprojection_error: null         # RMS error (px) — filled in after calibration
```

### Calibration Script — `calibrate_waveshare_camera.py`

**Corner detection helper:**
```python
def find_corners(image, board_size):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, board_size, None)
    if not ret:
        return False, None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners
```

**`cornerSubPix` refines the coarse corner location to sub-pixel accuracy** by finding the point
where the dot product of the image gradient and the displacement vector is zero — i.e. where the
gradient is perpendicular to the displacement in all directions, which only holds at a true
corner.

**Calibration call:**
```python
# Build identical object-point template for every image (Z=0 everywhere)
objp = np.zeros((board_w * board_h, 3), dtype=np.float32)
objp[:, :2] = np.mgrid[0:board_w, 0:board_h].T.reshape(-1, 2)
objp *= square_mm

object_points = [objp] * len(image_points)  # same board geometry for every image

rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
    object_points, image_points, image_size, None, None
)
```

**Live capture controls (on Pi with monitor/VNC):**
```
SPACE  — capture frame if corners are found
d      — discard last captured frame
q      — quit capture and run calibration
```

**Headless Pi workflow:** capture images separately (e.g. a short script that saves frames on
keypress over SSH), copy them to your laptop, then run the calibration script pointing at the
image folder.

---

## 6. Where This Fits in the Pipeline

Once calibration is done, $K$ and $D$ feed directly into Phase 3:

- $K$ is used to convert a **bounding-box pixel centroid offset** → **angular heading error**
  (degrees left/right of frame centre) via the inverse of the focal-length projection
- $D$ is used to **undistort** pixel coordinates before any geometric computation

The pattern already established in the codebase — load YAML → pydantic settings model — means
you would add a `WaveshareRGBSettings` model in `settings.py` with `camera_matrix` and
`dist_coeffs` fields loaded from `sensor_config.yaml`. No hardcoded values anywhere.

---

## 7. Testing

### Acceptance Criteria

| Check | Target |
|-------|--------|
| RMS reprojection error | < 1.0 px (ideally ≤ 0.5 px) |
| $c_x$ vs $W/2$ | within ~10% of frame width |
| $c_y$ vs $H/2$ | within ~10% of frame height |
| $f_x \approx f_y$ | within ~5% (square pixels) |
| $k_1$ magnitude | 0.0–0.4; very large values indicate a bad run |
| Per-image errors | no single image > 2× the mean |

### Sanity Checks in Code

```python
W, H = 640, 480
K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
D = np.array([k1, k2, p1, p2, k3], dtype=np.float64)

# 1 — Principal point near image centre
assert 0.4 * W < K[0, 2] < 0.6 * W, "cx way off-centre"
assert 0.4 * H < K[1, 2] < 0.6 * H, "cy way off-centre"

# 2 — Square pixels
assert abs(K[0, 0] - K[1, 1]) / K[0, 0] < 0.05, "fx and fy differ >5%"

# 3 — Visual check: straight lines in the scene should be straight after undistortion
undistorted = cv2.undistort(test_frame, K, D)
cv2.imshow("Undistorted", undistorted)
```

### Visual Sanity Check

`cv2.undistort` with your $K$ and $D$ should make straight lines in the scene (door frames,
table edges, walls) appear straight in the output image. This is the fastest way to spot a bad
calibration without needing more maths.

---

## Summary Flow

```
Print board → mount flat on rigid backing → lock pan-tilt to centre
    ↓
Capture 20+ images: varied angles, distances, all parts of the frame covered
    ↓
findChessboardCorners + cornerSubPix  →  sub-pixel corner locations
    ↓
cv2.calibrateCamera  →  K, D, per-image rvecs/tvecs
    ↓
Check RMS < 1.0 px and all sanity assertions pass
    ↓
Save K, D, resolution, RMS to sensor_config.yaml under waveshare_rgb
    ↓
Phase 3: K converts pixel centroid offset → angular heading error
```

> **Key practical tip:** image variety beats image count. Ten images from wildly different angles
> and distances will outperform thirty near-identical frontal shots.

---

## How to Run

Full operator workflow from zero to calibrated camera.

### Prerequisites

- Pi is powered on and accessible via SSH
- Checkerboard is printed and mounted flat on a rigid backing (clipboard or foam board)
- You are in the `ugv-follower/` directory and the project venv is active

```bash
# On the Pi
cd ~/git/knot-losing-you/ugv-follower
source .venv/bin/activate
```

- Confirm `configs/calibration_config.yaml` matches your hardware:
  - `checkerboard.inner_corners` — must match the printed board (default: `[9, 6]`)
  - `checkerboard.square_mm` — physical square size in mm (default: `28.0`)
  - `camera.device_index` — Waveshare RGB camera index (default: `0`)
  - `ugv.port` — serial port for pan-tilt (default: `/dev/ttyAMA0` for Pi 5)

### Step 1 — Start the capture script

```bash
python capture_calibration_images.py
```

The script will:
1. Zero the pan-tilt servo to (0°, 0°) via serial
2. Open the camera
3. Start an HTTP server on port 8080

Expected output:
```
INFO | Pan-tilt zeroed. Serial connection closed.
INFO | Camera opened.
INFO | HTTP server running on port 8080. Open http://<pi-ip>:8080/stream in your browser.
```

### Step 2 — Open the live stream

On your laptop, open a browser and navigate to:

```
http://<pi-ip>:8080/stream
```

You should see the live camera feed. When the checkerboard is detected, coloured corner markers will be overlaid on the image.

### Step 3 — Collect 20–30 images

Trigger captures by hitting the `/capture` endpoint — corners **must** be visible for a frame to be saved:

```bash
# From your laptop (repeat for each capture)
curl http://<pi-ip>:8080/capture

# Check progress at any time
curl http://<pi-ip>:8080/status
```

Example responses:
```json
{"saved": true,  "count": 7}   # corners detected — frame saved
{"saved": false, "count": 7}   # no corners — move the board and try again
{"count": 7, "corners_visible": true}   # /status
```

**Coverage tips for a good calibration:**
- Fill different parts of the frame (top-left, top-right, centre, corners)
- Vary the tilt angle of the board (±30–45° in both axes)
- Vary the distance (close, mid, far)
- Avoid motion blur — hold the board still before capturing
- Target 20–30 accepted images

### Step 4 — Stop the capture script

Press **Ctrl-C** on the Pi. The script prints the total saved count and exits cleanly.

### Step 5 — Run the calibration script

Run on the Pi (or SCP `calibration/images/` to your laptop first):

```bash
python calibrate_waveshare_camera.py \
    --images calibration/images \
    --square-mm 28
```

The script will:
1. Detect corners in all saved images (skipping any where detection fails)
2. Run `cv2.calibrateCamera`
3. Validate results against acceptance criteria
4. Print a results summary
5. Write K, D, resolution, and RMS into `configs/sensor_config.yaml` under `waveshare_rgb`

Example output on success:
```
--- Calibration results ---
  Images used   : 24
  Resolution    : 1280×720 px
  RMS error     : 0.312 px
  fx, fy        : 891.43, 889.77
  cx, cy        : 637.21, 360.84
  dist_coeffs   : [-0.312, 0.089, 0.001, -0.002, -0.011]

INFO | Validation passed.
Results written to configs/sensor_config.yaml under 'waveshare_rgb'.
```

### Step 6 — Verify the output

```bash
grep -A 8 "waveshare_rgb" configs/sensor_config.yaml
```

Confirm:
- `camera_matrix` contains a 3×3 list (not `null`)
- `rms_reprojection_error` is below 1.0
- `resolution` matches the camera output resolution

### Acceptance criteria

| Check | Threshold | Why |
|-------|-----------|-----|
| RMS reprojection error | < 1.0 px | > 1.0 px suggests poor coverage or a warped board |
| Principal point (cx, cy) | Within 10% of frame centre | Far off-centre suggests the wrong board size was configured |
| Focal length symmetry \|fx−fy\| / max | < 5% | Large asymmetry suggests lens or mount issues |

### Troubleshooting

**No corners detected in the stream:** Check lighting (avoid glare on the board), ensure the full board is visible, and confirm `inner_corners` in `calibration_config.yaml` matches the printed board.

**RMS > 1.0 px:** Recapture with more varied angles and better frame coverage. Discard any blurry images from `calibration/images/` and re-run the calibration script.

**`cv2.VideoCapture` fails to open:** Try `device_index: 1` (or higher) in `calibration_config.yaml` if another camera (e.g. the OAK-D) is assigned index 0.

**Serial error on pan-tilt zero:** Confirm `ugv.port` in `calibration_config.yaml` is correct and the UGV is powered on. The script will still proceed to camera capture if you Ctrl-C past the serial step, but the pan-tilt will not be zeroed.
