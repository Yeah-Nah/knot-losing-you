# knot-losing-you
Autonomous UGV that follows you around the boat park using OAK-D Lite and LiDAR. No sea legs required.

## **Check out my progress updates here:** ([`👉 Progress Updates 👈`](PROGRESS_UPDATES.md))

### Technical Specifications for This Project
- Raspberry Pi 5
- Waveshare UGV Rover
- OAK-D Lite

---

## Setup

### Development Machine

From the repo root:

```bash
cd ugv-follower
pip install -e ".[dev]"
```

### Pi

This assumes the rover is set up and accessible over WiFi. SSH in and run:

```bash
# Clone the repo
git clone https://github.com/Yeah-Nah/knot-losing-you.git
cd knot-losing-you/ugv-follower

# Create and activate a venv (working directory must be ugv-follower/)
python3 -m venv .venv
source .venv/bin/activate

# Install the project
pip install -e .

# Grant non-root access to the OAK-D Lite (required once per device)
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

## Calibration

### Step 1 — Capture calibration images (on the Pi)

Before running, check `configs/calibration_config.yaml` and confirm `inner_corners` matches your checkerboard (squares − 1 in each dimension) and `square_mm` is the physical side length of one square.

The rover's main process may be holding the camera open. Kill it first if needed:

```bash
sudo fuser -k /dev/video0
```

Then start the capture server:

```bash
ugv-capture-calibration
```

This zeros the pan-tilt servo and starts an HTTP server on port 8080. From your laptop browser:

- `http://<pi-ip>:8080/stream` — live MJPEG feed with checkerboard corners overlaid
- `http://<pi-ip>:8080/capture` — save the current frame (only saves if corners are detected); returns `{"saved": bool, "count": N}`
- `http://<pi-ip>:8080/status` — check save count and whether corners are currently visible

Collect at least 10 images with varied distances and angles. Press `Ctrl-C` on the Pi to stop. Images are saved to `calibration/images/`.

### Step 2 — Run the calibration (Pi or laptop)

```bash
ugv-calibrate-camera --images calibration/images --square-mm 28
```

On success, the calibration results (`camera_matrix`, `dist_coeffs`, `resolution`, `rms_reprojection_error`) are written into `configs/sensor_config.yaml` under the `waveshare_rgb` key.

### Step 3 — Pan-tilt servo curve calibration (on the Pi)

Measures the command-to-angle mapping of the pan axis so Phase 3 control loops can
work in physical degrees rather than raw command units.

Before running, check `configs/calibration_config.yaml` section `pan_tilt_servo` and
confirm the sweep range and timing values suit your setup.

Start the calibration server:

```bash
ugv-calibrate-pantilt
```

This zeros the pan-tilt and starts an HTTP server on port 8080. From your laptop browser:

- `http://<pi-ip>:8080/stream` — live MJPEG feed with a green guide line at `cx`

Place a distinct stationary target (e.g. coloured tape on a flat wall) roughly 1–2 m ahead
of the rover. Physically rotate the rover until the target is bisected by the green line.

Then interact via HTTP (e.g. `curl http://<pi-ip>:8080/<endpoint>`):

- `http://<pi-ip>:8080/confirm` — start the commanded sweep (forward then reverse)
- `http://<pi-ip>:8080/status` — JSON progress: `{state, progress_pct, current_step, total_steps}`
- `http://<pi-ip>:8080/abort` — cancel without saving
- `http://<pi-ip>:8080/save` — write results to `configs/sensor_config.yaml`
- `http://<pi-ip>:8080/reset` — reset to WAITING_ZERO from FAILED or COMPLETE

The tool sweeps pan commands from `sweep_min_deg` to `sweep_max_deg` and back, collecting
`frames_to_average` settled frames at each step. Press `Ctrl-C` to stop.

On success, `configs/sensor_config.yaml` gains a `pan_tilt_servo` section with the
piecewise-linear model, linear fit, dead-band, hysteresis, and fit metrics. Raw samples
are saved to `calibration/pantilt_servo/<timestamp>.csv`.

**Offline refit from an existing CSV** (no hardware required):

```bash
ugv-calibrate-pantilt --replay calibration/pantilt_servo/<timestamp>.csv
ugv-calibrate-pantilt --replay calibration/pantilt_servo/<timestamp>.csv --save
```

---

## Running the Follower

```bash
ugv-run
```

Optional config overrides:

```bash
ugv-run \
    --pipeline-config configs/pipeline_config.yaml \
    --model-config configs/model_config.yaml \
    --sensor-config configs/sensor_config.yaml
```

---

## Troubleshooting

### Serial port in use (`/dev/ttyAMA0`)

Waveshare's `ugv_rpi/app.py` starts automatically on boot via crontab and holds the serial port. Any script that uses the UGV sub-controller (`ugv-check-battery`, `ugv-capture-calibration`, `ugv-run`, etc.) will fail with a "device reports readiness to read but returned no data" error until it's stopped.

**One-time fix — kill the process for this session:**

```bash
# Find out what is holding the process
lsof -t /dev/ttyAMA0

# Then kill it (PID 848 is a placeholder)
sudo kill -9 848
```

**Permanent fix — disable it from autostarting:**

```bash
crontab -e
```

Comment out the `app.py` line by adding a `#` at the start:

```
# @reboot XDG_RUNTIME_DIR=/run/user/1000 ~/ugv_rpi/ugv-env/bin/python ~/ugv_rpi/app.py >> ~/ugv.log 2>&1
```

Save and exit. The process will no longer start on boot. The Jupyter line in crontab can be left as-is or also commented out if you don't use it.

---

## Sensor & Hardware Tests

All test scripts are run from `ugv-follower/` on the Pi.

```bash
# Check battery level
ugv-check-battery

# Test OAK-D Lite camera — prints frame stats (use --no-display for headless)
python -m tests.test_camera_access

# Test LiDAR connection — prints incoming packet breakdown
python -m tests.test_lidar_access

# Test motor control — drives forward 0.2 m over 1 s
python -m tests.test_ugv_controller
```
