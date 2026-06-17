# knot-losing-you
This project builds an autonomous person-following rover for use in a marina environment. A Waveshare UGV Rover running on a Raspberry Pi 5 uses an OAK-D Lite camera and a 2D LiDAR to detect and track a target person, then drive to maintain a set following distance and heading. The system includes a full calibration pipeline (camera intrinsics, pan-tilt servo curve, LiDAR-to-camera extrinsic offset, and drive model), a safe control loop with mode switching and emergency stop, and a YOLO-based object detection stage that is currently being integrated to close the autonomy loop.

## **Check out my progress updates here:** ([`👉 Progress Updates 👈`](PROGRESS_UPDATES.md))

### Technical Specifications for This Project

**Supplied separately:**
- Raspberry Pi 5
- OAK-D Lite

**[Waveshare UGV Rover kit](https://www.waveshare.com/ugv-rover.htm) included:**
- Waveshare UGV Rover (6WD chassis)
- LDRobot D500 LiDAR
- Waveshare Pan-Tilt Module

---

## Firmware

The stock firmware in the Waveshare UGV Rover's drive controller has a serial communication bug that prevents this project from working correctly. The [`waveshareteam/ugv_base_general`](https://github.com/waveshareteam/ugv_base_general) repository was forked and patched to add flushing functionality to `wFlushSCS` in the SCServo library, and the ESP32 drive controller was reflashed with the updated firmware. The patched fork is available at [`Yeah-Nah/ugv_base_general`](https://github.com/Yeah-Nah/ugv_base_general). **The drive controller must be flashed from this fork before any of the software in this repo will function correctly.**

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

Camera preflight is now automatic for camera-based calibration tools. If `/dev/video0` is held open, the script attempts to release it before opening `VideoCapture`.

If your user cannot terminate the holder process, stop it manually once and then re-run:

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
confirm the sweep range and timing values suit your setup. Set `precondition_cycles`
to the number of full forward+reverse passes to run before measurements are recorded
(default `1`) — set to `0` to skip warmup entirely.

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

### Step 4 — Angular offset calibration (on the Pi)

Measures the signed yaw offset between the LiDAR's 0° forward axis and the pan-tilt
camera's mechanical zero. Intrinsic calibration (Step 2) must be completed first.

```bash
ugv-calibrate-angular --distance 2.0
```

This zeros the pan-tilt and starts an HTTP server on port 8080. From your laptop browser:

- `http://<pi-ip>:8080/stream` — live MJPEG feed with a green guide line at `cx`

Place a flat-faced cardboard box (≈ 20 cm wide) roughly `--distance` metres directly in
front of the rover. Physically slide it left or right until its face is bisected by the
green line.

Then interact via HTTP (e.g. `curl http://<pi-ip>:8080/<endpoint>`):

- `http://<pi-ip>:8080/confirm` — start the LiDAR scan
- `http://<pi-ip>:8080/status` — JSON progress: `{state, ...}`
- `http://<pi-ip>:8080/abort` — cancel without saving
- `http://<pi-ip>:8080/save` — write result to `configs/sensor_config.yaml`

On success, `configs/sensor_config.yaml` gains an `extrinsic` section with
`lidar_to_pantilt_offset_deg`.

### Step 5 — Rover drive calibration (on the Pi)

Measures the rover's actual turn-rate response to commanded angular velocity and
identifies the angular dead-band. Intrinsic calibration (Step 2) must be completed first.

Before running, check `configs/calibration_config.yaml` and confirm `target_distance_m`
and other drive calibration parameters suit your setup.

```bash
ugv-calibrate-drive
```

This starts an HTTP server on port 8080. From your laptop browser:

- `http://<pi-ip>:8080/stream` — live MJPEG feed with a crosshair guide at `cx`

Place a high-contrast stationary target (e.g. tape cross or printed pattern) approximately
`target_distance_m` in front of the rover and verify it is visible and centred on the
crosshair.

Then interact via HTTP (e.g. `curl http://<pi-ip>:8080/<endpoint>`):

- `http://<pi-ip>:8080/confirm` — start the sweep
- `http://<pi-ip>:8080/status` — JSON progress: `{state, progress_pct, current_step, total_steps}`
- `http://<pi-ip>:8080/abort` — cancel without saving
- `http://<pi-ip>:8080/save` — write results to `configs/sensor_config.yaml`
- `http://<pi-ip>:8080/reset` — reset to WAITING_TARGET from FAILED or COMPLETE

The tool sweeps four directional blocks (gain CCW, gain CW, dead-band CCW, dead-band CW).
You will be prompted to re-centre the rover between blocks. On success,
`configs/sensor_config.yaml` gains a `ugv_drive` section with `effective_track_width_m`,
`turn_rate_gain`, and `angular_dead_band_rad_s`. Raw samples are saved to
`calibration/ugv_drive/<timestamp>.csv`.

**Offline refit from an existing CSV** (no hardware required):

```bash
ugv-calibrate-drive --replay calibration/ugv_drive/<timestamp>.csv
ugv-calibrate-drive --replay calibration/ugv_drive/<timestamp>.csv --save
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

### Camera device in use (`/dev/video0`)

Camera-based calibration commands now run an automatic preflight that checks whether
the camera device is busy and attempts to release it. If this still fails, the holder
process is usually started at boot by another service.

Use these checks:

```bash
# Show process IDs holding the camera device
lsof -t /dev/video0

# Kill holders manually (only needed if automatic preflight cannot)
sudo fuser -k /dev/video0
```

For a permanent fix, disable the process that auto-starts and takes ownership of
the camera device.

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

# Test OAK-D Lite camera — prints frame stats
ugv-check-camera

# Test LiDAR connection — prints incoming packet breakdown
ugv-check-lidar

# Test motor control — drives forward 0.2 m over 1 s
ugv-check-controller
```
