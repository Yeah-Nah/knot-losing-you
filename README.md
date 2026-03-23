# knot-losing-you
Autonomous UGV that follows you around the boat park using OAK-D Lite and LiDAR. No sea legs required.

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
python capture_calibration_images.py
```

This zeros the pan-tilt servo and starts an HTTP server on port 8080. From your laptop browser:

- `http://<pi-ip>:8080/stream` — live MJPEG feed with checkerboard corners overlaid
- `http://<pi-ip>:8080/capture` — save the current frame (only saves if corners are detected); returns `{"saved": bool, "count": N}`
- `http://<pi-ip>:8080/status` — check save count and whether corners are currently visible

Collect at least 10 images with varied distances and angles. Press `Ctrl-C` on the Pi to stop. Images are saved to `calibration/images/`.

### Step 2 — Run the calibration (Pi or laptop)

```bash
python calibrate_waveshare_camera.py \
    --images calibration/images \
    --square-mm 28
```

On success, the calibration results (`camera_matrix`, `dist_coeffs`, `resolution`, `rms_reprojection_error`) are written into `configs/sensor_config.yaml` under the `waveshare_rgb` key.

---

## Running the Follower

```bash
python run_follower.py
```

Optional config overrides:

```bash
python run_follower.py \
    --pipeline-config configs/pipeline_config.yaml \
    --model-config configs/model_config.yaml \
    --sensor-config configs/sensor_config.yaml
```

---

## Troubleshooting

### Serial port in use (`/dev/ttyAMA0`)

Waveshare's `ugv_rpi/app.py` starts automatically on boot via crontab and holds the serial port. Any script that uses the UGV sub-controller (`check_battery.py`, `capture_calibration_images.py`, `run_follower.py`, etc.) will fail with a "device reports readiness to read but returned no data" error until it's stopped.

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
python check_battery.py

# Test OAK-D Lite camera — prints frame stats (use --no-display for headless)
python test_camera_access.py

# Test LiDAR connection — prints incoming packet breakdown
python test_lidar_access.py

# Test motor control — drives forward 0.2 m over 1 s
python test_ugv_controller.py
```
