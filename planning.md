# Planning — knot-losing-you

Autonomous UGV that follows you around the boat park using OAK-D Lite and LiDAR.
No sea legs required.

---

## Phase 0 — Baseplate Setup ✅

> Establish repo structure, tooling, and coding conventions.

- [x] Repo structure mirroring `now-you-sea-me`
- [x] `pyproject.toml` with all dependencies
- [x] `.pre-commit-config.yaml` (ruff, mypy, nbstripout)
- [x] GitHub Actions CI (linting + validation)
- [x] `.claude/` context files (project overview, code standards)
- [x] Config YAML files (pipeline, model, sensor)
- [x] Source stubs for all modules (perception, inference, control, utils)

---

## Phase 1 — Hardware Integration ✅

> Get real data flowing from each sensor on Raspberry Pi.

- [x] OAK-D Lite colour camera frames via DepthAI V3 (`CameraAccess`)
- [x] LiDAR scan data via serial (`LidarAccess`)
- [x] Waveshare UGV Rover basic drive commands (`UGVController`)
- [x] Verify sensor pipeline end-to-end with logging

---

## Phase 2 — Sensor Calibration ✅

> Characterise all sensors and actuators offline before any runtime code depends on their outputs.
> Each item produces measured parameters stored in `sensor_config.yaml` — no changes to existing
> source modules. All subsequent phases depend on these values being measured first.
>
> **Camera decision note:** If the Waveshare RGB on the pan-tilt is used for detection instead of
> (or in addition to) the OAK-D Lite, its intrinsics must also be calibrated here.
> Set `chassis_module: 2` in `sensor_config.yaml` before running pan-tilt calibration steps.

- [x] **Intrinsic calibration — detection camera**: OpenCV checkerboard routine
  (`cv2.calibrateCamera`) → focal lengths $f_x, f_y$, principal point $c_x, c_y$, and distortion
  coefficients $D$; stored in `sensor_config.yaml`
  _Skills: camera calibration techniques · intrinsic/extrinsic calibration algorithms · OpenCV_

- [x] **Angular offset calibration — LiDAR ↔ pan-tilt**: measure the single yaw scalar
  $\delta_\text{offset}$ between the LiDAR forward axis and the pan-tilt mechanical zero
  position; place a stationary target at the pan-tilt zero bearing, identify the corresponding
  return in the LiDAR scan, compute the angular difference; stored in `sensor_config.yaml`
  _Skills: intrinsic/extrinsic calibration algorithms · coordinate system conversions ·
  integrate and calibrate sensors_

- [x] **Pan-tilt servo curve calibration**: command a sweep of servo values, measure resulting pan
  angles; characterise the command-to-degrees mapping and dead-band; stored in `sensor_config.yaml`
  _Skills: actuator calibration · integrate and calibrate sensors_

- [x] **Rover drive calibration**: measure actual turn rate per unit angular velocity command;
  identify dead-band (minimum differential wheel speed that overcomes static friction);
  stored in `sensor_config.yaml`
  _Skills: motor/actuator calibration · real-time robotic platform integration_

---

## Phase 3 — Detection, Spatial Transforms & Following

> Detect and track the target; convert sensor data to robot-frame control signals; drive the rover.
> Reads Phase 2 calibration parameters from config — no hardcoded sensor values anywhere.

> **3A-lite checkpoint (recommended before deeper Phase 3 integration):**
> The goal is to lock one safe command path, not to fully design the final architecture.

- [x] **3A-lite — define one motion command shape**: standardise the final drive command payload
  (e.g., `linear_m_s`, `angular_rad_s`, `source`, `timestamp`)
- [x] **3A-lite — single command decision point**: ensure only one pipeline point chooses the final
  motion command before calling `UGVController`
- [x] **3A-lite — safety-first override rule**: emergency stop preempts all other motion commands
- [x] **3A-lite — minimal mode model**: implement only `AUTONOMOUS` and `MANUAL`, with a mandatory
  zero-velocity command on every mode transition
- [x] **3A-lite — one thin end-to-end run, then tune**: run with real sensors/inputs, log edge
  cases, then refine thresholds and payload fields before continuing the rest of Phase 3

- [x] YOLO detection with configurable target class (set via `model_config.yaml classes`; default: `0` = person)
- [x] Support custom trained model via `model_config.yaml` (drop-in replacement — e.g. jacket detector trained externally)
- [ ] **Coordinate system conversion — LiDAR**: convert polar scan returns $(r, \theta)$ →
  robot body-frame Cartesian $(x, y)$; extract the forward arc for range-to-target
  _Skills: spatial transformations and coordinate system conversions_

- [ ] **Spatial transformation — bounding box → angular heading**: use intrinsic matrix $K$
  (from Phase 2) to convert pixel centroid offset to angular heading error; apply tilt
  correction $\delta_\text{heading} = \arctan(\Delta u / f_x) \cdot \cos\psi_\text{tilt}$
  to account for image-plane rotation at non-zero tilt angles
  _Skills: spatial transformations · implement transformations for accurate geolocation of
  detected objects · OpenCV_

- [ ] **LiDAR distance → range-to-target**: compute search bearing
  $\theta_\text{search} = \delta_\text{offset} + \phi_\text{pan} + \delta_\text{heading}$;
  find nearest LiDAR return within a tolerance window around $\theta_\text{search}$; range
  $r$ of the matched return is the distance to target
  _Skills: spatial transformations and coordinate system conversions_

- [ ] **Pan-tilt inner control loop**: angular heading error → pan servo command (proportional
  controller using servo curve from Phase 2); keeps target centred in frame
  _Skills: actuator control · real-time robotic platform integration_

- [ ] **Rover outer control loop**: pan angle → chassis angular velocity command; when pan angle
  exceeds dead-band threshold the rover turns to re-centre; cascade on top of inner loop; uses
  drive calibration from Phase 2
  _Skills: motor control calibration · cascade controller design_

- [ ] **Proportional controller — linear velocity**: range-to-target → linear velocity command
  (stop at safe distance, creep when near, drive at full speed when far)

- [ ] Integrate all modules into `Pipeline._main_loop()`

- [ ] Live view overlay (bounding box, track ID, distance, pan angle)

---

## Phase 4 — Sensor Fusion, IMU & Stabilisation

> Add probabilistic state estimation and inertial corrections on top of the working Phase 3
> follower. Each item is an additive enhancement — the Phase 3 controller interface is unchanged.

- [ ] **IMU integration — OAK-D Lite BMI270**: new `ImuAccess` perception module; stream
  accelerometer + gyroscope via DepthAI V3; expose roll, pitch, and angular rates as structured data
  _Skills: integrate and calibrate sensors (IMUs)_

- [ ] **Platform tilt compensation**: use IMU roll/pitch to correct the bounding-box centroid
  before it feeds the pan-tilt and rover controllers; prevents erroneous steering on uneven dock
  surfaces
  _Skills: integrate and calibrate sensors (IMUs) · spatial transformations · sensor fusion_

- [ ] **Pan-tilt stabilisation**: IMU gyroscope angular rate as a feedforward signal to the pan
  servo controller; counteracts chassis vibration before it appears as image jitter or centroid
  noise
  _Skills: integrate and calibrate sensors (IMUs) · actuator control_

- [ ] **Kalman filter — target state estimation**: probabilistic fusion of YOLO detections
  (noisy, intermittent) and IMU-derived chassis motion (continuous); maintain a smoothed
  $(x, y, \dot{x}, \dot{y})$ state estimate in robot body frame; enables graceful tracking
  through short occlusions without controller thrashing
  _Skills: sensor fusion algorithms combining multiple data sources · implement and optimise
  real-time perception systems_

---

## Phase 5 — Obstacle Avoidance

> Stop or steer around obstacles detected by LiDAR. Builds on Phase 3 coordinate conversions.

- [ ] LiDAR scan → nearest obstacle distance + bearing (reuses Phase 3 polar → Cartesian conversion)
- [ ] Emergency stop if obstacle within safety radius
- [ ] Basic avoidance steering (go-around logic)

---

## Phase 6 — Manual Controller Mode

> Allow the rover to be driven manually via the Waveshare gamepad controller. Autonomous and
> manual modes are exclusive — a dedicated button toggles between them.

> **Hardware note:** The Waveshare controller may communicate directly with the ESP32
> sub-controller, bypassing the Pi entirely. Before implementing, verify whether the Pi can see
> the controller as a USB HID device (check `lsusb` / `ls /dev/input/`). If the controller talks
> only to the ESP32, we intercept at the Pi serial layer or remap a spare button to signal
> mode switching.

- [ ] Confirm controller connection path (USB HID to Pi vs. direct to ESP32 sub-controller)
- [ ] Read gamepad input on the Pi (`inputs` or `pygame` library)
- [ ] Dedicated button toggles **AUTONOMOUS ↔ MANUAL** mode (exclusive, not an override)
- [ ] Zero velocity before every mode transition (safety stop)
- [ ] Manual drive: map joystick axes → `UGVController` linear/angular velocity commands
- [ ] Log active mode clearly on each transition

---

## Phase 7 — Web Dashboard & Remote Control

> Lightweight web UI served from the Pi over its WiFi hotspot. Access from any phone or tablet
> via browser — no app install required. Primary use: toggle recording per camera when operating
> in the field without a laptop.

- [ ] FastAPI server running on boot (served over Pi WiFi hotspot)
- [ ] Mobile-friendly browser UI
- [ ] Toggle recording independently: Waveshare RGB camera and OAK-D Lite RGB camera
- [ ] Mode toggle: autonomous ↔ manual (mirrors the controller button)
- [ ] Status display: active mode, recording state per camera, battery level

---

## Phase 8 — Stretch Goals

> Nice-to-haves once the core loop is solid.

- [ ] **LiDAR scan deskewing**: use IMU angular rate to compensate for rover rotation during a
  LiDAR sweep; corrects arc distortion on scans acquired while the rover is turning
  _Skills: sensor fusion algorithms · spatial transformations and coordinate system conversions_

- [ ] OAK-D Lite depth data for closer-range obstacle sensing
- [ ] Target re-identification after occlusion
- [ ] MLflow experiment logging (confidence, latency, tracking metrics)
- [ ] Jupyter notebook for offline analysis of recorded sessions
- [ ] Extend web dashboard: live camera feed + telemetry overlay (bounding boxes, distance, confidence)
